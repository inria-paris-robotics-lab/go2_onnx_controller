#include "controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <memory>
#include <string>

#include "motor_crc.hpp"
#include "onnx_actor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interface.hpp"

using namespace std::chrono_literals;

std::string get_model_path() {
  std::string home = std::getenv("HOME");

  if (home.empty()) {
    std::cerr << "Error: HOME environment variable not set." << std::endl;
  }

  std::string model_path = home + "/.local/share/.onnx-actor/model.onnx";

  return model_path;
}

ONNXController::ONNXController()
    : Node("onnx_controller"), joy_(std::make_shared<sensor_msgs::msg::Joy>()),
      obs_act_(std::make_shared<onnx_interfaces::msg::ObservationAction>()) {
  actor_ = std::make_unique<ONNXActor>(get_model_path(), observation_, action_),
  // Set up the robot interface
      robot_interface_ = std::make_unique<Go2RobotInterface>(
          *this, isaac_joint_names_, isaac_feet_names_);

  // Set parameters
  this->declare_parameter("kp", kp_);
  this->declare_parameter("kd", kd_);

  auto param_change_callback =
      [this](const std::vector<rclcpp::Parameter> &params) {
        return set_param_callback(params);
      };

  parameter_callback_handle_ =
      this->add_on_set_parameters_callback(param_change_callback);

  // Subscribe to the Joy topic
  auto joy_callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
    consume(msg);
  };

  joy_subscription_ =
      this->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joy_callback);

  // Print the ONNXActor
  actor_->print_model_info();

  // Print vectors
  print_vecs();

  RCLCPP_INFO(this->get_logger(),
              "ONNXController initialised, going to initial "
              "pose and waiting for Joy message.");

  // Go to the initial pose
  std::array<float, 12> q_des{};
  for (size_t i = 0; i < 12; i++) {
    q_des[i] = q0_[i];
  }
  robot_interface_->go_to_configuration(q_des, 5.0);

  // Set the timer to publish at 50 Hz
  timer_ =
      this->create_wall_timer(20ms, std::bind(&ONNXController::publish, this));
}

void ONNXController::consume(const sensor_msgs::msg::Joy::SharedPtr msg) {
  // Copy the Joy message
  joy_ = msg;
}

void ONNXController::print_vecs() {
  // Print observation and action
  std::cout << "Observation: ";
  for (size_t i = 0; i < observation_.size(); i++) {
    std::cout << observation_[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "gravity_b_hist_: ";
  for (size_t i = 0; i < gravity_b_hist_.size(); i++) {
    std::cout << gravity_b_hist_[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "base_ang_vel_hist_: ";
  for (size_t i = 0; i < base_ang_vel_hist_.size(); i++) {
    std::cout << base_ang_vel_hist_[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "imu_lin_acc_hist_: ";
  for (size_t i = 0; i < imu_lin_acc_hist_.size(); i++) {
    std::cout << imu_lin_acc_hist_[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "vel_cmd_hist_: ";
  for (size_t i = 0; i < vel_cmd_hist_.size(); i++) {
    std::cout << vel_cmd_hist_[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "q_hist_: ";
  for (size_t i = 0; i < q_hist_.size(); i++) {
    std::cout << q_hist_[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "dq_hist_: ";
  for (size_t i = 0; i < dq_hist_.size(); i++) {
    std::cout << dq_hist_[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "action_hist_: ";
  for (size_t i = 0; i < action_hist_.size(); i++) {
    std::cout << action_hist_[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "foot_forces_hist_: ";
  for (size_t i = 0; i < foot_forces_hist_.size(); i++) {
    std::cout << action_hist_[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "Action: " << std::endl;
  for (size_t i = 0; i < action_.size(); i++) {
    std::cout << i << ": " << action_[i] << std::endl;
  }
  std::cout << std::endl;

  std::cout << "Velocity command: " << std::endl;
  for (size_t i = 0; i < vel_cmd_.size(); i++) {
    std::cout << i << ": " << vel_cmd_[i] << std::endl;
  }
  std::cout << std::endl;

  std::cout << "kp: " << kp_ << std::endl;
  std::cout << "kd: " << kd_ << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
}

void ONNXController::publish() {
  if (!robot_interface_->is_ready()) {
    /*RCLCPP_WARN(this->get_logger(),
                "ONNXController::publish() Robot is not ready, cannot "
                "send command!"); */
    return;
  }
  if (!robot_interface_->is_safe()) {
    RCLCPP_WARN(this->get_logger(),
                "ONNXController::publish() Robot is not safe, cannot "
                "send command!");
    return;
  }

  if (joy_ && !joy_->axes.empty()) {
    // Reset to the original configuration if O is pressed
    if (joy_->buttons[1] != 0) {
      robot_interface_->go_to_configuration(q_des, 5.0);
    }

    // Ingest commanded velocity
    vel_cmd_[0] = joy_->axes[1];
    vel_cmd_[1] = pow(joy_->axes[0], 2) * ((joy_->axes[0] > 0) ? 1 : -1) * 0.8;
    vel_cmd_[2] = joy_->axes[3] * joy_->axes[1];
  }

  // Project the gravity into base frame
  std::array<float, 4> quat = robot_interface_->get_quaternion();
  quaternion_ = Eigen::Quaternion(quat[0], quat[1], quat[2], quat[3]);
  Eigen::Map<const Eigen::Vector3f> vec(gravity_w_.data());
  Eigen::Map<Eigen::Vector3f> gb_map(gravity_b_.data());
  gb_map = quaternion_.inverse() * vec;

  // Get the current state
  q_ = robot_interface_->get_q();
  dq_ = robot_interface_->get_dq();
  imu_lin_acc_ = robot_interface_->get_lin_acc();
  base_ang_vel_ = robot_interface_->get_ang_vel();

  // Read foot contact state
  for (uint8_t i = 0; i < 4; i++) {
    foot_forces_[i] = robot_interface_->get_forces()[i] >= 22;
  }

  // Subtract the q0_ initial pose from the joint positions
  for (uint8_t i = 0; i < 12; i++) {
    q_[i] -= q0_[i];
  }

  // Prepare the buffers
  populate_buffer(gravity_b_hist_, gravity_b_);
  populate_buffer(base_ang_vel_hist_, base_ang_vel_);
  // populate_buffer(imu_lin_acc_hist_, imu_lin_acc_);
  populate_buffer(vel_cmd_hist_, vel_cmd_);
  populate_buffer(q_hist_, q_);
  populate_buffer(dq_hist_, dq_);
  populate_buffer(action_hist_, action_);
  populate_buffer(foot_forces_hist_, foot_forces_);

  // Push all buffers into history
  populate_buffer(observation_, gravity_b_hist_, base_ang_vel_hist_,
                  /*imu_lin_acc_hist_,*/ vel_cmd_hist_, q_hist_, dq_hist_,
                  action_hist_, foot_forces_hist_);

  // Run the ONNX model (writes to action_)
  actor_->act();

  // Clamp the action between -+ kActionLimit
  for (float &a : action_) {
    a = std::clamp(a, -kActionLimit, kActionLimit);
    a *= joy_->buttons[0]; // lower button (i.e. X-button on PS) zeroes the
                           // action
  }

  // Populate the message
  obs_act_->observation = observation_;
  obs_act_->action = action_;

  robot_interface_->publish_obs_act(obs_act_);

  // Print observation and action
  print_vecs();

  // Prepare the arrays for the robot interface
  std::array<float, 12> q_des{};
  std::array<float, 12> zeroes{};
  std::array<float, 12> kp_array{};
  std::array<float, 12> kd_array{};

  for (size_t i = 0; i < 12; i++) {
    // The policy expects the prev. action in the scale it outputs them,
    // so we do the scaling only before sanding the commands to the actuators.
    q_des[i] = q0_[i] + action_[i] * 0.25;
    zeroes[i] = 0.0;
    kp_array[i] = kp_;
    kd_array[i] = kd_;
  }

  // Send the command
  robot_interface_->send_command(q_des, zeroes, zeroes, kp_array, kd_array);
}

rcl_interfaces::msg::SetParametersResult ONNXController::set_param_callback(
    const std::vector<rclcpp::Parameter> &params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &param : params) {
    if (param.get_name() == "kp") {
      kp_ = param.as_double();
    } else if (param.get_name() == "kd") {
      kd_ = param.as_double();
    } else {
      result.successful = false;
    }
  }

  return result;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ONNXController>());
  rclcpp::shutdown();
  return 0;
}
