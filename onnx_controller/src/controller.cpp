#include "controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
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
    : Node("onnx_controller"), joy_(std::make_shared<sensor_msgs::msg::Joy>()) {
  actor_ = std::make_unique<ONNXActor>(get_model_path(), observation_, action_),
  // Set up the robot interface
      robot_interface_ =
          std::make_unique<Go2RobotInterface>(*this, isaac_joint_names_);

  // Set parameters
  this->declare_parameter("kp", kp_);
  this->declare_parameter("kd", kd_);

  auto param_change_callback =
      [this](const std::vector<rclcpp::Parameter>& params) {
        return set_param_callback(params);
      };

  parameter_callback_handle_ =
      this->add_on_set_parameters_callback(param_change_callback);

  // Subscribe to the Joy topic
  auto joy_callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
    consume(msg);
  };

  joy_subscription_ =
      this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, joy_callback);

  // Print the ONNXActor
  actor_->print_model_info();

  // Print vectors
  // print_vecs();

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
  std::cout << "Observation: " << std::endl;
  for (size_t i = 0; i < observation_.size(); i++) {
    std::cout << i << ": " << observation_[i] << std::endl;
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
}

void ONNXController::publish() {
  if (!robot_interface_->is_ready()) {
    RCLCPP_WARN(this->get_logger(),
                "ONNXController::publish() Robot is not ready, cannot "
                "send command!");
    return;
  }
  if (!robot_interface_->is_safe()) {
    RCLCPP_WARN(this->get_logger(),
                "ONNXController::publish() Robot is not safe, cannot "
                "send command!");
    return;
  }

  if (joy_ && !joy_->axes.empty()) {
    // Ingest commanded velocity
    vel_cmd_[0] = joy_->axes[1];
    vel_cmd_[1] = pow(joy_->axes[0], 2) * ((joy_->axes[0] > 0) ? 1 : -1) * 0.8;
    vel_cmd_[2] = joy_->axes[3] * joy_->axes[1];
  }

  // Get the current state
  q_ = robot_interface_->get_q();
  dq_ = robot_interface_->get_dq();
  imu_lin_acc_ = robot_interface_->get_lin_acc();
  imu_ang_vel_ = robot_interface_->get_ang_vel();

  // Subtract the q0_ initial pose from the joint positions
  for (size_t i = 0; i < 12; i++) {
    q_[i] -= q0_[i];
  }

  // Run the ONNX model
  prepare_observation(imu_lin_acc_, imu_ang_vel_, vel_cmd_, q_, dq_, action_);
  actor_->act();

  // Print observation and action
  print_vecs();

  // Prepare the arrays for the robot interface
  std::array<float, 12> q_des{};
  std::array<float, 12> zeroes{};
  std::array<float, 12> kp_array{};
  std::array<float, 12> kd_array{};

  for (size_t i = 0; i < 12; i++) {
    q_des[i] = q0_[i] + (action_[i] * 0.25);
    zeroes[i] = 0.0;
    kp_array[i] = kp_;
    kd_array[i] = kd_;
  }

  // Send the command
  robot_interface_->send_command(q_des, zeroes, zeroes, kp_array, kd_array);
}

rcl_interfaces::msg::SetParametersResult ONNXController::set_param_callback(
    const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : params) {
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

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ONNXController>());
  rclcpp::shutdown();
  return 0;
}
