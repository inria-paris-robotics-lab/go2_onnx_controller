#include "controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <string>

#include "go2_control_interface_cpp/robot_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

std::string get_model_path()
{
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("onnx_inference");
  std::string model_path = package_share_dir + "/data/standing.onnx";

  return model_path;
}

ONNXController::ONNXController()
: Node("onnx_controller")
, joy_(std::make_shared<sensor_msgs::msg::Joy>())
, obs_act_(std::make_shared<onnx_interfaces::msg::ObservationAction>())
{
  actor_ = std::make_unique<ONNXActor>(get_model_path(), observation_, action_),
  // Set up the robot interface
    robot_interface_ = std::make_unique<Go2RobotInterface>(*this, simple_joint_names_);
  state_subscription_ = this->create_subscription<unitree_go::msg::LowState>(
    "/lowstate", 10, std::bind(&ONNXController::lowstate_cb_, this, std::placeholders::_1));

  obs_act_publisher_ = this->create_publisher<onnx_interfaces::msg::ObservationAction>("/observation_action", 10);

  // Set parameters
  this->declare_parameter("kp", kp_);
  this->declare_parameter("kd", kd_);

  auto param_change_callback = [this](const std::vector<rclcpp::Parameter> & params) {
    return set_param_callback(params);
  };

  parameter_callback_handle_ = this->add_on_set_parameters_callback(param_change_callback);

  // Subscribe to the Joy topic
  auto joy_callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) { consume(msg); };

  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joy_callback);

  // Print the ONNXActor
  actor_->print_model_info();

  // Print vectors
  print_vecs();

  RCLCPP_INFO(
    this->get_logger(), "ONNXController initialised, going to initial "
                        "pose and waiting for Joy message.");

  robot_interface_->start_async(q0_simple_, 8.0);

  // Set the timer to publish at 50 Hz
  timer_ = this->create_wall_timer(20ms, std::bind(&ONNXController::publish, this));
}

void ONNXController::consume(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Copy the Joy message
  joy_ = msg;
}

void ONNXController::print_vecs()
{
  // Print observation and action
  std::cout << "---- Observation Components ----" << std::endl;

  std::cout << "xyzw quaternion:         [";
  std::cout << std::fixed << std::setprecision(4) << quaternion_.x() << ", " << quaternion_.y() << ", "
            << quaternion_.z() << ", " << quaternion_.w();
  std::cout << "]" << std::endl;

  std::cout << "q (Simple order):        [";
  for (size_t i = 0; i < q_.size(); i++)
    std::cout << std::fixed << std::setprecision(4) << q_[i] << (i < q_.size() - 1 ? ", " : "");
  std::cout << "]" << std::endl;

  std::cout << "dq (Simple order):       [";
  for (size_t i = 0; i < dq_.size(); i++)
    std::cout << std::fixed << std::setprecision(4) << dq_[i] << (i < dq_.size() - 1 ? ", " : "");
  std::cout << "]" << std::endl;

  std::cout << "base_ang_vel:            [";
  for (size_t i = 0; i < base_ang_vel_.size(); i++)
    std::cout << std::fixed << std::setprecision(4) << base_ang_vel_[i] << (i < base_ang_vel_.size() - 1 ? ", " : "");
  std::cout << "]" << std::endl;

  std::cout << "gravity_b:               [";
  for (size_t i = 0; i < gravity_b_.size(); i++)
    std::cout << std::fixed << std::setprecision(4) << gravity_b_[i] << (i < gravity_b_.size() - 1 ? ", " : "");
  std::cout << "]" << std::endl;

  std::cout << "vel_cmd:                 [";
  for (size_t i = 0; i < vel_cmd_.size(); i++)
    std::cout << std::fixed << std::setprecision(4) << vel_cmd_[i] << (i < vel_cmd_.size() - 1 ? ", " : "");
  std::cout << "]" << std::endl;

  std::cout << "action (last, Simple):    [" << std::endl;
  for (size_t i = 0; i < action_.size(); i++)
    std::cout << std::fixed << std::setprecision(4) << action_[i] << (i < action_.size() - 1 ? ", " : "");
  std::cout << "]" << std::endl;

  std::cout << "foot_forces:             [";
  for (size_t i = 0; i < foot_forces_.size(); i++)
    std::cout << std::fixed << std::setprecision(4) << foot_forces_[i] << (i < foot_forces_.size() - 1 ? ", " : "");
  std::cout << "]" << std::endl;

  std::cout << "-------------------------------" << std::endl;

  std::cout << "kp: " << kp_ << std::endl;
  std::cout << "kd: " << kd_ << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
}

void ONNXController::publish()
{
  if (!robot_interface_->is_ready())
  {
    /*RCLCPP_WARN(this->get_logger(),
                "ONNXController::publish() Robot is not ready, cannot "
                "send command!"); */
    return;
  }
  if (!robot_interface_->is_safe())
  {
    RCLCPP_WARN(
      this->get_logger(), "ONNXController::publish() Robot is not safe, cannot "
                          "send command!");
    return;
  }

  if (joy_ && !joy_->axes.empty())
  {
    // Ingest commanded velocity
    vel_cmd_[0] = joy_->axes[1];
    vel_cmd_[1] = pow(joy_->axes[0], 2) * ((joy_->axes[0] > 0) ? 1 : -1) * 0.8;
    vel_cmd_[2] = joy_->axes[3] * joy_->axes[1];
  }

  // Project the gravity into base frame
  Eigen::Map<const Eigen::Vector3f> vec(gravity_w_.data());
  Eigen::Map<Eigen::Vector3f> gb_map(gravity_b_.data());
  gb_map = quaternion_.inverse() * vec;

  // Get the current state
  for (int i = 0; i < 12; i++)
  {
    q_[i] = robot_interface_->get_q()[i];
    dq_[i] = robot_interface_->get_dq()[i];
  }

  std::array<double, 4> xyzw_quat_{quaternion_.x(), quaternion_.y(), quaternion_.z(), quaternion_.w()};

  // Prepare the buffers
  populate_buffer(quaternion_hist, xyzw_quat_);
  populate_buffer(gravity_b_hist_, gb_map);
  populate_buffer(base_ang_vel_hist_, base_ang_vel_);
  // populate_buffer(imu_lin_acc_hist_, imu_lin_acc_);
  populate_buffer(vel_cmd_hist_, vel_cmd_);
  populate_buffer(q_hist_, q_);
  populate_buffer(dq_hist_, dq_);
  populate_buffer(action_hist_, action_);
  populate_buffer(foot_forces_hist_, foot_forces_);

  // Push all buffers into history
  populate_buffer(
    observation_, xyzw_quat_, q_,
    /*imu_lin_acc_hist_,*/ base_ang_vel_, dq_, action_, gravity_b_, vel_cmd_, foot_forces_);

  // Run the ONNX model (writes to action_)
  actor_->act();

  // Clamp the action between -+ kActionLimit
  for (float & a : action_)
  {
    a = std::clamp(a, -kActionLimit, kActionLimit);
    a *= joy_->buttons[0] == 0; // lower button (i.e. X-button on PS) zeroes the
                                // action
  }

  // Populate the message
  obs_act_->observation = observation_;
  obs_act_->action = action_;

  obs_act_publisher_->publish(*obs_act_.get());

  // Print observation and action
  print_vecs();

  // Prepare the arrays for the robot interface
  Eigen::Vector<double, 12> q_des{};
  Eigen::Vector<double, 12> zeroes = Eigen::Vector<double, 12>::Zero();
  Eigen::Vector<double, 12> kp_array{};
  Eigen::Vector<double, 12> kd_array{};

  for (size_t i = 0; i < 12; i++)
  {
    // The policy expects the prev. action in the scale it outputs them,
    // so we do the scaling only before sanding the commands to the actuators.
    q_des[i] = q0_simple_[i] - action_[i] * 0.8;
    zeroes[i] = 0.0;
    kp_array[i] = joy_->buttons[0] == 0 ? kp_ : 5;
    kd_array[i] = kd_;
  }

  // Send the command
  robot_interface_->send_command(q_des, zeroes, zeroes, kp_array, kd_array);
}

rcl_interfaces::msg::SetParametersResult
ONNXController::set_param_callback(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : params)
  {
    if (param.get_name() == "kp")
    {
      kp_ = param.as_double();
    }
    else if (param.get_name() == "kd")
    {
      kd_ = param.as_double();
    }
    else
    {
      result.successful = false;
    }
  }

  return result;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ONNXController>());
  rclcpp::shutdown();
  return 0;
}
