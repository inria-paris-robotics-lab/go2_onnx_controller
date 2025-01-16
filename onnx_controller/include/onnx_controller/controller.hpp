#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "onnx_actor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interface.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

class ONNXController : public rclcpp::Node {
 public:
  ONNXController();

  /**
   * @brief Consumes the Joy message.
   *
   * This function consumes the Joy message and stores it in the `joy_` member
   * variable.
   *
   * @param msg The Joy message to consume.
   */
  void consume(const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Publishes the command to the /lowcmd topic.
   */
  void publish();

 private:
  /**
   * @brief Populates the observation vector.
   *
   * This function populates the observation vector `observation_` with the
   * linear acceleration, angular velocity, commanded velocity, joint positions,
   * joint velocities, and previous action.
   */
  void prepare_observation();

  /**
   * @brief Prints the observation and action vectors.
   *
   * This function prints the observation and action vectors to the standard
   * output.
   */
  void print_vecs();

  /**
   * @brief Callback function for setting parameters.
   *
   * This function is called when the parameters are set.
   *
   * @param params The parameters to set.
   *
   * @return The result of setting the parameters.
   */
  rcl_interfaces::msg::SetParametersResult set_param_callback(
      const std::vector<rclcpp::Parameter> &params);

  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for publishing commands
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      joy_subscription_;  ///< Subscription to the Joy topic
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;  ///< Handle for the parameter callback

  // Torque control parameters
  float kp_ = 25.0;  ///< Proportional gain
  float kd_ = 0.5;   ///< Derivative gain

  sensor_msgs::msg::Joy::SharedPtr joy_;  ///< Pointer to the Joy message

  std::unique_ptr<Go2RobotInterface>
      robot_interface_;               ///< Robot interface object
  std::unique_ptr<ONNXActor> actor_;  ///< ONNXActor object

  // Inertial state
  std::array<float, 3> imu_lin_acc_{};  ///< Linear acceleration
  std::array<float, 3> imu_ang_vel_{};  ///< Angular velocity

  std::array<float, 3> vel_cmd_{};  ///< Linear velocity command

  // Proprioceptive state
  std::array<float, 12> q_{};   ///< Joint positions
  std::array<float, 12> dq_{};  ///< Joint velocities

  // Control state
  std::vector<float> action_;       ///< Action to be taken, of size 12
  std::vector<float> observation_;  ///< Observation vector, of size 45

  //! Initial pose (in radians, Isaac order)
  static constexpr std::array<const float, 12> q0_ = {
      0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5};

  //! Joint names (Isaac does breadth-first traversal)
  static constexpr std::array<std::string_view, 12> isaac_joint_names_ = {
      "FL_hip_joint",   "FR_hip_joint",   "RL_hip_joint",   "RR_hip_joint",
      "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
      "FL_calf_joint",  "FR_calf_joint",  "RL_calf_joint",  "RR_calf_joint"};
};
