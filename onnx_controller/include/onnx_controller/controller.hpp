#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "onnx_actor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interface.hpp"
#include "sensor_msgs/msg/joy.hpp"

constexpr size_t kDimObs = 52;
constexpr size_t kHistory = 2;

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
   * @brief Populates the target buffer.
   *
   * This function template populates the target buffer `target` with the
   * given arrays.
   */
  template <typename Target, typename Head, typename... Tail>
  void populate_buffer(Target &target, const Head &head, const Tail &...tail) {
    // Rotate the observation array to the left, and copy the new observation
    std::shift_left(target.begin(), target.end(), head.size());
    std::copy(head.begin(), head.end(), target.end() - head.size());

    // Recurse if there are more observations
    if constexpr (sizeof...(tail) > 0) {
      populate_buffer(target, tail...);
    } else {
	  size_t offset = target.size() - head.size();
      for (size_t i = 0; i < head.size(); i++) {
	    if (target[offset + i] != head[i]) {
		  exit(1);
		}
	  }
	}
  }

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
  float kp_ = 28.0;  ///< Proportional gain
  float kd_ = 0.5;   ///< Derivative gain

  sensor_msgs::msg::Joy::SharedPtr joy_;  ///< Pointer to the Joy message

  std::unique_ptr<Go2RobotInterface>
      robot_interface_;               ///< Robot interface object
  std::unique_ptr<ONNXActor> actor_;  ///< ONNXActor object

  // Gravity vector
  static constexpr std::array<float, 3> gravity_w_{0., 0., -1.}; ///< Gravity direction in the world
  std::array<float, 3> gravity_b_{}; ///< Gravity vector in the body frame

  // Inertial state
  Eigen::Quaternion<float> quaternion_{}; ///< Orientation (w, x, y, z)
  std::array<float, 3> base_lin_vel_{};  ///< Linear velocity
  std::array<float, 3> base_ang_vel_{};  ///< Angular velocity

  // Velocity command
  std::array<float, 3> vel_cmd_{};  ///< Velocity command

  // Proprioceptive state
  std::array<float, kDimDOF> q_{};   ///< Joint positions
  std::array<float, kDimDOF> dq_{};  ///< Joint velocities

  // Control state
  std::array<float, kDimDOF> action_{};  ///< Action to be taken, of size 12
  std::array<float, kDimObs * kHistory>
      observation_{};  ///< Observation array, with a kHistory-step history

  // Foot contacts
  std::array<uint16_t, 4> foot_forces_{1, 1, 1, 1};

  // History buffers
  std::array<float, 3 * kHistory> gravity_b_hist_{}; ///< Gravity vector history
  std::array<float, 3 * kHistory> base_lin_vel_hist_{};  ///< Linear velocity history
  std::array<float, 3 * kHistory> base_ang_vel_hist_{};  ///< Angular velocity history
  std::array<float, 3 * kHistory> vel_cmd_hist_{};  ///< Velocity command history
  std::array<float, kDimDOF * kHistory> q_hist_{};   ///< Joint positions history
  std::array<float, kDimDOF * kHistory> dq_hist_{};  ///< Joint velocities history
  std::array<float, kDimDOF * kHistory> action_hist_{};  ///< Action history
  std::array<uint16_t, 4 * kHistory> foot_forces_hist_{}; ///< Foot force history

  //! Initial pose (in radians, Isaac order)
  static constexpr std::array<const float, 12> q0_ = {
      0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5};

  //! Joint names (Isaac does breadth-first traversal)
  static constexpr std::array<std::string_view, 12> isaac_joint_names_ = {
      "FL_hip_joint",   "FR_hip_joint",   "RL_hip_joint",   "RR_hip_joint",
      "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
      "FL_calf_joint",  "FR_calf_joint",  "RL_calf_joint",  "RR_calf_joint"};
 
  //! Feet names (Isaac does breadth-first traversal)
  static constexpr std::array<std::string_view, 4> isaac_feet_names_ = {
      "FL", "FR", "RL", "RR"};
};
