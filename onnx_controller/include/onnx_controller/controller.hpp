#pragma once

#include <cstdint>
#include <memory>

#include "go2_control_interface_cpp/robot_interface.hpp"
#include "onnx_actor.hpp"
#include "onnx_interfaces/msg/observation_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "unitree_go/msg/low_state.hpp"

constexpr size_t kDimDOF = 12;
constexpr size_t kDimObs = 53;
constexpr size_t kHistory = 1;
constexpr float kActionLimit = 1000; // Clip the actions to -+ this limit

class ONNXController : public rclcpp::Node
{
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
  template<typename Target, typename Head, typename... Tail>
  void populate_buffer(Target & target, const Head & head, const Tail &... tail)
  {
    // Rotate the observation array to the left, and copy the new observation
    std::shift_left(target.begin(), target.end(), head.size());
    std::copy(head.begin(), head.end(), target.end() - head.size());

    // Recurse if there are more observations
    if constexpr (sizeof...(tail) > 0)
    {
      populate_buffer(target, tail...);
    }
    else
    {
      size_t offset = target.size() - head.size();
      for (size_t i = 0; i < head.size(); i++)
      {
        if (target[offset + i] != head[i])
        {
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
  rcl_interfaces::msg::SetParametersResult set_param_callback(const std::vector<rclcpp::Parameter> & params);

  /**
   * @brief Read IMU data from LowState message
   */
  void lowstate_cb_(const unitree_go::msg::LowState::SharedPtr msg)
  {
    // Process the quaternion and foot forces
    quaternion_ = Eigen::Quaternion(
      msg->imu_state.quaternion[0], msg->imu_state.quaternion[1], msg->imu_state.quaternion[2],
      msg->imu_state.quaternion[3]);

    // Swap feet because of unitree <-> simple conventions
    foot_forces_[0] = msg->foot_force[2] >= 22; // RL
    foot_forces_[1] = msg->foot_force[0] >= 22; // FR
    foot_forces_[2] = msg->foot_force[1] >= 22; // FL
    foot_forces_[3] = msg->foot_force[3] >= 22; // RR

    // Process the IMU state
    for (size_t i = 0; i < 3; i++)
    {
      imu_lin_acc_[i] = msg->imu_state.accelerometer[i];
      base_ang_vel_[i] = msg->imu_state.gyroscope[i];
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;                                      ///< Timer for publishing commands
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_; ///< Subscription to the Joy topic
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_; ///< Handle for the parameter callback

  // Torque control parameters
  float kp_ = 28.0; ///< Proportional gain
  float kd_ = 2.5;  ///< Derivative gain

  sensor_msgs::msg::Joy::SharedPtr joy_;                       ///< Pointer to the Joy message
  onnx_interfaces::msg::ObservationAction::SharedPtr obs_act_; ///< Pointer to the ObservationAction message
  rclcpp::Publisher<onnx_interfaces::msg::ObservationAction>::SharedPtr obs_act_publisher_;

  std::unique_ptr<Go2RobotInterface> robot_interface_; ///< Robot interface object
  std::unique_ptr<ONNXActor> actor_;                   ///< ONNXActor object
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr
    state_subscription_; ///< Get embeded imu & sensors via LowState

  // Gravity vector
  static constexpr std::array<float, 3> gravity_w_{0., 0., 1.}; ///< Gravity direction in the world
  std::array<float, 3> gravity_b_{};                            ///< Gravity vector in the body frame

  // Inertial state
  Eigen::Quaternion<float> quaternion_{}; ///< Orientation (w, x, y, z)
  std::array<float, 3> base_ang_vel_{};   ///< Angular velocity
  std::array<float, 3> imu_lin_acc_{};    ///< Linear acceleration

  // Velocity command
  std::array<float, 3> vel_cmd_{}; ///< Velocity command

  // Proprioceptive state
  std::array<float, kDimDOF> q_{};  ///< Joint positions
  std::array<float, kDimDOF> dq_{}; ///< Joint velocities

  // Control state
  std::array<float, kDimDOF> action_{};                 ///< Action to be taken, of size 12
  std::array<float, kDimObs * kHistory> observation_{}; ///< Observation array, with a kHistory-step history

  // Foot contacts
  std::array<uint16_t, 4> foot_forces_{1, 1, 1, 1};

  // History buffers
  std::array<float, 3 * kHistory> gravity_b_hist_{};      ///< Gravity vector history
  std::array<float, 3 * kHistory> base_ang_vel_hist_{};   ///< Angular velocity history
  std::array<float, 3 * kHistory> imu_lin_acc_hist_{};    ///< Linear acceleration history
  std::array<float, 3 * kHistory> vel_cmd_hist_{};        ///< Velocity command history
  std::array<float, kDimDOF * kHistory> q_hist_{};        ///< Joint positions history
  std::array<float, kDimDOF * kHistory> dq_hist_{};       ///< Joint velocities history
  std::array<float, kDimDOF * kHistory> action_hist_{};   ///< Action history
  std::array<uint16_t, 4 * kHistory> foot_forces_hist_{}; ///< Foot force history
  std::array<float, 4 * kHistory> quaternion_hist{};

  //! Initial pose (in radians, Isaac order)
  Eigen::Vector<double, 12> q0_isaac{0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5};
  Eigen::Vector<double, 12> q0_simple_ = {0.068, 0.785, -1.44, -0.068, 0.785, -1.44,
                                          0.068, 0.785, -1.44, -0.068, 0.785, -1.44};

  //! Joint names (Isaac does breadth-first traversal)
  static constexpr std::array<std::string_view, 12> isaac_joint_names_ = {
    "FL_hip_joint",   "FR_hip_joint",   "RL_hip_joint",  "RR_hip_joint",  "FL_thigh_joint", "FR_thigh_joint",
    "RL_thigh_joint", "RR_thigh_joint", "FL_calf_joint", "FR_calf_joint", "RL_calf_joint",  "RR_calf_joint"};

  static constexpr std::array<std::string_view, 12> simple_joint_names_ = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

  //! Feet names (Isaac does breadth-first traversal)
  static constexpr std::array<std::string_view, 4> isaac_feet_names_ = {"FL", "FR", "RL", "RR"};
  static constexpr std::array<std::string_view, 4> simple_feet_names_ = {"RL", "FR", "FL", "RR"};
};
