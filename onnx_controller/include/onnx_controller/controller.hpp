#include <chrono>
#include <memory>
#include <string>

#include "onnx_actor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

/**
 * Maps the indices of elements in the source array to their corresponding
 * indices in the target array.
 *
 * @tparam T The type of elements in the arrays.
 * @tparam N The size of the arrays.
 * @param source The source array.
 * @param target The target array.
 * @return An array of indices representing the mapping from source to target.
 */
template <typename T, size_t N>
constexpr std::array<std::uint8_t, N> map_indices(std::array<T, N> source,
                                                  std::array<T, N> target) {
  std::array<std::uint8_t, N> permutation{0};
  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      if (source[i] == target[j]) {
        permutation[i] = j;
        break;
      }
    }
  }
  return permutation;
}

class ONNXController : public rclcpp::Node {
 public:
  ONNXController();

  /**
   * @brief Consumes the state message.
   *
   * This function consumes the state message and stores it in the `state_`
   * member variable.
   *
   * @param msg The LowState message to consume.
   */
  void consume(const unitree_go::msg::LowState::SharedPtr msg);

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
   * @brief Initializes the command structure with default values.
   *
   * This function sets up the command structure `cmd_` with initial values:
   *
   * - `head = {0xEF, 0xEF}`,
   * - `level_flag = 0`,
   * - `frame_reserve = 0`,
   * - `sn = {0, 0}`,
   * - `bandwidth = 0`,
   * - `fan = {0, 0}`,
   * - `reserve = 0`,
   * - `led` is a 12-element array of zeros.
   */
  void initialize_command();

  /**
   * @brief Prepares the command structure for publishing.
   *
   * This function prepares the command structure `cmd_` for publishing by
   * setting the joint positions and velocities to the values in `action_`.
   */
  void prepare_command();

  /**
   * @brief Prints the observation and action vectors.
   *
   * This function prints the observation and action vectors to the standard
   * output.
   */
  void print_vecs();

  /**
   * @brief Moves the robot to the initial pose.
   *
   * This function moves the robot to the initial pose by interpolating the
   * joint positions from the current state to the initial pose.
   */
  void initial_pose();

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
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr
      state_subscription_;  ///< Subscription to the state topic
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr
      publisher_;  ///< Publisher for the command topic
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;  ///< Handle for the parameter callback

  // Torque control parameters
  float kp_ = 25.0;  ///< Proportional gain
  float kd_ = 0.5;  ///< Derivative gain

  // Messages
  sensor_msgs::msg::Joy::SharedPtr joy_;  ///< Pointer to the Joy message
  unitree_go::msg::LowState::SharedPtr
      state_;                               ///< Pointer to the LowState message
  unitree_go::msg::LowCmd::SharedPtr cmd_;  ///< Pointer to the LowCmd message

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

  //! Interpolation parameters to reach the initial pose
  static constexpr uint16_t num_interpolation_it_ =
      200;             ///< Number of iterations to reach the initial pose
  uint16_t it_count_;  ///< Current iteration count

  //! Initial pose (in radians, Isaac order)
  static constexpr std::array<const float, 12> q0_ = {
      0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5};

  //! Joint names (Isaac does breadth-first traversal)
  static constexpr std::array<std::string_view, 12> isaac_joint_names_ = {
      "FL_hip_joint",   "FR_hip_joint",   "RL_hip_joint",   "RR_hip_joint",
      "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
      "FL_calf_joint",  "FR_calf_joint",  "RL_calf_joint",  "RR_calf_joint"};

  //! Bullet joint names (Bullet does depth-first traversal)
  static constexpr std::array<std::string_view, 12> bullet_joint_names_ = {
      "FR_hip_joint",   "FR_thigh_joint", "FR_calf_joint",  "FL_hip_joint",
      "FL_thigh_joint", "FL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint",
      "RR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint", "RL_calf_joint"};

  //! @brief A mapping of indices from Isaac joint names to Bullet joint names.
  static constexpr auto isaac_in_bullet_ =
      map_indices(isaac_joint_names_, bullet_joint_names_);
};
