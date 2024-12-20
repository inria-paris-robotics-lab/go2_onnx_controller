#include <chrono>
#include <memory>
#include <string>

#include "onnx_actor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

constexpr std::array<uint8_t, 12> get_ros_to_urdf_idx() {
  std::array<uint8_t, 4> leg_order = {1, 0, 3, 2}; // {FL, FR, RL, RR} -> {FR, FL, RR, RL}
  std::array<uint8_t, 3> joint_order = {1, 2, 0}; // {calf, hip, thigh} -> {hip, thigh, calf}

  std::array<uint8_t, 12> ros_to_urdf_idx{0};
  
  size_t i = 0;
  for (auto leg_id: leg_order) {
    for (auto ji: joint_order) {
      ros_to_urdf_idx[i] = leg_id * 3 + ji;
      i++;
    }
  }
  // return {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  return ros_to_urdf_idx;
};

class ONNXController : public rclcpp::Node {
 public:
  ONNXController();

  void consume(const unitree_go::msg::LowState::SharedPtr msg);
  void publish();

  // const std::array<uint8_t, 12> urdf_to_ros(const std::array<uint8_t, 12>);
  // const std::array<uint8_t, 12> ros_to_urdf(const std::array<uint8_t, 12>);

 private:
  void prepare_observation();
  void initialize_command();
  void prepare_command();
  void print_vecs();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscription_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher_;
  size_t count_;

  unitree_go::msg::LowState::SharedPtr state_;
  unitree_go::msg::LowCmd::SharedPtr cmd_;

  std::unique_ptr<ONNXActor> actor_;

  // Index conversion
  const std::array<uint8_t, 12> ros_to_urdf_idx_ = get_ros_to_urdf_idx();

  // Inertial state
  std::array<float, 3> imu_lin_acc_;  ///< Linear acceleration
  std::array<float, 3> imu_ang_vel_;  ///< Angular velocity

  std::array<float, 3> vel_cmd_{0.0, 0.0, 0.0};  ///< Linear velocity command

  // Proprioceptive state
  std::array<float, 12> q_;   ///< Joint positions
  std::array<float, 12> dq_;  ///< Joint velocities

  // Control state
  std::vector<float> action_; ///< Action to be taken, of size 12
  std::vector<float> observation_; ///< Observation vector, of size 45

  // Initial pose
  std::array<float, 12> q0_ = {-1.5, 0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0};
};