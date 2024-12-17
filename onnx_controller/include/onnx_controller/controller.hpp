#include <chrono>
#include <memory>
#include <string>

#include "onnx_actor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

constexpr std::array<uint8_t, 12> get_ros_to_urdf_idx() {
  std::array<uint8_t, 4> foot_order = {1, 0, 3, 2};
  std::array<uint8_t, 12> ros_to_urdf_idx{0};

  for (size_t foot_idx = 0; foot_idx <= 3; foot_idx++) {
    uint8_t fid = foot_order[foot_idx];

    for (size_t i = 0; i < 3; i++) {
      ros_to_urdf_idx[foot_idx * 3 + i] = fid * 3 + i;
    }
  }

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
  void prepare_observation(std::vector<float>& observation);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  unitree_go::msg::LowState::SharedPtr last_msg_;
  unitree_go::msg::LowCmd::SharedPtr last_cmd_;

  std::shared_ptr<ONNXActor> actor_;

  // Index conversion
  const std::array<uint8_t, 12> ros_to_urdf_idx_ = get_ros_to_urdf_idx();

  // Inertial state
  std::array<float, 3> imu_lin_acc_;  ///< Linear acceleration
  std::array<float, 3> imu_ang_vel_;  ///< Angular velocity

  std::array<float, 3> vel_cmd_{0, 0, 0};  ///< Linear velocity command

  // Proprioceptive state
  std::array<float, 12> q_;   ///< Joint positions
  std::array<float, 12> dq_;  ///< Joint velocities

  // Control state
  std::array<float, 12> action_;  ///< Action to

  // Observation state
};