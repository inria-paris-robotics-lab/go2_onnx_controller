#include <chrono>
#include <memory>
#include <string>

#include "onnx_actor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

constexpr std::array<std::uint8_t, 12> get_permutation(const std::array<std::string, 12> source, const std::array<std::string, 12> target)
{
  std::array<std::uint8_t, 12> permutation{0};
  for (size_t i = 0; i < 12; i++)
  {
    auto it = std::find(target.begin(), target.end(), source[i]);
    permutation[i] = std::distance(target.begin(), it);
  }
  return permutation;
}

class ONNXController : public rclcpp::Node
{
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


  // Inertial state
  std::array<float, 3> imu_lin_acc_; ///< Linear acceleration
  std::array<float, 3> imu_ang_vel_; ///< Angular velocity

  std::array<float, 3> vel_cmd_{0.0, 0.0, 0.0}; ///< Linear velocity command

  // Proprioceptive state
  std::array<float, 12> q_;  ///< Joint positions
  std::array<float, 12> dq_; ///< Joint velocities

  // Control state
  std::vector<float> action_;      ///< Action to be taken, of size 12
  std::vector<float> observation_; ///< Observation vector, of size 45

  // Initial pose
  const std::array<float, 12> q0_ = {0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5};

  // Joint names (Isaac does breadth-first traversal)
  const std::array<std::string, 12> isaac_joint_names_ = {"FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint", "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint"};

  const std::array<std::string, 12> bullet_joint_names_ = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint", "FL_thigh_joint", 
    "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

  const std::array<uint8_t, 12> isaac_in_bullet_ = get_permutation(isaac_joint_names_, bullet_joint_names_);
};
