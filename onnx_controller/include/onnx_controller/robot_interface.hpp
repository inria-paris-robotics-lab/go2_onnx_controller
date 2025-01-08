#include "rclcpp/rclcpp.hpp"

class Go2RobotInterface {
 public:
  Go2RobotInterface(rclcpp::Node &node_);

 private:
  safety_cb_();

  rclcpp::Node &node_;
  bool is_ready_ = false;
  bool is_safe_ = false;

  static constexpr std::array<std::string_view, 12> target_joint_names_ = {
      "FR_hip_joint",   "FR_thigh_joint", "FR_calf_joint",  "FL_hip_joint",
      "FL_thigh_joint", "FL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint",
      "RR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint", "RL_calf_joint"};
}