#include "robot_interface.hpp"

Go2RobotInterface::Go2RobotInterface(
    rclcpp::Node &node,
    const std::array<std::string_view, 12> source_joint_names)
    : node_(node),
      source_joint_names_(source_joint_names),
      idx_source_in_target_(
          map_indices(source_joint_names_, target_joint_names_)),
      idx_target_in_source_(
          map_indices(target_joint_names_, source_joint_names_)) {
  // Set up publishers
  watchdog_publisher_ =
      node.create_publisher<std_msgs::msg::Bool>("/watchdog/arm", 10);
  command_publisher_ =
      node.create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

  // Subscribe to the /lowstate and /watchdog/is_safe topics
  auto state_callback = [this](const unitree_go::msg::LowState::SharedPtr msg) {
    consume(msg);
  };
  state_subscription_ = node_.create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, state_callback);

  auto watchdog_callback = [this](const std_msgs::msg::Bool::SharedPtr msg) {
    consume(msg);
  };
  watchdog_subscription_ = node.create_subscription<std_msgs::msg::Bool>(
      "/watchdog/is_safe", 10, watchdog_callback);
};

void Go2RobotInterface::consume(
    const unitree_go::msg::LowState::SharedPtr msg) {
  // Copy the /lowstate message
  state_ = msg;
}

void Go2RobotInterface::consume(const std_msgs::msg::Bool::SharedPtr msg) {
  // Copy the /watchdog/is_safe message
  is_safe_ = msg->data;
}