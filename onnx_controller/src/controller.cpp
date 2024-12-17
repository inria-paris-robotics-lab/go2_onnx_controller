#include "controller.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "onnx_actor.hpp"

using namespace std::chrono_literals;

ONNXController::ONNXController() : Node("onnx_controller"), count_(0), last_msg_(nullptr), last_cmd_(nullptr) {
  subscription_ = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10,
      std::bind(&ONNXController::consume, this, std::placeholders::_1));

  publisher_ = this->create_publisher<std_msgs::msg::String>("/lowcmd", 10);
  timer_ = this->create_wall_timer(1000ms, std::bind(&ONNXController::publish, this));
}

void ONNXController::consume(const unitree_go::msg::LowState::SharedPtr msg) {
  // Do something with the message
  last_msg_ = msg;

  // Ingest proprioceptive data
  for (size_t i = 0; i < 12; i++) {
    size_t urdf_i = ros_to_urdf_idx_[i];
    q_[urdf_i] = msg->motor_state[i].q;
    dq_[urdf_i] = msg->motor_state[i].dq;
  }

  // Ingest IMU data
  imu_lin_acc_ = {msg->imu_state.accelerometer[0], msg->imu_state.accelerometer[1], msg->imu_state.accelerometer[2]};
  imu_ang_vel_ = {msg->imu_state.gyroscope[0], msg->imu_state.gyroscope[1], msg->imu_state.gyroscope[2]};

  // Run the ONNX mode

}

void ONNXController::publish() {
  RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", (int)count_);
  count_++;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ONNXController>());
  rclcpp::shutdown();
  return 0;
}