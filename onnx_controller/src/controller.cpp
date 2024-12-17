#include "controller.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "onnx_actor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_go/msg/low_state.hpp"

using namespace std::chrono_literals;

ONNXController::ONNXController()
    : Node("onnx_controller"),
      count_(0),
      last_msg_(nullptr),
      last_cmd_(nullptr) {
  subscription_ = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10,
      std::bind(&ONNXController::consume, this, std::placeholders::_1));

  publisher_ = this->create_publisher<std_msgs::msg::String>("/lowcmd", 10);
  timer_ = this->create_wall_timer(1000ms,
                                   std::bind(&ONNXController::publish, this));

  // Initialize the ONNX actor
  std::string home = std::getenv("HOME");

  if (home.empty()) {
    std::cerr << "Error: HOME environment variable not set." << std::endl;
  }

  std::string model_path = home + "/.local/share/.onnx-actor/model.onnx";

  actor_ = std::make_shared<ONNXActor>(model_path);
}

void ONNXController::consume(const unitree_go::msg::LowState::SharedPtr msg) {
  // Do something with the message
  last_msg_ = msg;

  // Ingest proprioceptive data
  for (size_t i = 0; i < 12; i++) {
    size_t urdf_i = ros_to_urdf_idx_[i];
    q_[urdf_i] = msg->motor_state[i].q;
    dq_[urdf_i] = 1;  // msg->motor_state[i].dq;

    // Print joint positions and velocities
    std::cout << "Joint " << i << ": q = " << q_[urdf_i]
              << ", dq = " << dq_[urdf_i] << std::endl;
  }
  // Check that dq_ sums to 1
  float sum = 0;
  for (size_t i = 0; i < 12; i++) {
    sum += dq_[i];
    std::cout << "idx: " << i << ", urdf_idx: " << (int)ros_to_urdf_idx_[i]
              << std::endl;
  }
  std::cout << "Sum of dq_: " << sum << std::endl;

  // Ingest IMU data
  imu_lin_acc_ = {msg->imu_state.accelerometer[0],
                  msg->imu_state.accelerometer[1],
                  msg->imu_state.accelerometer[2]};
  imu_ang_vel_ = {msg->imu_state.gyroscope[0], msg->imu_state.gyroscope[1],
                  msg->imu_state.gyroscope[2]};

  // Run the ONNX mode
  std::vector<float> action(12, 0.0);
  std::vector<float> observation<45, 0.0>;
  prepare_observation(observation);

  // Print observation and action
  /*
  std::cout << "q_: ";
  for (const auto& q : q_) {
    std::cout << q << " ";
  }
  std::cout << std::endl;

  std::cout << "Observation: ";
  for (const auto& obs : observation) {
    std::cout << obs << " ";
  }
  std::cout << std::endl;
  */
  actor_->update_observation(observation);
  actor_->act(action);

  /*
  std::cout << "Action: ";
  for (const auto& act : action) {
    std::cout << act << " ";
  }
  std::cout << std::endl;
  */
}

void prepare_observation(std::vector<float>& observation) {
  observation.clear(); // Empty the observation vector
  observation.insert(observation.end(), imu_lin_acc_.begin(),
                     imu_lin_acc_.end());
  observation.insert(observation.end(), imu_ang_vel_.begin(),
                     imu_ang_vel_.end());
  observation.insert(observation.end(), vel_cmd_.begin(), vel_cmd_.end());
  observation.insert(observation.end(), q_.begin(), q_.end());
  observation.insert(observation.end(), dq_.begin(), dq_.end());
  observation.insert(observation.end(), action_.begin(), action_.end());
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