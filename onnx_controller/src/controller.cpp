#include "controller.hpp"

#include <array>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include "onnx_actor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_cmd.hpp"

using namespace std::chrono_literals;

std::string get_model_path() {
  std::string home = std::getenv("HOME");

  if (home.empty()) {
    std::cerr << "Error: HOME environment variable not set." << std::endl;
  }

  std::string model_path = home + "/.local/share/.onnx-actor/model.onnx";

  return model_path;
}

ONNXController::ONNXController()
    : Node("onnx_controller"),
      cmd_(),
      actor_(std::make_unique<ONNXActor>(get_model_path())),
      action_(12, 0.0),
      observation_(45, 0.0),
      it_count_(0) {
  // Subscribe to the lowstate topic
  state_subscription_ = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10,
      std::bind(&ONNXController::consume, this, std::placeholders::_1));

  // Publish to the lowcmd topic
  publisher_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

  // Set the timer to publish at 50 Hz
  timer_ =
      this->create_wall_timer(20ms, std::bind(&ONNXController::publish, this));

  // Initialize the command structure
  initialize_command();

  // Print the ONNXActor
  actor_->print_model_info();

  // Print observation and action
  print_vecs();
}

void ONNXController::initialize_command() {
  cmd_->head = {0xEF, 0xEF};
  cmd_->level_flag = 0;
  cmd_->frame_reserve = 0;
  cmd_->sn = {0, 0};
  cmd_->bandwidth = 0;
  cmd_->fan = {0, 0};
  cmd_->reserve = 0;
  // 1-> zeros for the 12 leds
  cmd_->led = std::array<uint8_t, 12>{0};

  /* Initialize the motor commands,
   * refer to
   * https://github.com/isaac-sim/IsaacLab/blob/874b7b628d501640399a241854c83262c5794a4b/source/extensions/omni.isaac.lab_assets/omni/isaac/lab_assets/unitree.py#L167
   * for the default values of kp and kd.
   */
  for (auto& m_cmd_ : cmd_->motor_cmd) {
    m_cmd_.mode = 0;
    m_cmd_.q = 0;
    m_cmd_.dq = 0;
    m_cmd_.kp = 30.0;
    m_cmd_.kd = 0.5;
    m_cmd_.tau = 0;
  }
}

void ONNXController::prepare_command() {
  for (size_t i = 0; i < 12; i++) {
    size_t bullet_idx = isaac_in_bullet_[i];
    cmd_->motor_cmd[bullet_idx].q = (action_[i] * 0.25) + q0_[i];
  }
}

void ONNXController::consume(const unitree_go::msg::LowState::SharedPtr msg) {
  // Copy the state message
  state_ = msg;
}

void ONNXController::prepare_observation() {
  observation_.clear();  // Clear the observation vector

  // Add the linear acceleration and angular velocity
  observation_.insert(observation_.end(), imu_lin_acc_.begin(),
                      imu_lin_acc_.end());
  observation_.insert(observation_.end(), imu_ang_vel_.begin(),
                      imu_ang_vel_.end());

  // Add the commanded velocity
  observation_.insert(observation_.end(), vel_cmd_.begin(), vel_cmd_.end());

  // Add the joint positions and velocities
  observation_.insert(observation_.end(), q_.begin(), q_.end());
  observation_.insert(observation_.end(), dq_.begin(), dq_.end());

  // Add the previous action
  observation_.insert(observation_.end(), action_.begin(), action_.end());
}

void ONNXController::print_vecs() {
  // Print observation and action
  std::cout << "Observation: " << std::endl;
  for (size_t i = 0; i < observation_.size(); i++) {
    std::cout << i << ": " << observation_[i] << std::endl;
  }
  std::cout << std::endl;

  std::cout << "Action: " << std::endl;
  for (size_t i = 0; i < action_.size(); i++) {
    std::cout << i << ": " << action_[i] << std::endl;
  }
  std::cout << std::endl;
}

void ONNXController::initial_pose() {
  for (size_t i = 0; i < 12; i++) {
    float alpha = (float)(num_interpolation_it_ - it_count_) /
                  num_interpolation_it_;  // Go from 1 to 0
    action_[i] = q_[i] * alpha;
  }
}

void ONNXController::publish() {
  // If the state is not available, do not publish
  if (!state_) {
    return;
  }

  // Ingest proprioceptive data
  for (size_t i = 0; i < 12; i++) {
    size_t bullet_idx = isaac_in_bullet_[i];
    q_[i] = state_->motor_state[bullet_idx].q -
            q0_[i];  // Subtract the initial pose
    dq_[i] = state_->motor_state[bullet_idx].dq;
  }

  // Ingest IMU data
  for (size_t i = 0; i < 3; i++) {
    imu_lin_acc_[i] = state_->imu_state.accelerometer[i];
    imu_ang_vel_[i] = state_->imu_state.gyroscope[i];
  }

  // Run the ONNX model
  if (it_count_ >= num_interpolation_it_) {
    prepare_observation();
    actor_->observe(observation_);
    actor_->act(action_);
  } else {
    initial_pose();
  }

  it_count_++;

  // Print observation and action
  print_vecs();

  // Prepare the command
  prepare_command();
  publisher_->publish(*cmd_.get());
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ONNXController>());
  rclcpp::shutdown();
  return 0;
}
