#include "controller.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "onnx_actor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"


using namespace std::chrono_literals;

ONNXController::ONNXController()
    : Node("onnx_controller"),
      count_(0) {
  subscription_ = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10,
      std::bind(&ONNXController::consume, this, std::placeholders::_1));

  publisher_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
  timer_ = this->create_wall_timer(20ms,
                                   std::bind(&ONNXController::publish, this));

  // Initialize the ONNX actor
  std::string home = std::getenv("HOME");

  if (home.empty()) {
    std::cerr << "Error: HOME environment variable not set." << std::endl;
  }

  std::string model_path = home + "/.local/share/.onnx-actor/model.onnx";

  actor_ = std::make_unique<ONNXActor>(model_path);

  cmd_ = std::make_shared<unitree_go::msg::LowCmd>();
  // Initialize the command structure
  initialize_command();


}

/**
 * @brief Initializes the command structure with default values.
 * 
 * This function sets up the command structure `cmd_` with initial values:
 * - `head` is set to {0xEF, 0xEF}.
 * - `level_flag` is set to 0.
 * - `frame_reserve` is set to 0.
 * - `sn` is set to {0, 0}.
 * - `bandwidth` is set to 0.
 * - `fan` is set to {0, 0}.
 * - `reserve` is set to 0.
 * - `led` is set to an array of 12 zeros.
 */
void ONNXController::initialize_command(){
  cmd_->head = {0xEF, 0xEF}; // Segfault here
  cmd_->level_flag = 0; // Segfault here too, and also likely below...
  cmd_->frame_reserve = 0;
  cmd_->sn = {0, 0};
  cmd_->bandwidth = 0;
  cmd_->fan = {0, 0};
  cmd_->reserve = 0;
  // 1-> zeros for the 12 leds
  cmd_->led = std::array<uint8_t, 12>{0};

  for(auto &m_cmd_: cmd_->motor_cmd){
    m_cmd_.mode = 0;
    m_cmd_.q = 0;
    m_cmd_.dq = 0;
    // c.f. 
    // https://github.com/isaac-sim/IsaacLab/blob/874b7b628d501640399a241854c83262c5794a4b/source/extensions/omni.isaac.lab_assets/omni/isaac/lab_assets/unitree.py#L167
    m_cmd_.kp = 15.0;
    m_cmd_.kd = 0.5;
    /////// c.f.
    m_cmd_.tau = 0;
  }
}

/**
 * @brief Prepares the command structure for publishing.
 * 
 * This function prepares the command structure `cmd_` for publishing by setting
 * the joint positions and velocities to the values in `action_`.
 */
void ONNXController::prepare_command(){
  for(size_t i = 0; i < 12; i++){
    size_t urdf_i = ros_to_urdf_idx_[i];
    cmd_->motor_cmd[i].q = action_[urdf_i];
    cmd_->motor_cmd[i].dq = 0;
  }
}

void ONNXController::consume(const unitree_go::msg::LowState::SharedPtr msg) {
  // Ingest proprioceptive data
  for (size_t i = 0; i < 12; i++) {
    size_t urdf_i = ros_to_urdf_idx_[i];
    q_[urdf_i] = msg->motor_state[i].q;
    dq_[urdf_i] = msg->motor_state[i].dq;
  }

  // Ingest IMU data
  imu_lin_acc_ = {msg->imu_state.accelerometer[0] * 0,
                  msg->imu_state.accelerometer[1] * 0,
                  msg->imu_state.accelerometer[2] * 0};
  imu_ang_vel_ = {msg->imu_state.gyroscope[0] * 0, msg->imu_state.gyroscope[1] * 0,
                  msg->imu_state.gyroscope[2] * 0};

  // Run the ONNX mode
  std::vector<float> action(12, 0.0);
  std::vector<float> observation(45, 0.0);
  prepare_observation(observation);

  actor_->observe(observation);
  actor_->act(action);

  std::copy(action.begin(), action.end(), action_.begin());

  // Print observation and action
  std::cout << "Observation: " << std::endl;
  for(int i = 0; i < observation.size(); i++){
    std::cout << i << ": " << observation[i] << std::endl;
  }
  std::cout << std::endl;

  std::cout << "Action: " << std::endl;
  for(int i = 0; i < action.size(); i++){
    std::cout << i << ": " << action[i] << std::endl;
  }
  std::cout << std::endl;
}

void ONNXController::prepare_observation(std::vector<float>& observation) {
  observation.clear(); // Empty the observation vector
  observation.insert(observation.end(), imu_lin_acc_.begin(),
                     imu_lin_acc_.end());
  observation.insert(observation.end(), imu_ang_vel_.begin(),
                     imu_ang_vel_.end());
  observation.insert(observation.end(), vel_cmd_.begin(), vel_cmd_.end());
  observation.insert(observation.end(), q_.begin(), q_.end());
  observation.insert(observation.end(), dq_.begin(), dq_.end());
  observation.insert(observation.end(), action_.begin(), action_.end());
  //observation.insert(observation.end(), 12, 0.0);
}

void ONNXController::publish() {
  // Prepare the command
  prepare_command();
  auto msg = *cmd_.get();
  publisher_->publish(msg);

}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ONNXController>());
  rclcpp::shutdown();
  return 0;
}

/*
q0 =
9: 0.135416
10: 1.22434
11: -2.7229
12: -0.13507
13: 1.22309
14: -2.72294
15: -0.00658123
16: 1.49209
17: -2.99261
18: -0.482452
19: 1.24558
20: -2.72279

*/