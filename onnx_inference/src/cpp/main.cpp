#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "onnx_actor.hpp"

constexpr unsigned int kInputSize = 98;
void print_vec(const std::span<float> &vec, const std::string &name) {
  std::cout << name << ": [";
  for (size_t i = 0; i < vec.size(); ++i) {
    std::cout << vec[i];
    if (i < vec.size() - 1) {
      std::cout << ",  ";
    }
  }
  std::cout << "]" << std::endl;
}

int main() {
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");

  // Build full path to model
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("onnx_inference");
  std::string model_path = package_share_dir + "/data/model.onnx";

  std::array<float, kInputSize> observation{0.0};
  std::array<float, 12> action{0.0};

  ONNXActor actor(model_path, observation, action);
  actor.print_model_info();

  const auto start = std::chrono::steady_clock::now();
  actor.act();
  const auto end = std::chrono::steady_clock::now();

  std::cout << "Inference took "
            << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                     start)
            << std::endl;

  // Print the action
  print_vec(action, "Action");

  return 0;
}
