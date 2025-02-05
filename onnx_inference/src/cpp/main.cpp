#include <chrono>
#include <cstdlib>
#include <iostream>

#include "onnx_actor.hpp"

// constexpr unsigned int inputSize = 45;
void print_vec(const std::span<float> &vec, const std::string &name){
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
  std::string home = std::getenv("HOME");

  if (home.empty()) {
    std::cerr << "Error: HOME environment variable not set." << std::endl;
    return 1;
  }

  std::string model_path = home + "/.local/share/.onnx-actor/model.onnx";

  std::array<float, 45> observation{0.0};
  std::array<float, 12> action{0.0};

  ONNXActor actor(model_path, observation, action);
  actor.print_model_info();
  actor.act();

  // Print the action
  print_vec(action, "Action");

  return 0;
}