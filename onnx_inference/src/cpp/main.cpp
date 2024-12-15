#include <chrono>
#include <iostream>
#include <vector>

#include "onnx_actor.hpp"

// constexpr unsigned int inputSize = 45;

int main() {
  std::cout << "Hello, World!" << std::endl;
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
  std::string model_path = "data/model.onnx";

  ONNXActor actor(model_path);
  actor.print_model_info();

  std::vector<float> observation(45, 2.0);
  std::vector<float> action(12, 0.0);

  actor.update_observation(observation);
  actor.act(action);

  // Print the action
  std::cout << "Action: [";
  for (size_t i = 0; i < action.size(); ++i) {
    std::cout << action[i];
    if (i < action.size() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << "]" << std::endl;

  return 0;
}