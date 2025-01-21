#include <chrono>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "onnx_actor.hpp"

// constexpr unsigned int inputSize = 45;

int main() {
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
  std::string home = std::getenv("HOME");

  if (home.empty()) {
    std::cerr << "Error: HOME environment variable not set." << std::endl;
    return 1;
  }

  std::string model_path = home + "/.local/share/.onnx-actor/model.onnx";

  std::vector<float> observation(45, 0.0);
  std::vector<float> action(12, 0.0);

  ONNXActor actor(model_path, observation, action);
  actor.print_model_info();
  actor.act();

  // Print the action
  std::cout << "Action: [";
  for (size_t i = 0; i < action.size(); ++i) {
    std::cout << action[i];
    if (i < action.size() - 1) {
      std::cout << ",  ";
    }
  }
  std::cout << "]" << std::endl;

  return 0;
}