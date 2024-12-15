#include <chrono>
#include <iostream>
#include <vector>

#include "onnxruntime_cxx_api.h"

// constexpr unsigned int inputSize = 45;

int main() {
  std::cout << "Hello, World!" << std::endl;
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
  std::string model_path = "data/model.onnx";
  Ort::Session session(env, model_path.c_str(), Ort::SessionOptions{nullptr});

  // Print model input information
  std::cout << "Number of model inputs: " << session.GetInputCount()
            << std::endl;
  std::cout << "Number of model outputs: " << session.GetOutputCount()
            << std::endl;

  // Get input name
  Ort::AllocatorWithDefaultOptions allocator;

  Ort::AllocatedStringPtr input_name =
      session.GetInputNameAllocated(0, allocator);
  std::cout << "Input name: " << input_name.get() << std::endl;

  // Get output name
  Ort::AllocatedStringPtr output_name =
      session.GetOutputNameAllocated(0, allocator);
  std::cout << "Output name: " << output_name.get() << std::endl;

  // Memory configuration
  Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  // Get input and output shapes from model
  std::vector<int64_t> input_shape = session.GetInputTypeInfo(0)
                                         .GetTensorTypeAndShapeInfo()
                                         .GetShape();  // Likely {1, 45}
  std::vector<int64_t> output_shape = session.GetOutputTypeInfo(0)
                                          .GetTensorTypeAndShapeInfo()
                                          .GetShape();  // Likely {1, 12}

  // Create input and output vectors
  std::vector<float> observation = std::vector<float>(input_shape.at(1), 2.0);
  std::vector<float> action = std::vector<float>(output_shape.at(1), 1.0);

  // Allocate memory for input and output
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      memory_info, observation.data(), observation.size(), input_shape.data(),
      input_shape.size());

  Ort::Value output_tensor =
      Ort::Value::CreateTensor<float>(memory_info, action.data(), action.size(),
                                      output_shape.data(), output_shape.size());

  const std::array<const char*, 1> input_names = {input_name.get()};
  const std::array<const char*, 1> output_names = {output_name.get()};

  input_name.release();
  output_name.release();

  Ort::RunOptions runOptions;

  // run inference
  try {
    // Time it
    auto start = std::chrono::high_resolution_clock::now();
    session.Run(runOptions, input_names.data(), &input_tensor, 1,
                output_names.data(), &output_tensor, 1);
    auto end = std::chrono::high_resolution_clock::now();

    // Print time
    std::chrono::duration<double, std::milli> elapsed_ms = end - start;
    std::cout << "Inference time: " << elapsed_ms.count() << " ms"
              << std::endl;

  } catch (Ort::Exception& e) {
    std::cout << e.what() << std::endl;
    return 1;
  }

  // Print output
  for (int i = 0; i < output_shape.at(1); i++) {
    std::cout << action.at(i) << " ";
  }

  std::cout << std::endl;

  return 0;
}
