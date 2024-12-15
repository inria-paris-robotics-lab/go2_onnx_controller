#include "onnx_actor.hpp"

#include <iostream>

ONNXActor::ONNXActor(const std::string& model_path, OrtLoggingLevel log_level)
    : log_level_(log_level),
      env_(log_level, "ONNXActor"),
      model_path_(model_path),
      memory_info_(
          Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
      run_options_(nullptr),
      session_(env_, model_path.c_str(), Ort::SessionOptions{nullptr}) {
  // Get input and output shapes from model
  Ort::AllocatorWithDefaultOptions allocator;

  input_name_ = session_.GetInputNameAllocated(0, allocator).get();
  output_name_ = session_.GetOutputNameAllocated(0, allocator).get();

  input_shape_ = session_.GetInputTypeInfo(0)
                     .GetTensorTypeAndShapeInfo()
                     .GetShape();  // Likely {1, 45}

  output_shape_ = session_.GetOutputTypeInfo(0)
                      .GetTensorTypeAndShapeInfo()
                      .GetShape();  // Likely {1, 12}

  observation_ = std::vector<float>(input_shape_.at(1), 0.0);
  action_ = std::vector<float>(output_shape_.at(1), 0.0);

  // Define input and output tensors which wrap the input and output buffers
  input_tensor_ = std::make_unique<Ort::Value>(Ort::Value::CreateTensor<float>(
      memory_info_, observation_.data(), observation_.size(),
      input_shape_.data(), input_shape_.size()));

  output_tensor_ = std::make_unique<Ort::Value>(Ort::Value::CreateTensor<float>(
      memory_info_, action_.data(), action_.size(), output_shape_.data(),
      output_shape_.size()));
};

void ONNXActor::update_observation(const std::vector<float>& obs) {
  // Check if observation size matches input shape
  if (obs.size() != input_shape_.at(1)) {
    throw std::runtime_error(
        "Observation size does not match the expected shape.");
  }

  // Copy observation to internal buffer
  std::copy(obs.begin(), obs.end(), observation_.begin());
}

void ONNXActor::act(std::vector<float>& action) {
  // Check if observation size matches input shape
  const char* cstr_in = input_name_.c_str();
  const char* const* input_names = &cstr_in;

  const char* cstr_out = output_name_.c_str();
  const char* const* output_names = &cstr_out;

  session_.Run(run_options_, input_names, input_tensor_.get(), 1, output_names,
               output_tensor_.get(), 1);

  // Copy action to output buffer
  action.resize(output_shape_.at(1));
  std::copy(action_.begin(), action_.end(), action.begin());
}

void ONNXActor::print_model_info() {
  std::cout << "Number of model inputs: " << session_.GetInputCount()
            << std::endl;
  std::cout << "Number of model outputs: " << session_.GetOutputCount()
            << std::endl;
  std::cout << "Input name: " << input_name_ << std::endl;
  std::cout << "Output name: " << output_name_ << std::endl;
}
