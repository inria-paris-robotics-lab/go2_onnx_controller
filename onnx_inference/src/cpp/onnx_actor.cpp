#include "onnx_actor.hpp"

#include <iostream>
#include <span>

ONNXActor::ONNXActor(const std::string& model_path,
                     const std::span<float> observation,
                     const std::span<float> action,
                     OrtLoggingLevel log_level)
    : log_level_(log_level),
      env_(log_level, "ONNXActor"),
      model_path_(model_path),
      memory_info_(
          Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
      run_options_(nullptr),
      session_(env_, model_path.c_str(), Ort::SessionOptions{nullptr}),
      observation_(observation),
      action_(action) {
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

  // Define input and output tensors which wrap the input and output buffers
  input_tensor_ = std::make_unique<Ort::Value>(Ort::Value::CreateTensor<float>(
      memory_info_, observation_.data(), input_shape_.at(1),
      input_shape_.data(), input_shape_.size()));

  output_tensor_ = std::make_unique<Ort::Value>(Ort::Value::CreateTensor<float>(
      memory_info_, action_.data(), output_shape_.at(1), output_shape_.data(),
      output_shape_.size()));
};

void ONNXActor::act() {
  // Check if observation size matches input shape
  const char* cstr_in = input_name_.c_str();
  const char* const* input_names = &cstr_in;

  const char* cstr_out = output_name_.c_str();
  const char* const* output_names = &cstr_out;

  session_.Run(run_options_, input_names, input_tensor_.get(), 1, output_names,
               output_tensor_.get(), 1);
}

bool ONNXActor::check_dims() {
  bool result = true;

  result &= observation_.size() == input_shape_.at(1);
  result &= action_.size() == output_shape_.at(1);

  return result;
}

void ONNXActor::print_model_info() {
  std::cout << "Input dimension: " << input_shape_.at(1) << std::endl;
  std::cout << "Output dimension: " << output_shape_.at(1) << std::endl;
  std::cout << "Input name: " << input_name_ << std::endl;
  std::cout << "Output name: " << output_name_ << std::endl;
}
