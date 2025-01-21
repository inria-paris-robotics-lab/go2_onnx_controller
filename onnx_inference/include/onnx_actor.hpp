#pragma once
#include <memory>
#include <string>
#include <vector>

#include "onnxruntime_cxx_api.h"

/**
 * @class ONNXActor
 * @brief A class to handle ONNX model inference for reinforcement learning
 * policies.
 */
class ONNXActor {
 public:
  /**
   * @brief Constructor for ONNXActor.
   * @param model_path Path to the ONNX model file.
   * @param observation_ptr Pointer to the observation buffer.
   * @param action_ptr Pointer to the action buffer.
   * @param log_level Logging level for ONNX Runtime (default:
   * ORT_LOGGING_LEVEL_WARNING).
   *
   * @note The observation and action buffers must be shared pointers to
   * ensure that the buffers are not deallocated while the ONNXActor is
   * running.
   */
  ONNXActor(const std::string& model_path,
            const std::shared_ptr<std::vector<float>> observation,
            const std::shared_ptr<std::vector<float>> action,
            OrtLoggingLevel log_level = ORT_LOGGING_LEVEL_WARNING);

  /**
   * @brief Destructor for ONNXActor.
   */
  ~ONNXActor() = default;

  /**
   * @brief Compute the action based on the given observation.
   */
  void act();

  /**
   * @brief Print information about the loaded ONNX model.
   */
  void print_model_info();

  /**
   * @brief Check that both observation and action have the correct dimensions.
   */
  bool check_dims();

 private:
  OrtLoggingLevel log_level_;  ///< Logging level for ONNX Runtime.
  Ort::Env env_;               ///< ONNX Runtime environment.
  Ort::Session session_;       ///< ONNX Runtime session for model inference.

  std::shared_ptr<std::vector<float>>
      observation_;  ///< Buffer for storing observations.
  std::shared_ptr<std::vector<float>> action_;  ///< Buffer for storing actions.

  Ort::MemoryInfo memory_info_;  ///< Memory information for ONNX Runtime.
  Ort::RunOptions run_options_;  ///< Run options for ONNX Runtime.

  const std::string model_path_;  ///< Path to the ONNX model file.*/

  std::vector<int64_t> input_shape_;   ///< Shape of the model input.
  std::vector<int64_t> output_shape_;  ///< Shape of the model output.

  std::string input_name_;   ///< Name of the model input.
  std::string output_name_;  ///< Name of the model output.

  std::unique_ptr<Ort::Value> input_tensor_;   ///< Tensor for model input.
  std::unique_ptr<Ort::Value> output_tensor_;  ///< Tensor for model output.
};