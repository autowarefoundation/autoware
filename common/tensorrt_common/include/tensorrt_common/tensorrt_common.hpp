// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TENSORRT_COMMON__TENSORRT_COMMON_HPP_
#define TENSORRT_COMMON__TENSORRT_COMMON_HPP_

#include <rclcpp/rclcpp.hpp>

#include <NvInfer.h>
#include <NvOnnxParser.h>

#if (defined(_MSC_VER) or (defined(__GNUC__) and (7 <= __GNUC_MAJOR__)))
#include <filesystem>
namespace fs = ::std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = ::std::experimental::filesystem;
#endif

#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace tensorrt_common
{
class Logger : public nvinfer1::ILogger  // NOLINT
{
public:
  Logger() : Logger(Severity::kINFO) {}

  explicit Logger(Severity severity) : reportable_severity_(severity) {}

  void log(Severity severity, const char * msg) noexcept override
  {
    // suppress messages with severity enum value greater than the reportable
    if (severity > reportable_severity_) {
      return;
    }

    switch (severity) {
      case Severity::kINTERNAL_ERROR:
        RCLCPP_ERROR_STREAM(logger_, msg);
        break;
      case Severity::kERROR:
        RCLCPP_ERROR_STREAM(logger_, msg);
        break;
      case Severity::kWARNING:
        RCLCPP_WARN_STREAM(logger_, msg);
        break;
      case Severity::kINFO:
        RCLCPP_INFO_STREAM(logger_, msg);
        break;
      default:
        RCLCPP_INFO_STREAM(logger_, msg);
        break;
    }
  }

  Severity reportable_severity_{Severity::kWARNING};
  rclcpp::Logger logger_{rclcpp::get_logger("tensorrt_common")};
};

template <typename T>
struct InferDeleter  // NOLINT
{
  void operator()(T * obj) const
  {
    if (obj) {
#if TENSORRT_VERSION_MAJOR >= 8
      delete obj;
#else
      obj->destroy();
#endif
    }
  }
};

template <typename T>
using TrtUniquePtr = std::unique_ptr<T, InferDeleter<T>>;

using BatchConfig = std::array<int32_t, 3>;

class TrtCommon  // NOLINT
{
public:
  TrtCommon(
    const std::string & model_path, const std::string & precision,
    std::unique_ptr<nvinfer1::IInt8EntropyCalibrator2> calibrator = nullptr,
    const BatchConfig & batch_config = {1, 1, 1}, const size_t max_workspace_size = (16 << 20),
    const std::vector<std::string> & plugin_paths = {});

  bool loadEngine(const std::string & engine_file_path);
  bool buildEngineFromOnnx(
    const std::string & onnx_file_path, const std::string & output_engine_file_path);
  void setup();

  bool isInitialized();

  nvinfer1::Dims getBindingDimensions(const int32_t index) const;
  int32_t getNbBindings();
  bool setBindingDimensions(const int32_t index, const nvinfer1::Dims & dimensions) const;
  bool enqueueV2(void ** bindings, cudaStream_t stream, cudaEvent_t * input_consumed);

private:
  Logger logger_;
  fs::path model_file_path_;
  TrtUniquePtr<nvinfer1::IRuntime> runtime_;
  TrtUniquePtr<nvinfer1::ICudaEngine> engine_;
  TrtUniquePtr<nvinfer1::IExecutionContext> context_;
  std::unique_ptr<nvinfer1::IInt8EntropyCalibrator2> calibrator_;

  nvinfer1::Dims input_dims_;
  nvinfer1::Dims output_dims_;
  std::string precision_;
  BatchConfig batch_config_;
  size_t max_workspace_size_;
  bool is_initialized_{false};
};

}  // namespace tensorrt_common

#endif  // TENSORRT_COMMON__TENSORRT_COMMON_HPP_
