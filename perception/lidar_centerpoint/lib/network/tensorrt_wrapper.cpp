// Copyright 2021 TIER IV, Inc.
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

#include "lidar_centerpoint/network/tensorrt_wrapper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <NvOnnxParser.h>

#include <fstream>
#include <memory>
#include <string>

namespace centerpoint
{
TensorRTWrapper::TensorRTWrapper(const CenterPointConfig & config) : config_(config)
{
}

TensorRTWrapper::~TensorRTWrapper()
{
  context_.reset();
  runtime_.reset();
  plan_.reset();
  engine_.reset();
}

bool TensorRTWrapper::init(
  const std::string & onnx_path, const std::string & engine_path, const std::string & precision)
{
  runtime_ =
    tensorrt_common::TrtUniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    RCLCPP_ERROR(rclcpp::get_logger("lidar_centerpoint"), "Failed to create runtime");
    return false;
  }

  bool success;
  std::ifstream engine_file(engine_path);
  if (engine_file.is_open()) {
    success = loadEngine(engine_path);
  } else {
    success = parseONNX(onnx_path, engine_path, precision);
  }
  success &= createContext();

  return success;
}

bool TensorRTWrapper::createContext()
{
  if (!engine_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("lidar_centerpoint"), "Failed to create context: Engine was not created");
    return false;
  }

  context_ =
    tensorrt_common::TrtUniquePtr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  if (!context_) {
    RCLCPP_ERROR(rclcpp::get_logger("lidar_centerpoint"), "Failed to create context");
    return false;
  }

  return true;
}

bool TensorRTWrapper::parseONNX(
  const std::string & onnx_path, const std::string & engine_path, const std::string & precision,
  const size_t workspace_size)
{
  auto builder =
    tensorrt_common::TrtUniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(logger_));
  if (!builder) {
    RCLCPP_ERROR(rclcpp::get_logger("lidar_centerpoint"), "Failed to create builder");
    return false;
  }

  auto config =
    tensorrt_common::TrtUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) {
    RCLCPP_ERROR(rclcpp::get_logger("lidar_centerpoint"), "Failed to create config");
    return false;
  }
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8400
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, workspace_size);
#else
  config->setMaxWorkspaceSize(workspace_size);
#endif
  if (precision == "fp16") {
    if (builder->platformHasFastFp16()) {
      RCLCPP_INFO(rclcpp::get_logger("lidar_centerpoint"), "Using TensorRT FP16 Inference");
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("lidar_centerpoint"),
        "TensorRT FP16 Inference isn't supported in this environment");
    }
  }

  const auto flag =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network =
    tensorrt_common::TrtUniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(flag));
  if (!network) {
    RCLCPP_ERROR(rclcpp::get_logger("lidar_centerpoint"), "Failed to create network");
    return false;
  }

  auto parser = tensorrt_common::TrtUniquePtr<nvonnxparser::IParser>(
    nvonnxparser::createParser(*network, logger_));
  parser->parseFromFile(onnx_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR));

  if (!setProfile(*builder, *network, *config)) {
    RCLCPP_ERROR(rclcpp::get_logger("lidar_centerpoint"), "Failed to set profile");
    return false;
  }

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("lidar_centerpoint"),
    "Applying optimizations and building TRT CUDA engine (" << onnx_path << ") ...");
  plan_ = tensorrt_common::TrtUniquePtr<nvinfer1::IHostMemory>(
    builder->buildSerializedNetwork(*network, *config));
  if (!plan_) {
    RCLCPP_ERROR(rclcpp::get_logger("lidar_centerpoint"), "Failed to create serialized network");
    return false;
  }
  engine_ = tensorrt_common::TrtUniquePtr<nvinfer1::ICudaEngine>(
    runtime_->deserializeCudaEngine(plan_->data(), plan_->size()));
  if (!engine_) {
    RCLCPP_ERROR(rclcpp::get_logger("lidar_centerpoint"), "Failed to create engine");
    return false;
  }

  return saveEngine(engine_path);
}

bool TensorRTWrapper::saveEngine(const std::string & engine_path)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_centerpoint"), "Writing to " << engine_path);
  std::ofstream file(engine_path, std::ios::out | std::ios::binary);
  file.write(reinterpret_cast<const char *>(plan_->data()), plan_->size());
  return true;
}

bool TensorRTWrapper::loadEngine(const std::string & engine_path)
{
  std::ifstream engine_file(engine_path);
  std::stringstream engine_buffer;
  engine_buffer << engine_file.rdbuf();
  std::string engine_str = engine_buffer.str();
  engine_ = tensorrt_common::TrtUniquePtr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(
    reinterpret_cast<const void *>(engine_str.data()), engine_str.size()));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_centerpoint"), "Loaded engine from " << engine_path);
  return true;
}

}  // namespace centerpoint
