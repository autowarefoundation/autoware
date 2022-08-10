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

#include <NvOnnxParser.h>

#include <fstream>
#include <memory>
#include <string>

namespace centerpoint
{
TensorRTWrapper::TensorRTWrapper(const CenterPointConfig & config, const bool verbose)
: config_(config), logger_(Logger(verbose))
{
}

bool TensorRTWrapper::init(
  const std::string & onnx_path, const std::string & engine_path, const std::string & precision)
{
  runtime_ = unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    std::cout << "Fail to create runtime" << std::endl;
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
    std::cout << "Fail to create context: Engine isn't created" << std::endl;
    return false;
  }

  context_ = unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  if (!context_) {
    std::cout << "Fail to create context" << std::endl;
    return false;
  }

  return true;
}

bool TensorRTWrapper::parseONNX(
  const std::string & onnx_path, const std::string & engine_path, const std::string & precision,
  const size_t workspace_size)
{
  auto builder = unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(logger_));
  if (!builder) {
    std::cout << "Fail to create builder" << std::endl;
    return false;
  }

  auto config = unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) {
    std::cout << "Fail to create config" << std::endl;
    return false;
  }
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8400
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, workspace_size);
#else
  config->setMaxWorkspaceSize(workspace_size);
#endif
  if (precision == "fp16") {
    if (builder->platformHasFastFp16()) {
      std::cout << "use TensorRT FP16 Inference" << std::endl;
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
    } else {
      std::cout << "TensorRT FP16 Inference isn't supported in this environment" << std::endl;
    }
  }

  const auto flag =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(flag));
  if (!network) {
    std::cout << "Fail to create network" << std::endl;
    return false;
  }

  auto parser = unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, logger_));
  parser->parseFromFile(onnx_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR));

  if (!setProfile(*builder, *network, *config)) {
    std::cout << "Fail to set profile" << std::endl;
    return false;
  }

  std::cout << "Applying optimizations and building TRT CUDA engine (" << onnx_path << ") ..."
            << std::endl;
  plan_ = unique_ptr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));
  if (!plan_) {
    std::cout << "Fail to create serialized network" << std::endl;
    return false;
  }
  engine_ = unique_ptr<nvinfer1::ICudaEngine>(
    runtime_->deserializeCudaEngine(plan_->data(), plan_->size()));
  if (!engine_) {
    std::cout << "Fail to create engine" << std::endl;
    return false;
  }

  return saveEngine(engine_path);
}

bool TensorRTWrapper::saveEngine(const std::string & engine_path)
{
  std::cout << "Writing to " << engine_path << std::endl;
  std::ofstream file(engine_path, std::ios::out | std::ios::binary);
  file.write(reinterpret_cast<const char *>(plan_->data()), plan_->size());
  return true;
}

bool TensorRTWrapper::loadEngine(const std::string & engine_path)
{
  std::ifstream file(engine_path, std::ios::in | std::ios::binary);
  file.seekg(0, std::ifstream::end);
  const size_t size = file.tellg();
  file.seekg(0, std::ifstream::beg);

  std::unique_ptr<char[]> buffer{new char[size]};
  file.read(buffer.get(), size);
  file.close();

  if (!runtime_) {
    std::cout << "Fail to load engine: Runtime isn't created" << std::endl;
    return false;
  }

  std::cout << "Loading from " << engine_path << std::endl;
  engine_ = unique_ptr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(buffer.get(), size));
  return true;
}

}  // namespace centerpoint
