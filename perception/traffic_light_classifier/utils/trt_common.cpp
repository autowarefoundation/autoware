// Copyright 2020 Tier IV, Inc.
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

#include <trt_common.hpp>

#if (defined(_MSC_VER) or (defined(__GNUC__) and (7 <= __GNUC_MAJOR__)))
#include <filesystem>
namespace fs = ::std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = ::std::experimental::filesystem;
#endif

#include <functional>
#include <string>

namespace Tn
{
void check_error(const ::cudaError_t e, decltype(__FILE__) f, decltype(__LINE__) n)
{
  if (e != ::cudaSuccess) {
    std::stringstream s;
    s << ::cudaGetErrorName(e) << " (" << e << ")@" << f << "#L" << n << ": "
      << ::cudaGetErrorString(e);
    throw std::runtime_error{s.str()};
  }
}

TrtCommon::TrtCommon(std::string model_path, std::string precision)
: model_file_path_(model_path),
  precision_(precision),
  input_name_("input_0"),
  output_name_("output_0"),
  is_initialized_(false)
{
  runtime_ = UniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
}

void TrtCommon::setup()
{
  const fs::path path(model_file_path_);
  std::string extension = path.extension().string();

  if (fs::exists(path)) {
    if (extension == ".engine") {
      loadEngine(model_file_path_);
    } else if (extension == ".onnx") {
      fs::path cache_engine_path{model_file_path_};
      cache_engine_path.replace_extension("engine");
      if (fs::exists(cache_engine_path)) {
        loadEngine(cache_engine_path.string());
      } else {
        logger_.log(nvinfer1::ILogger::Severity::kINFO, "start build engine");
        buildEngineFromOnnx(model_file_path_, cache_engine_path.string());
        logger_.log(nvinfer1::ILogger::Severity::kINFO, "end build engine");
      }
    } else {
      is_initialized_ = false;
      return;
    }
  } else {
    is_initialized_ = false;
    return;
  }

  context_ = UniquePtr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  input_dims_ = engine_->getBindingDimensions(getInputBindingIndex());
  output_dims_ = engine_->getBindingDimensions(getOutputBindingIndex());
  is_initialized_ = true;
}

bool TrtCommon::loadEngine(std::string engine_file_path)
{
  std::ifstream engine_file(engine_file_path);
  std::stringstream engine_buffer;
  engine_buffer << engine_file.rdbuf();
  std::string engine_str = engine_buffer.str();
  engine_ = UniquePtr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(
    reinterpret_cast<const void *>(engine_str.data()), engine_str.size()));
  return true;
}

bool TrtCommon::buildEngineFromOnnx(std::string onnx_file_path, std::string output_engine_file_path)
{
  auto builder = UniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(logger_));
  const auto explicitBatch =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = UniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
  auto config = UniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());

  auto parser = UniquePtr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, logger_));
  if (!parser->parseFromFile(
        onnx_file_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR))) {
    return false;
  }

#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8400
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 16 << 20);
#else
  config->setMaxWorkspaceSize(16 << 20);
#endif

  if (precision_ == "fp16") {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  } else if (precision_ == "int8") {
    config->setFlag(nvinfer1::BuilderFlag::kINT8);
  } else {
    return false;
  }

  auto plan = UniquePtr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));
  if (!plan) {
    return false;
  }
  engine_ =
    UniquePtr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(plan->data(), plan->size()));
  if (!engine_) {
    return false;
  }

  // save engine
  std::ofstream file;
  file.open(output_engine_file_path, std::ios::binary | std::ios::out);
  if (!file.is_open()) {
    return false;
  }
  file.write((const char *)plan->data(), plan->size());
  file.close();

  return true;
}

bool TrtCommon::isInitialized() { return is_initialized_; }

int TrtCommon::getNumInput()
{
  return std::accumulate(
    input_dims_.d, input_dims_.d + input_dims_.nbDims, 1, std::multiplies<int>());
}

int TrtCommon::getNumOutput()
{
  return std::accumulate(
    output_dims_.d, output_dims_.d + output_dims_.nbDims, 1, std::multiplies<int>());
}

int TrtCommon::getInputBindingIndex() { return engine_->getBindingIndex(input_name_.c_str()); }

int TrtCommon::getOutputBindingIndex() { return engine_->getBindingIndex(output_name_.c_str()); }

}  // namespace Tn
