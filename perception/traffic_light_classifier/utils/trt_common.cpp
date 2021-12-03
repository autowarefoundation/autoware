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

TrtCommon::TrtCommon(std::string model_path, std::string cache_dir, std::string precision)
: model_file_path_(model_path),
  cache_dir_(cache_dir),
  precision_(precision),
  input_name_("input_0"),
  output_name_("output_0"),
  is_initialized_(false),
  max_batch_size_(1)
{
}

void TrtCommon::setup()
{
  const boost::filesystem::path path(model_file_path_);
  std::string extension = path.extension().string();

  if (boost::filesystem::exists(path)) {
    if (extension == ".engine") {
      loadEngine(model_file_path_);
    } else if (extension == ".onnx") {
      std::string cache_engine_path = cache_dir_ + "/" + path.stem().string() + ".engine";
      const boost::filesystem::path cache_path(cache_engine_path);
      if (boost::filesystem::exists(cache_path)) {
        loadEngine(cache_engine_path);
      } else {
        logger_.log(nvinfer1::ILogger::Severity::kINFO, "start build engine");
        buildEngineFromOnnx(model_file_path_, cache_engine_path);
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
  runtime_ = UniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
  engine_ = UniquePtr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(
    reinterpret_cast<const void *>(engine_str.data()), engine_str.size(), nullptr));
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

  builder->setMaxBatchSize(max_batch_size_);
  config->setMaxWorkspaceSize(16 << 20);

  if (precision_ == "fp16") {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  } else if (precision_ == "int8") {
    config->setFlag(nvinfer1::BuilderFlag::kINT8);
  } else {
    return false;
  }

  engine_ = UniquePtr<nvinfer1::ICudaEngine>(builder->buildEngineWithConfig(*network, *config));
  if (!engine_) {
    return false;
  }

  // save engine
  nvinfer1::IHostMemory * data = engine_->serialize();
  std::ofstream file;
  file.open(output_engine_file_path, std::ios::binary | std::ios::out);
  if (!file.is_open()) {
    return false;
  }
  file.write((const char *)data->data(), data->size());
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
