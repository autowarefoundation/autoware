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

#include <trt_ssd.hpp>

#include <NvOnnxConfig.h>
#include <NvOnnxParser.h>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace ssd
{
void Net::load(const std::string & path)
{
  std::ifstream file(path, std::ios::in | std::ios::binary);
  file.seekg(0, file.end);
  size_t size = file.tellg();
  file.seekg(0, file.beg);

  char * buffer = new char[size];
  file.read(buffer, size);
  file.close();
  if (runtime_) {
    engine_ = unique_ptr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(buffer, size));
  }
  delete[] buffer;
}

void Net::prepare()
{
  if (engine_) {
    context_ = unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  }
  cudaStreamCreate(&stream_);
}

Net::Net(const std::string & path, bool verbose)
{
  Logger logger(verbose);
  runtime_ = unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger));
  load(path);
  prepare();
}

Net::~Net()
{
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

Net::Net(
  const std::string & onnx_file_path, const std::string & precision, const int max_batch_size,
  bool verbose, size_t workspace_size)
{
  Logger logger(verbose);
  runtime_ = unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger));
  if (!runtime_) {
    std::cout << "Fail to create runtime" << std::endl;
    return;
  }
  bool fp16 = precision.compare("FP16") == 0;
  bool int8 = precision.compare("INT8") == 0;

  // Create builder
  auto builder = unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(logger));
  if (!builder) {
    std::cout << "Fail to create builder" << std::endl;
    return;
  }
  auto config = unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) {
    std::cout << "Fail to create builder config" << std::endl;
    return;
  }
  // Allow use of FP16 layers when running in INT8
  if (fp16 || int8) {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8400
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, workspace_size);
#else
  config->setMaxWorkspaceSize(workspace_size);
#endif

  // Parse ONNX FCN
  std::cout << "Building " << precision << " core model..." << std::endl;
  const auto flag =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(flag));
  if (!network) {
    std::cout << "Fail to create network" << std::endl;
    return;
  }
  auto parser = unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, logger));
  if (!parser) {
    std::cout << "Fail to create parser" << std::endl;
    return;
  }
  parser->parseFromFile(
    onnx_file_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR));

  // TODO(someone): int8 calibrator
  /* std::unique_ptr<nvinfer1::Int8EntropyCalibrator> calib;
    if (int8) {
        config->setFlag(BuilderFlag::kINT8);
        ImageStream stream(batch, inputDims, calibration_images);
        calib = std::unique_ptr<Int8EntropyCalibrator>(new Int8EntropyCalibrator(stream, model_name,
    calibration_table)); config->setInt8Calibrator(calib.get());
    }*/

  // create profile
  auto profile = builder->createOptimizationProfile();
  profile->setDimensions(
    network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kMIN,
    nvinfer1::Dims4{1, 3, 300, 300});
  profile->setDimensions(
    network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kOPT,
    nvinfer1::Dims4{max_batch_size, 3, 300, 300});
  profile->setDimensions(
    network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kMAX,
    nvinfer1::Dims4{max_batch_size, 3, 300, 300});
  config->addOptimizationProfile(profile);

  // Build engine
  std::cout << "Applying optimizations and building TRT CUDA engine..." << std::endl;
  plan_ = unique_ptr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));
  if (!plan_) {
    std::cout << "Fail to create serialized network" << std::endl;
    return;
  }
  engine_ = unique_ptr<nvinfer1::ICudaEngine>(
    runtime_->deserializeCudaEngine(plan_->data(), plan_->size()));
  if (!engine_) {
    std::cout << "Fail to create engine" << std::endl;
    return;
  }
  context_ = unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  if (!context_) {
    std::cout << "Fail to create context" << std::endl;
    return;
  }
}

void Net::save(const std::string & path)
{
  std::cout << "Writing to " << path << "..." << std::endl;
  std::ofstream file(path, std::ios::out | std::ios::binary);
  file.write(reinterpret_cast<const char *>(plan_->data()), plan_->size());
}

void Net::infer(std::vector<void *> & buffers, const int batch_size)
{
  if (!context_) {
    throw std::runtime_error("Fail to create context");
  }
  auto input_dims = engine_->getBindingDimensions(0);
  context_->setBindingDimensions(
    0, nvinfer1::Dims4(batch_size, input_dims.d[1], input_dims.d[2], input_dims.d[3]));
  context_->enqueueV2(buffers.data(), stream_, nullptr);
  cudaStreamSynchronize(stream_);
}

std::vector<int> Net::getInputSize()
{
  auto dims = engine_->getBindingDimensions(0);
  return {dims.d[1], dims.d[2], dims.d[3]};
}

std::vector<int> Net::getOutputScoreSize()
{
  auto dims = engine_->getBindingDimensions(1);
  return {dims.d[1], dims.d[2]};
}

int Net::getMaxBatchSize()
{
  return engine_->getProfileDimensions(0, 0, nvinfer1::OptProfileSelector::kMAX).d[0];
}

int Net::getMaxDetections() { return engine_->getBindingDimensions(1).d[1]; }

}  // namespace ssd
