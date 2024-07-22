// Copyright 2024 TIER IV, Inc.
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

#include "autoware/lidar_transfusion/network/network_trt.hpp"

#include <NvOnnxParser.h>

#include <fstream>
#include <memory>
#include <string>

namespace autoware::lidar_transfusion
{

std::ostream & operator<<(std::ostream & os, const ProfileDimension & profile)
{
  std::string delim = "";
  os << "min->[";
  for (int i = 0; i < profile.min.nbDims; ++i) {
    os << delim << profile.min.d[i];
    delim = ", ";
  }
  os << "], opt->[";
  delim = "";
  for (int i = 0; i < profile.opt.nbDims; ++i) {
    os << delim << profile.opt.d[i];
    delim = ", ";
  }
  os << "], max->[";
  delim = "";
  for (int i = 0; i < profile.max.nbDims; ++i) {
    os << delim << profile.max.d[i];
    delim = ", ";
  }
  os << "]";
  return os;
}

NetworkTRT::NetworkTRT(const TransfusionConfig & config) : config_(config)
{
  ProfileDimension voxels_dims = {
    nvinfer1::Dims3(
      config_.min_voxel_size_, config_.min_point_in_voxel_size_, config_.min_network_feature_size_),
    nvinfer1::Dims3(
      config_.opt_voxel_size_, config_.opt_point_in_voxel_size_, config_.opt_network_feature_size_),
    nvinfer1::Dims3(
      config_.max_voxel_size_, config_.max_point_in_voxel_size_,
      config_.max_network_feature_size_)};
  ProfileDimension num_points_dims = {
    nvinfer1::Dims{1, {static_cast<int32_t>(config_.min_points_size_)}},
    nvinfer1::Dims{1, {static_cast<int32_t>(config_.opt_points_size_)}},
    nvinfer1::Dims{1, {static_cast<int32_t>(config_.max_points_size_)}}};
  ProfileDimension coors_dims = {
    nvinfer1::Dims2(config_.min_coors_size_, config_.min_coors_dim_size_),
    nvinfer1::Dims2(config_.opt_coors_size_, config_.opt_coors_dim_size_),
    nvinfer1::Dims2(config_.max_coors_size_, config_.max_coors_dim_size_)};
  in_profile_dims_ = {voxels_dims, num_points_dims, coors_dims};
}

NetworkTRT::~NetworkTRT()
{
  context.reset();
  runtime_.reset();
  plan_.reset();
  engine.reset();
}

bool NetworkTRT::init(
  const std::string & onnx_path, const std::string & engine_path, const std::string & precision)
{
  runtime_ =
    tensorrt_common::TrtUniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create runtime" << std::endl;
    return false;
  }

  bool success;
  std::ifstream engine_file(engine_path);
  if (engine_file.is_open()) {
    success = loadEngine(engine_path);
  } else {
    auto log_thread = logger_.log_throttle(
      nvinfer1::ILogger::Severity::kINFO,
      "Applying optimizations and building TRT CUDA engine. Please wait a minutes...", 5);
    success = parseONNX(onnx_path, engine_path, precision);
    logger_.stop_throttle(log_thread);
  }
  success &= createContext();

  return success;
}

bool NetworkTRT::setProfile(
  nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
  nvinfer1::IBuilderConfig & config)
{
  auto profile = builder.createOptimizationProfile();

  auto voxels_name = network.getInput(NetworkIO::voxels)->getName();
  auto num_points_name = network.getInput(NetworkIO::num_points)->getName();
  auto coors_name = network.getInput(NetworkIO::coors)->getName();

  profile->setDimensions(
    voxels_name, nvinfer1::OptProfileSelector::kMIN, in_profile_dims_[NetworkIO::voxels].min);
  profile->setDimensions(
    voxels_name, nvinfer1::OptProfileSelector::kOPT, in_profile_dims_[NetworkIO::voxels].opt);
  profile->setDimensions(
    voxels_name, nvinfer1::OptProfileSelector::kMAX, in_profile_dims_[NetworkIO::voxels].max);

  profile->setDimensions(
    num_points_name, nvinfer1::OptProfileSelector::kMIN,
    in_profile_dims_[NetworkIO::num_points].min);
  profile->setDimensions(
    num_points_name, nvinfer1::OptProfileSelector::kOPT,
    in_profile_dims_[NetworkIO::num_points].opt);
  profile->setDimensions(
    num_points_name, nvinfer1::OptProfileSelector::kMAX,
    in_profile_dims_[NetworkIO::num_points].max);

  profile->setDimensions(
    coors_name, nvinfer1::OptProfileSelector::kMIN, in_profile_dims_[NetworkIO::coors].min);
  profile->setDimensions(
    coors_name, nvinfer1::OptProfileSelector::kOPT, in_profile_dims_[NetworkIO::coors].opt);
  profile->setDimensions(
    coors_name, nvinfer1::OptProfileSelector::kMAX, in_profile_dims_[NetworkIO::coors].max);

  config.addOptimizationProfile(profile);
  return true;
}

bool NetworkTRT::createContext()
{
  if (!engine) {
    tensorrt_common::LOG_ERROR(logger_)
      << "Failed to create context: Engine was not created" << std::endl;
    return false;
  }

  context =
    tensorrt_common::TrtUniquePtr<nvinfer1::IExecutionContext>(engine->createExecutionContext());
  if (!context) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create context" << std::endl;
    return false;
  }

  return true;
}

bool NetworkTRT::parseONNX(
  const std::string & onnx_path, const std::string & engine_path, const std::string & precision,
  const size_t workspace_size)
{
  auto builder =
    tensorrt_common::TrtUniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(logger_));
  if (!builder) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create builder" << std::endl;
    return false;
  }

  auto config =
    tensorrt_common::TrtUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create config" << std::endl;
    return false;
  }
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8400
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, workspace_size);
#else
  config->setMaxWorkspaceSize(workspace_size);
#endif
  if (precision == "fp16") {
    if (builder->platformHasFastFp16()) {
      tensorrt_common::LOG_INFO(logger_) << "Using TensorRT FP16 Inference" << std::endl;
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
    } else {
      tensorrt_common::LOG_INFO(logger_)
        << "TensorRT FP16 Inference isn't supported in this environment" << std::endl;
    }
  }

  const auto flag =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network =
    tensorrt_common::TrtUniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(flag));
  if (!network) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create network" << std::endl;
    return false;
  }

  auto parser = tensorrt_common::TrtUniquePtr<nvonnxparser::IParser>(
    nvonnxparser::createParser(*network, logger_));
  parser->parseFromFile(onnx_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR));

  if (!setProfile(*builder, *network, *config)) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to set profile" << std::endl;
    return false;
  }

  plan_ = tensorrt_common::TrtUniquePtr<nvinfer1::IHostMemory>(
    builder->buildSerializedNetwork(*network, *config));
  if (!plan_) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create serialized network" << std::endl;
    return false;
  }
  engine = tensorrt_common::TrtUniquePtr<nvinfer1::ICudaEngine>(
    runtime_->deserializeCudaEngine(plan_->data(), plan_->size()));
  if (!engine) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create engine" << std::endl;
    return false;
  }

  return saveEngine(engine_path);
}

bool NetworkTRT::saveEngine(const std::string & engine_path)
{
  tensorrt_common::LOG_INFO(logger_) << "Writing to " << engine_path << std::endl;
  std::ofstream file(engine_path, std::ios::out | std::ios::binary);
  file.write(reinterpret_cast<const char *>(plan_->data()), plan_->size());
  return validateNetworkIO();
}

bool NetworkTRT::loadEngine(const std::string & engine_path)
{
  std::ifstream engine_file(engine_path);
  std::stringstream engine_buffer;
  engine_buffer << engine_file.rdbuf();
  std::string engine_str = engine_buffer.str();
  engine = tensorrt_common::TrtUniquePtr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(
    reinterpret_cast<const void *>(engine_str.data()), engine_str.size()));
  tensorrt_common::LOG_INFO(logger_) << "Loaded engine from " << engine_path << std::endl;
  return validateNetworkIO();
}

bool NetworkTRT::validateNetworkIO()
{
  // Whether the number of IO match the expected size
  if (engine->getNbIOTensors() != NetworkIO::ENUM_SIZE) {
    tensorrt_common::LOG_ERROR(logger_)
      << "Invalid network IO. Expected size: " << NetworkIO::ENUM_SIZE
      << ". Actual size: " << engine->getNbIOTensors() << "." << std::endl;
    throw std::runtime_error("Failed to initialize TRT network.");
  }
  for (int i = 0; i < NetworkIO::ENUM_SIZE; ++i) {
    tensors_names_.push_back(engine->getIOTensorName(i));
  }

  // Log the network IO
  std::string tensors = std::accumulate(
    tensors_names_.begin(), tensors_names_.end(), std::string(),
    [](const std::string & a, const std::string & b) -> std::string { return a + b + " "; });
  tensorrt_common::LOG_INFO(logger_) << "Network IO: " << tensors << std::endl;

  // Whether the current engine input profile match the config input profile
  for (int i = 0; i <= NetworkIO::coors; ++i) {
    ProfileDimension engine_dims{
      engine->getProfileShape(tensors_names_[i], 0, nvinfer1::OptProfileSelector::kMIN),
      engine->getProfileShape(tensors_names_[i], 0, nvinfer1::OptProfileSelector::kOPT),
      engine->getProfileShape(tensors_names_[i], 0, nvinfer1::OptProfileSelector::kMAX)};

    tensorrt_common::LOG_INFO(logger_)
      << "Profile for " << tensors_names_[i] << ": " << engine_dims << std::endl;

    if (engine_dims != in_profile_dims_[i]) {
      tensorrt_common::LOG_ERROR(logger_)
        << "Invalid network input dimension. Config: " << in_profile_dims_[i]
        << ". Please change the input profile or delete the engine file and build engine again."
        << std::endl;
      throw std::runtime_error("Failed to initialize TRT network.");
    }
  }

  // Whether the IO tensor shapes match the network config, -1 for dynamic size
  validateTensorShape(
    NetworkIO::voxels, {-1, static_cast<int>(config_.points_per_voxel_),
                        static_cast<int>(config_.num_point_feature_size_)});
  validateTensorShape(NetworkIO::num_points, {-1});
  validateTensorShape(NetworkIO::coors, {-1, static_cast<int>(config_.num_point_values_)});
  auto cls_score = validateTensorShape(
    NetworkIO::cls_score,
    {static_cast<int>(config_.batch_size_), static_cast<int>(config_.num_classes_),
     static_cast<int>(config_.num_proposals_)});
  tensorrt_common::LOG_INFO(logger_) << "Network num classes: " << cls_score.d[1] << std::endl;
  validateTensorShape(
    NetworkIO::dir_pred,
    {static_cast<int>(config_.batch_size_), 2, static_cast<int>(config_.num_proposals_)});  // x, y
  validateTensorShape(
    NetworkIO::bbox_pred,
    {static_cast<int>(config_.batch_size_), static_cast<int>(config_.num_box_values_),
     static_cast<int>(config_.num_proposals_)});

  return true;
}

const char * NetworkTRT::getTensorName(NetworkIO name)
{
  return tensors_names_.at(name);
}

nvinfer1::Dims NetworkTRT::validateTensorShape(NetworkIO name, const std::vector<int> shape)
{
  auto tensor_shape = engine->getTensorShape(tensors_names_[name]);
  if (tensor_shape.nbDims != static_cast<int>(shape.size())) {
    tensorrt_common::LOG_ERROR(logger_)
      << "Invalid tensor shape for " << tensors_names_[name] << ". Expected size: " << shape.size()
      << ". Actual size: " << tensor_shape.nbDims << "." << std::endl;
    throw std::runtime_error("Failed to initialize TRT network.");
  }
  for (int i = 0; i < tensor_shape.nbDims; ++i) {
    if (tensor_shape.d[i] != static_cast<int>(shape[i])) {
      tensorrt_common::LOG_ERROR(logger_)
        << "Invalid tensor shape for " << tensors_names_[name] << ". Expected: " << shape[i]
        << ". Actual: " << tensor_shape.d[i] << "." << std::endl;
      throw std::runtime_error("Failed to initialize TRT network.");
    }
  }
  return tensor_shape;
}

}  // namespace autoware::lidar_transfusion
