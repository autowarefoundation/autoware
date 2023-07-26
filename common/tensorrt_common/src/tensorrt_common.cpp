// Copyright 2023 TIER IV, Inc.
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

#include <tensorrt_common/tensorrt_common.hpp>

#include <NvInferPlugin.h>
#include <dlfcn.h>

#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace
{
template <class T>
bool contain(const std::string & s, const T & v)
{
  return s.find(v) != std::string::npos;
}
}  // anonymous namespace

namespace tensorrt_common
{
nvinfer1::Dims get_input_dims(const std::string & onnx_file_path)
{
  Logger logger_;
  auto builder = TrtUniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(logger_));
  if (!builder) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create builder");
  }

  const auto explicitBatch =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);

  auto network =
    TrtUniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
  if (!network) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create network");
  }

  auto config = TrtUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create builder config");
  }

  auto parser = TrtUniquePtr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, logger_));
  if (!parser->parseFromFile(
        onnx_file_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR))) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Failed to parse onnx file");
  }

  const auto input = network->getInput(0);
  return input->getDimensions();
}

bool is_valid_precision_string(const std::string & precision)
{
  if (
    std::find(valid_precisions.begin(), valid_precisions.end(), precision) ==
    valid_precisions.end()) {
    std::stringstream message;
    message << "Invalid precision was specified: " << precision << std::endl
            << "Valid string is one of: [";
    for (const auto & s : valid_precisions) {
      message << s << ", ";
    }
    message << "] (case sensitive)" << std::endl;
    std::cerr << message.str();
    return false;
  } else {
    return true;
  }
}

TrtCommon::TrtCommon(
  const std::string & model_path, const std::string & precision,
  std::unique_ptr<nvinfer1::IInt8Calibrator> calibrator, const BatchConfig & batch_config,
  const size_t max_workspace_size, const BuildConfig & build_config,
  const std::vector<std::string> & plugin_paths)
: model_file_path_(model_path),
  calibrator_(std::move(calibrator)),
  precision_(precision),
  batch_config_(batch_config),
  max_workspace_size_(max_workspace_size),
  model_profiler_("Model"),
  host_profiler_("Host")
{
  // Check given precision is valid one
  if (!is_valid_precision_string(precision)) {
    return;
  }
  build_config_ = std::make_unique<const BuildConfig>(build_config);

  for (const auto & plugin_path : plugin_paths) {
    int32_t flags{RTLD_LAZY};
// cspell: ignore asan
#if ENABLE_ASAN
    // https://github.com/google/sanitizers/issues/89
    // asan doesn't handle module unloading correctly and there are no plans on doing
    // so. In order to get proper stack traces, don't delete the shared library on
    // close so that asan can resolve the symbols correctly.
    flags |= RTLD_NODELETE;
#endif  // ENABLE_ASAN
    void * handle = dlopen(plugin_path.c_str(), flags);
    if (!handle) {
      logger_.log(nvinfer1::ILogger::Severity::kERROR, "Could not load plugin library");
    }
  }
  runtime_ = TrtUniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
  if (build_config_->dla_core_id != -1) {
    runtime_->setDLACore(build_config_->dla_core_id);
  }
  initLibNvInferPlugins(&logger_, "");
}

TrtCommon::~TrtCommon()
{
}

void TrtCommon::setup()
{
  if (!fs::exists(model_file_path_)) {
    is_initialized_ = false;
    return;
  }
  std::string engine_path = model_file_path_;
  if (model_file_path_.extension() == ".engine") {
    std::cout << "Load ... " << model_file_path_ << std::endl;
    loadEngine(model_file_path_);
  } else if (model_file_path_.extension() == ".onnx") {
    fs::path cache_engine_path{model_file_path_};
    std::string ext;
    std::string calib_name = "";
    if (precision_ == "int8") {
      if (build_config_->calib_type_str == "Entropy") {
        calib_name = "EntropyV2-";
      } else if (
        build_config_->calib_type_str == "Legacy" ||
        build_config_->calib_type_str == "Percentile") {
        calib_name = "Legacy-";
      } else {
        calib_name = "MinMax-";
      }
    }
    if (build_config_->dla_core_id != -1) {
      ext = "DLA" + std::to_string(build_config_->dla_core_id) + "-" + calib_name + precision_;
      if (build_config_->quantize_first_layer) {
        ext += "-firstFP16";
      }
      if (build_config_->quantize_last_layer) {
        ext += "-lastFP16";
      }
      ext += "-batch" + std::to_string(batch_config_[0]) + ".engine";
    } else {
      ext = calib_name + precision_;
      if (build_config_->quantize_first_layer) {
        ext += "-firstFP16";
      }
      if (build_config_->quantize_last_layer) {
        ext += "-lastFP16";
      }
      ext += "-batch" + std::to_string(batch_config_[0]) + ".engine";
    }
    cache_engine_path.replace_extension(ext);

    // Output Network Information
    printNetworkInfo(model_file_path_);

    if (fs::exists(cache_engine_path)) {
      std::cout << "Loading... " << cache_engine_path << std::endl;
      loadEngine(cache_engine_path);
    } else {
      std::cout << "Building... " << cache_engine_path << std::endl;
      logger_.log(nvinfer1::ILogger::Severity::kINFO, "Start build engine");
      buildEngineFromOnnx(model_file_path_, cache_engine_path);
      logger_.log(nvinfer1::ILogger::Severity::kINFO, "End build engine");
    }
    engine_path = cache_engine_path;
  } else {
    is_initialized_ = false;
    return;
  }

  context_ = TrtUniquePtr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  if (!context_) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create context");
    is_initialized_ = false;
    return;
  }

  if (build_config_->profile_per_layer) {
    context_->setProfiler(&model_profiler_);
  }
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8200
  // Write profiles for trt-engine-explorer
  // See: https://github.com/NVIDIA/TensorRT/tree/main/tools/experimental/trt-engine-explorer
  std::string j_ext = ".json";
  fs::path json_path{engine_path};
  json_path.replace_extension(j_ext);
  std::string ret = getLayerInformation(nvinfer1::LayerInformationFormat::kJSON);
  std::ofstream os(json_path, std::ofstream::trunc);
  os << ret << std::flush;
#endif

  is_initialized_ = true;
}

bool TrtCommon::loadEngine(const std::string & engine_file_path)
{
  std::ifstream engine_file(engine_file_path);
  std::stringstream engine_buffer;
  engine_buffer << engine_file.rdbuf();
  std::string engine_str = engine_buffer.str();
  engine_ = TrtUniquePtr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(
    reinterpret_cast<const void *>(engine_str.data()), engine_str.size()));
  return true;
}

void TrtCommon::printNetworkInfo(const std::string & onnx_file_path)
{
  auto builder = TrtUniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(logger_));
  if (!builder) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create builder");
    return;
  }

  const auto explicitBatch =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);

  auto network =
    TrtUniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
  if (!network) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create network");
    return;
  }

  auto config = TrtUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create builder config");
    return;
  }

  if (precision_ == "fp16" || precision_ == "int8") {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8400
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, max_workspace_size_);
#else
  config->setMaxWorkspaceSize(max_workspace_size_);
#endif

  auto parser = TrtUniquePtr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, logger_));
  if (!parser->parseFromFile(
        onnx_file_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR))) {
    return;
  }
  int num = network->getNbLayers();
  float total_gflops = 0.0;
  int total_params = 0;
  for (int i = 0; i < num; i++) {
    nvinfer1::ILayer * layer = network->getLayer(i);
    auto layer_type = layer->getType();
    std::string name = layer->getName();
    if (build_config_->profile_per_layer) {
      model_profiler_.setProfDict(layer);
    }
    if (layer_type == nvinfer1::LayerType::kCONSTANT) {
      continue;
    }
    nvinfer1::ITensor * in = layer->getInput(0);
    nvinfer1::Dims dim_in = in->getDimensions();
    nvinfer1::ITensor * out = layer->getOutput(0);
    nvinfer1::Dims dim_out = out->getDimensions();

    if (layer_type == nvinfer1::LayerType::kCONVOLUTION) {
      nvinfer1::IConvolutionLayer * conv = (nvinfer1::IConvolutionLayer *)layer;
      nvinfer1::Dims k_dims = conv->getKernelSizeNd();
      nvinfer1::Dims s_dims = conv->getStrideNd();
      int groups = conv->getNbGroups();
      int stride = s_dims.d[0];
      int num_weights = (dim_in.d[1] / groups) * dim_out.d[1] * k_dims.d[0] * k_dims.d[1];
      float gflops = (2 * num_weights) * (dim_in.d[3] / stride * dim_in.d[2] / stride / 1e9);
      ;
      total_gflops += gflops;
      total_params += num_weights;
      std::cout << "L" << i << " [conv " << k_dims.d[0] << "x" << k_dims.d[1] << " (" << groups
                << ") "
                << "/" << s_dims.d[0] << "] " << dim_in.d[3] << "x" << dim_in.d[2] << "x"
                << dim_in.d[1] << " -> " << dim_out.d[3] << "x" << dim_out.d[2] << "x"
                << dim_out.d[1];
      std::cout << " weights:" << num_weights;
      std::cout << " GFLOPs:" << gflops;
      std::cout << std::endl;
    } else if (layer_type == nvinfer1::LayerType::kPOOLING) {
      nvinfer1::IPoolingLayer * pool = (nvinfer1::IPoolingLayer *)layer;
      auto p_type = pool->getPoolingType();
      nvinfer1::Dims dim_stride = pool->getStrideNd();
      nvinfer1::Dims dim_window = pool->getWindowSizeNd();

      std::cout << "L" << i << " [";
      if (p_type == nvinfer1::PoolingType::kMAX) {
        std::cout << "max ";
      } else if (p_type == nvinfer1::PoolingType::kAVERAGE) {
        std::cout << "avg ";
      } else if (p_type == nvinfer1::PoolingType::kMAX_AVERAGE_BLEND) {
        std::cout << "max avg blend ";
      }
      float gflops = dim_in.d[1] * dim_window.d[0] / dim_stride.d[0] * dim_window.d[1] /
                     dim_stride.d[1] * dim_in.d[2] * dim_in.d[3] / 1e9;
      total_gflops += gflops;
      std::cout << "pool " << dim_window.d[0] << "x" << dim_window.d[1] << "]";
      std::cout << " GFLOPs:" << gflops;
      std::cout << std::endl;
    } else if (layer_type == nvinfer1::LayerType::kRESIZE) {
      std::cout << "L" << i << " [resize]" << std::endl;
    }
  }
  std::cout << "Total " << total_gflops << " GFLOPs" << std::endl;
  std::cout << "Total " << total_params / 1000.0 / 1000.0 << " M params" << std::endl;
  return;
}

bool TrtCommon::buildEngineFromOnnx(
  const std::string & onnx_file_path, const std::string & output_engine_file_path)
{
  auto builder = TrtUniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(logger_));
  if (!builder) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create builder");
    return false;
  }

  const auto explicitBatch =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);

  auto network =
    TrtUniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
  if (!network) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create network");
    return false;
  }

  auto config = TrtUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create builder config");
    return false;
  }

  int num_available_dla = builder->getNbDLACores();
  if (build_config_->dla_core_id != -1) {
    if (num_available_dla > 0) {
      std::cout << "###" << num_available_dla << " DLAs are supported! ###" << std::endl;
    } else {
      std::cout << "###Warning : "
                << "No DLA is supported! ###" << std::endl;
    }
    config->setDefaultDeviceType(nvinfer1::DeviceType::kDLA);
    config->setDLACore(build_config_->dla_core_id);
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8200
    config->setFlag(nvinfer1::BuilderFlag::kPREFER_PRECISION_CONSTRAINTS);
#else
    config->setFlag(nvinfer1::BuilderFlag::kSTRICT_TYPES);
#endif
    config->setFlag(nvinfer1::BuilderFlag::kGPU_FALLBACK);
  }
  if (precision_ == "fp16" || precision_ == "int8") {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8400
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, max_workspace_size_);
#else
  config->setMaxWorkspaceSize(max_workspace_size_);
#endif

  auto parser = TrtUniquePtr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, logger_));
  if (!parser->parseFromFile(
        onnx_file_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR))) {
    std::cout << "Failed to parse onnx file" << std::endl;
    return false;
  }

  const int num = network->getNbLayers();
  bool first = build_config_->quantize_first_layer;
  bool last = build_config_->quantize_last_layer;
  // Partial Quantization
  if (precision_ == "int8") {
    network->getInput(0)->setDynamicRange(0, 255.0);
    for (int i = 0; i < num; i++) {
      nvinfer1::ILayer * layer = network->getLayer(i);
      auto layer_type = layer->getType();
      std::string name = layer->getName();
      nvinfer1::ITensor * out = layer->getOutput(0);
      if (build_config_->clip_value > 0.0) {
        std::cout << "Set max value for outputs : " << build_config_->clip_value << "  " << name
                  << std::endl;
        out->setDynamicRange(0.0, build_config_->clip_value);
      }

      if (layer_type == nvinfer1::LayerType::kCONVOLUTION) {
        if (first) {
          layer->setPrecision(nvinfer1::DataType::kHALF);
          std::cout << "Set kHALF in " << name << std::endl;
          first = false;
        }
        if (last) {
          // cspell: ignore preds
          if (
            contain(name, "reg_preds") || contain(name, "cls_preds") ||
            contain(name, "obj_preds")) {
            layer->setPrecision(nvinfer1::DataType::kHALF);
            std::cout << "Set kHALF in " << name << std::endl;
          }
          for (int i = num - 1; i >= 0; i--) {
            nvinfer1::ILayer * layer = network->getLayer(i);
            auto layer_type = layer->getType();
            std::string name = layer->getName();
            if (layer_type == nvinfer1::LayerType::kCONVOLUTION) {
              layer->setPrecision(nvinfer1::DataType::kHALF);
              std::cout << "Set kHALF in " << name << std::endl;
              break;
            }
            if (layer_type == nvinfer1::LayerType::kMATRIX_MULTIPLY) {
              layer->setPrecision(nvinfer1::DataType::kHALF);
              std::cout << "Set kHALF in " << name << std::endl;
              break;
            }
          }
        }
      }
    }
  }

  const auto input = network->getInput(0);
  const auto input_dims = input->getDimensions();
  const auto input_channel = input_dims.d[1];
  const auto input_height = input_dims.d[2];
  const auto input_width = input_dims.d[3];
  const auto input_batch = input_dims.d[0];

  if (input_batch > 1) {
    batch_config_[0] = input_batch;
  }

  if (batch_config_.at(0) > 1 && (batch_config_.at(0) == batch_config_.at(2))) {
    // Attention : below API is deprecated in TRT8.4
    builder->setMaxBatchSize(batch_config_.at(2));
  } else {
    if (build_config_->profile_per_layer) {
      auto profile = builder->createOptimizationProfile();
      profile->setDimensions(
        network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kMIN,
        nvinfer1::Dims4{batch_config_.at(0), input_channel, input_height, input_width});
      profile->setDimensions(
        network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kOPT,
        nvinfer1::Dims4{batch_config_.at(1), input_channel, input_height, input_width});
      profile->setDimensions(
        network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kMAX,
        nvinfer1::Dims4{batch_config_.at(2), input_channel, input_height, input_width});
      config->addOptimizationProfile(profile);
    }
  }
  if (precision_ == "int8" && calibrator_) {
    config->setFlag(nvinfer1::BuilderFlag::kINT8);
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8200
    config->setFlag(nvinfer1::BuilderFlag::kPREFER_PRECISION_CONSTRAINTS);
#else
    config->setFlag(nvinfer1::BuilderFlag::kSTRICT_TYPES);
#endif
    // QAT requires no calibrator.
    //    assert((calibrator != nullptr) && "Invalid calibrator for INT8 precision");
    config->setInt8Calibrator(calibrator_.get());
  }
  if (build_config_->profile_per_layer) {
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8200
    config->setProfilingVerbosity(nvinfer1::ProfilingVerbosity::kDETAILED);
#else
    config->setProfilingVerbosity(nvinfer1::ProfilingVerbosity::kVERBOSE);
#endif
  }

#if TENSORRT_VERSION_MAJOR >= 8
  auto plan =
    TrtUniquePtr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));
  if (!plan) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create host memory");
    return false;
  }
  engine_ = TrtUniquePtr<nvinfer1::ICudaEngine>(
    runtime_->deserializeCudaEngine(plan->data(), plan->size()));
#else
  engine_ = TrtUniquePtr<nvinfer1::ICudaEngine>(builder->buildEngineWithConfig(*network, *config));
#endif

  if (!engine_) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create engine");
    return false;
  }

  // save engine
#if TENSORRT_VERSION_MAJOR < 8
  auto data = TrtUniquePtr<nvinfer1::IHostMemory>(engine_->serialize());
#endif
  std::ofstream file;
  file.open(output_engine_file_path, std::ios::binary | std::ios::out);
  if (!file.is_open()) {
    return false;
  }
#if TENSORRT_VERSION_MAJOR < 8
  file.write(reinterpret_cast<const char *>(data->data()), data->size());
#else
  file.write(reinterpret_cast<const char *>(plan->data()), plan->size());
#endif

  file.close();

  return true;
}

bool TrtCommon::isInitialized()
{
  return is_initialized_;
}

nvinfer1::Dims TrtCommon::getBindingDimensions(const int32_t index) const
{
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + (NV_TENSOR_PATCH * 10) >= 8500
  auto const & name = engine_->getIOTensorName(index);
  auto dims = context_->getTensorShape(name);
  bool const has_runtime_dim =
    std::any_of(dims.d, dims.d + dims.nbDims, [](int32_t dim) { return dim == -1; });

  if (has_runtime_dim) {
    return dims;
  } else {
    return context_->getBindingDimensions(index);
  }
#else
  return context_->getBindingDimensions(index);
#endif
}

int32_t TrtCommon::getNbBindings()
{
  return engine_->getNbBindings();
}

bool TrtCommon::setBindingDimensions(const int32_t index, const nvinfer1::Dims & dimensions) const
{
  return context_->setBindingDimensions(index, dimensions);
}

bool TrtCommon::enqueueV2(void ** bindings, cudaStream_t stream, cudaEvent_t * input_consumed)
{
  if (build_config_->profile_per_layer) {
    auto inference_start = std::chrono::high_resolution_clock::now();

    bool ret = context_->enqueueV2(bindings, stream, input_consumed);

    auto inference_end = std::chrono::high_resolution_clock::now();
    host_profiler_.reportLayerTime(
      "inference",
      std::chrono::duration<float, std::milli>(inference_end - inference_start).count());
    return ret;
  } else {
    return context_->enqueueV2(bindings, stream, input_consumed);
  }
}

void TrtCommon::printProfiling()
{
  std::cout << host_profiler_;
  std::cout << std::endl;
  std::cout << model_profiler_;
}

#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8200
std::string TrtCommon::getLayerInformation(nvinfer1::LayerInformationFormat format)
{
  auto runtime = std::unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
  auto inspector = std::unique_ptr<nvinfer1::IEngineInspector>(engine_->createEngineInspector());
  if (context_ != nullptr) {
    inspector->setExecutionContext(&(*context_));
  }
  std::string result = inspector->getEngineInformation(format);
  return result;
}
#endif

}  // namespace tensorrt_common
