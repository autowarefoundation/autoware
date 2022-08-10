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

#ifndef PERCEPTION__TRAFFIC_LIGHT_CLASSIFIER__UTILS__TRT_COMMON_HPP_
#define PERCEPTION__TRAFFIC_LIGHT_CLASSIFIER__UTILS__TRT_COMMON_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <./cudnn.h>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <stdio.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>

#define CHECK_CUDA_ERROR(e) (Tn::check_error(e, __FILE__, __LINE__))

namespace Tn
{
class Logger : public nvinfer1::ILogger
{
public:
  Logger() : Logger(Severity::kINFO) {}

  explicit Logger(Severity severity) : reportableSeverity(severity) {}

  void log(Severity severity, const char * msg) noexcept override
  {
    // suppress messages with severity enum value greater than the reportable
    if (severity > reportableSeverity) {
      return;
    }

    switch (severity) {
      case Severity::kINTERNAL_ERROR:
        std::cerr << "[TRT_COMMON][INTERNAL_ERROR]: ";
        break;
      case Severity::kERROR:
        std::cerr << "[TRT_COMMON][ERROR]: ";
        break;
      case Severity::kWARNING:
        std::cerr << "[TRT_COMMON][WARNING]: ";
        break;
      case Severity::kINFO:
        std::cerr << "[TRT_COMMON][INFO]: ";
        break;
      default:
        std::cerr << "[TRT_COMMON][UNKNOWN]: ";
        break;
    }
    std::cerr << msg << std::endl;
  }

  Severity reportableSeverity{Severity::kWARNING};
};

void check_error(const ::cudaError_t e, decltype(__FILE__) f, decltype(__LINE__) n);

struct InferDeleter
{
  void operator()(void * p) const { ::cudaFree(p); }
};

template <typename T>
using UniquePtr = std::unique_ptr<T, InferDeleter>;

// auto array = Tn::make_unique<float[]>(n);
// ::cudaMemcpy(array.get(), src_array, sizeof(float)*n, ::cudaMemcpyHostToDevice);
template <typename T>
typename std::enable_if<std::is_array<T>::value, Tn::UniquePtr<T>>::type make_unique(
  const std::size_t n)
{
  using U = typename std::remove_extent<T>::type;
  U * p;
  ::cudaMalloc(reinterpret_cast<void **>(&p), sizeof(U) * n);
  return Tn::UniquePtr<T>{p};
}

// auto value = Tn::make_unique<my_class>();
// ::cudaMemcpy(value.get(), src_value, sizeof(my_class), ::cudaMemcpyHostToDevice);
template <typename T>
Tn::UniquePtr<T> make_unique()
{
  T * p;
  ::cudaMalloc(reinterpret_cast<void **>(&p), sizeof(T));
  return Tn::UniquePtr<T>{p};
}

class TrtCommon
{
public:
  TrtCommon(std::string model_path, std::string precision);
  ~TrtCommon() {}

  bool loadEngine(std::string engine_file_path);
  bool buildEngineFromOnnx(std::string onnx_file_path, std::string output_engine_file_path);
  void setup();

  bool isInitialized();
  int getNumInput();
  int getNumOutput();
  int getInputBindingIndex();
  int getOutputBindingIndex();

  UniquePtr<nvinfer1::IExecutionContext> context_;

private:
  Logger logger_;
  std::string model_file_path_;
  UniquePtr<nvinfer1::IRuntime> runtime_;
  UniquePtr<nvinfer1::ICudaEngine> engine_;

  nvinfer1::Dims input_dims_;
  nvinfer1::Dims output_dims_;
  std::string cache_dir_;
  std::string precision_;
  std::string input_name_;
  std::string output_name_;
  bool is_initialized_;
};

}  // namespace Tn

#endif  // PERCEPTION__TRAFFIC_LIGHT_CLASSIFIER__UTILS__TRT_COMMON_HPP_
