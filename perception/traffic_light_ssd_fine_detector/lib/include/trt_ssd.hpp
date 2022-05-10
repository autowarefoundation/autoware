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

#ifndef TRT_SSD_HPP_
#define TRT_SSD_HPP_

#include <./cuda_runtime.h>
#include <NvInfer.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace ssd
{
struct Deleter
{
  template <typename T>
  void operator()(T * obj) const
  {
    if (obj) {
      delete obj;
    }
  }
};

template <typename T>
using unique_ptr = std::unique_ptr<T, Deleter>;

class Logger : public nvinfer1::ILogger
{
public:
  explicit Logger(bool verbose) : verbose_(verbose) {}

  void log(Severity severity, const char * msg) noexcept override
  {
    if (verbose_ || ((severity != Severity::kINFO) && (severity != Severity::kVERBOSE))) {
      std::cout << msg << std::endl;
    }
  }

private:
  bool verbose_{false};
};

class Net
{
public:
  // Create engine from engine path
  explicit Net(const std::string & engine_path, bool verbose = false);

  // Create engine from serialized onnx model
  Net(
    const std::string & onnx_file_path, const std::string & precision, const int max_batch_size,
    bool verbose = false, size_t workspace_size = (1ULL << 30));

  ~Net();

  // Save model to path
  void save(const std::string & path);

  // Infer using pre-allocated GPU buffers {data, scores, boxes}
  void infer(std::vector<void *> & buffers, const int batch_size);

  // Get (c, h, w) size of the fixed input
  std::vector<int> getInputSize();

  std::vector<int> getOutputScoreSize();

  // Get max allowed batch size
  int getMaxBatchSize();

  // Get max number of detections
  int getMaxDetections();

private:
  unique_ptr<nvinfer1::IRuntime> runtime_ = nullptr;
  unique_ptr<nvinfer1::IHostMemory> plan_ = nullptr;
  unique_ptr<nvinfer1::ICudaEngine> engine_ = nullptr;
  unique_ptr<nvinfer1::IExecutionContext> context_ = nullptr;
  cudaStream_t stream_ = nullptr;

  void load(const std::string & path);
  void prepare();
};

}  // namespace ssd

#endif  // TRT_SSD_HPP_
