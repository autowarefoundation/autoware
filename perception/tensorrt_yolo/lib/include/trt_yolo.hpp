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

/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef TRT_YOLO_HPP_
#define TRT_YOLO_HPP_

#include <cuda_utils.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <yolo_layer.hpp>

#include <NvInfer.h>
#include <cuda_runtime.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace yolo
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

struct Config
{
  int num_anchors;
  std::vector<float> anchors;
  std::vector<float> scale_x_y;
  float score_thresh;
  float iou_thresh;
  int detections_per_im;
  bool use_darknet_layer;
  float ignore_thresh;
};

class Net
{
public:
  // Create engine from engine path
  explicit Net(const std::string & engine_path, bool verbose = false);

  // Create engine from serialized onnx model
  Net(
    const std::string & onnx_file_path, const std::string & precision, const int max_batch_size,
    const Config & yolo_config, const std::vector<std::string> & calibration_images,
    const std::string & calibration_table, bool verbose = false,
    size_t workspace_size = (1ULL << 30));

  ~Net();

  // Save model to path
  void save(const std::string & path) const;

  bool detect(const cv::Mat & in_img, float * out_scores, float * out_boxes, float * out_classes);

  // Get (c, h, w) size of the fixed input
  std::vector<int> getInputDims() const;

  std::vector<int> getOutputScoreSize() const;

  // Get max allowed batch size
  int getMaxBatchSize() const;

  // Get max number of detections
  int getMaxDetections() const;

  int getInputSize() const;

private:
  unique_ptr<nvinfer1::IRuntime> runtime_ = nullptr;
  unique_ptr<nvinfer1::IHostMemory> plan_ = nullptr;
  unique_ptr<nvinfer1::ICudaEngine> engine_ = nullptr;
  unique_ptr<nvinfer1::IExecutionContext> context_ = nullptr;
  cudaStream_t stream_ = nullptr;
  cuda::unique_ptr<float[]> input_d_ = nullptr;
  cuda::unique_ptr<float[]> out_scores_d_ = nullptr;
  cuda::unique_ptr<float[]> out_boxes_d_ = nullptr;
  cuda::unique_ptr<float[]> out_classes_d_ = nullptr;

  void load(const std::string & path);
  bool prepare();
  std::vector<float> preprocess(
    const cv::Mat & in_img, const int c, const int h, const int w) const;
  // Infer using pre-allocated GPU buffers {data, scores, boxes}
  void infer(std::vector<void *> & buffers, const int batch_size);
};

bool set_cuda_device(int gpu_id)
{
  cudaError_t status = cudaSetDevice(gpu_id);
  if (status != cudaSuccess) {
    return false;
  } else {
    return true;
  }
}

}  // namespace yolo

#endif  // TRT_YOLO_HPP_
