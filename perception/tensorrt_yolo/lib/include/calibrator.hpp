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

#ifndef CALIBRATOR_HPP_
#define CALIBRATOR_HPP_

#include <cuda_utils.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <NvInfer.h>
#include <assert.h>

#include <algorithm>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace yolo
{
class ImageStream
{
public:
  ImageStream(
    int batch_size, nvinfer1::Dims input_dims, const std::vector<std::string> calibration_images)
  : batch_size_(batch_size),
    calibration_images_(calibration_images),
    current_batch_(0),
    max_batches_(calibration_images.size() / batch_size_),
    input_dims_(input_dims)
  {
    batch_.resize(batch_size_ * input_dims_.d[1] * input_dims_.d[2] * input_dims_.d[3]);
  }

  int getBatchSize() const { return batch_size_; }

  int getMaxBatches() const { return max_batches_; }

  float * getBatch() { return &batch_[0]; }

  nvinfer1::Dims getInputDims() { return input_dims_; }

  std::vector<float> preprocess(const cv::Mat & in_img, const int c, const int w, const int h) const
  {
    cv::Mat rgb;
    cv::cvtColor(in_img, rgb, cv::COLOR_BGR2RGB);

    cv::resize(rgb, rgb, cv::Size(w, h));

    cv::Mat img_float;
    rgb.convertTo(img_float, CV_32FC3, 1 / 255.0);

    // HWC TO CHW
    std::vector<cv::Mat> input_channels(c);
    cv::split(img_float, input_channels);

    std::vector<float> result(h * w * c);
    auto data = result.data();
    int channel_length = h * w;
    for (int i = 0; i < c; ++i) {
      memcpy(data, input_channels[i].data, channel_length * sizeof(float));
      data += channel_length;
    }

    return result;
  }

  bool next()
  {
    if (current_batch_ == max_batches_) {
      return false;
    }

    for (int i = 0; i < batch_size_; ++i) {
      auto image =
        cv::imread(calibration_images_[batch_size_ * current_batch_ + i].c_str(), cv::IMREAD_COLOR);
      auto input = preprocess(image, input_dims_.d[1], input_dims_.d[3], input_dims_.d[2]);
      batch_.insert(
        batch_.begin() + i * input_dims_.d[1] * input_dims_.d[2] * input_dims_.d[3], input.begin(),
        input.end());
    }

    ++current_batch_;
    return true;
  }

  void reset() { current_batch_ = 0; }

private:
  int batch_size_;
  std::vector<std::string> calibration_images_;
  int current_batch_;
  int max_batches_;
  nvinfer1::Dims input_dims_;

  std::vector<float> batch_;
};

class Int8EntropyCalibrator : public nvinfer1::IInt8EntropyCalibrator2
{
public:
  Int8EntropyCalibrator(
    ImageStream & stream, const std::string calibration_cache_file, bool read_cache = true)
  : stream_(stream), calibration_cache_file_(calibration_cache_file), read_cache_(read_cache)
  {
    auto d = stream_.getInputDims();
    input_count_ = stream_.getBatchSize() * d.d[1] * d.d[2] * d.d[3];
    CHECK_CUDA_ERROR(cudaMalloc(&device_input_, input_count_ * sizeof(float)));
  }

  int getBatchSize() const noexcept override { return stream_.getBatchSize(); }

  virtual ~Int8EntropyCalibrator() { CHECK_CUDA_ERROR(cudaFree(device_input_)); }

  bool getBatch(void * bindings[], const char * names[], int nb_bindings) noexcept override
  {
    (void)names;
    (void)nb_bindings;

    if (!stream_.next()) {
      return false;
    }

    try {
      CHECK_CUDA_ERROR(cudaMemcpy(
        device_input_, stream_.getBatch(), input_count_ * sizeof(float), cudaMemcpyHostToDevice));
    } catch (const std::exception & e) {
      // Do nothing
    }
    bindings[0] = device_input_;
    return true;
  }

  const void * readCalibrationCache(size_t & length) noexcept override
  {
    calib_cache_.clear();
    std::ifstream input(calibration_cache_file_, std::ios::binary);
    input >> std::noskipws;
    if (read_cache_ && input.good()) {
      std::copy(
        std::istream_iterator<char>(input), std::istream_iterator<char>(),
        std::back_inserter(calib_cache_));
    }

    length = calib_cache_.size();
    return length ? &calib_cache_[0] : nullptr;
  }

  void writeCalibrationCache(const void * cache, size_t length) noexcept override
  {
    std::ofstream output(calibration_cache_file_, std::ios::binary);
    output.write(reinterpret_cast<const char *>(cache), length);
  }

private:
  ImageStream stream_;
  const std::string calibration_cache_file_;
  bool read_cache_{true};
  size_t input_count_;
  void * device_input_{nullptr};
  std::vector<char> calib_cache_;
};
}  // namespace yolo

#endif  // CALIBRATOR_HPP_
