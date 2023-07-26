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

#ifndef TENSORRT_YOLOX__CALIBRATOR_HPP_
#define TENSORRT_YOLOX__CALIBRATOR_HPP_

#include "cuda_utils/cuda_check_error.hpp"
#include "cuda_utils/cuda_unique_ptr.hpp"

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

namespace tensorrt_yolox
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

  /**
   * @brief Preprocess in calibration
   * @param[in] images input images
   * @param[in] input_dims input dimensions
   * @param[in] norm normalization (0.0-1.0)
   * @return vector<float> preprocessed data
   */
  std::vector<float> preprocess(
    const std::vector<cv::Mat> & images, nvinfer1::Dims input_dims, double norm)
  {
    std::vector<float> input_h_;
    const auto batch_size = images.size();
    input_dims.d[0] = batch_size;
    const float input_height = static_cast<float>(input_dims.d[2]);
    const float input_width = static_cast<float>(input_dims.d[3]);
    std::vector<cv::Mat> dst_images;
    std::vector<float> scales_;
    scales_.clear();
    for (const auto & image : images) {
      cv::Mat dst_image;
      const float scale = std::min(input_width / image.cols, input_height / image.rows);
      scales_.emplace_back(scale);
      const auto scale_size = cv::Size(image.cols * scale, image.rows * scale);
      cv::resize(image, dst_image, scale_size, 0, 0, cv::INTER_CUBIC);
      const auto bottom = input_height - dst_image.rows;
      const auto right = input_width - dst_image.cols;
      copyMakeBorder(
        dst_image, dst_image, 0, bottom, 0, right, cv::BORDER_CONSTANT, {114, 114, 114});
      dst_images.emplace_back(dst_image);
    }
    const auto chw_images =
      cv::dnn::blobFromImages(dst_images, norm, cv::Size(), cv::Scalar(), false, false, CV_32F);

    const auto data_length = chw_images.total();
    input_h_.reserve(data_length);
    const auto flat = chw_images.reshape(1, data_length);
    input_h_ = chw_images.isContinuous() ? flat : flat.clone();
    return input_h_;
  }

  /**
   * @brief Decode data in calibration
   * @param[in] scale normalization (0.0-1.0)
   * @return bool succ or fail
   */
  bool next(double scale)
  {
    if (current_batch_ == max_batches_) {
      return false;
    }

    for (int i = 0; i < batch_size_; ++i) {
      auto image =
        cv::imread(calibration_images_[batch_size_ * current_batch_ + i].c_str(), cv::IMREAD_COLOR);
      std::cout << current_batch_ << " " << i << " Preprocess "
                << calibration_images_[batch_size_ * current_batch_ + i].c_str() << std::endl;
      auto input = preprocess({image}, input_dims_, scale);
      batch_.insert(
        batch_.begin() + i * input_dims_.d[1] * input_dims_.d[2] * input_dims_.d[3], input.begin(),
        input.end());
    }

    ++current_batch_;
    return true;
  }

  /**
   * @brief Reset calibration
   */
  void reset() { current_batch_ = 0; }

private:
  int batch_size_;
  std::vector<std::string> calibration_images_;
  int current_batch_;
  int max_batches_;
  nvinfer1::Dims input_dims_;
  std::vector<float> batch_;
};

/**Percentile calibration using legacy calibrator*/
/**
 * @class Int8LegacyCalibrator
 * @brief Calibrator for Percentile
 * @warning We are confirming bug on Tegra like Xavier and Orin. We recommend use MinMax calibrator
 */
class Int8LegacyCalibrator : public nvinfer1::IInt8LegacyCalibrator
{
public:
  Int8LegacyCalibrator(
    ImageStream & stream, const std::string calibration_cache_file,
    const std::string histogram_cache_file, double scale = 1.0, bool read_cache = true,
    double quantile = 0.999999, double cutoff = 0.999999)
  : stream_(stream),
    calibration_cache_file_(calibration_cache_file),
    histogram_cache_file_(histogram_cache_file),
    read_cache_(read_cache)
  {
    auto d = stream_.getInputDims();
    input_count_ = stream_.getBatchSize() * d.d[1] * d.d[2] * d.d[3];
    CHECK_CUDA_ERROR(cudaMalloc(&device_input_, input_count_ * sizeof(float)));
    scale_ = scale;
    quantile_ = quantile;
    cutoff_ = cutoff;
    auto algType = getAlgorithm();
    switch (algType) {
      case (nvinfer1::CalibrationAlgoType::kLEGACY_CALIBRATION):
        std::cout << "CalibrationAlgoType : kLEGACY_CALIBRATION" << std::endl;
        break;
      case (nvinfer1::CalibrationAlgoType::kENTROPY_CALIBRATION):
        std::cout << "CalibrationAlgoType : kENTROPY_CALIBRATION" << std::endl;
        break;
      case (nvinfer1::CalibrationAlgoType::kENTROPY_CALIBRATION_2):
        std::cout << "CalibrationAlgoType : kENTROPY_CALIBRATION_2" << std::endl;
        break;
      case (nvinfer1::CalibrationAlgoType::kMINMAX_CALIBRATION):
        std::cout << "CalibrationAlgoType : kMINMAX_CALIBRATION" << std::endl;
        break;
      default:
        std::cout << "No CalibrationAlgType" << std::endl;
        break;
    }
  }
  int getBatchSize() const noexcept override { return stream_.getBatchSize(); }

  virtual ~Int8LegacyCalibrator() { CHECK_CUDA_ERROR(cudaFree(device_input_)); }

  bool getBatch(void * bindings[], const char * names[], int nb_bindings) noexcept override
  {
    (void)names;
    (void)nb_bindings;

    if (!stream_.next(scale_)) {
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
    if (length) {
      std::cout << "Using cached calibration table to build the engine" << std::endl;
    } else {
      std::cout << "New calibration table will be created to build the engine" << std::endl;
    }
    return length ? &calib_cache_[0] : nullptr;
  }

  void writeCalibrationCache(const void * cache, size_t length) noexcept override
  {
    std::ofstream output(calibration_cache_file_, std::ios::binary);
    output.write(reinterpret_cast<const char *>(cache), length);
  }

  double getQuantile() const noexcept
  {
    printf("Quantile %f\n", quantile_);
    return quantile_;
  }

  double getRegressionCutoff(void) const noexcept
  {
    printf("Cutoff %f\n", cutoff_);
    return cutoff_;
  }

  const void * readHistogramCache(std::size_t & length) noexcept
  {
    hist_cache_.clear();
    std::ifstream input(histogram_cache_file_, std::ios::binary);
    input >> std::noskipws;
    if (read_cache_ && input.good()) {
      std::copy(
        std::istream_iterator<char>(input), std::istream_iterator<char>(),
        std::back_inserter(hist_cache_));
    }

    length = hist_cache_.size();
    if (length) {
      std::cout << "Using cached histogram table to build the engine" << std::endl;
    } else {
      std::cout << "New histogram table will be created to build the engine" << std::endl;
    }
    return length ? &hist_cache_[0] : nullptr;
  }
  void writeHistogramCache(void const * ptr, std::size_t length) noexcept
  {
    std::ofstream output(histogram_cache_file_, std::ios::binary);
    output.write(reinterpret_cast<const char *>(ptr), length);
  }

private:
  ImageStream stream_;
  const std::string calibration_cache_file_;
  const std::string histogram_cache_file_;
  bool read_cache_{true};
  size_t input_count_;
  void * device_input_{nullptr};
  std::vector<char> calib_cache_;
  std::vector<char> hist_cache_;
  double scale_;
  double quantile_;
  double cutoff_;
};

/**
 * @class Int8LegacyCalibrator
 * @brief Calibrator for Percentile
 * @warning This calibrator causes crucial accuracy drop for YOLOX.
 */
class Int8EntropyCalibrator : public nvinfer1::IInt8EntropyCalibrator2
{
public:
  Int8EntropyCalibrator(
    ImageStream & stream, const std::string calibration_cache_file, double scale = 1.0,
    bool read_cache = true)
  : stream_(stream), calibration_cache_file_(calibration_cache_file), read_cache_(read_cache)
  {
    auto d = stream_.getInputDims();
    input_count_ = stream_.getBatchSize() * d.d[1] * d.d[2] * d.d[3];
    CHECK_CUDA_ERROR(cudaMalloc(&device_input_, input_count_ * sizeof(float)));
    scale_ = scale;
    auto algType = getAlgorithm();
    switch (algType) {
      case (nvinfer1::CalibrationAlgoType::kLEGACY_CALIBRATION):
        std::cout << "CalibrationAlgoType : kLEGACY_CALIBRATION" << std::endl;
        break;
      case (nvinfer1::CalibrationAlgoType::kENTROPY_CALIBRATION):
        std::cout << "CalibrationAlgoType : kENTROPY_CALIBRATION" << std::endl;
        break;
      case (nvinfer1::CalibrationAlgoType::kENTROPY_CALIBRATION_2):
        std::cout << "CalibrationAlgoType : kENTROPY_CALIBRATION_2" << std::endl;
        break;
      case (nvinfer1::CalibrationAlgoType::kMINMAX_CALIBRATION):
        std::cout << "CalibrationAlgoType : kMINMAX_CALIBRATION" << std::endl;
        break;
      default:
        std::cout << "No CalibrationAlgType" << std::endl;
        break;
    }
  }
  int getBatchSize() const noexcept override { return stream_.getBatchSize(); }

  virtual ~Int8EntropyCalibrator() { CHECK_CUDA_ERROR(cudaFree(device_input_)); }

  bool getBatch(void * bindings[], const char * names[], int nb_bindings) noexcept override
  {
    (void)names;
    (void)nb_bindings;

    if (!stream_.next(scale_)) {
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
    if (length) {
      std::cout << "Using cached calibration table to build the engine" << std::endl;
    } else {
      std::cout << "New calibration table will be created to build the engine" << std::endl;
    }
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
  std::vector<char> hist_cache_;
  double scale_;
};

/**
 * @class Int8MinMaxCalibrator
 * @brief Calibrator for MinMax
 * @warning We strongly recommend MinMax calibrator for YOLOX
 */
class Int8MinMaxCalibrator : public nvinfer1::IInt8MinMaxCalibrator
{
public:
  Int8MinMaxCalibrator(
    ImageStream & stream, const std::string calibration_cache_file, double scale = 1.0,
    bool read_cache = true)
  : stream_(stream), calibration_cache_file_(calibration_cache_file), read_cache_(read_cache)
  {
    auto d = stream_.getInputDims();
    input_count_ = stream_.getBatchSize() * d.d[1] * d.d[2] * d.d[3];
    CHECK_CUDA_ERROR(cudaMalloc(&device_input_, input_count_ * sizeof(float)));
    scale_ = scale;
    auto algType = getAlgorithm();
    switch (algType) {
      case (nvinfer1::CalibrationAlgoType::kLEGACY_CALIBRATION):
        std::cout << "CalibrationAlgoType : kLEGACY_CALIBRATION" << std::endl;
        break;
      case (nvinfer1::CalibrationAlgoType::kENTROPY_CALIBRATION):
        std::cout << "CalibrationAlgoType : kENTROPY_CALIBRATION" << std::endl;
        break;
      case (nvinfer1::CalibrationAlgoType::kENTROPY_CALIBRATION_2):
        std::cout << "CalibrationAlgoType : kENTROPY_CALIBRATION_2" << std::endl;
        break;
      case (nvinfer1::CalibrationAlgoType::kMINMAX_CALIBRATION):
        std::cout << "CalibrationAlgoType : kMINMAX_CALIBRATION" << std::endl;
        break;
      default:
        std::cout << "No CalibrationAlgType" << std::endl;
        break;
    }
  }
  int getBatchSize() const noexcept override { return stream_.getBatchSize(); }

  virtual ~Int8MinMaxCalibrator() { CHECK_CUDA_ERROR(cudaFree(device_input_)); }

  bool getBatch(void * bindings[], const char * names[], int nb_bindings) noexcept override
  {
    (void)names;
    (void)nb_bindings;

    if (!stream_.next(scale_)) {
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
    if (length) {
      std::cout << "Using cached calibration table to build the engine" << std::endl;
    } else {
      std::cout << "New calibration table will be created to build the engine" << std::endl;
    }
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
  std::vector<char> hist_cache_;
  double scale_;
};
}  // namespace tensorrt_yolox

#endif  // TENSORRT_YOLOX__CALIBRATOR_HPP_
