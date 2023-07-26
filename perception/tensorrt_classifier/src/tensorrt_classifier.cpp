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

#include <tensorrt_classifier/calibrator.hpp>
#include <tensorrt_classifier/tensorrt_classifier.hpp>

#include <omp.h>
#include <tensorrt_classifier/preprocess.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

static void trimLeft(std::string & s)
{
  s.erase(s.begin(), find_if(s.begin(), s.end(), [](int ch) { return !isspace(ch); }));
}

static void trimRight(std::string & s)
{
  s.erase(find_if(s.rbegin(), s.rend(), [](int ch) { return !isspace(ch); }).base(), s.end());
}

std::string trim(std::string & s)
{
  trimLeft(s);
  trimRight(s);
  return s;
}

bool fileExists(const std::string & file_name, bool verbose)
{
  if (!std::experimental::filesystem::exists(std::experimental::filesystem::path(file_name))) {
    if (verbose) {
      std::cout << "File does not exist : " << file_name << std::endl;
    }
    return false;
  }
  return true;
}

std::vector<std::string> loadListFromTextFile(const std::string filename)
{
  assert(fileExists(filename, true));
  std::vector<std::string> list;

  std::ifstream f(filename);
  if (!f) {
    std::cout << "failed to open " << filename;
    assert(0);
  }

  std::string line;
  while (std::getline(f, line)) {
    if (line.empty())
      continue;

    else
      list.push_back(trim(line));
  }

  return list;
}

std::vector<std::string> loadImageList(const std::string & filename, const std::string & prefix)
{
  std::vector<std::string> fileList = loadListFromTextFile(filename);
  for (auto & file : fileList) {
    if (fileExists(file, false)) {
      continue;
    } else {
      std::string prefixed = prefix + file;
      if (fileExists(prefixed, false))
        file = prefixed;
      else
        std::cerr << "WARNING: couldn't find: " << prefixed << " while loading: " << filename
                  << std::endl;
    }
  }
  return fileList;
}

namespace tensorrt_classifier
{
TrtClassifier::TrtClassifier(
  const std::string & model_path, const std::string & precision,
  const tensorrt_common::BatchConfig & batch_config, const std::vector<float> & mean,
  const std::vector<float> & std, const size_t max_workspace_size,
  const std::string & calibration_image_list_path, tensorrt_common::BuildConfig build_config,
  const bool cuda)
{
  src_width_ = -1;
  src_height_ = -1;
  mean_ = mean;
  std_ = std;
  inv_std_ = std;
  for (size_t i = 0; i < inv_std_.size(); i++) {
    inv_std_[i] = 1.0 / inv_std_[i];
  }
  batch_size_ = batch_config[2];
  if (precision == "int8") {
    int max_batch_size = batch_config[2];
    nvinfer1::Dims input_dims = tensorrt_common::get_input_dims(model_path);
    std::vector<std::string> calibration_images;
    if (calibration_image_list_path != "") {
      calibration_images = loadImageList(calibration_image_list_path, "");
    }
    tensorrt_classifier::ImageStream stream(max_batch_size, input_dims, calibration_images);
    fs::path calibration_table{model_path};
    std::string calibName = "";
    std::string ext = "";
    if (build_config.calib_type_str == "Entropy") {
      ext = "EntropyV2-";
    } else if (
      build_config.calib_type_str == "Legacy" || build_config.calib_type_str == "Percentile") {
      ext = "Legacy-";
    } else {
      ext = "MinMax-";
    }
    ext += "calibration.table";
    calibration_table.replace_extension(ext);
    fs::path histogram_table{model_path};
    ext = "histogram.table";
    histogram_table.replace_extension(ext);

    std::unique_ptr<nvinfer1::IInt8Calibrator> calibrator;
    if (build_config.calib_type_str == "Entropy") {
      calibrator.reset(
        new tensorrt_classifier::Int8EntropyCalibrator(stream, calibration_table, mean_, std_));
    } else if (
      build_config.calib_type_str == "Legacy" || build_config.calib_type_str == "Percentile") {
      double quantile = 0.999999;
      double cutoff = 0.999999;
      calibrator.reset(new tensorrt_classifier::Int8LegacyCalibrator(
        stream, calibration_table, histogram_table, mean_, std_, true, quantile, cutoff));
    } else {
      calibrator.reset(
        new tensorrt_classifier::Int8MinMaxCalibrator(stream, calibration_table, mean_, std_));
    }
    trt_common_ = std::make_unique<tensorrt_common::TrtCommon>(
      model_path, precision, std::move(calibrator), batch_config, max_workspace_size, build_config);
  } else {
    trt_common_ = std::make_unique<tensorrt_common::TrtCommon>(
      model_path, precision, nullptr, batch_config, max_workspace_size, build_config);
  }
  trt_common_->setup();

  if (!trt_common_->isInitialized()) {
    return;
  }

  // GPU memory allocation
  const auto input_dims = trt_common_->getBindingDimensions(0);
  const auto input_size =
    std::accumulate(input_dims.d + 1, input_dims.d + input_dims.nbDims, 1, std::multiplies<int>());

  const auto output_dims = trt_common_->getBindingDimensions(1);
  input_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * input_size);
  out_elem_num_ = std::accumulate(
    output_dims.d + 1, output_dims.d + output_dims.nbDims, 1, std::multiplies<int>());
  out_elem_num_ = out_elem_num_ * batch_config[2];
  out_elem_num_per_batch_ = static_cast<int>(out_elem_num_ / batch_config[2]);
  out_prob_d_ = cuda_utils::make_unique<float[]>(out_elem_num_);
  out_prob_h_ = cuda_utils::make_unique_host<float[]>(out_elem_num_, cudaHostAllocPortable);

  if (cuda) {
    m_cuda = true;
    h_img_ = NULL;
    d_img_ = NULL;

  } else {
    m_cuda = false;
  }
}

TrtClassifier::~TrtClassifier()
{
  if (m_cuda) {
    if (h_img_) CHECK_CUDA_ERROR(cudaFreeHost(h_img_));
    if (d_img_) CHECK_CUDA_ERROR(cudaFree(d_img_));
  }
}

void TrtClassifier::initPreprocessBuffer(int width, int height)
{
  // if size of source input has been changed...
  if (src_width_ != -1 || src_height_ != -1) {
    if (width != src_width_ || height != src_height_) {
      // Free cuda memory to reallocate
      if (h_img_) {
        CHECK_CUDA_ERROR(cudaFreeHost(h_img_));
        h_img_ = NULL;
      }
      if (d_img_) {
        CHECK_CUDA_ERROR(cudaFree(d_img_));
        d_img_ = NULL;
      }
    }
  }
  src_width_ = width;
  src_height_ = height;
  if (m_cuda) {
    auto input_dims = trt_common_->getBindingDimensions(0);
    bool const hasRuntimeDim = std::any_of(
      input_dims.d, input_dims.d + input_dims.nbDims,
      [](int32_t input_dim) { return input_dim == -1; });
    if (hasRuntimeDim) {
      input_dims.d[0] = batch_size_;
    }
    if (!h_img_) {
      trt_common_->setBindingDimensions(0, input_dims);
    }
    if (!h_img_) {
      CHECK_CUDA_ERROR(cudaMallocHost(
        reinterpret_cast<void **>(&h_img_),
        sizeof(unsigned char) * width * height * 3 * batch_size_));
      CHECK_CUDA_ERROR(cudaMalloc(
        reinterpret_cast<void **>(&d_img_),
        sizeof(unsigned char) * width * height * 3 * batch_size_));
    }
  }
}

void TrtClassifier::preprocessGpu(const std::vector<cv::Mat> & images)
{
  const auto batch_size = images.size();
  auto input_dims = trt_common_->getBindingDimensions(0);

  input_dims.d[0] = batch_size;
  for (const auto & image : images) {
    // if size of source input has been changed...
    int width = image.cols;
    int height = image.rows;
    if (src_width_ != -1 || src_height_ != -1) {
      if (width != src_width_ || height != src_height_) {
        // Free cuda memory to reallocate
        if (h_img_) {
          CHECK_CUDA_ERROR(cudaFreeHost(h_img_));
          h_img_ = NULL;
        }
        if (d_img_) {
          CHECK_CUDA_ERROR(cudaFree(d_img_));
          d_img_ = NULL;
        }
      }
    }
    src_width_ = width;
    src_height_ = height;
  }
  if (!h_img_) {
    trt_common_->setBindingDimensions(0, input_dims);
  }
  const float input_height = static_cast<float>(input_dims.d[2]);
  const float input_width = static_cast<float>(input_dims.d[3]);
  int b = 0;
  for (const auto & image : images) {
    if (!h_img_) {
      CHECK_CUDA_ERROR(cudaMallocHost(
        reinterpret_cast<void **>(&h_img_),
        sizeof(unsigned char) * image.cols * image.rows * 3 * batch_size));
      CHECK_CUDA_ERROR(cudaMalloc(
        reinterpret_cast<void **>(&d_img_),
        sizeof(unsigned char) * image.cols * image.rows * 3 * batch_size));
    }
    int index = b * image.cols * image.rows * 3;
    // Copy into pinned memory
    memcpy(&(h_img_[index]), &image.data[0], image.cols * image.rows * 3 * sizeof(unsigned char));
    b++;
  }
  // Copy into device memory
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_img_, h_img_, images[0].cols * images[0].rows * 3 * batch_size * sizeof(unsigned char),
    cudaMemcpyHostToDevice, *stream_));
  resize_bilinear_letterbox_nhwc_to_nchw32_batch_gpu(
    input_d_.get(), d_img_, input_width, input_height, 3, images[0].cols, images[0].rows, 3,
    batch_size, static_cast<float>(1.0), *stream_);
  // No Need for Sync and used for timer
  // CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
}

void TrtClassifier::preprocess_opt(const std::vector<cv::Mat> & images)
{
  int batch_size = static_cast<int>(images.size());
  auto input_dims = trt_common_->getBindingDimensions(0);
  input_dims.d[0] = batch_size;
  trt_common_->setBindingDimensions(0, input_dims);
  const float input_chan = static_cast<float>(input_dims.d[1]);
  const float input_height = static_cast<float>(input_dims.d[2]);
  const float input_width = static_cast<float>(input_dims.d[3]);
  std::vector<cv::Mat> dst_images;
  int volume = batch_size * input_chan * input_height * input_width;
  input_h_.resize(volume);
  dst_images.resize(images.size());
  // NHWC
  const size_t strides_cv[4] = {
    static_cast<size_t>(input_width * input_chan * input_height),
    static_cast<size_t>(input_width * input_chan), static_cast<size_t>(input_chan), 1};
  // NCHW
  const size_t strides[4] = {
    static_cast<size_t>(input_height * input_width * input_chan),
    static_cast<size_t>(input_height * input_width), static_cast<size_t>(input_width), 1};
#pragma omp parallel for
  for (int n = 0; n < batch_size; n++) {
    const float scale = std::min(input_width / images[n].cols, input_height / images[n].rows);
    const auto scale_size = cv::Size(images[n].cols * scale, images[n].rows * scale);
    cv::resize(images[n], dst_images[n], scale_size, 0, 0, cv::INTER_LINEAR);
    const auto bottom = input_height - dst_images[n].rows;
    const auto right = input_width - dst_images[n].cols;
    copyMakeBorder(
      dst_images[n], dst_images[n], 0, bottom, 0, right, cv::BORDER_CONSTANT, {0, 0, 0});
    for (int h = 0; h < input_height; h++) {
      for (int w = 0; w < input_width; w++) {
        for (int c = 0; c < input_chan; c++) {
          // NHWC (needs RBswap)
          const size_t offset_cv = h * strides_cv[1] + w * strides_cv[2] + c * strides_cv[3];
          // NCHW
          const size_t offset = n * strides[0] + (c)*strides[1] + h * strides[2] + w * strides[3];
          input_h_[offset] =
            (static_cast<float>(dst_images[n].data[offset_cv]) - mean_[c]) * inv_std_[c];
        }
      }
    }
  }
  CHECK_CUDA_ERROR(cudaMemcpy(
    input_d_.get(), input_h_.data(), input_h_.size() * sizeof(float), cudaMemcpyHostToDevice));
  // No Need for Sync
}

bool TrtClassifier::doInference(
  const std::vector<cv::Mat> & images, std::vector<int> & results,
  std::vector<float> & probabilities)
{
  if (!trt_common_->isInitialized()) {
    return false;
  }
  preprocess_opt(images);

  return feedforwardAndDecode(images, results, probabilities);
}

bool TrtClassifier::feedforwardAndDecode(
  const std::vector<cv::Mat> & images, std::vector<int> & results,
  std::vector<float> & probabilities)
{
  results.clear();
  probabilities.clear();
  std::vector<void *> buffers = {input_d_.get(), out_prob_d_.get()};
  trt_common_->enqueueV2(buffers.data(), *stream_, nullptr);

  int batch_size = static_cast<int>(images.size());

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_prob_h_.get(), out_prob_d_.get(), sizeof(float) * out_elem_num_, cudaMemcpyDeviceToHost,
    *stream_));
  cudaStreamSynchronize(*stream_);

  for (int i = 0; i < batch_size; ++i) {
    float max = 0.0;
    int index = 0;
    float * output = out_prob_h_.get();
    for (size_t j = 0; j < out_elem_num_per_batch_; j++) {
      if (max < output[j + i * out_elem_num_per_batch_]) {
        max = output[j + i * out_elem_num_per_batch_];
        index = j;
      }
    }
    probabilities.push_back(max);
    results.push_back(index);
  }
  return true;
}
}  // namespace tensorrt_classifier
