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

#include <calibrator.hpp>
#include <cuda_utils.hpp>
#include <mish_plugin.hpp>
#include <nms_plugin.hpp>
#include <trt_yolo.hpp>
#include <yolo_layer_plugin.hpp>

#include <NvOnnxParser.h>

#include <algorithm>
#include <fstream>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace yolo
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

bool Net::prepare()
{
  if (!engine_) {
    return false;
  }
  context_ = unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  if (!context_) {
    return false;
  }
  input_d_ = cuda::make_unique<float[]>(getMaxBatchSize() * getInputSize());
  out_scores_d_ = cuda::make_unique<float[]>(getMaxBatchSize() * getMaxDetections());
  out_boxes_d_ = cuda::make_unique<float[]>(getMaxBatchSize() * getMaxDetections() * 4);
  out_classes_d_ = cuda::make_unique<float[]>(getMaxBatchSize() * getMaxDetections());
  cudaStreamCreate(&stream_);
  return true;
}

std::vector<float> Net::preprocess(
  const cv::Mat & in_img, const int c, const int w, const int h) const
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

Net::Net(const std::string & path, bool verbose)
{
  Logger logger(verbose);
  runtime_ = unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger));
  load(path);
  if (!prepare()) {
    std::cout << "Fail to prepare engine" << std::endl;
    return;
  }
}

Net::~Net()
{
  if (stream_) {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
  }
}

Net::Net(
  const std::string & onnx_file_path, const std::string & precision, const int max_batch_size,
  const Config & yolo_config, const std::vector<std::string> & calibration_images,
  const std::string & calibration_table, bool verbose, size_t workspace_size)
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
  std::vector<nvinfer1::ITensor *> scores, boxes, classes;
  const auto input = network->getInput(0);
  const auto num_outputs = network->getNbOutputs();
  const auto input_dims = input->getDimensions();
  const auto input_channel = input_dims.d[1];
  const auto input_height = input_dims.d[2];
  const auto input_width = input_dims.d[3];
  for (int i = 0; i < num_outputs; ++i) {
    auto output = network->getOutput(i);
    std::vector<float> anchor(
      yolo_config.anchors.begin() + i * yolo_config.num_anchors * 2,
      yolo_config.anchors.begin() + (i + 1) * yolo_config.num_anchors * 2);
    auto yoloLayerPlugin = yolo::YoloLayerPlugin(
      input_width, input_height, 3, anchor, yolo_config.scale_x_y[i], yolo_config.score_thresh,
      yolo_config.use_darknet_layer);
    std::vector<nvinfer1::ITensor *> inputs = {output};
    auto layer = network->addPluginV2(inputs.data(), inputs.size(), yoloLayerPlugin);
    scores.push_back(layer->getOutput(0));
    boxes.push_back(layer->getOutput(1));
    classes.push_back(layer->getOutput(2));
  }

  // Cleanup outputs
  for (int i = 0; i < num_outputs; i++) {
    auto output = network->getOutput(0);
    network->unmarkOutput(*output);
  }

  // Concat tensors from each feature map
  std::vector<nvinfer1::ITensor *> concat;
  for (auto tensors : {scores, boxes, classes}) {
    auto layer = network->addConcatenation(tensors.data(), tensors.size());
    layer->setAxis(1);
    auto output = layer->getOutput(0);
    concat.push_back(output);
  }

  // Add NMS plugin
  auto nmsPlugin = yolo::NMSPlugin(yolo_config.iou_thresh, yolo_config.detections_per_im);
  auto layer = network->addPluginV2(concat.data(), concat.size(), nmsPlugin);
  for (int i = 0; i < layer->getNbOutputs(); i++) {
    auto output = layer->getOutput(i);
    network->markOutput(*output);
  }

  // create profile
  auto profile = builder->createOptimizationProfile();
  profile->setDimensions(
    network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kMIN,
    nvinfer1::Dims4{max_batch_size, input_channel, input_height, input_width});
  profile->setDimensions(
    network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kOPT,
    nvinfer1::Dims4{max_batch_size, input_channel, input_height, input_width});
  profile->setDimensions(
    network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kMAX,
    nvinfer1::Dims4{max_batch_size, input_channel, input_height, input_width});
  config->addOptimizationProfile(profile);

  std::unique_ptr<yolo::Int8EntropyCalibrator> calib{nullptr};
  if (int8) {
    if (calibration_images.size() >= static_cast<size_t>(max_batch_size)) {
      config->setFlag(nvinfer1::BuilderFlag::kINT8);
      yolo::ImageStream stream(max_batch_size, input_dims, calibration_images);
      calib = std::make_unique<yolo::Int8EntropyCalibrator>(stream, calibration_table);
      config->setInt8Calibrator(calib.get());
    } else {
      std::cout << "Fail to find enough images for INT8 calibration. Build FP mode..." << std::endl;
    }
  }

  // Build engine
  std::cout << "Applying optimizations and building TRT CUDA engine..." << std::endl;
  plan_ = unique_ptr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));
  if (!plan_) {
    std::cout << "Fail to create serialized network" << std::endl;
    return;
  }
  engine_ = unique_ptr<nvinfer1::ICudaEngine>(
    runtime_->deserializeCudaEngine(plan_->data(), plan_->size()));
  if (!prepare()) {
    std::cout << "Fail to create engine" << std::endl;
    return;
  }
}

void Net::save(const std::string & path) const
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

bool Net::detect(const cv::Mat & in_img, float * out_scores, float * out_boxes, float * out_classes)
{
  const auto input_dims = getInputDims();
  const auto input = preprocess(in_img, input_dims.at(0), input_dims.at(2), input_dims.at(1));
  CHECK_CUDA_ERROR(
    cudaMemcpy(input_d_.get(), input.data(), input.size() * sizeof(float), cudaMemcpyHostToDevice));
  std::vector<void *> buffers = {
    input_d_.get(), out_scores_d_.get(), out_boxes_d_.get(), out_classes_d_.get()};
  try {
    infer(buffers, 1);
  } catch (const std::runtime_error & e) {
    return false;
  }
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_scores, out_scores_d_.get(), sizeof(float) * getMaxDetections(), cudaMemcpyDeviceToHost,
    stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_boxes, out_boxes_d_.get(), sizeof(float) * 4 * getMaxDetections(), cudaMemcpyDeviceToHost,
    stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_classes, out_classes_d_.get(), sizeof(float) * getMaxDetections(), cudaMemcpyDeviceToHost,
    stream_));
  cudaStreamSynchronize(stream_);
  return true;
}

std::vector<int> Net::getInputDims() const
{
  auto dims = engine_->getBindingDimensions(0);
  return {dims.d[1], dims.d[2], dims.d[3]};
}

int Net::getMaxBatchSize() const
{
  return engine_->getProfileDimensions(0, 0, nvinfer1::OptProfileSelector::kMAX).d[0];
}

int Net::getInputSize() const
{
  const auto input_dims = getInputDims();
  const auto input_size =
    std::accumulate(std::begin(input_dims), std::end(input_dims), 1, std::multiplies<int>());
  return input_size;
}

int Net::getMaxDetections() const
{
  return engine_->getBindingDimensions(1).d[1];
}

}  // namespace yolo
