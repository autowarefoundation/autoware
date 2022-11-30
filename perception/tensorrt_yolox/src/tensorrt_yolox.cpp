// Copyright 2022 Tier IV, Inc.
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

#include "cuda_utils/cuda_check_error.hpp"
#include "cuda_utils/cuda_unique_ptr.hpp"

#include <tensorrt_yolox/tensorrt_yolox.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace tensorrt_yolox
{
TrtYoloX::TrtYoloX(
  const std::string & model_path, const std::string & precision, const int num_class,
  const float score_threshold, const float nms_threshold,
  [[maybe_unused]] const std::string & cache_dir, const tensorrt_common::BatchConfig & batch_config,
  const size_t max_workspace_size)
{
  trt_common_ = std::make_unique<tensorrt_common::TrtCommon>(
    model_path, precision, nullptr, batch_config, max_workspace_size);
  trt_common_->setup();

  if (!trt_common_->isInitialized()) {
    return;
  }

  // Judge whether decoding output is required
  // Plain models require decoding, while models with EfficientNMS_TRT module don't.
  // If getNbBindings == 5, the model contains EfficientNMS_TRT
  switch (trt_common_->getNbBindings()) {
    case 2:
      // Specified model is plain one.
      // Bindings are: [input, output]
      needs_output_decode_ = true;
      // The following three values are considered only if the specified model is plain one
      num_class_ = num_class;
      score_threshold_ = score_threshold;
      nms_threshold_ = nms_threshold;
      break;
    case 5:
      // Specified model contains Efficient NMS_TRT.
      // Bindings are[input, detection_classes, detection_scores, detection_boxes, num_detections]
      needs_output_decode_ = false;
      break;
    default:
      std::stringstream s;
      s << "\"" << model_path << "\" is unsuppoerted format";
      std::runtime_error{s.str()};
  }

  // GPU memory allocation
  const auto input_dims = trt_common_->getBindingDimensions(0);
  const auto input_size =
    std::accumulate(input_dims.d + 1, input_dims.d + input_dims.nbDims, 1, std::multiplies<int>());
  if (needs_output_decode_) {
    const auto output_dims = trt_common_->getBindingDimensions(1);
    input_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * input_size);
    out_elem_num_ = std::accumulate(
      output_dims.d + 1, output_dims.d + output_dims.nbDims, 1, std::multiplies<int>());
    out_elem_num_per_batch_ = static_cast<int>(out_elem_num_ / batch_config[2]);
    out_prob_d_ = cuda_utils::make_unique<float[]>(out_elem_num_);
    out_prob_h_ = cuda_utils::make_unique_host<float[]>(out_elem_num_, cudaHostAllocPortable);
  } else {
    const auto out_scores_dims = trt_common_->getBindingDimensions(3);
    max_detections_ = out_scores_dims.d[1];
    input_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * input_size);
    out_num_detections_d_ = cuda_utils::make_unique<int32_t[]>(batch_config[2]);
    out_boxes_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * max_detections_ * 4);
    out_scores_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * max_detections_);
    out_classes_d_ = cuda_utils::make_unique<int32_t[]>(batch_config[2] * max_detections_);
  }
}

void TrtYoloX::preprocess(const std::vector<cv::Mat> & images)
{
  const auto batch_size = images.size();
  auto input_dims = trt_common_->getBindingDimensions(0);
  input_dims.d[0] = batch_size;
  trt_common_->setBindingDimensions(0, input_dims);
  const float input_height = static_cast<float>(input_dims.d[2]);
  const float input_width = static_cast<float>(input_dims.d[3]);
  std::vector<cv::Mat> dst_images;
  scales_.clear();
  for (const auto & image : images) {
    cv::Mat dst_image;
    const float scale = std::min(input_width / image.cols, input_height / image.rows);
    scales_.emplace_back(scale);
    const auto scale_size = cv::Size(image.cols * scale, image.rows * scale);
    cv::resize(image, dst_image, scale_size, 0, 0, cv::INTER_CUBIC);
    const auto bottom = input_height - dst_image.rows;
    const auto right = input_width - dst_image.cols;
    copyMakeBorder(dst_image, dst_image, 0, bottom, 0, right, cv::BORDER_CONSTANT, {114, 114, 114});
    dst_images.emplace_back(dst_image);
  }
  const auto chw_images =
    cv::dnn::blobFromImages(dst_images, 1.0, cv::Size(), cv::Scalar(), false, false, CV_32F);

  const auto data_length = chw_images.total();
  input_h_.reserve(data_length);
  const auto flat = chw_images.reshape(1, data_length);
  input_h_ = chw_images.isContinuous() ? flat : flat.clone();
}

bool TrtYoloX::doInference(const std::vector<cv::Mat> & images, ObjectArrays & objects)
{
  if (!trt_common_->isInitialized()) {
    return false;
  }

  preprocess(images);

  CHECK_CUDA_ERROR(cudaMemcpy(
    input_d_.get(), input_h_.data(), input_h_.size() * sizeof(float), cudaMemcpyHostToDevice));

  if (needs_output_decode_) {
    return feedforwardAndDecode(images, objects);
  } else {
    return feedforward(images, objects);
  }
}

// This method is assumed to be called when specified YOLOX model contains
// EfficientNMS_TRT module.
bool TrtYoloX::feedforward(const std::vector<cv::Mat> & images, ObjectArrays & objects)
{
  std::vector<void *> buffers = {
    input_d_.get(), out_num_detections_d_.get(), out_boxes_d_.get(), out_scores_d_.get(),
    out_classes_d_.get()};

  trt_common_->enqueueV2(buffers.data(), *stream_, nullptr);

  const auto batch_size = images.size();
  auto out_num_detections = std::make_unique<int32_t[]>(batch_size);
  auto out_boxes = std::make_unique<float[]>(4 * batch_size * max_detections_);
  auto out_scores = std::make_unique<float[]>(batch_size * max_detections_);
  auto out_classes = std::make_unique<int32_t[]>(batch_size * max_detections_);

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_num_detections.get(), out_num_detections_d_.get(), sizeof(int32_t) * batch_size,
    cudaMemcpyDeviceToHost, *stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_boxes.get(), out_boxes_d_.get(), sizeof(float) * 4 * batch_size * max_detections_,
    cudaMemcpyDeviceToHost, *stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_scores.get(), out_scores_d_.get(), sizeof(float) * batch_size * max_detections_,
    cudaMemcpyDeviceToHost, *stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_classes.get(), out_classes_d_.get(), sizeof(int32_t) * batch_size * max_detections_,
    cudaMemcpyDeviceToHost, *stream_));
  cudaStreamSynchronize(*stream_);
  objects.clear();
  for (size_t i = 0; i < batch_size; ++i) {
    const size_t num_detection = static_cast<size_t>(out_num_detections[i]);
    ObjectArray object_array(num_detection);
    for (size_t j = 0; j < num_detection; ++j) {
      Object object{};
      const auto x1 = out_boxes[i * max_detections_ * 4 + j * 4] / scales_[i];
      const auto y1 = out_boxes[i * max_detections_ * 4 + j * 4 + 1] / scales_[i];
      const auto x2 = out_boxes[i * max_detections_ * 4 + j * 4 + 2] / scales_[i];
      const auto y2 = out_boxes[i * max_detections_ * 4 + j * 4 + 3] / scales_[i];
      object.x_offset = std::clamp(0, static_cast<int32_t>(x1), images[i].cols);
      object.y_offset = std::clamp(0, static_cast<int32_t>(y1), images[i].rows);
      object.width = static_cast<int32_t>(std::max(0.0F, x2 - x1));
      object.height = static_cast<int32_t>(std::max(0.0F, y2 - y1));
      object.score = out_scores[i * max_detections_ + j];
      object.type = out_classes[i * max_detections_ + j];
      object_array.emplace_back(object);
    }
    objects.emplace_back(object_array);
  }
  return true;
}

bool TrtYoloX::feedforwardAndDecode(const std::vector<cv::Mat> & images, ObjectArrays & objects)
{
  std::vector<void *> buffers = {input_d_.get(), out_prob_d_.get()};

  trt_common_->enqueueV2(buffers.data(), *stream_, nullptr);

  const auto batch_size = images.size();

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_prob_h_.get(), out_prob_d_.get(), sizeof(float) * out_elem_num_, cudaMemcpyDeviceToHost,
    *stream_));
  cudaStreamSynchronize(*stream_);
  objects.clear();

  for (size_t i = 0; i < batch_size; ++i) {
    auto image_size = images[i].size();
    float * batch_prob = out_prob_h_.get() + (i * out_elem_num_per_batch_);
    ObjectArray object_array;
    decodeOutputs(batch_prob, object_array, scales_[i], image_size);
    objects.emplace_back(object_array);
  }
  return true;
}

void TrtYoloX::decodeOutputs(
  float * prob, ObjectArray & objects, float scale, cv::Size & img_size) const
{
  ObjectArray proposals;
  auto input_dims = trt_common_->getBindingDimensions(0);
  const float input_height = static_cast<float>(input_dims.d[2]);
  const float input_width = static_cast<float>(input_dims.d[3]);
  std::vector<int> strides = {8, 16, 32};
  std::vector<GridAndStride> grid_strides;
  generateGridsAndStride(input_width, input_height, strides, grid_strides);
  generateYoloxProposals(grid_strides, prob, score_threshold_, proposals);

  qsortDescentInplace(proposals);

  std::vector<int> picked;
  nmsSortedBboxes(proposals, picked, nms_threshold_);

  int count = static_cast<int>(picked.size());
  objects.resize(count);
  for (int i = 0; i < count; i++) {
    objects[i] = proposals[picked[i]];

    // adjust offset to original unpadded
    float x0 = (objects[i].x_offset) / scale;
    float y0 = (objects[i].y_offset) / scale;
    float x1 = (objects[i].x_offset + objects[i].width) / scale;
    float y1 = (objects[i].y_offset + objects[i].height) / scale;

    // clip
    x0 = std::clamp(x0, 0.f, static_cast<float>(img_size.width - 1));
    y0 = std::clamp(y0, 0.f, static_cast<float>(img_size.height - 1));
    x1 = std::clamp(x1, 0.f, static_cast<float>(img_size.width - 1));
    y1 = std::clamp(y1, 0.f, static_cast<float>(img_size.height - 1));

    objects[i].x_offset = x0;
    objects[i].y_offset = y0;
    objects[i].width = x1 - x0;
    objects[i].height = y1 - y0;
  }
}

void TrtYoloX::generateGridsAndStride(
  const int target_w, const int target_h, std::vector<int> & strides,
  std::vector<GridAndStride> & grid_strides) const
{
  for (auto stride : strides) {
    int num_grid_w = target_w / stride;
    int num_grid_h = target_h / stride;
    for (int g1 = 0; g1 < num_grid_h; g1++) {
      for (int g0 = 0; g0 < num_grid_w; g0++) {
        grid_strides.push_back(GridAndStride{g0, g1, stride});
      }
    }
  }
}

void TrtYoloX::generateYoloxProposals(
  std::vector<GridAndStride> grid_strides, float * feat_blob, float prob_threshold,
  ObjectArray & objects) const
{
  const int num_anchors = grid_strides.size();

  for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++) {
    const int grid0 = grid_strides[anchor_idx].grid0;
    const int grid1 = grid_strides[anchor_idx].grid1;
    const int stride = grid_strides[anchor_idx].stride;

    const int basic_pos = anchor_idx * (num_class_ + 5);

    // yolox/models/yolo_head.py decode logic
    // To apply this logic, YOLOX head must output raw value
    // (i.e., `decode_in_inference` should be False)
    float x_center = (feat_blob[basic_pos + 0] + grid0) * stride;
    float y_center = (feat_blob[basic_pos + 1] + grid1) * stride;
    float w = exp(feat_blob[basic_pos + 2]) * stride;
    float h = exp(feat_blob[basic_pos + 3]) * stride;
    float x0 = x_center - w * 0.5f;
    float y0 = y_center - h * 0.5f;

    float box_objectness = feat_blob[basic_pos + 4];
    for (int class_idx = 0; class_idx < num_class_; class_idx++) {
      float box_cls_score = feat_blob[basic_pos + 5 + class_idx];
      float box_prob = box_objectness * box_cls_score;
      if (box_prob > prob_threshold) {
        Object obj;
        obj.x_offset = x0;
        obj.y_offset = y0;
        obj.height = h;
        obj.width = w;
        obj.type = class_idx;
        obj.score = box_prob;

        objects.push_back(obj);
      }
    }  // class loop
  }    // point anchor loop
}

void TrtYoloX::qsortDescentInplace(ObjectArray & faceobjects, int left, int right) const
{
  int i = left;
  int j = right;
  float p = faceobjects[(left + right) / 2].score;

  while (i <= j) {
    while (faceobjects[i].score > p) {
      i++;
    }

    while (faceobjects[j].score < p) {
      j--;
    }

    if (i <= j) {
      // swap
      std::swap(faceobjects[i], faceobjects[j]);

      i++;
      j--;
    }
  }

#pragma omp parallel sections
  {
#pragma omp section
    {
      if (left < j) {
        qsortDescentInplace(faceobjects, left, j);
      }
    }
#pragma omp section
    {
      if (i < right) {
        qsortDescentInplace(faceobjects, i, right);
      }
    }
  }
}

void TrtYoloX::nmsSortedBboxes(
  const ObjectArray & faceobjects, std::vector<int> & picked, float nms_threshold) const
{
  picked.clear();
  const int n = faceobjects.size();

  std::vector<float> areas(n);
  for (int i = 0; i < n; i++) {
    cv::Rect rect(
      faceobjects[i].x_offset, faceobjects[i].y_offset, faceobjects[i].width,
      faceobjects[i].height);
    areas[i] = rect.area();
  }

  for (int i = 0; i < n; i++) {
    const Object & a = faceobjects[i];

    int keep = 1;
    for (int j = 0; j < static_cast<int>(picked.size()); j++) {
      const Object & b = faceobjects[picked[j]];

      // intersection over union
      float inter_area = intersectionArea(a, b);
      float union_area = areas[i] + areas[picked[j]] - inter_area;
      // float IoU = inter_area / union_area
      if (inter_area / union_area > nms_threshold) {
        keep = 0;
      }
    }

    if (keep) {
      picked.push_back(i);
    }
  }
}

}  // namespace tensorrt_yolox
