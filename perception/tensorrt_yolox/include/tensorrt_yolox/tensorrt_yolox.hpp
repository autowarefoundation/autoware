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

#ifndef TENSORRT_YOLOX__TENSORRT_YOLOX_HPP_
#define TENSORRT_YOLOX__TENSORRT_YOLOX_HPP_

#include <cuda_utils/cuda_unique_ptr.hpp>
#include <cuda_utils/stream_unique_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <tensorrt_common/tensorrt_common.hpp>

#include <memory>
#include <string>
#include <vector>

namespace tensorrt_yolox
{
using cuda_utils::CudaUniquePtr;
using cuda_utils::CudaUniquePtrHost;
using cuda_utils::makeCudaStream;
using cuda_utils::StreamUniquePtr;

struct Object
{
  int32_t x_offset;
  int32_t y_offset;
  int32_t height;
  int32_t width;
  float score;
  int32_t type;
};

using ObjectArray = std::vector<Object>;
using ObjectArrays = std::vector<ObjectArray>;

struct GridAndStride
{
  int grid0;
  int grid1;
  int stride;
};

class TrtYoloX
{
public:
  TrtYoloX(
    const std::string & model_path, const std::string & precision, const int num_class = 8,
    const float score_threshold = 0.3, const float nms_threshold = 0.7,
    const std::string & cache_dir = "",
    const tensorrt_common::BatchConfig & batch_config = {1, 1, 1},
    const size_t max_workspace_size = (1 << 30));

  bool doInference(const std::vector<cv::Mat> & images, ObjectArrays & objects);

private:
  void preprocess(const std::vector<cv::Mat> & images);
  bool feedforward(const std::vector<cv::Mat> & images, ObjectArrays & objects);
  bool feedforwardAndDecode(const std::vector<cv::Mat> & images, ObjectArrays & objects);
  void decodeOutputs(float * prob, ObjectArray & objects, float scale, cv::Size & img_size) const;
  void generateGridsAndStride(
    const int target_w, const int target_h, std::vector<int> & strides,
    std::vector<GridAndStride> & grid_strides) const;
  void generateYoloxProposals(
    std::vector<GridAndStride> grid_strides, float * feat_blob, float prob_threshold,
    ObjectArray & objects) const;
  void qsortDescentInplace(ObjectArray & faceobjects, int left, int right) const;
  inline void qsortDescentInplace(ObjectArray & objects) const
  {
    if (objects.empty()) {
      return;
    }
    qsortDescentInplace(objects, 0, objects.size() - 1);
  }
  inline float intersectionArea(const Object & a, const Object & b) const
  {
    cv::Rect a_rect(a.x_offset, a.y_offset, a.width, a.height);
    cv::Rect b_rect(b.x_offset, b.y_offset, b.width, b.height);
    cv::Rect_<float> inter = a_rect & b_rect;
    return inter.area();
  }
  void nmsSortedBboxes(
    const ObjectArray & faceobjects, std::vector<int> & picked, float nms_threshold) const;

  std::unique_ptr<tensorrt_common::TrtCommon> trt_common_;

  std::vector<float> input_h_;
  CudaUniquePtr<float[]> input_d_;
  CudaUniquePtr<int32_t[]> out_num_detections_d_;
  CudaUniquePtr<float[]> out_boxes_d_;
  CudaUniquePtr<float[]> out_scores_d_;
  CudaUniquePtr<int32_t[]> out_classes_d_;

  bool needs_output_decode_;
  size_t out_elem_num_;
  size_t out_elem_num_per_batch_;
  CudaUniquePtr<float[]> out_prob_d_;

  StreamUniquePtr stream_{makeCudaStream()};

  int32_t max_detections_;
  std::vector<float> scales_;

  int num_class_;
  float score_threshold_;
  float nms_threshold_;

  CudaUniquePtrHost<float[]> out_prob_h_;
};

}  // namespace tensorrt_yolox

#endif  // TENSORRT_YOLOX__TENSORRT_YOLOX_HPP_
