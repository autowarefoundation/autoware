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

#ifndef TENSORRT_YOLOX__TENSORRT_YOLOX_HPP_
#define TENSORRT_YOLOX__TENSORRT_YOLOX_HPP_

#include <cuda_utils/cuda_unique_ptr.hpp>
#include <cuda_utils/stream_unique_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <tensorrt_common/tensorrt_common.hpp>
#include <tensorrt_yolox/preprocess.hpp>

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

/**
 * @class TrtYoloX
 * @brief TensorRT YOLOX for faster inference
 * @warning  Regarding quantization, we recommend use MinMax calibration due to accuracy drop with
 * Entropy calibration.
 */
class TrtYoloX
{
public:
  /**
   * @brief Construct TrtYoloX.
   * @param[in] mode_path ONNX model_path
   * @param[in] precision precision for inference
   * @param[in] num_class classifier-ed num
   * @param[in] score_threshold threshold for detection
   * @param[in] nms_threshold threshold for NMS
   * @param[in] build_config configuration including precision, calibration method, DLA, remaining
   * fp16 for first layer,  remaining fp16 for last layer and profiler for builder
   * @param[in] use_gpu_preprocess whether use cuda gpu for preprocessing
   * @param[in] calibration_image_list_file path for calibration files (only require for
   * quantization)
   * @param[in] norm_factor scaling factor for preprocess
   * @param[in] cache_dir unused variable
   * @param[in] batch_config configuration for batched execution
   * @param[in] max_workspace_size maximum workspace for building TensorRT engine
   */
  TrtYoloX(
    const std::string & model_path, const std::string & precision, const int num_class = 8,
    const float score_threshold = 0.3, const float nms_threshold = 0.7,
    const tensorrt_common::BuildConfig build_config = tensorrt_common::BuildConfig(),
    const bool use_gpu_preprocess = false, std::string calibration_image_list_file = std::string(),
    const double norm_factor = 1.0, [[maybe_unused]] const std::string & cache_dir = "",
    const tensorrt_common::BatchConfig & batch_config = {1, 1, 1},
    const size_t max_workspace_size = (1 << 30));
  /**
   * @brief Deconstruct TrtYoloX
   */
  ~TrtYoloX();

  /**
   * @brief run inference including pre-process and post-process
   * @param[out] objects results for object detection
   * @param[in] images batched images
   */
  bool doInference(const std::vector<cv::Mat> & images, ObjectArrays & objects);

  /**
   * @brief run inference including pre-process and post-process
   * @param[out] objects results for object detection
   * @param[in] images batched images
   * @param[in] rois region of interest for inference
   */
  bool doInferenceWithRoi(
    const std::vector<cv::Mat> & images, ObjectArrays & objects, const std::vector<cv::Rect> & roi);

  /**
   * @brief run multi-scale inference including pre-process and post-process
   * @param[out] objects results for object detection
   * @param[in] image
   * @param[in] rois region of interest for inference
   */
  bool doMultiScaleInference(
    const cv::Mat & image, ObjectArrays & objects, const std::vector<cv::Rect> & roi);

  /**
   * @brief allocate buffer for preprocess on GPU
   * @param[in] width original image width
   * @param[in] height original image height
   * @warning if we don't allocate buffers using it, "preprocessGpu" allocates buffers at the
   * beginning
   */
  void initPreprocessBuffer(int width, int height);

  /**
   * @brief output TensorRT profiles for each layer
   */
  void printProfiling(void);

private:
  /**
   * @brief run preprocess including resizing, letterbox, NHWC2NCHW and toFloat on CPU
   * @param[in] images batching images
   */
  void preprocess(const std::vector<cv::Mat> & images);

  /**
   * @brief run preprocess on GPU
   * @param[in] images batching images
   */
  void preprocessGpu(const std::vector<cv::Mat> & images);

  /**
   * @brief run preprocess including resizing, letterbox, NHWC2NCHW and toFloat on CPU
   * @param[in] images batching images
   * @param[in] rois region of interest
   */
  void preprocessWithRoi(const std::vector<cv::Mat> & images, const std::vector<cv::Rect> & rois);

  /**
   * @brief run preprocess on GPU
   * @param[in] images batching images
   * @param[in] rois region of interest
   */
  void preprocessWithRoiGpu(
    const std::vector<cv::Mat> & images, const std::vector<cv::Rect> & rois);

  /**
   * @brief run multi-scale preprocess including resizing, letterbox, NHWC2NCHW and toFloat on CPU
   * @param[in] images batching images
   * @param[in] rois region of interest
   */
  void multiScalePreprocess(const cv::Mat & image, const std::vector<cv::Rect> & rois);

  /**
   * @brief run multi-scale preprocess including resizing, letterbox, NHWC2NCHW and toFloat on GPU
   * @param[in] images batching images
   * @param[in] rois region of interest
   */
  void multiScalePreprocessGpu(const cv::Mat & image, const std::vector<cv::Rect> & rois);

  bool multiScaleFeedforward(const cv::Mat & image, int batch_size, ObjectArrays & objects);
  bool multiScaleFeedforwardAndDecode(
    const cv::Mat & images, int batch_size, ObjectArrays & objects);

  bool feedforward(const std::vector<cv::Mat> & images, ObjectArrays & objects);
  bool feedforwardAndDecode(const std::vector<cv::Mat> & images, ObjectArrays & objects);
  void decodeOutputs(float * prob, ObjectArray & objects, float scale, cv::Size & img_size) const;
  void generateGridsAndStride(
    const int target_w, const int target_h, const std::vector<int> & strides,
    std::vector<GridAndStride> & grid_strides) const;
  void generateYoloxProposals(
    std::vector<GridAndStride> grid_strides, float * feat_blob, float prob_threshold,
    ObjectArray & objects) const;
  void qsortDescentInplace(ObjectArray & face_objects, int left, int right) const;
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

  // cspell: ignore Bboxes
  void nmsSortedBboxes(
    const ObjectArray & face_objects, std::vector<int> & picked, float nms_threshold) const;

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
  int batch_size_;
  CudaUniquePtrHost<float[]> out_prob_h_;

  // flag whether preprocess are performed on GPU
  bool use_gpu_preprocess_;
  // host buffer for preprocessing on GPU
  CudaUniquePtrHost<unsigned char[]> image_buf_h_;
  // device buffer for preprocessing on GPU
  CudaUniquePtr<unsigned char[]> image_buf_d_;
  // normalization factor used for preprocessing
  double norm_factor_;

  std::vector<int> output_strides_;

  int src_width_;
  int src_height_;

  // host pointer for ROI
  CudaUniquePtrHost<Roi[]> roi_h_;
  // device pointer for ROI
  CudaUniquePtr<Roi[]> roi_d_;
};

}  // namespace tensorrt_yolox

#endif  // TENSORRT_YOLOX__TENSORRT_YOLOX_HPP_
