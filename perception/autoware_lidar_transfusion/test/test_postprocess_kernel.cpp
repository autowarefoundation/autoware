// Copyright 2024 TIER IV, Inc.
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

#include "test_postprocess_kernel.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::lidar_transfusion
{

void PostprocessKernelTest::SetUp()
{
  cudaStreamCreate(&stream_);

  auto cloud_capacity = 2000000;
  auto voxels_num = std::vector<int64_t>{1, 3, 5};
  auto point_cloud_range = std::vector<double>{-76.8, -76.8, -3.0, 76.8, 76.8, 5.0};
  auto voxel_size = std::vector<double>{0.3, 0.3, 8.0};
  auto num_proposals = 500;
  auto score_threshold = 0.2f;
  auto circle_nms_dist_threshold = 0.5f;
  auto yaw_norm_thresholds = std::vector<double>{0.5, 0.5, 0.5};
  config_ptr_ = std::make_unique<TransfusionConfig>(
    cloud_capacity, voxels_num, point_cloud_range, voxel_size, num_proposals,
    circle_nms_dist_threshold, yaw_norm_thresholds, score_threshold);
  post_ptr_ = std::make_unique<PostprocessCuda>(*config_ptr_, stream_);

  cls_size_ = config_ptr_->num_proposals_ * config_ptr_->num_classes_;
  box_size_ = config_ptr_->num_proposals_ * config_ptr_->num_box_values_;
  dir_cls_size_ = config_ptr_->num_proposals_ * 2;  // x, y

  cls_output_d_ = cuda::make_unique<float[]>(cls_size_);
  box_output_d_ = cuda::make_unique<float[]>(box_size_);
  dir_cls_output_d_ = cuda::make_unique<float[]>(dir_cls_size_);

  cuda::clear_async(cls_output_d_.get(), cls_size_, stream_);
  cuda::clear_async(box_output_d_.get(), box_size_, stream_);
  cuda::clear_async(dir_cls_output_d_.get(), dir_cls_size_, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

void PostprocessKernelTest::TearDown()
{
}

TEST_F(PostprocessKernelTest, EmptyTensorTest)
{
  std::vector<Box3D> det_boxes3d;

  CHECK_CUDA_ERROR(post_ptr_->generateDetectedBoxes3D_launch(
    cls_output_d_.get(), box_output_d_.get(), dir_cls_output_d_.get(), det_boxes3d, stream_));

  EXPECT_EQ(0, det_boxes3d.size());
}

TEST_F(PostprocessKernelTest, SingleDetectionTest)
{
  cuda::clear_async(cls_output_d_.get(), cls_size_, stream_);
  cuda::clear_async(box_output_d_.get(), box_size_, stream_);
  cuda::clear_async(dir_cls_output_d_.get(), dir_cls_size_, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  std::vector<Box3D> det_boxes3d;
  unsigned int det_idx = 42;
  unsigned int cls_idx = 2;

  constexpr float detection_score = 1.f;
  constexpr float detection_x = 50.f;
  constexpr float detection_y = -38.4f;
  constexpr float detection_z = 1.0;
  const float detection_raw_x = (detection_x - config_ptr_->min_x_range_) /
                                (config_ptr_->num_point_values_ * config_ptr_->voxel_x_size_);
  const float detection_raw_y = (detection_y - config_ptr_->min_y_range_) /
                                (config_ptr_->num_point_values_ * config_ptr_->voxel_y_size_);
  const float detection_raw_z = detection_z;

  constexpr float detection_w = 3.2f;
  constexpr float detection_l = 9.3f;
  constexpr float detection_h = 1.7f;
  constexpr float detection_log_w = std::log(detection_w);
  constexpr float detection_log_l = std::log(detection_l);
  constexpr float detection_log_h = std::log(detection_h);

  constexpr float detection_yaw = M_PI_4;
  constexpr float detection_yaw_sin = std::sin(detection_yaw);
  constexpr float detection_yaw_cos = std::cos(detection_yaw);

  // Set the values in the cuda tensor
  cudaMemcpy(
    &cls_output_d_[cls_idx * config_ptr_->num_proposals_ + det_idx], &detection_score,
    1 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(
    &dir_cls_output_d_[det_idx], &detection_yaw_sin, 1 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(
    &dir_cls_output_d_[det_idx + config_ptr_->num_proposals_], &detection_yaw_cos,
    1 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(&box_output_d_[det_idx], &detection_raw_x, 1 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(
    &box_output_d_[det_idx + config_ptr_->num_proposals_], &detection_raw_y, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &box_output_d_[det_idx + 2 * config_ptr_->num_proposals_], &detection_raw_z, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &box_output_d_[det_idx + 3 * config_ptr_->num_proposals_], &detection_log_w, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &box_output_d_[det_idx + 4 * config_ptr_->num_proposals_], &detection_log_l, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &box_output_d_[det_idx + 5 * config_ptr_->num_proposals_], &detection_log_h, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  auto code1 = cudaGetLastError();
  ASSERT_EQ(cudaSuccess, code1);

  // Extract the boxes
  auto code2 = post_ptr_->generateDetectedBoxes3D_launch(
    cls_output_d_.get(), box_output_d_.get(), dir_cls_output_d_.get(), det_boxes3d, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  ASSERT_EQ(cudaSuccess, code2);
  ASSERT_EQ(1, det_boxes3d.size());

  const auto det_box3d = det_boxes3d[0];
  EXPECT_EQ(cls_idx, det_box3d.label);
  EXPECT_EQ(detection_score, det_box3d.score);
  EXPECT_EQ(detection_x, det_box3d.x);
  EXPECT_EQ(detection_y, det_box3d.y);
  EXPECT_EQ(detection_z, det_box3d.z);
  EXPECT_NEAR(detection_w, det_box3d.width, 1e-6);
  EXPECT_NEAR(detection_l, det_box3d.length, 1e-6);
  EXPECT_NEAR(detection_h, det_box3d.height, 1e-6);
  EXPECT_EQ(detection_yaw, det_box3d.yaw);
}

TEST_F(PostprocessKernelTest, InvalidYawTest)
{
  cuda::clear_async(cls_output_d_.get(), cls_size_, stream_);
  cuda::clear_async(box_output_d_.get(), box_size_, stream_);
  cuda::clear_async(dir_cls_output_d_.get(), dir_cls_size_, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  std::vector<Box3D> det_boxes3d;
  unsigned int det_idx = 42;
  unsigned int cls_idx = 2;

  constexpr float detection_score = 1.f;
  constexpr float detection_x = 50.f;
  constexpr float detection_y = -38.4f;
  constexpr float detection_z = 1.0;
  const float detection_raw_x = (detection_x - config_ptr_->min_x_range_) /
                                (config_ptr_->num_point_values_ * config_ptr_->voxel_x_size_);
  const float detection_raw_y = (detection_y - config_ptr_->min_y_range_) /
                                (config_ptr_->num_point_values_ * config_ptr_->voxel_y_size_);
  const float detection_raw_z = detection_z;

  constexpr float detection_w = 3.2f;
  constexpr float detection_l = 9.3f;
  constexpr float detection_h = 1.7f;
  constexpr float detection_log_w = std::log(detection_w);
  constexpr float detection_log_l = std::log(detection_l);
  constexpr float detection_log_h = std::log(detection_h);

  constexpr float detection_yaw_sin = 0.f;
  constexpr float detection_yaw_cos = 0.2f;

  // Set the values in the cuda tensor
  cudaMemcpy(
    &cls_output_d_[cls_idx * config_ptr_->num_proposals_ + det_idx], &detection_score,
    1 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(
    &dir_cls_output_d_[det_idx], &detection_yaw_sin, 1 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(
    &dir_cls_output_d_[det_idx + config_ptr_->num_proposals_], &detection_yaw_cos,
    1 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(&box_output_d_[det_idx], &detection_raw_x, 1 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(
    &box_output_d_[det_idx + config_ptr_->num_proposals_], &detection_raw_y, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &box_output_d_[det_idx + 2 * config_ptr_->num_proposals_], &detection_raw_z, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &box_output_d_[det_idx + 3 * config_ptr_->num_proposals_], &detection_log_w, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &box_output_d_[det_idx + 4 * config_ptr_->num_proposals_], &detection_log_l, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &box_output_d_[det_idx + 5 * config_ptr_->num_proposals_], &detection_log_h, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  auto code1 = cudaGetLastError();
  ASSERT_EQ(cudaSuccess, code1);

  // Extract the boxes
  auto code2 = post_ptr_->generateDetectedBoxes3D_launch(
    cls_output_d_.get(), box_output_d_.get(), dir_cls_output_d_.get(), det_boxes3d, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  ASSERT_EQ(cudaSuccess, code2);
  EXPECT_EQ(0, det_boxes3d.size());
}

TEST_F(PostprocessKernelTest, CircleNMSTest)
{
  cuda::clear_async(cls_output_d_.get(), cls_size_, stream_);
  cuda::clear_async(box_output_d_.get(), box_size_, stream_);
  cuda::clear_async(dir_cls_output_d_.get(), dir_cls_size_, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  std::vector<Box3D> det_boxes3d;
  unsigned int det_idx = 42;
  unsigned int det_num = 2;
  unsigned int cls_idx = 2;

  constexpr float detection_score = 1.f;
  constexpr float detection_x = 50.f;
  constexpr float detection_y = -38.4f;
  constexpr float detection_z = 1.0;
  const float detection_raw_x = (detection_x - config_ptr_->min_x_range_) /
                                (config_ptr_->num_point_values_ * config_ptr_->voxel_x_size_);
  const float detection_raw_y = (detection_y - config_ptr_->min_y_range_) /
                                (config_ptr_->num_point_values_ * config_ptr_->voxel_y_size_);
  const float detection_raw_z = detection_z;

  constexpr float detection_w = 3.2f;
  constexpr float detection_l = 9.3f;
  constexpr float detection_h = 1.7f;
  constexpr float detection_log_w = std::log(detection_w);
  constexpr float detection_log_l = std::log(detection_l);
  constexpr float detection_log_h = std::log(detection_h);

  constexpr float detection_yaw = M_PI_4;
  constexpr float detection_yaw_sin = std::sin(detection_yaw);
  constexpr float detection_yaw_cos = std::cos(detection_yaw);

  // Set the values in the cuda tensor for 2 detections
  for (std::size_t i = 0; i < det_num; ++i) {
    cudaMemcpy(
      &cls_output_d_[cls_idx * config_ptr_->num_proposals_ + det_idx + i], &detection_score,
      1 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(
      &dir_cls_output_d_[det_idx + i], &detection_yaw_sin, 1 * sizeof(float),
      cudaMemcpyHostToDevice);
    cudaMemcpy(
      &dir_cls_output_d_[det_idx + i + config_ptr_->num_proposals_], &detection_yaw_cos,
      1 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(
      &box_output_d_[det_idx + i], &detection_raw_x, 1 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(
      &box_output_d_[det_idx + i + config_ptr_->num_proposals_], &detection_raw_y,
      1 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(
      &box_output_d_[det_idx + i + 2 * config_ptr_->num_proposals_], &detection_raw_z,
      1 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(
      &box_output_d_[det_idx + i + 3 * config_ptr_->num_proposals_], &detection_log_w,
      1 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(
      &box_output_d_[det_idx + i + 4 * config_ptr_->num_proposals_], &detection_log_l,
      1 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(
      &box_output_d_[det_idx + i + 5 * config_ptr_->num_proposals_], &detection_log_h,
      1 * sizeof(float), cudaMemcpyHostToDevice);
  }

  auto code1 = cudaGetLastError();
  ASSERT_EQ(cudaSuccess, code1);

  // Extract the boxes
  auto code2 = post_ptr_->generateDetectedBoxes3D_launch(
    cls_output_d_.get(), box_output_d_.get(), dir_cls_output_d_.get(), det_boxes3d, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  ASSERT_EQ(cudaSuccess, code2);
  EXPECT_EQ(1, det_boxes3d.size());
}

}  // namespace autoware::lidar_transfusion

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
