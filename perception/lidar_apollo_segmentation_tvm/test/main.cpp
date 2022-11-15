// Copyright 2021-2022 Arm Ltd.
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

#include <common/types.hpp>
#include <lidar_apollo_segmentation_tvm/lidar_apollo_segmentation_tvm.hpp>
#include <tvm_utility/pipeline.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <random>
#include <string>
#include <vector>

using autoware::common::types::float32_t;
using autoware::common::types::uchar8_t;
using autoware::perception::lidar_apollo_segmentation_tvm::ApolloLidarSegmentation;

void test_segmentation(bool use_intensity_feature, bool use_constant_feature, bool expect_throw)
{
  // Instantiate the pipeline
  const int width = 1;
  const int height = 10000;
  const int range = 70;
  const float score_threshold = 0.8f;
  const float z_offset = 1.0f;
  const float32_t min_height = -5.0f;
  const float32_t max_height = 5.0f;
  const float32_t objectness_thresh = 0.5f;
  const int32_t min_pts_num = 3;
  const float32_t height_thresh = 0.5f;
  ApolloLidarSegmentation segmentation(
    range, score_threshold, use_intensity_feature, use_constant_feature, z_offset, min_height,
    max_height, objectness_thresh, min_pts_num, height_thresh);

  auto version_status = segmentation.version_check();
  EXPECT_NE(version_status, tvm_utility::Version::Unsupported);

  std::random_device rd;
  std::mt19937 gen(42);
  std::uniform_real_distribution<float32_t> dis(-50.0, 50.0);
  std::vector<uchar8_t> v(width * height * sizeof(float32_t) * 4);
  for (size_t i = 0; i < width * height * 4; i++) {
    reinterpret_cast<float32_t *>(v.data())[i] = dis(gen);
  }

  sensor_msgs::msg::PointCloud2 input{};
  input.header.frame_id = "base_link";
  input.fields.resize(4U);
  input.fields[0U].name = "x";
  input.fields[1U].name = "y";
  input.fields[2U].name = "z";
  input.fields[3U].name = "intensity";
  for (uint32_t idx = 0U; idx < 4U; ++idx) {
    input.fields[idx].offset = static_cast<uint32_t>(idx * sizeof(float32_t));
    input.fields[idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
    input.fields[idx].count = 1U;
    input.point_step += static_cast<uint32_t>(sizeof(float32_t));
  }
  input.height = static_cast<uint32_t>(height);
  input.width = static_cast<uint32_t>(width);
  input.is_bigendian = false;
  input.row_step = input.point_step * input.width;
  input.is_dense = false;
  input.data = v;

  std::shared_ptr<const tier4_perception_msgs::msg::DetectedObjectsWithFeature> output;
  bool has_thrown = false;
  try {
    output = segmentation.detectDynamicObjects(input);
  } catch (const std::exception & e) {
    has_thrown = true;
  }
  EXPECT_EQ(expect_throw, has_thrown);
}

// Other test configurations to increase code coverage.
TEST(lidar_apollo_segmentation_tvm, others)
{
  test_segmentation(false, true, false);
  test_segmentation(true, true, false);
  test_segmentation(false, false, false);
  test_segmentation(true, false, false);
}
