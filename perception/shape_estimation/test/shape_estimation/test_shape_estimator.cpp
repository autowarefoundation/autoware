
// Copyright 2024 TIER IV, inc.
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

#include "shape_estimation/corrector/corrector.hpp"
#include "shape_estimation/filter/filter.hpp"
#include "shape_estimation/model/model.hpp"
#include "shape_estimation/shape_estimator.hpp"

#include <gtest/gtest.h>
#include <math.h>

namespace
{
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double deg2rad(const double deg)
{
  return deg / 180.0 * M_PI;
}

pcl::PointCloud<pcl::PointXYZ> createLShapeCluster(
  const double length, const double width, const double height, const double yaw,
  const double offset_x, const double offset_y)
{
  pcl::PointCloud<pcl::PointXYZ> cluster;
  for (double x = -length / 2; x < length / 2; x += 0.4) {
    cluster.push_back(pcl::PointXYZ(x, width / 2, 0.0));
  }
  for (double y = -width / 2; y < width / 2; y += 0.2) {
    cluster.push_back(pcl::PointXYZ(-length / 2, y, 0.0));
  }
  cluster.push_back(pcl::PointXYZ(length / 2, -width / 2, 0.0));
  cluster.push_back(pcl::PointXYZ(length / 2, width / 2, 0.0));
  cluster.push_back(pcl::PointXYZ(-length / 2, -width / 2, 0.0));
  cluster.push_back(pcl::PointXYZ(-length / 2, width / 2, 0.0));
  cluster.push_back(pcl::PointXYZ(0.0, 0.0, height));

  for (auto & point : cluster) {
    const double x = point.x;
    const double y = point.y;
    // rotate
    point.x = x * cos(yaw) - y * sin(yaw);
    point.y = x * sin(yaw) + y * cos(yaw);
    // offset
    point.x += offset_x;
    point.y += offset_y;
  }

  return cluster;
}
}  // namespace

// test BoundingBoxShapeModel
// 1. base case
TEST(BoundingBoxShapeModel, test_estimateShape)
{
  // Generate BoundingBoxShapeModel
  BoundingBoxShapeModel bbox_shape_model = BoundingBoxShapeModel();

  // Generate cluster
  const double length = 4.0;
  const double width = 2.0;
  const double height = 1.0;

  pcl::PointCloud<pcl::PointXYZ> cluster =
    createLShapeCluster(length, width, height, 0.0, 0.0, 0.0);

  // Generate shape and pose output
  autoware_perception_msgs::msg::Shape shape_output;
  geometry_msgs::msg::Pose pose_output;

  // Test estimateShape
  const bool result = bbox_shape_model.estimate(cluster, shape_output, pose_output);
  EXPECT_TRUE(result);

  // Check shape_output
  EXPECT_EQ(shape_output.type, autoware_perception_msgs::msg::Shape::BOUNDING_BOX);
  EXPECT_NEAR(shape_output.dimensions.x, length, length * 0.1);
  EXPECT_NEAR(shape_output.dimensions.y, width, width * 0.1);
  EXPECT_NEAR(shape_output.dimensions.z, height, height * 0.1);

  // Check pose_output
  EXPECT_NEAR(pose_output.position.x, 0.0, 1e-2);
  EXPECT_NEAR(pose_output.position.y, 0.0, 1e-2);
  EXPECT_NEAR(pose_output.position.z, height / 2, 1e-2);

  // transform quaternion to yaw
  const double pose_output_yaw = yawFromQuaternion(pose_output.orientation);
  EXPECT_NEAR(pose_output_yaw, 0, 1e-3);
}

// 2. rotated case
TEST(BoundingBoxShapeModel, test_estimateShape_rotated)
{
  // Generate cluster
  const double length = 4.0;
  const double width = 2.0;
  const double height = 1.0;
  const double yaw = deg2rad(30.0);
  const double offset_x = 10.0;
  const double offset_y = 1.0;
  pcl::PointCloud<pcl::PointXYZ> cluster =
    createLShapeCluster(length, width, height, yaw, offset_x, offset_y);

  const auto ref_yaw_info =
    ReferenceYawInfo{static_cast<float>(yaw), static_cast<float>(deg2rad(10.0))};
  const bool use_boost_bbox_optimizer = true;
  // Generate BoundingBoxShapeModel
  BoundingBoxShapeModel bbox_shape_model =
    BoundingBoxShapeModel(ref_yaw_info, use_boost_bbox_optimizer);

  // Generate shape and pose output
  autoware_perception_msgs::msg::Shape shape_output;
  geometry_msgs::msg::Pose pose_output;

  // Test estimateShape
  const bool result = bbox_shape_model.estimate(cluster, shape_output, pose_output);
  EXPECT_TRUE(result);

  // Check shape_output
  EXPECT_EQ(shape_output.type, autoware_perception_msgs::msg::Shape::BOUNDING_BOX);
  EXPECT_NEAR(shape_output.dimensions.x, length, length * 0.1);
  EXPECT_NEAR(shape_output.dimensions.y, width, width * 0.1);
  EXPECT_NEAR(shape_output.dimensions.z, height, height * 0.1);

  // Check pose_output
  EXPECT_NEAR(pose_output.position.x, offset_x, 1.0);
  EXPECT_NEAR(pose_output.position.y, offset_y, 1.0);
  EXPECT_NEAR(pose_output.position.z, height / 2, 1.0);

  // transform quaternion to yaw
  const double pose_output_yaw = yawFromQuaternion(pose_output.orientation);
  EXPECT_NEAR(pose_output_yaw, yaw, deg2rad(15.0));
}

// test CylinderShapeModel
TEST(CylinderShapeModel, test_estimateShape)
{
  // Generate CylinderShapeModel
  CylinderShapeModel cylinder_shape_model = CylinderShapeModel();

  // Generate cluster
  pcl::PointCloud<pcl::PointXYZ> cluster = createLShapeCluster(4.0, 2.0, 1.0, 0.0, 0.0, 0.0);
  // Generate shape and pose output
  autoware_perception_msgs::msg::Shape shape_output;
  geometry_msgs::msg::Pose pose_output;

  // Test estimateShape
  const bool result = cylinder_shape_model.estimate(cluster, shape_output, pose_output);
  EXPECT_TRUE(result);
}

// test ConvexHullShapeModel
TEST(ConvexHullShapeModel, test_estimateShape)
{
  // Generate ConvexHullShapeModel
  ConvexHullShapeModel convex_hull_shape_model = ConvexHullShapeModel();

  // Generate cluster
  pcl::PointCloud<pcl::PointXYZ> cluster = createLShapeCluster(2.0, 1.0, 1.0, 0.0, 0.0, 0.0);

  // Generate shape and pose output
  autoware_perception_msgs::msg::Shape shape_output;
  geometry_msgs::msg::Pose pose_output;

  // Test estimateShape
  const bool result = convex_hull_shape_model.estimate(cluster, shape_output, pose_output);
  EXPECT_TRUE(result);
}

// test ShapeEstimator
TEST(ShapeEstimator, test_estimateShapeAndPose)
{
  // Generate cluster
  double length = 4.0;
  double width = 2.0;
  double height = 1.0;
  const double yaw = deg2rad(60.0);
  const double offset_x = 6.0;
  const double offset_y = -2.0;
  pcl::PointCloud<pcl::PointXYZ> cluster =
    createLShapeCluster(length, width, height, yaw, offset_x, offset_y);

  // Generate ShapeEstimator
  const bool use_corrector = true;
  const bool use_filter = true;
  const bool use_boost_bbox_optimizer = true;
  ShapeEstimator shape_estimator =
    ShapeEstimator(use_corrector, use_filter, use_boost_bbox_optimizer);

  // Generate ref_yaw_info
  boost::optional<ReferenceYawInfo> ref_yaw_info = boost::none;
  boost::optional<ReferenceShapeSizeInfo> ref_shape_size_info = boost::none;

  ref_yaw_info = ReferenceYawInfo{static_cast<float>(yaw), static_cast<float>(deg2rad(10.0))};
  const auto label = autoware_perception_msgs::msg::ObjectClassification::CAR;

  // Generate shape and pose output
  autoware_perception_msgs::msg::Shape shape_output;
  geometry_msgs::msg::Pose pose_output;

  // Test estimateShapeAndPose
  const bool result = shape_estimator.estimateShapeAndPose(
    label, cluster, ref_yaw_info, ref_shape_size_info, shape_output, pose_output);
  EXPECT_TRUE(result);

  // Check shape_output
  EXPECT_EQ(shape_output.type, autoware_perception_msgs::msg::Shape::BOUNDING_BOX);
  EXPECT_NEAR(shape_output.dimensions.x, length, length * 0.1);
  EXPECT_NEAR(shape_output.dimensions.y, width, width * 0.1);
  EXPECT_NEAR(shape_output.dimensions.z, height, height * 0.1);

  // Check pose_output
  EXPECT_NEAR(pose_output.position.x, offset_x, 1.0);
  EXPECT_NEAR(pose_output.position.y, offset_y, 1.0);
  EXPECT_NEAR(pose_output.position.z, height / 2, 1.0);

  // transform quaternion to yaw
  const double pose_output_yaw = yawFromQuaternion(pose_output.orientation);
  EXPECT_NEAR(pose_output_yaw, yaw, deg2rad(15.0));
}
