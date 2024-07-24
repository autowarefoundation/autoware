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

#include "autoware/image_projection_based_fusion/utils/geometry.hpp"

#include <gtest/gtest.h>

TEST(objectToVertices, test_objectToVertices)
{
  // Test `boundingBoxToVertices()` and `cylinderToVertices()` simultaneously
  // Test at Shape::BOUNDING_BOX
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 3.0;
  const double angle = M_PI / 12;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(angle);
  pose.orientation.w = std::cos(angle);
  {
    autoware_perception_msgs::msg::Shape shape;
    shape.type = 0;
    shape.dimensions.x = 4.0;
    shape.dimensions.y = 6.0;
    shape.dimensions.z = 8.0;
    std::vector<Eigen::Vector3d> vertices;

    autoware::image_projection_based_fusion::objectToVertices(pose, shape, vertices);

    EXPECT_FALSE(vertices.empty());
    EXPECT_NEAR(vertices.at(0).x(), 1.2320508075688772935274, 1e-6);
    EXPECT_NEAR(vertices.at(0).y(), 5.598076211353315940291, 1e-6);
    EXPECT_NEAR(vertices.at(0).z(), -1.0, 1e-6);
    EXPECT_NEAR(vertices.at(7).x(), -2.232050807568877293527, 1e-6);
    EXPECT_NEAR(vertices.at(7).y(), 3.598076211353315940291, 1e-6);
    EXPECT_NEAR(vertices.at(7).z(), 7.0, 1e-6);
  }

  {
    // Test at Shape::CYLINDER
    autoware_perception_msgs::msg::Shape shape;
    shape.type = 1;
    shape.dimensions.x = 4.0;
    shape.dimensions.y = 6.0;
    shape.dimensions.z = 8.0;
    std::vector<Eigen::Vector3d> vertices;

    autoware::image_projection_based_fusion::objectToVertices(pose, shape, vertices);

    EXPECT_FALSE(vertices.empty());
    EXPECT_NEAR(vertices.at(0).x(), 2.732050807568877293528, 1e-6);
    EXPECT_NEAR(vertices.at(0).y(), 3.0, 1e-6);
    EXPECT_NEAR(vertices.at(0).z(), 7.0, 1e-6);
    EXPECT_NEAR(vertices.at(11).x(), 2.732050807568877293528, 1e-6);
    EXPECT_NEAR(vertices.at(11).y(), 1.0, 1e-6);
    EXPECT_NEAR(vertices.at(11).z(), -1.0, 1e-6);
  }

  {
    // Test at Shape::POLYGON (Nothing to do)
    autoware_perception_msgs::msg::Shape shape;
    shape.type = 2;
    std::vector<Eigen::Vector3d> vertices;

    autoware::image_projection_based_fusion::objectToVertices(pose, shape, vertices);

    EXPECT_TRUE(vertices.empty());
  }
}

TEST(transformPoints, test_transformPoints)
{
  std::vector<Eigen::Vector3d> input_points;
  Eigen::Vector3d point(0.0, 0.0, 0.0);
  input_points.push_back(point);
  Eigen::Translation<double, 3> translation(1.0, 1.0, 1.0);
  Eigen::Matrix3d rotation = (Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()))
                               .toRotationMatrix();
  Eigen::Affine3d affine_transform = rotation * translation;
  std::vector<Eigen::Vector3d> output_points;

  autoware::image_projection_based_fusion::transformPoints(
    input_points, affine_transform, output_points);

  EXPECT_FALSE(output_points.empty());
  EXPECT_NEAR(output_points.at(0).x(), 0.7071067811865475244008, 1e-6);
  EXPECT_NEAR(output_points.at(0).y(), 0.5, 1e-6);
  EXPECT_NEAR(output_points.at(0).z(), 1.5, 1e-6);
}

TEST(is_inside, test_is_inside)
{
  // Test default pattern
  sensor_msgs::msg::RegionOfInterest outer;
  outer.x_offset = 30;
  outer.y_offset = 40;
  outer.height = 400;
  outer.width = 300;
  const double outer_offset_scale = 1.0;
  {
    sensor_msgs::msg::RegionOfInterest inner;
    inner.x_offset = 31;
    inner.y_offset = 41;
    inner.height = 399;
    inner.width = 299;

    const bool inside_flag =
      autoware::image_projection_based_fusion::is_inside(outer, inner, outer_offset_scale);

    EXPECT_TRUE(inside_flag);
  }

  {
    // Test left-top outside pattern
    sensor_msgs::msg::RegionOfInterest inner;
    inner.x_offset = 29;
    inner.y_offset = 39;

    const bool inside_flag =
      autoware::image_projection_based_fusion::is_inside(outer, inner, outer_offset_scale);

    EXPECT_FALSE(inside_flag);
  }

  {
    // Test right-bottom outside pattern
    sensor_msgs::msg::RegionOfInterest inner;
    inner.x_offset = 31;
    inner.y_offset = 41;
    inner.height = 401;
    inner.width = 301;

    const bool inside_flag =
      autoware::image_projection_based_fusion::is_inside(outer, inner, outer_offset_scale);

    EXPECT_FALSE(inside_flag);
  }
}

TEST(sanitizeROI, test_sanitizeROI)
{
  {
    // Test default pattern
    sensor_msgs::msg::RegionOfInterest roi;
    roi.x_offset = 10;
    roi.y_offset = 20;
    roi.height = 200;
    roi.width = 100;
    int height = 400;  // image height
    int width = 300;   // image width

    autoware::image_projection_based_fusion::sanitizeROI(roi, width, height);

    EXPECT_EQ(roi.height, 200);
    EXPECT_EQ(roi.width, 100);
  }

  {
    // Test pattern that x_offset or y_offset is not in image
    sensor_msgs::msg::RegionOfInterest roi;
    roi.x_offset = 100;
    roi.y_offset = 200;
    int height = 100;
    int width = 50;

    autoware::image_projection_based_fusion::sanitizeROI(roi, width, height);

    EXPECT_EQ(roi.height, 0);
    EXPECT_EQ(roi.width, 0);
  }

  {
    // Test patten that roi does not fit within image
    sensor_msgs::msg::RegionOfInterest roi;
    roi.x_offset = 10;
    roi.y_offset = 20;
    roi.height = 500;
    roi.width = 400;
    int height = 100;
    int width = 50;

    autoware::image_projection_based_fusion::sanitizeROI(roi, width, height);

    EXPECT_EQ(roi.height, 80);
    EXPECT_EQ(roi.width, 40);
  }
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
