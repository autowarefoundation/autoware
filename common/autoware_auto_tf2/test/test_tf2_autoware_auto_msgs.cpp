// Copyright 2020, The Autoware Foundation.
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
/// \file
/// \brief This file includes common transform functionally for autoware_auto_msgs

#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <rclcpp/clock.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

std::unique_ptr<tf2_ros::Buffer> tf_buffer = nullptr;
constexpr double EPS = 1e-3;

geometry_msgs::msg::TransformStamped filled_transform()
{
  geometry_msgs::msg::TransformStamped t;
  t.transform.translation.x = 10;
  t.transform.translation.y = 20;
  t.transform.translation.z = 30;
  t.transform.rotation.w = 0;
  t.transform.rotation.x = 1;
  t.transform.rotation.y = 0;
  t.transform.rotation.z = 0;
  t.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
  t.header.frame_id = "A";
  t.child_frame_id = "B";

  return t;
}

TEST(Tf2AutowareAuto, DoTransformPoint32)
{
  const auto trans = filled_transform();
  geometry_msgs::msg::Point32 p1;
  p1.x = 1;
  p1.y = 2;
  p1.z = 3;

  // doTransform
  geometry_msgs::msg::Point32 p_out;
  tf2::doTransform(p1, p_out, trans);

  EXPECT_NEAR(p_out.x, 11, EPS);
  EXPECT_NEAR(p_out.y, 18, EPS);
  EXPECT_NEAR(p_out.z, 27, EPS);
}

TEST(Tf2AutowareAuto, DoTransformPolygon)
{
  const auto trans = filled_transform();
  geometry_msgs::msg::Polygon poly;
  geometry_msgs::msg::Point32 p1;
  p1.x = 1;
  p1.y = 2;
  p1.z = 3;
  poly.points.push_back(p1);
  // doTransform
  geometry_msgs::msg::Polygon poly_out;
  tf2::doTransform(poly, poly_out, trans);

  EXPECT_NEAR(poly_out.points[0].x, 11, EPS);
  EXPECT_NEAR(poly_out.points[0].y, 18, EPS);
  EXPECT_NEAR(poly_out.points[0].z, 27, EPS);
}

TEST(Tf2AutowareAuto, DoTransformQuaternion32)
{
  const auto trans = filled_transform();
  autoware_auto_geometry_msgs::msg::Quaternion32 q1;
  q1.w = 0;
  q1.x = 0;
  q1.y = 0;
  q1.z = 1;

  // doTransform
  autoware_auto_geometry_msgs::msg::Quaternion32 q_out;
  tf2::doTransform(q1, q_out, trans);

  EXPECT_NEAR(q_out.x, 0.0, EPS);
  EXPECT_NEAR(q_out.y, 1.0, EPS);
  EXPECT_NEAR(q_out.z, 0.0, EPS);
  EXPECT_NEAR(q_out.w, 0.0, EPS);
}

TEST(Tf2AutowareAuto, DoTransformBoundingBox)
{
  const auto trans = filled_transform();
  BoundingBox bb1;
  bb1.orientation.w = 0;
  bb1.orientation.x = 0;
  bb1.orientation.y = 0;
  bb1.orientation.z = 1;
  bb1.centroid.x = 1;
  bb1.centroid.y = 2;
  bb1.centroid.z = 3;
  bb1.corners[0].x = 4;
  bb1.corners[0].y = 5;
  bb1.corners[0].z = 6;
  bb1.corners[1].x = 7;
  bb1.corners[1].y = 8;
  bb1.corners[1].z = 9;
  bb1.corners[2].x = 10;
  bb1.corners[2].y = 11;
  bb1.corners[2].z = 12;
  bb1.corners[3].x = 13;
  bb1.corners[3].y = 14;
  bb1.corners[3].z = 15;

  // doTransform
  BoundingBox bb_out;
  tf2::doTransform(bb1, bb_out, trans);

  EXPECT_NEAR(bb_out.orientation.x, 0.0, EPS);
  EXPECT_NEAR(bb_out.orientation.y, 1.0, EPS);
  EXPECT_NEAR(bb_out.orientation.z, 0.0, EPS);
  EXPECT_NEAR(bb_out.orientation.w, 0.0, EPS);
  EXPECT_NEAR(bb_out.centroid.x, 11, EPS);
  EXPECT_NEAR(bb_out.centroid.y, 18, EPS);
  EXPECT_NEAR(bb_out.centroid.z, 27, EPS);
  EXPECT_NEAR(bb_out.corners[0].x, 14, EPS);
  EXPECT_NEAR(bb_out.corners[0].y, 15, EPS);
  EXPECT_NEAR(bb_out.corners[0].z, 24, EPS);
  EXPECT_NEAR(bb_out.corners[1].x, 17, EPS);
  EXPECT_NEAR(bb_out.corners[1].y, 12, EPS);
  EXPECT_NEAR(bb_out.corners[1].z, 21, EPS);
  EXPECT_NEAR(bb_out.corners[2].x, 20, EPS);
  EXPECT_NEAR(bb_out.corners[2].y, 9, EPS);
  EXPECT_NEAR(bb_out.corners[2].z, 18, EPS);
  EXPECT_NEAR(bb_out.corners[3].x, 23, EPS);
  EXPECT_NEAR(bb_out.corners[3].y, 6, EPS);
  EXPECT_NEAR(bb_out.corners[3].z, 15, EPS);

  // Testing unused fields are unmodified
  EXPECT_EQ(bb_out.size, bb1.size);
  EXPECT_EQ(bb_out.heading, bb1.heading);
  EXPECT_EQ(bb_out.heading_rate, bb1.heading_rate);
  EXPECT_EQ(bb_out.variance, bb1.variance);
  EXPECT_EQ(bb_out.vehicle_label, bb1.vehicle_label);
  EXPECT_EQ(bb_out.signal_label, bb1.signal_label);
  EXPECT_EQ(bb_out.class_likelihood, bb1.class_likelihood);
}

TEST(Tf2AutowareAuto, TransformBoundingBoxArray)
{
  BoundingBox bb1;
  bb1.orientation.w = 0;
  bb1.orientation.x = 0;
  bb1.orientation.y = 0;
  bb1.orientation.z = 1;
  bb1.centroid.x = 20;
  bb1.centroid.y = 21;
  bb1.centroid.z = 22;
  bb1.corners[0].x = 23;
  bb1.corners[0].y = 24;
  bb1.corners[0].z = 25;
  bb1.corners[1].x = 26;
  bb1.corners[1].y = 27;
  bb1.corners[1].z = 28;
  bb1.corners[2].x = 29;
  bb1.corners[2].y = 30;
  bb1.corners[2].z = 31;
  bb1.corners[3].x = 32;
  bb1.corners[3].y = 33;
  bb1.corners[3].z = 34;

  BoundingBox bb2;
  bb2.orientation.w = 0.707f;
  bb2.orientation.x = -0.706f;
  bb2.orientation.y = 0;
  bb2.orientation.z = 0;
  bb2.centroid.x = 50;
  bb2.centroid.y = 51;
  bb2.centroid.z = 52;
  bb2.corners[0].x = 53;
  bb2.corners[0].y = 54;
  bb2.corners[0].z = 55;
  bb2.corners[1].x = 56;
  bb2.corners[1].y = 57;
  bb2.corners[1].z = 58;
  bb2.corners[2].x = 59;
  bb2.corners[2].y = 50;
  bb2.corners[2].z = 51;
  bb2.corners[3].x = 52;
  bb2.corners[3].y = 53;
  bb2.corners[3].z = 54;

  BoundingBoxArray bba1;
  bba1.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
  bba1.header.frame_id = "A";
  bba1.boxes.push_back(bb1);
  bba1.boxes.push_back(bb2);

  // simple api
  const auto bba_simple = tf_buffer->transform(bba1, "B", tf2::durationFromSec(2.0));

  EXPECT_EQ(bba_simple.header.frame_id, "B");

  // checking boxes[0]
  EXPECT_NEAR(bba_simple.boxes[0].orientation.x, 0.0, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].orientation.y, 1.0, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].orientation.z, 0.0, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].orientation.w, 0.0, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].centroid.x, 10, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].centroid.y, -1, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].centroid.z, 8, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].corners[0].x, 13, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].corners[0].y, -4, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].corners[0].z, 5, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].corners[1].x, 16, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].corners[1].y, -7, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].corners[1].z, 2, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].corners[2].x, 19, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].corners[2].y, -10, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].corners[2].z, -1, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].corners[3].x, 22, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].corners[3].y, -13, EPS);
  EXPECT_NEAR(bba_simple.boxes[0].corners[3].z, -4, EPS);

  // checking boxes[1]
  EXPECT_NEAR(bba_simple.boxes[1].orientation.x, 0.707, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].orientation.y, 0.0, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].orientation.z, 0.0, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].orientation.w, 0.707, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].centroid.x, 40, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].centroid.y, -31, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].centroid.z, -22, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].corners[0].x, 43, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].corners[0].y, -34, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].corners[0].z, -25, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].corners[1].x, 46, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].corners[1].y, -37, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].corners[1].z, -28, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].corners[2].x, 49, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].corners[2].y, -30, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].corners[2].z, -21, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].corners[3].x, 42, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].corners[3].y, -33, EPS);
  EXPECT_NEAR(bba_simple.boxes[1].corners[3].z, -24, EPS);

  // advanced api
  const auto bba_advanced =
    tf_buffer->transform(bba1, "B", tf2::timeFromSec(2.0), "A", tf2::durationFromSec(3.0));

  EXPECT_EQ(bba_advanced.header.frame_id, "B");

  // checking boxes[0]
  EXPECT_NEAR(bba_advanced.boxes[0].orientation.x, 0.0, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].orientation.y, 1.0, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].orientation.z, 0.0, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].orientation.w, 0.0, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].centroid.x, 10, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].centroid.y, -1, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].centroid.z, 8, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].corners[0].x, 13, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].corners[0].y, -4, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].corners[0].z, 5, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].corners[1].x, 16, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].corners[1].y, -7, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].corners[1].z, 2, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].corners[2].x, 19, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].corners[2].y, -10, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].corners[2].z, -1, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].corners[3].x, 22, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].corners[3].y, -13, EPS);
  EXPECT_NEAR(bba_advanced.boxes[0].corners[3].z, -4, EPS);

  // checking boxes[1]
  EXPECT_NEAR(bba_advanced.boxes[1].orientation.x, 0.707, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].orientation.y, 0.0, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].orientation.z, 0.0, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].orientation.w, 0.707, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].centroid.x, 40, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].centroid.y, -31, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].centroid.z, -22, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].corners[0].x, 43, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].corners[0].y, -34, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].corners[0].z, -25, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].corners[1].x, 46, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].corners[1].y, -37, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].corners[1].z, -28, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].corners[2].x, 49, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].corners[2].y, -30, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].corners[2].z, -21, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].corners[3].x, 42, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].corners[3].y, -33, EPS);
  EXPECT_NEAR(bba_advanced.boxes[1].corners[3].z, -24, EPS);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf_buffer = std::make_unique<tf2_ros::Buffer>(clock);
  tf_buffer->setUsingDedicatedThread(true);

  // populate buffer
  const geometry_msgs::msg::TransformStamped t = filled_transform();
  tf_buffer->setTransform(t, "test");

  int ret = RUN_ALL_TESTS();
  return ret;
}
