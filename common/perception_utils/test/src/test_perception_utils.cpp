// Copyright 2020 TIER IV, Inc.
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

#include "perception_utils/perception_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <gtest/gtest.h>

using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Point3d;

constexpr double epsilon = 1e-06;

namespace
{
geometry_msgs::msg::Point32 createPoint32(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;

  return p;
}

geometry_msgs::msg::Pose createPose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = 0.0;
  p.orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);

  return p;
}

autoware_auto_perception_msgs::msg::ObjectClassification createObjectClassification(
  const std::uint8_t label, const double probability)
{
  autoware_auto_perception_msgs::msg::ObjectClassification classification;
  classification.label = label;
  classification.probability = probability;

  return classification;
}
}  // namespace

TEST(perception_utils, test_getHighestProbLabel)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using perception_utils::getHighestProbLabel;

  {  // empty
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
    std::uint8_t label = getHighestProbLabel(classification);
    EXPECT_EQ(label, ObjectClassification::UNKNOWN);
  }

  {  // normal case
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
    classification.push_back(createObjectClassification(ObjectClassification::CAR, 0.5));
    classification.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classification.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    std::uint8_t label = getHighestProbLabel(classification);
    EXPECT_EQ(label, ObjectClassification::TRUCK);
  }

  {  // labels with the same probability
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
    classification.push_back(createObjectClassification(ObjectClassification::CAR, 0.8));
    classification.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classification.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    std::uint8_t label = getHighestProbLabel(classification);
    EXPECT_EQ(label, ObjectClassification::CAR);
  }
}

// NOTE: covariance is not checked
TEST(perception_utils, test_toDetectedObject)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using perception_utils::toDetectedObject;

  autoware_auto_perception_msgs::msg::TrackedObject tracked_obj;
  // existence probability
  tracked_obj.existence_probability = 1.0;
  // classification
  tracked_obj.classification.push_back(createObjectClassification(ObjectClassification::CAR, 0.8));
  tracked_obj.classification.push_back(
    createObjectClassification(ObjectClassification::TRUCK, 0.8));
  tracked_obj.classification.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));
  // kinematics
  tracked_obj.kinematics.pose_with_covariance.pose.position.x = 1.0;
  tracked_obj.kinematics.pose_with_covariance.pose.position.y = 2.0;
  tracked_obj.kinematics.pose_with_covariance.pose.position.z = 3.0;
  tracked_obj.kinematics.pose_with_covariance.pose.orientation.x = 4.0;
  tracked_obj.kinematics.pose_with_covariance.pose.orientation.y = 5.0;
  tracked_obj.kinematics.pose_with_covariance.pose.orientation.z = 6.0;
  tracked_obj.kinematics.pose_with_covariance.pose.orientation.w = 7.0;
  tracked_obj.kinematics.twist_with_covariance.twist.linear.x = 1.0;
  tracked_obj.kinematics.twist_with_covariance.twist.linear.y = 2.0;
  tracked_obj.kinematics.twist_with_covariance.twist.linear.z = 3.0;
  tracked_obj.kinematics.twist_with_covariance.twist.angular.x = 4.0;
  tracked_obj.kinematics.twist_with_covariance.twist.angular.y = 5.0;
  tracked_obj.kinematics.twist_with_covariance.twist.angular.z = 6.0;
  tracked_obj.kinematics.orientation_availability = 1;
  // shape
  tracked_obj.shape.type = 1;
  tracked_obj.shape.dimensions.x = 1.0;
  tracked_obj.shape.dimensions.y = 2.0;
  tracked_obj.shape.dimensions.z = 3.0;
  tracked_obj.shape.footprint.points.push_back(createPoint32(1.0, 2.0, 3.0));
  tracked_obj.shape.footprint.points.push_back(createPoint32(2.0, 4.0, 6.0));

  const auto detected_obj = toDetectedObject(tracked_obj);

  // existence probability
  EXPECT_DOUBLE_EQ(tracked_obj.existence_probability, detected_obj.existence_probability);
  // classification
  EXPECT_EQ(tracked_obj.classification.size(), detected_obj.classification.size());
  EXPECT_EQ(tracked_obj.classification.at(0).label, detected_obj.classification.at(0).label);
  EXPECT_DOUBLE_EQ(
    tracked_obj.classification.at(0).probability, detected_obj.classification.at(0).probability);
  EXPECT_EQ(tracked_obj.classification.at(1).label, detected_obj.classification.at(1).label);
  EXPECT_DOUBLE_EQ(
    tracked_obj.classification.at(1).probability, detected_obj.classification.at(1).probability);
  EXPECT_EQ(tracked_obj.classification.at(2).label, detected_obj.classification.at(2).label);
  EXPECT_DOUBLE_EQ(
    tracked_obj.classification.at(2).probability, detected_obj.classification.at(2).probability);
  // kinematics
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.pose_with_covariance.pose.position.x,
    detected_obj.kinematics.pose_with_covariance.pose.position.x);
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.pose_with_covariance.pose.position.y,
    detected_obj.kinematics.pose_with_covariance.pose.position.y);
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.pose_with_covariance.pose.position.z,
    detected_obj.kinematics.pose_with_covariance.pose.position.z);
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.pose_with_covariance.pose.orientation.x,
    detected_obj.kinematics.pose_with_covariance.pose.orientation.x);
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.pose_with_covariance.pose.orientation.y,
    detected_obj.kinematics.pose_with_covariance.pose.orientation.y);
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.pose_with_covariance.pose.orientation.z,
    detected_obj.kinematics.pose_with_covariance.pose.orientation.z);
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.pose_with_covariance.pose.orientation.w,
    detected_obj.kinematics.pose_with_covariance.pose.orientation.w);
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.twist_with_covariance.twist.linear.x,
    detected_obj.kinematics.twist_with_covariance.twist.linear.x);
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.twist_with_covariance.twist.linear.y,
    detected_obj.kinematics.twist_with_covariance.twist.linear.y);
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.twist_with_covariance.twist.linear.z,
    detected_obj.kinematics.twist_with_covariance.twist.linear.z);
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.twist_with_covariance.twist.angular.x,
    detected_obj.kinematics.twist_with_covariance.twist.angular.x);
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.twist_with_covariance.twist.angular.y,
    detected_obj.kinematics.twist_with_covariance.twist.angular.y);
  EXPECT_DOUBLE_EQ(
    tracked_obj.kinematics.twist_with_covariance.twist.angular.z,
    detected_obj.kinematics.twist_with_covariance.twist.angular.z);
  EXPECT_EQ(
    tracked_obj.kinematics.orientation_availability,
    detected_obj.kinematics.orientation_availability);
  EXPECT_EQ(detected_obj.kinematics.has_position_covariance, true);
  EXPECT_EQ(detected_obj.kinematics.has_twist, true);
  EXPECT_EQ(detected_obj.kinematics.has_twist_covariance, true);
  // shape
  EXPECT_EQ(tracked_obj.shape.type, detected_obj.shape.type);
  EXPECT_DOUBLE_EQ(tracked_obj.shape.dimensions.x, detected_obj.shape.dimensions.x);
  EXPECT_DOUBLE_EQ(tracked_obj.shape.dimensions.y, detected_obj.shape.dimensions.y);
  EXPECT_DOUBLE_EQ(tracked_obj.shape.dimensions.z, detected_obj.shape.dimensions.z);
  EXPECT_EQ(tracked_obj.shape.footprint.points.size(), detected_obj.shape.footprint.points.size());
  EXPECT_DOUBLE_EQ(
    tracked_obj.shape.footprint.points.at(0).x, detected_obj.shape.footprint.points.at(0).x);
  EXPECT_DOUBLE_EQ(
    tracked_obj.shape.footprint.points.at(0).y, detected_obj.shape.footprint.points.at(0).y);
  EXPECT_DOUBLE_EQ(
    tracked_obj.shape.footprint.points.at(0).z, detected_obj.shape.footprint.points.at(0).z);
  EXPECT_DOUBLE_EQ(
    tracked_obj.shape.footprint.points.at(1).x, detected_obj.shape.footprint.points.at(1).x);
  EXPECT_DOUBLE_EQ(
    tracked_obj.shape.footprint.points.at(1).y, detected_obj.shape.footprint.points.at(1).y);
  EXPECT_DOUBLE_EQ(
    tracked_obj.shape.footprint.points.at(1).z, detected_obj.shape.footprint.points.at(1).z);
}

// NOTE: covariance is not checked
TEST(perception_utils, test_toTrackedObject)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using perception_utils::toTrackedObject;

  autoware_auto_perception_msgs::msg::DetectedObject detected_obj;
  // existence probability
  detected_obj.existence_probability = 1.0;
  // classification
  detected_obj.classification.push_back(createObjectClassification(ObjectClassification::CAR, 0.8));
  detected_obj.classification.push_back(
    createObjectClassification(ObjectClassification::TRUCK, 0.8));
  detected_obj.classification.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));
  // kinematics
  detected_obj.kinematics.pose_with_covariance.pose.position.x = 1.0;
  detected_obj.kinematics.pose_with_covariance.pose.position.y = 2.0;
  detected_obj.kinematics.pose_with_covariance.pose.position.z = 3.0;
  detected_obj.kinematics.pose_with_covariance.pose.orientation.x = 4.0;
  detected_obj.kinematics.pose_with_covariance.pose.orientation.y = 5.0;
  detected_obj.kinematics.pose_with_covariance.pose.orientation.z = 6.0;
  detected_obj.kinematics.pose_with_covariance.pose.orientation.w = 7.0;
  detected_obj.kinematics.twist_with_covariance.twist.linear.x = 1.0;
  detected_obj.kinematics.twist_with_covariance.twist.linear.y = 2.0;
  detected_obj.kinematics.twist_with_covariance.twist.linear.z = 3.0;
  detected_obj.kinematics.twist_with_covariance.twist.angular.x = 4.0;
  detected_obj.kinematics.twist_with_covariance.twist.angular.y = 5.0;
  detected_obj.kinematics.twist_with_covariance.twist.angular.z = 6.0;
  detected_obj.kinematics.orientation_availability = 1;
  // shape
  detected_obj.shape.type = 1;
  detected_obj.shape.dimensions.x = 1.0;
  detected_obj.shape.dimensions.y = 2.0;
  detected_obj.shape.dimensions.z = 3.0;
  detected_obj.shape.footprint.points.push_back(createPoint32(1.0, 2.0, 3.0));
  detected_obj.shape.footprint.points.push_back(createPoint32(2.0, 4.0, 6.0));

  const auto tracked_obj = toTrackedObject(detected_obj);

  // existence probability
  EXPECT_DOUBLE_EQ(detected_obj.existence_probability, tracked_obj.existence_probability);
  // classification
  EXPECT_EQ(detected_obj.classification.size(), tracked_obj.classification.size());
  EXPECT_EQ(detected_obj.classification.at(0).label, tracked_obj.classification.at(0).label);
  EXPECT_DOUBLE_EQ(
    detected_obj.classification.at(0).probability, tracked_obj.classification.at(0).probability);
  EXPECT_EQ(detected_obj.classification.at(1).label, tracked_obj.classification.at(1).label);
  EXPECT_DOUBLE_EQ(
    detected_obj.classification.at(1).probability, tracked_obj.classification.at(1).probability);
  EXPECT_EQ(detected_obj.classification.at(2).label, tracked_obj.classification.at(2).label);
  EXPECT_DOUBLE_EQ(
    detected_obj.classification.at(2).probability, tracked_obj.classification.at(2).probability);
  // kinematics
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.pose_with_covariance.pose.position.x,
    tracked_obj.kinematics.pose_with_covariance.pose.position.x);
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.pose_with_covariance.pose.position.y,
    tracked_obj.kinematics.pose_with_covariance.pose.position.y);
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.pose_with_covariance.pose.position.z,
    tracked_obj.kinematics.pose_with_covariance.pose.position.z);
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.pose_with_covariance.pose.orientation.x,
    tracked_obj.kinematics.pose_with_covariance.pose.orientation.x);
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.pose_with_covariance.pose.orientation.y,
    tracked_obj.kinematics.pose_with_covariance.pose.orientation.y);
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.pose_with_covariance.pose.orientation.z,
    tracked_obj.kinematics.pose_with_covariance.pose.orientation.z);
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.pose_with_covariance.pose.orientation.w,
    tracked_obj.kinematics.pose_with_covariance.pose.orientation.w);
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.twist_with_covariance.twist.linear.x,
    tracked_obj.kinematics.twist_with_covariance.twist.linear.x);
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.twist_with_covariance.twist.linear.y,
    tracked_obj.kinematics.twist_with_covariance.twist.linear.y);
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.twist_with_covariance.twist.linear.z,
    tracked_obj.kinematics.twist_with_covariance.twist.linear.z);
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.twist_with_covariance.twist.angular.x,
    tracked_obj.kinematics.twist_with_covariance.twist.angular.x);
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.twist_with_covariance.twist.angular.y,
    tracked_obj.kinematics.twist_with_covariance.twist.angular.y);
  EXPECT_DOUBLE_EQ(
    detected_obj.kinematics.twist_with_covariance.twist.angular.z,
    tracked_obj.kinematics.twist_with_covariance.twist.angular.z);
  EXPECT_EQ(
    detected_obj.kinematics.orientation_availability,
    tracked_obj.kinematics.orientation_availability);
  // shape
  EXPECT_EQ(detected_obj.shape.type, tracked_obj.shape.type);
  EXPECT_DOUBLE_EQ(detected_obj.shape.dimensions.x, tracked_obj.shape.dimensions.x);
  EXPECT_DOUBLE_EQ(detected_obj.shape.dimensions.y, tracked_obj.shape.dimensions.y);
  EXPECT_DOUBLE_EQ(detected_obj.shape.dimensions.z, tracked_obj.shape.dimensions.z);
  EXPECT_EQ(detected_obj.shape.footprint.points.size(), tracked_obj.shape.footprint.points.size());
  EXPECT_DOUBLE_EQ(
    detected_obj.shape.footprint.points.at(0).x, tracked_obj.shape.footprint.points.at(0).x);
  EXPECT_DOUBLE_EQ(
    detected_obj.shape.footprint.points.at(0).y, tracked_obj.shape.footprint.points.at(0).y);
  EXPECT_DOUBLE_EQ(
    detected_obj.shape.footprint.points.at(0).z, tracked_obj.shape.footprint.points.at(0).z);
  EXPECT_DOUBLE_EQ(
    detected_obj.shape.footprint.points.at(1).x, tracked_obj.shape.footprint.points.at(1).x);
  EXPECT_DOUBLE_EQ(
    detected_obj.shape.footprint.points.at(1).y, tracked_obj.shape.footprint.points.at(1).y);
  EXPECT_DOUBLE_EQ(
    detected_obj.shape.footprint.points.at(1).z, tracked_obj.shape.footprint.points.at(1).z);
}

TEST(perception_utils, test_get2dIoU)
{
  using perception_utils::get2dIoU;
  const double quart_circle = 0.16237976320958225;

  {  // non overlapped
    autoware_auto_perception_msgs::msg::DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(1.5, 1.5, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    autoware_auto_perception_msgs::msg::DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double iou = get2dIoU(source_obj, target_obj);
    EXPECT_DOUBLE_EQ(iou, 0.0);
  }

  {  // partially overlapped
    autoware_auto_perception_msgs::msg::DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.5, 0.5, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    autoware_auto_perception_msgs::msg::DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double iou = get2dIoU(source_obj, target_obj);
    EXPECT_NEAR(iou, quart_circle / (1.0 + quart_circle * 3), epsilon);
  }

  {  // fully overlapped
    autoware_auto_perception_msgs::msg::DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    autoware_auto_perception_msgs::msg::DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double iou = get2dIoU(source_obj, target_obj);
    EXPECT_DOUBLE_EQ(iou, quart_circle * 4);
  }
}

TEST(perception_utils, test_get2dGeneralizedIoU)
{
  using perception_utils::get2dGeneralizedIoU;
  // TODO(Shin-kyoto):
  // get2dGeneralizedIoU uses outer points of each polygon.
  // But these points contain an sampling error of outer line,
  // so get2dGeneralizedIoU icludes an error of about 0.03.
  // Threfore, in this test, epsilon is set to 0.04.
  constexpr double epsilon_giou = 4 * 1e-02;
  const double quart_circle = 0.16237976320958225;

  {  // non overlapped
    autoware_auto_perception_msgs::msg::DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(1.5, 0.0, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    autoware_auto_perception_msgs::msg::DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double giou = get2dGeneralizedIoU(source_obj, target_obj);
    EXPECT_NEAR(giou, (2 * quart_circle - 1) / (2 * quart_circle + 2), epsilon_giou);  // not 0
  }

  {  // partially overlapped
    autoware_auto_perception_msgs::msg::DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.5, 0.0, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    autoware_auto_perception_msgs::msg::DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double giou = get2dGeneralizedIoU(source_obj, target_obj);
    EXPECT_NEAR(giou, 2 * quart_circle / (2 * quart_circle + 1), epsilon_giou);
  }

  {  // fully overlapped
    autoware_auto_perception_msgs::msg::DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    autoware_auto_perception_msgs::msg::DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double giou = get2dGeneralizedIoU(source_obj, target_obj);
    EXPECT_NEAR(giou, quart_circle * 4, epsilon_giou);  // giou equals iou
  }
}

TEST(perception_utils, test_get2dPrecision)
{
  using perception_utils::get2dPrecision;
  const double quart_circle = 0.16237976320958225;

  {  // non overlapped
    autoware_auto_perception_msgs::msg::DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(1.5, 1.5, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    autoware_auto_perception_msgs::msg::DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double precision = get2dPrecision(source_obj, target_obj);
    EXPECT_DOUBLE_EQ(precision, 0.0);

    // reverse source and target object
    const double reversed_precision = get2dPrecision(target_obj, source_obj);
    EXPECT_DOUBLE_EQ(reversed_precision, 0.0);
  }

  {  // partially overlapped
    autoware_auto_perception_msgs::msg::DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.5, 0.5, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    autoware_auto_perception_msgs::msg::DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double precision = get2dPrecision(source_obj, target_obj);
    EXPECT_NEAR(precision, quart_circle, epsilon);

    // reverse source and target object
    const double reversed_precision = get2dPrecision(target_obj, source_obj);
    EXPECT_NEAR(reversed_precision, 1 / 4.0, epsilon);
  }

  {  // fully overlapped
    autoware_auto_perception_msgs::msg::DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    autoware_auto_perception_msgs::msg::DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double precision = get2dPrecision(source_obj, target_obj);
    EXPECT_DOUBLE_EQ(precision, quart_circle * 4);

    // reverse source and target object
    const double reversed_precision = get2dPrecision(target_obj, source_obj);
    EXPECT_DOUBLE_EQ(reversed_precision, 1.0);
  }
}

TEST(perception_utils, test_get2dRecall)
{
  using perception_utils::get2dRecall;
  const double quart_circle = 0.16237976320958225;

  {  // non overlapped
    autoware_auto_perception_msgs::msg::DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(1.5, 1.5, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    autoware_auto_perception_msgs::msg::DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double recall = get2dRecall(source_obj, target_obj);
    EXPECT_DOUBLE_EQ(recall, 0.0);

    // reverse source and target object
    const double reversed_recall = get2dRecall(target_obj, source_obj);
    EXPECT_DOUBLE_EQ(reversed_recall, 0.0);
  }

  {  // partially overlapped
    autoware_auto_perception_msgs::msg::DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.5, 0.5, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    autoware_auto_perception_msgs::msg::DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double recall = get2dRecall(source_obj, target_obj);
    EXPECT_NEAR(recall, 1 / 4.0, epsilon);

    // reverse source and target object
    const double reversed_recall = get2dRecall(target_obj, source_obj);
    EXPECT_NEAR(reversed_recall, quart_circle, epsilon);
  }

  {  // fully overlapped
    autoware_auto_perception_msgs::msg::DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    autoware_auto_perception_msgs::msg::DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double recall = get2dRecall(source_obj, target_obj);
    EXPECT_DOUBLE_EQ(recall, 1.0);

    // reverse source and target object
    const double reversed_recall = get2dRecall(target_obj, source_obj);
    EXPECT_DOUBLE_EQ(reversed_recall, quart_circle * 4);
  }
}
