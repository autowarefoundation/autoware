// Copyright 2022 TIER IV, Inc.
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

#include "perception_utils/matching.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <autoware_auto_perception_msgs/msg/detected_object.hpp>

#include <gtest/gtest.h>

using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Point3d;

constexpr double epsilon = 1e-06;

namespace
{
geometry_msgs::msg::Pose createPose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position = geometry_msgs::build<geometry_msgs::msg::Point>().x(x).y(y).z(0.0);
  p.orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);
  return p;
}
}  // namespace

TEST(matching, test_get2dIoU)
{
  using autoware_auto_perception_msgs::msg::DetectedObject;
  using perception_utils::get2dIoU;

  const double quart_circle = 0.16237976320958225;

  {  // non overlapped
    DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(1.5, 1.5, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double iou = get2dIoU(source_obj, target_obj);
    EXPECT_DOUBLE_EQ(iou, 0.0);
  }

  {  // partially overlapped
    DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.5, 0.5, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    DetectedObject target_obj;
    target_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI_2);
    target_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    target_obj.shape.dimensions.x = 1.0;

    const double iou = get2dIoU(source_obj, target_obj);
    EXPECT_NEAR(iou, quart_circle / (1.0 + quart_circle * 3), epsilon);
  }

  {  // fully overlapped
    DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    DetectedObject target_obj;
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
  // so get2dGeneralizedIoU includes an error of about 0.03.
  // Therefore, in this test, epsilon is set to 0.04.
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

TEST(matching, test_get2dPrecision)
{
  using autoware_auto_perception_msgs::msg::DetectedObject;
  using perception_utils::get2dPrecision;
  const double quart_circle = 0.16237976320958225;

  {  // non overlapped
    DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(1.5, 1.5, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    DetectedObject target_obj;
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
    DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.5, 0.5, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    DetectedObject target_obj;
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
    DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    DetectedObject target_obj;
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

TEST(matching, test_get2dRecall)
{
  using autoware_auto_perception_msgs::msg::DetectedObject;
  using perception_utils::get2dRecall;
  const double quart_circle = 0.16237976320958225;

  {  // non overlapped
    DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(1.5, 1.5, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    DetectedObject target_obj;
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
    DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.5, 0.5, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    DetectedObject target_obj;
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
    DetectedObject source_obj;
    source_obj.kinematics.pose_with_covariance.pose = createPose(0.0, 0.0, M_PI);
    source_obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    source_obj.shape.dimensions.x = 1.0;
    source_obj.shape.dimensions.y = 1.0;

    DetectedObject target_obj;
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
