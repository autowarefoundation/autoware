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

#ifndef PERCEPTION_UTILS__PERCEPTION_UTILS_HPP_
#define PERCEPTION_UTILS__PERCEPTION_UTILS_HPP_

#include "perception_utils/geometry.hpp"
#include "tier4_autoware_utils/geometry/boost_geometry.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"

#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include <boost/geometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace
{
[[maybe_unused]] boost::optional<geometry_msgs::msg::Transform> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer.lookupTransform(
      target_frame_id, source_frame_id, time, rclcpp::Duration::from_seconds(0.5));
    return self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("perception_utils"), ex.what());
    return boost::none;
  }
}

inline double getConvexShapeArea(
  const tier4_autoware_utils::Polygon2d & source_polygon,
  const tier4_autoware_utils::Polygon2d & target_polygon)
{
  boost::geometry::model::multi_polygon<tier4_autoware_utils::Polygon2d> union_polygons;
  boost::geometry::union_(source_polygon, target_polygon, union_polygons);

  tier4_autoware_utils::Polygon2d hull;
  boost::geometry::convex_hull(union_polygons, hull);
  return boost::geometry::area(hull);
}

}  // namespace

namespace perception_utils
{
std::uint8_t getHighestProbLabel(
  const std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> & classification);

autoware_auto_perception_msgs::msg::DetectedObject toDetectedObject(
  const autoware_auto_perception_msgs::msg::TrackedObject & tracked_object);

autoware_auto_perception_msgs::msg::DetectedObjects toDetectedObjects(
  const autoware_auto_perception_msgs::msg::TrackedObjects & tracked_objects);

autoware_auto_perception_msgs::msg::TrackedObject toTrackedObject(
  const autoware_auto_perception_msgs::msg::DetectedObject & detected_object);

autoware_auto_perception_msgs::msg::TrackedObjects toTrackedObjects(
  const autoware_auto_perception_msgs::msg::DetectedObjects & detected_objects);

template <class T1, class T2>
inline double get2dIoU(const T1 source_object, const T2 target_object)
{
  const auto & source_pose = getPose(source_object);
  const auto & target_pose = getPose(target_object);

  const auto source_polygon = tier4_autoware_utils::toPolygon2d(source_pose, source_object.shape);
  const auto target_polygon = tier4_autoware_utils::toPolygon2d(target_pose, target_object.shape);

  std::vector<tier4_autoware_utils::Polygon2d> union_polygons;
  std::vector<tier4_autoware_utils::Polygon2d> intersection_polygons;
  boost::geometry::union_(source_polygon, target_polygon, union_polygons);
  boost::geometry::intersection(source_polygon, target_polygon, intersection_polygons);

  double intersection_area = 0.0;
  double union_area = 0.0;
  for (const auto & intersection_polygon : intersection_polygons) {
    intersection_area += boost::geometry::area(intersection_polygon);
  }
  if (intersection_area == 0.0) return 0.0;

  for (const auto & union_polygon : union_polygons) {
    union_area += boost::geometry::area(union_polygon);
  }

  const double iou = union_area < 0.01 ? 0.0 : std::min(1.0, intersection_area / union_area);
  return iou;
}

template <class T1, class T2>
inline double get2dGeneralizedIoU(const T1 & source_object, const T2 & target_object)
{
  const auto & source_pose = getPose(source_object);
  const auto & target_pose = getPose(target_object);

  const auto & source_polygon = tier4_autoware_utils::toPolygon2d(source_pose, source_object.shape);
  const auto & target_polygon = tier4_autoware_utils::toPolygon2d(target_pose, target_object.shape);

  std::vector<tier4_autoware_utils::Polygon2d> union_polygons;
  std::vector<tier4_autoware_utils::Polygon2d> intersection_polygons;
  boost::geometry::union_(source_polygon, target_polygon, union_polygons);
  boost::geometry::intersection(source_polygon, target_polygon, intersection_polygons);

  double intersection_area = 0.0;
  double union_area = 0.0;
  for (const auto & intersection_polygon : intersection_polygons) {
    intersection_area += boost::geometry::area(intersection_polygon);
  }

  for (const auto & union_polygon : union_polygons) {
    union_area += boost::geometry::area(union_polygon);
  }

  const double iou = union_area < 0.01 ? 0.0 : std::min(1.0, intersection_area / union_area);
  const double convex_shape_area = getConvexShapeArea(source_polygon, target_polygon);
  return iou - (convex_shape_area - union_area) / convex_shape_area;
}

template <class T1, class T2>
inline double get2dPrecision(const T1 source_object, const T2 target_object)
{
  const auto & source_pose = getPose(source_object);
  const auto & target_pose = getPose(target_object);

  const auto source_polygon = tier4_autoware_utils::toPolygon2d(source_pose, source_object.shape);
  const auto target_polygon = tier4_autoware_utils::toPolygon2d(target_pose, target_object.shape);

  std::vector<tier4_autoware_utils::Polygon2d> intersection_polygons;
  boost::geometry::intersection(source_polygon, target_polygon, intersection_polygons);

  double intersection_area = 0.0;
  double source_area = 0.0;
  for (const auto & intersection_polygon : intersection_polygons) {
    intersection_area += boost::geometry::area(intersection_polygon);
  }
  if (intersection_area == 0.0) return 0.0;

  source_area = boost::geometry::area(source_polygon);

  const double precision = std::min(1.0, intersection_area / source_area);
  return precision;
}

template <class T1, class T2>
inline double get2dRecall(const T1 source_object, const T2 target_object)
{
  const auto & source_pose = getPose(source_object);
  const auto & target_pose = getPose(target_object);

  const auto source_polygon = tier4_autoware_utils::toPolygon2d(source_pose, source_object.shape);
  const auto target_polygon = tier4_autoware_utils::toPolygon2d(target_pose, target_object.shape);

  std::vector<tier4_autoware_utils::Polygon2d> intersection_polygons;
  boost::geometry::intersection(source_polygon, target_polygon, intersection_polygons);

  double intersection_area = 0.0;
  double target_area = 0.0;
  for (const auto & intersection_polygon : intersection_polygons) {
    intersection_area += boost::geometry::area(intersection_polygon);
  }
  if (intersection_area == 0.0) return 0.0;

  target_area += boost::geometry::area(target_polygon);

  const double recall = std::min(1.0, intersection_area / target_area);
  return recall;
}

template <class T>
bool transformObjects(
  const T & input_msg, const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
  T & output_msg)
{
  output_msg = input_msg;

  // transform to world coordinate
  if (input_msg.header.frame_id != target_frame_id) {
    output_msg.header.frame_id = target_frame_id;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    {
      const auto ros_target2objects_world =
        getTransform(tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
      if (!ros_target2objects_world) {
        return false;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (auto & object : output_msg.objects) {
      tf2::fromMsg(object.kinematics.pose_with_covariance.pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      tf2::toMsg(tf_target2objects, object.kinematics.pose_with_covariance.pose);
      // TODO(yukkysaito) transform covariance
    }
  }
  return true;
}
}  // namespace perception_utils
#endif  // PERCEPTION_UTILS__PERCEPTION_UTILS_HPP_
