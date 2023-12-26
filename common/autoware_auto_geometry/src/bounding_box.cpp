// Copyright 2017-2019 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "autoware_auto_geometry/bounding_box/bounding_box_common.hpp"
#include "autoware_auto_geometry/bounding_box/eigenbox_2d.hpp"

#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
// cspell: ignore eigenbox
#include "autoware_auto_geometry/bounding_box/lfit.hpp"
// cspell: ignore lfit
#include "autoware_auto_geometry/bounding_box/rotating_calipers.hpp"

#include <geometry_msgs/msg/point32.hpp>

#include <algorithm>
#include <limits>
#include <list>
#include <utility>
#include <vector>

namespace autoware
{
namespace common
{
namespace geometry
{
namespace bounding_box
{
namespace details
{
////////////////////////////////////////////////////////////////////////////////
void size_2d(const decltype(BoundingBox::corners) & corners, geometry_msgs::msg::Point32 & ret)
{
  ret.x = std::max(
    norm_2d(minus_2d(corners[1U], corners[0U])), std::numeric_limits<float32_t>::epsilon());
  ret.y = std::max(
    norm_2d(minus_2d(corners[2U], corners[1U])), std::numeric_limits<float32_t>::epsilon());
}
////////////////////////////////////////////////////////////////////////////////
void finalize_box(const decltype(BoundingBox::corners) & corners, BoundingBox & box)
{
  (void)std::copy(&corners[0U], &corners[corners.size()], &box.corners[0U]);
  // orientation: this is robust to lines
  const float32_t yaw_rad_2 =
    atan2f(box.corners[2U].y - box.corners[1U].y, box.corners[2U].x - box.corners[1U].x) * 0.5F;
  box.orientation.w = cosf(yaw_rad_2);
  box.orientation.z = sinf(yaw_rad_2);
  // centroid
  box.centroid = times_2d(plus_2d(corners[0U], corners[2U]), 0.5F);
}

autoware_auto_perception_msgs::msg::Shape make_shape(const BoundingBox & box)
{
  autoware_auto_perception_msgs::msg::Shape retval;
  // Polygon is 2D rectangle
  geometry_msgs::msg::Polygon polygon;
  auto & points = polygon.points;
  points.resize(4);

  // Note that the x and y of size field in BoundingBox should be swapped when being used to
  // compute corners.
  // origin of reference system: centroid of bbox
  points[0].x = -0.5F * box.size.y;
  points[0].y = -0.5F * box.size.x;
  points[0].z = -box.size.z;

  points[1] = points[0];
  points[1].y += box.size.x;

  points[2] = points[1];
  points[2].x += box.size.y;

  points[3] = points[2];
  points[3].y -= box.size.x;

  retval.footprint = polygon;
  retval.dimensions.z = 2 * box.size.z;

  return retval;
}

autoware_auto_perception_msgs::msg::DetectedObject make_detected_object(const BoundingBox & box)
{
  autoware_auto_perception_msgs::msg::DetectedObject ret;

  ret.kinematics.pose_with_covariance.pose.position.x = static_cast<double>(box.centroid.x);
  ret.kinematics.pose_with_covariance.pose.position.y = static_cast<double>(box.centroid.y);
  ret.kinematics.pose_with_covariance.pose.position.z = static_cast<double>(box.centroid.z);
  ret.kinematics.pose_with_covariance.pose.orientation.x = static_cast<double>(box.orientation.x);
  ret.kinematics.pose_with_covariance.pose.orientation.y = static_cast<double>(box.orientation.y);
  ret.kinematics.pose_with_covariance.pose.orientation.z = static_cast<double>(box.orientation.z);
  ret.kinematics.pose_with_covariance.pose.orientation.w = static_cast<double>(box.orientation.w);
  ret.kinematics.orientation_availability =
    autoware_auto_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;

  ret.shape = make_shape(box);

  ret.existence_probability = 1.0F;

  autoware_auto_perception_msgs::msg::ObjectClassification label;
  label.label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
  label.probability = 1.0F;
  ret.classification.emplace_back(std::move(label));

  return ret;
}

std::vector<geometry_msgs::msg::Point32> GEOMETRY_PUBLIC get_transformed_corners(
  const autoware_auto_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation)
{
  std::vector<geometry_msgs::msg::Point32> retval(shape_msg.footprint.points.size());
  geometry_msgs::msg::TransformStamped tf;
  tf.transform.rotation = orientation;
  tf.transform.translation.x = centroid.x;
  tf.transform.translation.y = centroid.y;
  tf.transform.translation.z = centroid.z;

  for (size_t i = 0U; i < shape_msg.footprint.points.size(); ++i) {
    tf2::doTransform(shape_msg.footprint.points[i], retval[i], tf);
  }
  return retval;
}

}  // namespace details
///////////////////////////////////////////////////////////////////////////////
// pre-compilation
using autoware::common::types::PointXYZIF;
template BoundingBox minimum_area_bounding_box<PointXYZIF>(std::list<PointXYZIF> & list);
template BoundingBox minimum_perimeter_bounding_box<PointXYZIF>(std::list<PointXYZIF> & list);
using PointXYZIFVIT = std::vector<PointXYZIF>::iterator;
template BoundingBox eigenbox_2d<PointXYZIFVIT>(const PointXYZIFVIT begin, const PointXYZIFVIT end);
template BoundingBox lfit_bounding_box_2d<PointXYZIFVIT>(
  const PointXYZIFVIT begin, const PointXYZIFVIT end);
using geometry_msgs::msg::Point32;
template BoundingBox minimum_area_bounding_box<Point32>(std::list<Point32> & list);
template BoundingBox minimum_perimeter_bounding_box<Point32>(std::list<Point32> & list);
using Point32VIT = std::vector<Point32>::iterator;
template BoundingBox eigenbox_2d<Point32VIT>(const Point32VIT begin, const Point32VIT end);
template BoundingBox lfit_bounding_box_2d<Point32VIT>(const Point32VIT begin, const Point32VIT end);
}  // namespace bounding_box
}  // namespace geometry
}  // namespace common
}  // namespace autoware
