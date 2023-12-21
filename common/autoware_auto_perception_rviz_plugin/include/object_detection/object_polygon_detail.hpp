// Copyright 2021 Apex.AI, Inc.
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
/// \brief This file defines some helper functions used by ObjectPolygonDisplayBase class
#ifndef OBJECT_DETECTION__OBJECT_POLYGON_DETAIL_HPP_
#define OBJECT_DETECTION__OBJECT_POLYGON_DETAIL_HPP_

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visibility_control.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_path.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
namespace detail
{
// Struct to define all the configurable visual properties of an object of a particular
// classification type
struct ObjectPropertyValues
{
  // Classified type of the object
  std::string label;
  // Color for the type of the object
  std::array<int, 3> color;
  // Alpha values for the type of the object
  float alpha{0.999F};
};

// Map defining colors according to value of label field in ObjectClassification msg
const std::map<
  autoware_auto_perception_msgs::msg::ObjectClassification::_label_type, ObjectPropertyValues>
  // Color map is based on cityscapes color
  kDefaultObjectPropertyValues = {
    {autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN,
     {"UNKNOWN", {255, 255, 255}}},
    {autoware_auto_perception_msgs::msg::ObjectClassification::CAR, {"CAR", {30, 144, 255}}},
    {autoware_auto_perception_msgs::msg::ObjectClassification::BUS, {"BUS", {30, 144, 255}}},
    {autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN,
     {"PEDESTRIAN", {255, 192, 203}}},
    {autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE, {"CYCLIST", {119, 11, 32}}},
    {autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE,
     {"MOTORCYCLE", {119, 11, 32}}},
    {autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER,
     {"TRAILER", {30, 144, 255}}},
    {autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK, {"TRUCK", {30, 144, 255}}}};

/// \brief Convert the given polygon into a marker representing the shape in 3d
/// \param shape_msg Shape msg to be converted. Corners should be in object-local frame
/// \param centroid Centroid position of the shape in Object.header.frame_id frame
/// \param orientation Orientation of the shape in Object.header.frame_id frame
/// \param color_rgba Color and alpha values to use for the marker
/// \param line_width Line thickness around the object
/// \return Marker ptr. Id and header will have to be set by the caller
AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::msg::Marker::SharedPtr
get_shape_marker_ptr(
  const autoware_auto_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const std_msgs::msg::ColorRGBA & color_rgba, const double & line_width);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::msg::Marker::SharedPtr
get_2d_shape_marker_ptr(
  const autoware_auto_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const std_msgs::msg::ColorRGBA & color_rgba, const double & line_width);

/// \brief Convert the given polygon into a marker representing the shape in 3d
/// \param centroid Centroid position of the shape in Object.header.frame_id frame
/// \return Marker ptr. Id and header will have to be set by the caller
AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::msg::Marker::SharedPtr
get_label_marker_ptr(
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const std::string label, const std_msgs::msg::ColorRGBA & color_rgba);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::msg::Marker::SharedPtr
get_uuid_marker_ptr(
  const std::string & uuid, const geometry_msgs::msg::Point & centroid,
  const std_msgs::msg::ColorRGBA & color_rgba);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::msg::Marker::SharedPtr
get_pose_with_covariance_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::msg::Marker::SharedPtr
get_velocity_text_marker_ptr(
  const geometry_msgs::msg::Twist & twist, const geometry_msgs::msg::Point & vis_pos,
  const std_msgs::msg::ColorRGBA & color_rgba);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::msg::Marker::SharedPtr
get_acceleration_text_marker_ptr(
  const geometry_msgs::msg::Accel & accel, const geometry_msgs::msg::Point & vis_pos,
  const std_msgs::msg::ColorRGBA & color_rgba);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::msg::Marker::SharedPtr
get_twist_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance, const double & line_width);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::msg::Marker::SharedPtr
get_predicted_path_marker_ptr(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const std_msgs::msg::ColorRGBA & predicted_path_color, const bool is_simple = false);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC visualization_msgs::msg::Marker::SharedPtr
get_path_confidence_marker_ptr(
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const std_msgs::msg::ColorRGBA & path_confidence_color);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_bounding_box_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_2d_bounding_box_bottom_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_cylinder_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_2d_cylinder_bottom_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_circle_line_list(
  const geometry_msgs::msg::Point center, const double radius,
  std::vector<geometry_msgs::msg::Point> & points, const int n);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_polygon_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_2d_polygon_bottom_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points);

AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC void calc_path_line_list(
  const autoware_auto_perception_msgs::msg::PredictedPath & paths,
  std::vector<geometry_msgs::msg::Point> & points, const bool is_simple = false);

/// \brief Convert Point32 to Point
/// \param val Point32 to be converted
/// \return Point type
inline AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC geometry_msgs::msg::Point to_point(
  const geometry_msgs::msg::Point32 & val)
{
  geometry_msgs::msg::Point ret;
  ret.x = static_cast<double>(val.x);
  ret.y = static_cast<double>(val.y);
  ret.z = static_cast<double>(val.z);
  return ret;
}

/// \brief Convert to Pose from Point and Quaternion
/// \param point
/// \param orientation
/// \return Pose type
inline AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC geometry_msgs::msg::Pose to_pose(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Quaternion & orientation)
{
  geometry_msgs::msg::Pose ret;
  ret.position = point;
  ret.orientation = orientation;
  return ret;
}

inline AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC geometry_msgs::msg::Pose initPose()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

/// \brief Get the best classification from the list of classifications based on max probability
/// \tparam ClassificationContainerT List type with ObjectClassificationMsg
/// \param labels List of ObjectClassificationMsg objects
/// \param logger_name Name to use for logger in case of a warning (if labels is empty)
/// \return Id of the best classification, Unknown if there is no best label
template <typename ClassificationContainerT>
AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC
  autoware_auto_perception_msgs::msg::ObjectClassification::_label_type
  get_best_label(ClassificationContainerT labels, const std::string & logger_name)
{
  const auto best_class_label = std::max_element(
    labels.begin(), labels.end(),
    [](const auto & a, const auto & b) -> bool { return a.probability < b.probability; });
  if (best_class_label == labels.end()) {
    RCLCPP_WARN(
      rclcpp::get_logger(logger_name),
      "Empty classification field. "
      "Treating as unknown");
    return autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
  }
  return best_class_label->label;
}

}  // namespace detail
}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // OBJECT_DETECTION__OBJECT_POLYGON_DETAIL_HPP_
