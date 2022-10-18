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
// limitations under the License..

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <object_detection/object_polygon_detail.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{
// get string of double value rounded after first decimal place
// e.g. roundAfterFirstDecimalPlace(12.345) -> "1.2"
std::string getRoundedDoubleString(const double val)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(1) << val;
  return ss.str();
}
}  // namespace

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
namespace detail
{
using Marker = visualization_msgs::msg::Marker;

visualization_msgs::msg::Marker::SharedPtr get_path_confidence_marker_ptr(
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const std_msgs::msg::ColorRGBA & path_confidence_color)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("path confidence");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_ptr->scale.x = 0.5;
  marker_ptr->scale.y = 0.5;
  marker_ptr->scale.z = 0.5;
  marker_ptr->pose = initPose();
  marker_ptr->color = path_confidence_color;
  marker_ptr->pose.position = predicted_path.path.back().position;
  marker_ptr->text = std::to_string(predicted_path.confidence);
  marker_ptr->color.a = std::max(
    static_cast<double>(std::min(static_cast<double>(predicted_path.confidence), 0.999)), 0.5);
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_predicted_path_marker_ptr(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const std_msgs::msg::ColorRGBA & predicted_path_color)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_ptr->ns = std::string("path");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_ptr->pose = initPose();
  marker_ptr->color = predicted_path_color;
  marker_ptr->color.a = std::max(
    static_cast<double>(std::min(static_cast<double>(predicted_path.confidence), 0.999)), 0.5);
  marker_ptr->scale.x = 0.03 * marker_ptr->color.a;
  calc_path_line_list(predicted_path, marker_ptr->points);
  for (size_t k = 0; k < marker_ptr->points.size(); ++k) {
    marker_ptr->points.at(k).z -= shape.dimensions.z / 2.0;
  }
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_twist_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_ptr->ns = std::string("twist");
  marker_ptr->scale.x = 0.03;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;

  geometry_msgs::msg::Point pt_s;
  pt_s.x = 0.0;
  pt_s.y = 0.0;
  pt_s.z = 0.0;
  marker_ptr->points.push_back(pt_s);

  geometry_msgs::msg::Point pt_e;
  pt_e.x = twist_with_covariance.twist.linear.x;
  pt_e.y = twist_with_covariance.twist.linear.y;
  pt_e.z = twist_with_covariance.twist.linear.z;
  marker_ptr->points.push_back(pt_e);

  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_ptr->color.a = 0.999;
  marker_ptr->color.r = 1.0;
  marker_ptr->color.g = 0.0;
  marker_ptr->color.b = 0.0;

  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_velocity_text_marker_ptr(
  const geometry_msgs::msg::Twist & twist, const geometry_msgs::msg::Point & vis_pos,
  const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("velocity");
  marker_ptr->scale.x = 0.5;
  marker_ptr->scale.z = 0.5;

  double vel = std::sqrt(
    twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y +
    twist.linear.z * twist.linear.z);
  marker_ptr->text = std::to_string(static_cast<int>(vel * 3.6)) + std::string("[km/h]");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose.position = vis_pos;
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_ptr->color = color_rgba;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_acceleration_text_marker_ptr(
  const geometry_msgs::msg::Accel & accel, const geometry_msgs::msg::Point & vis_pos,
  const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("acceleration");
  marker_ptr->scale.x = 0.5;
  marker_ptr->scale.z = 0.5;

  double acc = std::sqrt(
    accel.linear.x * accel.linear.x + accel.linear.y * accel.linear.y +
    accel.linear.z * accel.linear.z);
  marker_ptr->text = getRoundedDoubleString(acc) + std::string("[m/s^2]");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose.position = vis_pos;
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_ptr->color = color_rgba;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_pose_with_covariance_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_ptr->ns = std::string("position covariance");
  marker_ptr->scale.x = 0.03;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;
  marker_ptr->pose.orientation.x = 0.0;
  marker_ptr->pose.orientation.y = 0.0;
  marker_ptr->pose.orientation.z = 0.0;
  marker_ptr->pose.orientation.w = 1.0;
  geometry_msgs::msg::Point point;
  Eigen::Matrix2d eigen_pose_with_covariance;
  eigen_pose_with_covariance << pose_with_covariance.covariance[0],
    pose_with_covariance.covariance[1], pose_with_covariance.covariance[6],
    pose_with_covariance.covariance[7];
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(eigen_pose_with_covariance);
  double sigma1 = 2.448 * std::sqrt(solver.eigenvalues().x());  // 2.448 sigma is 95%
  double sigma2 = 2.448 * std::sqrt(solver.eigenvalues().y());  // 2.448 sigma is 95%
  Eigen::Vector2d e1 = solver.eigenvectors().col(0);
  Eigen::Vector2d e2 = solver.eigenvectors().col(1);
  point.x = -e1.x() * sigma1;
  point.y = -e1.y() * sigma1;
  point.z = 0;
  marker_ptr->points.push_back(point);
  point.x = e1.x() * sigma1;
  point.y = e1.y() * sigma1;
  point.z = 0;
  marker_ptr->points.push_back(point);
  point.x = -e2.x() * sigma2;
  point.y = -e2.y() * sigma2;
  point.z = 0;
  marker_ptr->points.push_back(point);
  point.x = e2.x() * sigma2;
  point.y = e2.y() * sigma2;
  point.z = 0;
  marker_ptr->points.push_back(point);
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_ptr->color.a = 0.999;
  marker_ptr->color.r = 1.0;
  marker_ptr->color.g = 1.0;
  marker_ptr->color.b = 1.0;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_uuid_marker_ptr(
  const std::string & uuid, const geometry_msgs::msg::Point & centroid,
  const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("uuid");
  marker_ptr->text = uuid.substr(0, 4);
  marker_ptr->action = Marker::MODIFY;
  marker_ptr->scale.z = 0.5;
  marker_ptr->color = color_rgba;
  marker_ptr->pose.position = centroid;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_label_marker_ptr(
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const std::string label, const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("label");
  marker_ptr->scale.x = 0.5;
  marker_ptr->scale.z = 0.5;
  marker_ptr->text = label;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = marker_ptr->pose = to_pose(centroid, orientation);
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_ptr->color = color_rgba;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_shape_marker_ptr(
  const autoware_auto_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const std_msgs::msg::ColorRGBA & color_rgba, const double & line_width)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->ns = std::string("shape");

  using autoware_auto_perception_msgs::msg::Shape;
  if (shape_msg.type == Shape::BOUNDING_BOX) {
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_bounding_box_line_list(shape_msg, marker_ptr->points);
  } else if (shape_msg.type == Shape::CYLINDER) {
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_cylinder_line_list(shape_msg, marker_ptr->points);
  } else if (shape_msg.type == Shape::POLYGON) {
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_polygon_line_list(shape_msg, marker_ptr->points);
  } else {
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_polygon_line_list(shape_msg, marker_ptr->points);
  }

  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = to_pose(centroid, orientation);
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_ptr->scale.x = line_width;
  marker_ptr->color = color_rgba;

  return marker_ptr;
}

void calc_bounding_box_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  geometry_msgs::msg::Point point;
  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  // up surface
  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);

  // down surface
  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
}

void calc_cylinder_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double radius = shape.dimensions.x * 0.5;
  {
    constexpr int n = 20;
    geometry_msgs::msg::Point center;
    center.x = 0.0;
    center.y = 0.0;
    center.z = shape.dimensions.z * 0.5;
    calc_circle_line_list(center, radius, points, n);
    center.z = -shape.dimensions.z * 0.5;
    calc_circle_line_list(center, radius, points, n);
  }
  {
    constexpr int n = 4;
    for (int i = 0; i < n; ++i) {
      geometry_msgs::msg::Point point;
      point.x = std::cos(
                  (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(n)) *
                radius;
      point.y = std::sin(
                  (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(n)) *
                radius;
      point.z = shape.dimensions.z * 0.5;
      points.push_back(point);
      point.x = std::cos(
                  (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(n)) *
                radius;
      point.y = std::sin(
                  (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(n)) *
                radius;
      point.z = -shape.dimensions.z * 0.5;
      points.push_back(point);
    }
  }
}

void calc_circle_line_list(
  const geometry_msgs::msg::Point center, const double radius,
  std::vector<geometry_msgs::msg::Point> & points, const int n)
{
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::Point point;
    point.x = std::cos(
                (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                M_PI / static_cast<double>(n)) *
                radius +
              center.x;
    point.y = std::sin(
                (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                M_PI / static_cast<double>(n)) *
                radius +
              center.y;
    point.z = center.z;
    points.push_back(point);
    point.x = std::cos(
                (static_cast<double>(i + 1.0) / static_cast<double>(n)) * 2.0 * M_PI +
                M_PI / static_cast<double>(n)) *
                radius +
              center.x;
    point.y = std::sin(
                (static_cast<double>(i + 1.0) / static_cast<double>(n)) * 2.0 * M_PI +
                M_PI / static_cast<double>(n)) *
                radius +
              center.y;
    point.z = center.z;
    points.push_back(point);
  }
}

void calc_polygon_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  if (shape.footprint.points.size() < 2) {
    RCLCPP_WARN(
      rclcpp::get_logger("ObjectPolygonDisplayBase"),
      "there are no enough footprint to visualize polygon");
    return;
  }
  for (size_t i = 0; i < shape.footprint.points.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = shape.dimensions.z / 2.0;
    points.push_back(point);
    point.x = shape.footprint.points
                .at(static_cast<int>(i + 1) % static_cast<int>(shape.footprint.points.size()))
                .x;
    point.y = shape.footprint.points
                .at(static_cast<int>(i + 1) % static_cast<int>(shape.footprint.points.size()))
                .y;
    point.z = shape.dimensions.z / 2.0;
    points.push_back(point);
  }
  for (size_t i = 0; i < shape.footprint.points.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = -shape.dimensions.z / 2.0;
    points.push_back(point);
    point.x = shape.footprint.points
                .at(static_cast<int>(i + 1) % static_cast<int>(shape.footprint.points.size()))
                .x;
    point.y = shape.footprint.points
                .at(static_cast<int>(i + 1) % static_cast<int>(shape.footprint.points.size()))
                .y;
    point.z = -shape.dimensions.z / 2.0;
    points.push_back(point);
  }
  for (size_t i = 0; i < shape.footprint.points.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = shape.dimensions.z / 2.0;
    points.push_back(point);
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = -shape.dimensions.z / 2.0;
    points.push_back(point);
  }
}

void calc_path_line_list(
  const autoware_auto_perception_msgs::msg::PredictedPath & paths,
  std::vector<geometry_msgs::msg::Point> & points)
{
  for (int i = 0; i < static_cast<int>(paths.path.size()) - 1; ++i) {
    geometry_msgs::msg::Point point;
    point.x = paths.path.at(i).position.x;
    point.y = paths.path.at(i).position.y;
    point.z = paths.path.at(i).position.z;
    points.push_back(point);
    point.x = paths.path.at(i + 1).position.x;
    point.y = paths.path.at(i + 1).position.y;
    point.z = paths.path.at(i + 1).position.z;
    points.push_back(point);
    calc_circle_line_list(point, 0.25, points, 10);
  }
}

}  // namespace detail
}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware
