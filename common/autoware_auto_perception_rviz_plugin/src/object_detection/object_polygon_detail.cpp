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

#include "autoware_auto_perception_rviz_plugin/object_detection/object_polygon_detail.hpp"

#include <Eigen/Core>
#include <Eigen/Eigen>

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
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->scale.x = 0.5;
  marker_ptr->scale.y = 0.5;
  marker_ptr->scale.z = 0.5;
  marker_ptr->pose = initPose();
  marker_ptr->color = path_confidence_color;
  marker_ptr->pose.position = predicted_path.path.back().position;
  marker_ptr->text = std::to_string(predicted_path.confidence);
  marker_ptr->color.a = 0.5;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_predicted_path_marker_ptr(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const std_msgs::msg::ColorRGBA & predicted_path_color, const bool is_simple)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_ptr->ns = std::string("path");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->pose = initPose();
  marker_ptr->color = predicted_path_color;
  marker_ptr->color.a = 0.6;
  marker_ptr->scale.x = 0.015;
  calc_path_line_list(predicted_path, marker_ptr->points, is_simple);
  for (size_t k = 0; k < marker_ptr->points.size(); ++k) {
    marker_ptr->points.at(k).z -= shape.dimensions.z * 0.5;
  }
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_twist_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance, const double & line_width)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_ptr->ns = std::string("twist");
  marker_ptr->scale.x = line_width;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;

  // velocity line
  geometry_msgs::msg::Point point;
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;
  marker_ptr->points.push_back(point);
  point.x = twist_with_covariance.twist.linear.x;
  point.y = twist_with_covariance.twist.linear.y;
  point.z = twist_with_covariance.twist.linear.z;
  marker_ptr->points.push_back(point);

  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color.a = 0.999;
  marker_ptr->color.r = 1.0;
  marker_ptr->color.g = 0.0;
  marker_ptr->color.b = 0.0;

  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_twist_covariance_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance,
  const double & confidence_interval_coefficient)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::CYLINDER;
  marker_ptr->ns = std::string("twist covariance");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;

  // position is the tip of the velocity vector
  const double velocity = std::sqrt(
    twist_with_covariance.twist.linear.x * twist_with_covariance.twist.linear.x +
    twist_with_covariance.twist.linear.y * twist_with_covariance.twist.linear.y);
  const double velocity_angle =
    std::atan2(twist_with_covariance.twist.linear.y, twist_with_covariance.twist.linear.x);
  const double pos_yaw_angle = 2.0 * std::atan2(
                                       pose_with_covariance.pose.orientation.z,
                                       pose_with_covariance.pose.orientation.w);  // [rad]
  marker_ptr->pose.position.x += velocity * std::cos(pos_yaw_angle + velocity_angle);
  marker_ptr->pose.position.y += velocity * std::sin(pos_yaw_angle + velocity_angle);

  // velocity covariance
  // extract eigen values and eigen vectors
  Eigen::Matrix2d eigen_twist_covariance;
  eigen_twist_covariance << twist_with_covariance.covariance[0],
    twist_with_covariance.covariance[1], twist_with_covariance.covariance[6],
    twist_with_covariance.covariance[7];
  double phi, sigma1, sigma2;
  calc_covariance_eigen_vectors(eigen_twist_covariance, sigma1, sigma2, phi);
  phi = pos_yaw_angle + phi;
  double area = sigma1 * sigma2;
  double alpha = std::min(0.5, 1.0 / area);
  alpha = std::max(0.1, alpha);

  // ellipse orientation
  marker_ptr->pose.orientation.x = 0.0;
  marker_ptr->pose.orientation.y = 0.0;
  marker_ptr->pose.orientation.z = std::sin(phi * 0.5);
  marker_ptr->pose.orientation.w = std::cos(phi * 0.5);

  // ellipse size
  marker_ptr->scale.x = sigma1 * confidence_interval_coefficient;
  marker_ptr->scale.y = sigma2 * confidence_interval_coefficient;
  marker_ptr->scale.z = 0.05;

  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color.a = alpha;
  marker_ptr->color.r = 0.2;
  marker_ptr->color.g = 0.4;
  marker_ptr->color.b = 0.9;

  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_yaw_rate_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance, const double & line_width)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_ptr->ns = std::string("yaw rate");
  marker_ptr->scale.x = line_width;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;

  // yaw rate
  const double yaw_rate = twist_with_covariance.twist.angular.z;
  const double velocity = std::sqrt(
    twist_with_covariance.twist.linear.x * twist_with_covariance.twist.linear.x +
    twist_with_covariance.twist.linear.y * twist_with_covariance.twist.linear.y +
    twist_with_covariance.twist.linear.z * twist_with_covariance.twist.linear.z);
  const double velocity_angle =
    std::atan2(twist_with_covariance.twist.linear.y, twist_with_covariance.twist.linear.x);
  const double yaw_mark_length = velocity * 0.8;

  geometry_msgs::msg::Point point;
  // first point
  point.x = 0;
  point.y = 0;
  point.z = 0;
  marker_ptr->points.push_back(point);
  // yaw rate arc
  calc_arc_line_strip(
    velocity_angle, velocity_angle + yaw_rate, yaw_mark_length, marker_ptr->points);
  // last point
  point.x = 0;
  point.y = 0;
  point.z = 0;
  marker_ptr->points.push_back(point);

  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color.a = 0.9;
  marker_ptr->color.r = 1.0;
  marker_ptr->color.g = 0.0;
  marker_ptr->color.b = 0.0;

  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_yaw_rate_covariance_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance,
  const double & confidence_interval_coefficient, const double & line_width)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_ptr->ns = std::string("yaw rate covariance");
  marker_ptr->scale.x = line_width;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;

  // yaw rate covariance
  const double yaw_rate_covariance = twist_with_covariance.covariance[35];
  const double yaw_rate_sigma = std::sqrt(yaw_rate_covariance) * confidence_interval_coefficient;
  const double yaw_rate = twist_with_covariance.twist.angular.z;
  const double velocity = std::sqrt(
    twist_with_covariance.twist.linear.x * twist_with_covariance.twist.linear.x +
    twist_with_covariance.twist.linear.y * twist_with_covariance.twist.linear.y +
    twist_with_covariance.twist.linear.z * twist_with_covariance.twist.linear.z);
  const double velocity_angle =
    std::atan2(twist_with_covariance.twist.linear.y, twist_with_covariance.twist.linear.x);
  const double yaw_mark_length = velocity * 0.8;
  const double bar_width = std::max(velocity * 0.05, 0.1);
  const double velocity_yaw_angle = velocity_angle + yaw_rate;
  const double velocity_yaw_p_sigma_angle = velocity_yaw_angle + yaw_rate_sigma;
  const double velocity_yaw_n_sigma_angle = velocity_yaw_angle - yaw_rate_sigma;

  const double point_list[7][3] = {
    {yaw_mark_length * std::cos(velocity_yaw_angle), yaw_mark_length * std::sin(velocity_yaw_angle),
     0},
    {yaw_mark_length * std::cos(velocity_yaw_p_sigma_angle),
     yaw_mark_length * std::sin(velocity_yaw_p_sigma_angle), 0},
    {yaw_mark_length * std::cos(velocity_yaw_n_sigma_angle),
     yaw_mark_length * std::sin(velocity_yaw_n_sigma_angle), 0},
    {(yaw_mark_length + bar_width) * std::cos(velocity_yaw_p_sigma_angle),
     (yaw_mark_length + bar_width) * std::sin(velocity_yaw_p_sigma_angle), 0},
    {(yaw_mark_length - bar_width) * std::cos(velocity_yaw_p_sigma_angle),
     (yaw_mark_length - bar_width) * std::sin(velocity_yaw_p_sigma_angle), 0},
    {(yaw_mark_length + bar_width) * std::cos(velocity_yaw_n_sigma_angle),
     (yaw_mark_length + bar_width) * std::sin(velocity_yaw_n_sigma_angle), 0},
    {(yaw_mark_length - bar_width) * std::cos(velocity_yaw_n_sigma_angle),
     (yaw_mark_length - bar_width) * std::sin(velocity_yaw_n_sigma_angle), 0},
  };
  const int point_pairs[4][2] = {
    {0, 1},
    {0, 2},
    {3, 4},
    {5, 6},
  };
  calc_line_list_from_points(point_list, point_pairs, 4, marker_ptr->points);

  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color.a = 0.9;
  marker_ptr->color.r = 1.0;
  marker_ptr->color.g = 0.2;
  marker_ptr->color.b = 0.4;

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
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
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
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color = color_rgba;
  return marker_ptr;
}

void calc_arc_line_strip(
  const double start_angle, const double end_angle, const double radius,
  std::vector<geometry_msgs::msg::Point> & points)
{
  geometry_msgs::msg::Point point;
  // arc points
  const double maximum_delta_angle = 10.0 * M_PI / 180.0;
  const int num_points =
    std::max(3, static_cast<int>(std::abs(end_angle - start_angle) / maximum_delta_angle));
  for (int i = 0; i < num_points; ++i) {
    const double angle = start_angle + (end_angle - start_angle) * static_cast<double>(i) /
                                         static_cast<double>(num_points - 1);
    point.x = radius * std::cos(angle);
    point.y = radius * std::sin(angle);
    point.z = 0;
    points.push_back(point);
  }
}

void calc_covariance_eigen_vectors(
  const Eigen::Matrix2d & matrix, double & sigma1, double & sigma2, double & yaw)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(matrix);
  Eigen::Vector2d eigen_values = solver.eigenvalues();
  // eigen values
  sigma1 = std::sqrt(eigen_values.x());
  sigma2 = std::sqrt(eigen_values.y());
  // orientation of covariance ellipse
  Eigen::Vector2d e1 = solver.eigenvectors().col(0);
  yaw = std::atan2(e1.y(), e1.x());
}

visualization_msgs::msg::Marker::SharedPtr get_pose_covariance_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
  const double & confidence_interval_coefficient)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::CYLINDER;
  marker_ptr->ns = std::string("position covariance");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;

  // position covariance
  // extract eigen values and eigen vectors
  Eigen::Matrix2d eigen_pose_covariance;
  eigen_pose_covariance << pose_with_covariance.covariance[0], pose_with_covariance.covariance[1],
    pose_with_covariance.covariance[6], pose_with_covariance.covariance[7];
  double yaw, sigma1, sigma2;
  calc_covariance_eigen_vectors(eigen_pose_covariance, sigma1, sigma2, yaw);

  // ellipse orientation
  marker_ptr->pose.orientation.x = 0.0;
  marker_ptr->pose.orientation.y = 0.0;
  marker_ptr->pose.orientation.z = std::sin(yaw * 0.5);
  marker_ptr->pose.orientation.w = std::cos(yaw * 0.5);

  // ellipse size
  marker_ptr->scale.x = sigma1 * confidence_interval_coefficient;
  marker_ptr->scale.y = sigma2 * confidence_interval_coefficient;
  marker_ptr->scale.z = 0.05;

  // ellipse color density
  double area = sigma1 * sigma2;
  double alpha = std::min(0.5, 3.0 / area);
  alpha = std::max(0.1, alpha);

  // marker configuration
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color.a = alpha;
  marker_ptr->color.r = 0.8;
  marker_ptr->color.g = 0.8;
  marker_ptr->color.b = 0.8;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_yaw_covariance_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance, const double & length,
  const double & confidence_interval_coefficient, const double & line_width)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_ptr->ns = std::string("yaw covariance");
  marker_ptr->scale.x = line_width;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;
  geometry_msgs::msg::Point point;

  // orientation covariance
  double yaw_vector_length = std::max(length, 1.0);
  double yaw_sigma =
    std::sqrt(pose_with_covariance.covariance[35]) * confidence_interval_coefficient;
  // get arc points
  if (yaw_sigma > M_PI) {
    yaw_vector_length = 1.0;
  }
  // first point
  point.x = 0;
  point.y = 0;
  point.z = 0;
  marker_ptr->points.push_back(point);
  // arc points
  calc_arc_line_strip(-yaw_sigma, yaw_sigma, yaw_vector_length, marker_ptr->points);
  // last point
  point.x = 0;
  point.y = 0;
  point.z = 0;
  marker_ptr->points.push_back(point);

  // marker configuration
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color.a = 0.9;
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
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color = color_rgba;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_existence_probability_marker_ptr(
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const float existence_probability, const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("existence probability");
  marker_ptr->scale.x = 0.5;
  marker_ptr->scale.z = 0.5;
  marker_ptr->text = std::to_string(existence_probability);
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = marker_ptr->pose = to_pose(centroid, orientation);
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color = color_rgba;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_shape_marker_ptr(
  const autoware_auto_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const std_msgs::msg::ColorRGBA & color_rgba, const double & line_width,
  const bool & is_orientation_available, const ObjectFillType fill_type)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->ns = std::string("shape");
  marker_ptr->color = color_rgba;
  marker_ptr->scale.x = line_width;

  using autoware_auto_perception_msgs::msg::Shape;
  if (shape_msg.type == Shape::BOUNDING_BOX) {
    if (fill_type == ObjectFillType::Skeleton) {
      marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
      calc_bounding_box_line_list(shape_msg, marker_ptr->points);
    } else if (fill_type == ObjectFillType::Fill) {
      marker_ptr->type = visualization_msgs::msg::Marker::CUBE;
      marker_ptr->scale = shape_msg.dimensions;
      marker_ptr->color.a = 0.75f;
    }
    if (is_orientation_available) {
      calc_bounding_box_direction_line_list(shape_msg, marker_ptr->points);
    } else {
      calc_bounding_box_orientation_line_list(shape_msg, marker_ptr->points);
    }
  } else if (shape_msg.type == Shape::CYLINDER) {
    if (fill_type == ObjectFillType::Skeleton) {
      marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
      calc_cylinder_line_list(shape_msg, marker_ptr->points);
    } else if (fill_type == ObjectFillType::Fill) {
      marker_ptr->type = visualization_msgs::msg::Marker::CYLINDER;
      marker_ptr->scale = shape_msg.dimensions;
      marker_ptr->color.a = 0.75f;
    }
  } else if (shape_msg.type == Shape::POLYGON) {
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_polygon_line_list(shape_msg, marker_ptr->points);
  } else {
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_polygon_line_list(shape_msg, marker_ptr->points);
  }

  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = to_pose(centroid, orientation);
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);

  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_2d_shape_marker_ptr(
  const autoware_auto_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const std_msgs::msg::ColorRGBA & color_rgba, const double & line_width,
  const bool & is_orientation_available)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->ns = std::string("shape");

  using autoware_auto_perception_msgs::msg::Shape;
  if (shape_msg.type == Shape::BOUNDING_BOX) {
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_2d_bounding_box_bottom_line_list(shape_msg, marker_ptr->points);
    if (is_orientation_available) {
      calc_2d_bounding_box_bottom_direction_line_list(shape_msg, marker_ptr->points);
    } else {
      calc_2d_bounding_box_bottom_orientation_line_list(shape_msg, marker_ptr->points);
    }
  } else if (shape_msg.type == Shape::CYLINDER) {
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_2d_cylinder_bottom_line_list(shape_msg, marker_ptr->points);
  } else if (shape_msg.type == Shape::POLYGON) {
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_2d_polygon_bottom_line_list(shape_msg, marker_ptr->points);
  } else {
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_2d_polygon_bottom_line_list(shape_msg, marker_ptr->points);
  }

  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = to_pose(centroid, orientation);
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->scale.x = line_width;
  marker_ptr->color = color_rgba;

  return marker_ptr;
}

void calc_line_list_from_points(
  const double point_list[][3], const int point_pairs[][2], const int & num_pairs,
  std::vector<geometry_msgs::msg::Point> & points)
{
  geometry_msgs::msg::Point point;
  for (int i = 0; i < num_pairs; ++i) {
    point.x = point_list[point_pairs[i][0]][0];
    point.y = point_list[point_pairs[i][0]][1];
    point.z = point_list[point_pairs[i][0]][2];
    points.push_back(point);
    point.x = point_list[point_pairs[i][1]][0];
    point.y = point_list[point_pairs[i][1]][1];
    point.z = point_list[point_pairs[i][1]][2];
    points.push_back(point);
  }
}

void calc_bounding_box_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double length_half = shape.dimensions.x * 0.5;
  const double width_half = shape.dimensions.y * 0.5;
  const double height_half = shape.dimensions.z * 0.5;
  geometry_msgs::msg::Point point;

  // bounding box corner points
  // top and bottom surface, clockwise
  const double point_list[8][3] = {
    {length_half, width_half, height_half},    {length_half, -width_half, height_half},
    {-length_half, -width_half, height_half},  {-length_half, width_half, height_half},
    {length_half, width_half, -height_half},   {length_half, -width_half, -height_half},
    {-length_half, -width_half, -height_half}, {-length_half, width_half, -height_half},
  };
  const int point_pairs[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };
  calc_line_list_from_points(point_list, point_pairs, 12, points);
}

void calc_bounding_box_direction_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  // direction triangle
  const double length_half = shape.dimensions.x * 0.5;
  const double width_half = shape.dimensions.y * 0.5;
  const double height_half = shape.dimensions.z * 0.5;
  const double triangle_size_half = std::min(width_half * 1.4, shape.dimensions.x);
  geometry_msgs::msg::Point point;

  // triangle-shaped direction indicator
  const double point_list[6][3] = {
    {length_half, 0, height_half},
    {length_half - triangle_size_half, width_half, height_half},
    {length_half - triangle_size_half, -width_half, height_half},
    {length_half, 0, -height_half},
    {length_half, width_half, height_half},
    {length_half, -width_half, height_half},
  };
  const int point_pairs[5][2] = {
    {0, 1}, {1, 2}, {0, 2}, {3, 4}, {3, 5},
  };
  calc_line_list_from_points(point_list, point_pairs, 5, points);
}

void calc_bounding_box_orientation_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double length_half = shape.dimensions.x * 0.5;
  const double width_half = shape.dimensions.y * 0.5;
  const double height_half = shape.dimensions.z * 0.5;
  const double tick_width = width_half * 0.5;
  const double tick_length = std::min(tick_width, length_half);
  geometry_msgs::msg::Point point;

  // front corner cuts for orientation
  const double point_list[4][3] = {
    {length_half, width_half - tick_width, height_half},
    {length_half - tick_length, width_half, height_half},
    {length_half, -width_half + tick_width, height_half},
    {length_half - tick_length, -width_half, height_half},
  };
  const int point_pairs[2][2] = {
    {0, 1},
    {2, 3},
  };
  calc_line_list_from_points(point_list, point_pairs, 2, points);
}

void calc_2d_bounding_box_bottom_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double length_half = shape.dimensions.x * 0.5;
  const double width_half = shape.dimensions.y * 0.5;
  const double height_half = shape.dimensions.z * 0.5;
  geometry_msgs::msg::Point point;

  // bounding box corner points
  // top surface, clockwise
  const double point_list[4][3] = {
    {length_half, width_half, -height_half},
    {length_half, -width_half, -height_half},
    {-length_half, -width_half, -height_half},
    {-length_half, width_half, -height_half},
  };
  const int point_pairs[4][2] = {
    {0, 1},
    {1, 2},
    {2, 3},
    {3, 0},
  };
  calc_line_list_from_points(point_list, point_pairs, 4, points);
}

void calc_2d_bounding_box_bottom_direction_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double length_half = shape.dimensions.x * 0.5;
  const double width_half = shape.dimensions.y * 0.5;
  const double height_half = shape.dimensions.z * 0.5;
  const double triangle_size_half = std::min(width_half * 1.4, shape.dimensions.x);
  geometry_msgs::msg::Point point;

  // triangle-shaped direction indicator
  const double point_list[6][3] = {
    {length_half, 0, -height_half},
    {length_half - triangle_size_half, width_half, -height_half},
    {length_half - triangle_size_half, -width_half, -height_half},
  };
  const int point_pairs[3][2] = {
    {0, 1},
    {1, 2},
    {0, 2},
  };
  calc_line_list_from_points(point_list, point_pairs, 3, points);
}

void calc_2d_bounding_box_bottom_orientation_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double length_half = shape.dimensions.x * 0.5;
  const double width_half = shape.dimensions.y * 0.5;
  const double height_half = shape.dimensions.z * 0.5;
  const double tick_width = width_half * 0.5;
  const double tick_length = std::min(tick_width, length_half);
  geometry_msgs::msg::Point point;

  // front corner cuts for orientation
  const double point_list[4][3] = {
    {length_half, width_half - tick_width, height_half},
    {length_half - tick_length, width_half, height_half},
    {length_half, -width_half + tick_width, height_half},
    {length_half - tick_length, -width_half, height_half},
  };
  const int point_pairs[2][2] = {
    {0, 1},
    {2, 3},
  };
  calc_line_list_from_points(point_list, point_pairs, 2, points);
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

void calc_2d_cylinder_bottom_line_list(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double radius = shape.dimensions.x * 0.5;
  {
    constexpr int n = 20;
    geometry_msgs::msg::Point center;
    center.x = 0.0;
    center.y = 0.0;
    center.z = -shape.dimensions.z * 0.5;
    calc_circle_line_list(center, radius, points, n);
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
    point.z = shape.dimensions.z * 0.5;
    points.push_back(point);
    point.x = shape.footprint.points
                .at(static_cast<int>(i + 1) % static_cast<int>(shape.footprint.points.size()))
                .x;
    point.y = shape.footprint.points
                .at(static_cast<int>(i + 1) % static_cast<int>(shape.footprint.points.size()))
                .y;
    point.z = shape.dimensions.z * 0.5;
    points.push_back(point);
  }
  for (size_t i = 0; i < shape.footprint.points.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = -shape.dimensions.z * 0.5;
    points.push_back(point);
    point.x = shape.footprint.points
                .at(static_cast<int>(i + 1) % static_cast<int>(shape.footprint.points.size()))
                .x;
    point.y = shape.footprint.points
                .at(static_cast<int>(i + 1) % static_cast<int>(shape.footprint.points.size()))
                .y;
    point.z = -shape.dimensions.z * 0.5;
    points.push_back(point);
  }
  for (size_t i = 0; i < shape.footprint.points.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = shape.dimensions.z * 0.5;
    points.push_back(point);
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = -shape.dimensions.z * 0.5;
    points.push_back(point);
  }
}

void calc_2d_polygon_bottom_line_list(
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
    point.z = -shape.dimensions.z * 0.5;
    points.push_back(point);
    point.x = shape.footprint.points
                .at(static_cast<int>(i + 1) % static_cast<int>(shape.footprint.points.size()))
                .x;
    point.y = shape.footprint.points
                .at(static_cast<int>(i + 1) % static_cast<int>(shape.footprint.points.size()))
                .y;
    point.z = -shape.dimensions.z * 0.5;
    points.push_back(point);
  }
}

void calc_path_line_list(
  const autoware_auto_perception_msgs::msg::PredictedPath & paths,
  std::vector<geometry_msgs::msg::Point> & points, const bool is_simple)
{
  const int circle_line_num = is_simple ? 5 : 10;

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
    if (!is_simple || i % 2 == 0) {
      calc_circle_line_list(point, 0.25, points, circle_line_num);
    }
  }
}

}  // namespace detail
}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware
