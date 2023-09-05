// Copyright 2020 Tier IV, Inc.
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
//
// Author: v1.0 Yukihiro Saito
//

#ifndef MULTI_OBJECT_TRACKER__UTILS__UTILS_HPP_
#define MULTI_OBJECT_TRACKER__UTILS__UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

namespace utils
{
enum MSG_COV_IDX {
  X_X = 0,
  X_Y = 1,
  X_Z = 2,
  X_ROLL = 3,
  X_PITCH = 4,
  X_YAW = 5,
  Y_X = 6,
  Y_Y = 7,
  Y_Z = 8,
  Y_ROLL = 9,
  Y_PITCH = 10,
  Y_YAW = 11,
  Z_X = 12,
  Z_Y = 13,
  Z_Z = 14,
  Z_ROLL = 15,
  Z_PITCH = 16,
  Z_YAW = 17,
  ROLL_X = 18,
  ROLL_Y = 19,
  ROLL_Z = 20,
  ROLL_ROLL = 21,
  ROLL_PITCH = 22,
  ROLL_YAW = 23,
  PITCH_X = 24,
  PITCH_Y = 25,
  PITCH_Z = 26,
  PITCH_ROLL = 27,
  PITCH_PITCH = 28,
  PITCH_YAW = 29,
  YAW_X = 30,
  YAW_Y = 31,
  YAW_Z = 32,
  YAW_ROLL = 33,
  YAW_PITCH = 34,
  YAW_YAW = 35
};

enum BBOX_IDX {
  FRONT_SURFACE = 0,
  RIGHT_SURFACE = 1,
  REAR_SURFACE = 2,
  LEFT_SURFACE = 3,
  FRONT_R_CORNER = 4,
  REAR_R_CORNER = 5,
  REAR_L_CORNER = 6,
  FRONT_L_CORNER = 7,
  INSIDE = 8,
  INVALID = -1
};

/**
 * @brief check if object label belongs to "large vehicle"
 * @param label: input object label
 * @return True if object label means large vehicle
 */
inline bool isLargeVehicleLabel(const uint8_t label)
{
  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;
  return label == Label::BUS || label == Label::TRUCK || label == Label::TRAILER;
}

/**
 * @brief Determine the Nearest Corner or Surface of detected object observed from ego vehicle
 *
 * @param x: object x coordinate in map frame
 * @param y: object y coordinate in map frame
 * @param yaw: object yaw orientation in map frame
 * @param width: object bounding box width
 * @param length: object bounding box length
 * @param self_transform: Ego vehicle position in map frame
 * @return int index
 */
inline int getNearestCornerOrSurface(
  const double x, const double y, const double yaw, const double width, const double length,
  const geometry_msgs::msg::Transform & self_transform)
{
  // get local vehicle pose
  const double x0 = self_transform.translation.x;
  const double y0 = self_transform.translation.y;

  // localize self vehicle pose to object coordinate
  // R.T (X0-X)
  const double xl = std::cos(yaw) * (x0 - x) + std::sin(yaw) * (y0 - y);
  const double yl = -std::sin(yaw) * (x0 - x) + std::cos(yaw) * (y0 - y);

  // Determine Index
  //     x+ (front)
  //         __
  // y+     |  | y-
  // (left) |  | (right)
  //         --
  //     x- (rear)
  int xgrid, ygrid;
  const int labels[3][3] = {
    {BBOX_IDX::FRONT_L_CORNER, BBOX_IDX::FRONT_SURFACE, BBOX_IDX::FRONT_R_CORNER},
    {BBOX_IDX::LEFT_SURFACE, BBOX_IDX::INSIDE, BBOX_IDX::RIGHT_SURFACE},
    {BBOX_IDX::REAR_L_CORNER, BBOX_IDX::REAR_SURFACE, BBOX_IDX::REAR_R_CORNER}};
  if (xl > length / 2.0) {
    xgrid = 0;  // front
  } else if (xl > -length / 2.0) {
    xgrid = 1;  // middle
  } else {
    xgrid = 2;  // rear
  }
  if (yl > width / 2.0) {
    ygrid = 0;  // left
  } else if (yl > -width / 2.0) {
    ygrid = 1;  // middle
  } else {
    ygrid = 2;  // right
  }

  return labels[xgrid][ygrid];  // 0 to 7 + 1(null) value
}

/**
 * @brief Get the Nearest Corner or Surface from detected object
 * @param object: input object
 * @param yaw: object yaw angle (after solved front and back uncertainty)
 * @param self_transform
 * @return nearest corner or surface index
 */
inline int getNearestCornerOrSurfaceFromObject(
  const autoware_auto_perception_msgs::msg::DetectedObject & object, const double & yaw,
  const geometry_msgs::msg::Transform & self_transform)
{
  // only work for BBOX shape
  if (object.shape.type != autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return BBOX_IDX::INVALID;
  }

  // extract necessary information from input object
  double x, y, width, length;
  x = object.kinematics.pose_with_covariance.pose.position.x;
  y = object.kinematics.pose_with_covariance.pose.position.y;
  // yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation); //not use raw yaw
  // now
  width = object.shape.dimensions.y;
  length = object.shape.dimensions.x;

  return getNearestCornerOrSurface(x, y, yaw, width, length, self_transform);
}

/**
 * @brief Calc bounding box center offset caused by shape change
 * @param dw: width update [m] =  w_new - w_old
 * @param dl: length update [m] = l_new - l_old
 * @param indx: nearest corner index
 * @return 2d offset vector caused by shape change
 */
inline Eigen::Vector2d calcOffsetVectorFromShapeChange(
  const double dw, const double dl, const int indx)
{
  Eigen::Vector2d offset;
  // if surface
  if (indx == BBOX_IDX::FRONT_SURFACE) {
    offset(0, 0) = dl / 2.0;  // move forward
    offset(1, 0) = 0;
  } else if (indx == BBOX_IDX::RIGHT_SURFACE) {
    offset(0, 0) = 0;
    offset(1, 0) = -dw / 2.0;  // move right
  } else if (indx == BBOX_IDX::REAR_SURFACE) {
    offset(0, 0) = -dl / 2.0;  // move backward
    offset(1, 0) = 0;
  } else if (indx == BBOX_IDX::LEFT_SURFACE) {
    offset(0, 0) = 0;
    offset(1, 0) = dw / 2.0;  // move left
  }
  // if corner
  if (indx == BBOX_IDX::FRONT_R_CORNER) {
    offset(0, 0) = dl / 2.0;   // move forward
    offset(1, 0) = -dw / 2.0;  // move right
  } else if (indx == BBOX_IDX::REAR_R_CORNER) {
    offset(0, 0) = -dl / 2.0;  // move backward
    offset(1, 0) = -dw / 2.0;  // move right
  } else if (indx == BBOX_IDX::REAR_L_CORNER) {
    offset(0, 0) = -dl / 2.0;  // move backward
    offset(1, 0) = dw / 2.0;   // move left
  } else if (indx == BBOX_IDX::FRONT_L_CORNER) {
    offset(0, 0) = dl / 2.0;  // move forward
    offset(1, 0) = dw / 2.0;  // move left
  }
  return offset;  // do nothing if indx == INVALID or INSIDE
}

/**
 * @brief post-processing to recover bounding box center from tracking point and offset vector
 * @param x: x of tracking point estimated with ekf
 * @param y: y of tracking point estimated with ekf
 * @param yaw: yaw of tracking point estimated with ekf
 * @param dw: diff of width: w_estimated - w_input
 * @param dl: diff of length: l_estimated - l_input
 * @param indx: closest corner or surface index
 * @param tracking_offset: tracking offset between bounding box center and tracking point
 */
inline Eigen::Vector2d recoverFromTrackingPoint(
  const double x, const double y, const double yaw, const double dw, const double dl,
  const int indx, const Eigen::Vector2d & tracking_offset)
{
  const Eigen::Vector2d tracking_point{x, y};
  const Eigen::Matrix2d R = Eigen::Rotation2Dd(yaw).toRotationMatrix();

  const Eigen::Vector2d shape_change_offset = calcOffsetVectorFromShapeChange(dw, dl, indx);

  Eigen::Vector2d output_center = tracking_point - R * tracking_offset - R * shape_change_offset;

  return output_center;
}

/**
 * @brief Convert input object center to tracking point based on nearest corner information
 * 1. update anchor offset vector, 2. offset input bbox based on tracking_offset vector and
 * prediction yaw angle
 * @param w: last input bounding box width
 * @param l: last input bounding box length
 * @param indx: last input bounding box closest corner index
 * @param input_object: input object bounding box
 * @param yaw: current yaw estimation
 * @param offset_object: output tracking measurement to feed ekf
 * @return nearest corner index(int)
 */
inline void calcAnchorPointOffset(
  const double w, const double l, const int indx,
  const autoware_auto_perception_msgs::msg::DetectedObject & input_object, const double & yaw,
  autoware_auto_perception_msgs::msg::DetectedObject & offset_object,
  Eigen::Vector2d & tracking_offset)
{
  // copy value
  offset_object = input_object;
  // invalid index
  if (indx == BBOX_IDX::INSIDE) {
    return;  // do nothing
  }

  // current object width and height
  double w_n, l_n;
  l_n = input_object.shape.dimensions.x;
  w_n = input_object.shape.dimensions.y;

  // update offset
  const Eigen::Vector2d offset = calcOffsetVectorFromShapeChange(w_n - w, l_n - l, indx);
  tracking_offset += offset;

  // offset input object
  const Eigen::Matrix2d R = Eigen::Rotation2Dd(yaw).toRotationMatrix();
  const Eigen::Vector2d rotated_offset = R * tracking_offset;
  offset_object.kinematics.pose_with_covariance.pose.position.x += rotated_offset.x();
  offset_object.kinematics.pose_with_covariance.pose.position.y += rotated_offset.y();
}

/**
 * @brief convert convex hull shape object to bounding box object
 * @param input_object: input convex hull objects
 * @param output_object: output bounding box objects
 */
inline void convertConvexHullToBoundingBox(
  const autoware_auto_perception_msgs::msg::DetectedObject & input_object,
  autoware_auto_perception_msgs::msg::DetectedObject & output_object)
{
  const Eigen::Vector2d center{
    input_object.kinematics.pose_with_covariance.pose.position.x,
    input_object.kinematics.pose_with_covariance.pose.position.y};
  const auto yaw = tf2::getYaw(input_object.kinematics.pose_with_covariance.pose.orientation);
  const Eigen::Matrix2d R_inv = Eigen::Rotation2Dd(-yaw).toRotationMatrix();

  double max_x = 0;
  double max_y = 0;
  double min_x = 0;
  double min_y = 0;
  double max_z = 0;

  // look for bounding box boundary
  for (size_t i = 0; i < input_object.shape.footprint.points.size(); ++i) {
    Eigen::Vector2d vertex{
      input_object.shape.footprint.points.at(i).x, input_object.shape.footprint.points.at(i).y};

    const Eigen::Vector2d local_vertex = R_inv * (vertex - center);
    max_x = std::max(max_x, local_vertex.x());
    max_y = std::max(max_y, local_vertex.y());
    min_x = std::min(min_x, local_vertex.x());
    min_y = std::min(min_y, local_vertex.y());

    max_z = std::max(max_z, static_cast<double>(input_object.shape.footprint.points.at(i).z));
  }

  // calc bounding box state
  const double length = max_x - min_x;
  const double width = max_y - min_y;
  const double height = max_z;
  const Eigen::Vector2d new_local_center{(max_x + min_x) / 2.0, (max_y + min_y) / 2.0};
  const Eigen::Vector2d new_center = center + R_inv.transpose() * new_local_center;

  // set output parameters
  output_object = input_object;
  output_object.kinematics.pose_with_covariance.pose.position.x = new_center.x();
  output_object.kinematics.pose_with_covariance.pose.position.y = new_center.y();

  output_object.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
  output_object.shape.dimensions.x = length;
  output_object.shape.dimensions.y = width;
  output_object.shape.dimensions.z = height;
}

}  // namespace utils

#endif  // MULTI_OBJECT_TRACKER__UTILS__UTILS_HPP_
