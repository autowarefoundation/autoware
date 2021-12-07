// Copyright 2021 Tier IV, Inc.
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

#include "behavior_path_planner/scene_module/avoidance/debug.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <tf2/utils.h>

#include <string>
#include <vector>

namespace marker_utils
{
using behavior_path_planner::util::calcPathArcLengthArray;
using behavior_path_planner::util::shiftPose;
using visualization_msgs::msg::Marker;

inline int64_t bitShift(int64_t original_id) { return original_id << (sizeof(int32_t) * 8 / 2); }

MarkerArray createShiftLengthMarkerArray(
  const std::vector<double> shift_distance,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & reference, const std::string & ns,
  const double r, const double g, const double b)
{
  MarkerArray ma;

  if (shift_distance.size() != reference.points.size()) {
    return ma;
  }

  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.action = Marker::ADD;
  marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  marker.scale = autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
  marker.color = autoware_utils::createMarkerColor(r, g, b, 0.9);
  marker.type = Marker::LINE_STRIP;

  for (size_t i = 0; i < shift_distance.size(); ++i) {
    auto p = reference.points.at(i).point.pose;
    shiftPose(&p, shift_distance.at(i));
    marker.points.push_back(p.position);
  }

  ma.markers.push_back(marker);
  return ma;
}

MarkerArray createAvoidPointMarkerArray(
  const AvoidPointArray & shift_points, const std::string & ns, const double r, const double g,
  const double b, const double w)
{
  int32_t id = 0;
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  AvoidPointArray shift_points_local = shift_points;
  if (shift_points_local.empty()) {
    shift_points_local.push_back(AvoidPoint());
  }

  for (const auto & sp : shift_points_local) {
    // ROS_ERROR("sp: s = (%f, %f), g = (%f, %f)", sp.start.x, sp.start.y, sp.end.x, sp.end.y);
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = ns;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.action = Marker::ADD;
    marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = autoware_utils::createMarkerScale(0.5, 0.5, 0.5);
    marker.color = autoware_utils::createMarkerColor(r, g, b, 0.9);
    {
      marker.type = Marker::CUBE;

      // start point
      auto marker_s = marker;
      marker_s.id = id++;
      marker_s.pose = sp.start;
      // shiftPose(&marker_s.pose, current_shift);  // old
      shiftPose(&marker_s.pose, sp.start_length);
      msg.markers.push_back(marker_s);

      // end point
      auto marker_e = marker;
      marker_e.id = id++;
      marker_e.pose = sp.end;
      shiftPose(&marker_e.pose, sp.length);
      msg.markers.push_back(marker_e);

      // start-to-end line
      auto marker_l = marker;
      marker_l.id = id++;
      marker_l.type = Marker::LINE_STRIP;
      marker_l.scale = autoware_utils::createMarkerScale(w, 0.0, 0.0);
      marker_l.points.push_back(marker_s.pose.position);
      marker_l.points.push_back(marker_e.pose.position);
      msg.markers.push_back(marker_l);
    }
    // current_shift = sp.length;
  }

  return msg;
}

MarkerArray createShiftPointMarkerArray(
  const ShiftPointArray & shift_points, const double base_shift, const std::string & ns,
  const double r, const double g, const double b, const double w)
{
  int32_t id = 0;
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  ShiftPointArray shift_points_local = shift_points;
  if (shift_points_local.empty()) {
    shift_points_local.push_back(ShiftPoint());
  }

  // TODO(Horibe) now assuming the shift point is aligned in longitudinal distance order
  double current_shift = base_shift;
  for (const auto & sp : shift_points_local) {
    // ROS_ERROR("sp: s = (%f, %f), g = (%f, %f)", sp.start.x, sp.start.y, sp.end.x, sp.end.y);
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = ns;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.action = Marker::ADD;
    marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = autoware_utils::createMarkerScale(0.5, 0.5, 0.5);
    marker.color = autoware_utils::createMarkerColor(r, g, b, 0.5);
    {
      marker.type = Marker::CUBE;

      // start point
      auto marker_s = marker;
      marker_s.id = id++;
      marker_s.pose = sp.start;
      shiftPose(&marker_s.pose, current_shift);  // old
      msg.markers.push_back(marker_s);

      // end point
      auto marker_e = marker;
      marker_e.id = id++;
      marker_e.pose = sp.end;
      shiftPose(&marker_e.pose, sp.length);
      msg.markers.push_back(marker_e);

      // start-to-end line
      auto marker_l = marker;
      marker_l.id = id++;
      marker_l.type = Marker::LINE_STRIP;
      marker_l.scale = autoware_utils::createMarkerScale(w, 0.0, 0.0);
      marker_l.points.push_back(marker_s.pose.position);
      marker_l.points.push_back(marker_e.pose.position);
      msg.markers.push_back(marker_l);
    }
    current_shift = sp.length;
  }

  return msg;
}

MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  for (const auto & lanelet : lanelets) {
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = ns;
    marker.id = lanelet.id();
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
    marker.color = autoware_utils::createMarkerColor(r, g, b, 0.999);
    for (const auto & p : lanelet.polygon3d()) {
      Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createLaneletPolygonsMarkerArray(
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const std::string & ns,
  const int64_t lane_id)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  int32_t i = 0;
  int32_t uid = bitShift(lane_id);
  for (const auto & polygon : polygons) {
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
    marker.color = autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999);
    for (const auto & p : polygon) {
      Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createPolygonMarkerArray(
  const Polygon & polygon, const std::string & ns, const int64_t lane_id, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;

  marker.ns = ns;
  marker.id = lane_id;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  marker.scale = autoware_utils::createMarkerScale(0.3, 0.0, 0.0);
  marker.color = autoware_utils::createMarkerColor(r, g, b, 0.8);
  for (const auto & p : polygon.points) {
    Point point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    marker.points.push_back(point);
  }
  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }
  msg.markers.push_back(marker);

  return msg;
}

MarkerArray createObjectsMarkerArray(
  const PredictedObjects & objects, const std::string & ns, const int64_t lane_id, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int32_t uid = bitShift(lane_id);
  int32_t i = 0;
  for (const auto & object : objects.objects) {
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.type = Marker::CUBE;
    marker.action = Marker::ADD;
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    marker.scale = autoware_utils::createMarkerScale(3.0, 1.0, 1.0);
    marker.color = autoware_utils::createMarkerColor(r, g, b, 0.8);
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createAvoidanceObjectsMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects, const std::string & ns)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  const auto normal_color = autoware_utils::createMarkerColor(0.9, 0.0, 0.0, 0.8);
  const auto disappearing_color = autoware_utils::createMarkerColor(0.9, 0.5, 0.9, 0.6);

  int32_t i = 0;
  for (const auto & object : objects) {
    marker.id = i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.type = Marker::CUBE;
    marker.action = Marker::ADD;
    marker.pose = object.object.kinematics.initial_pose_with_covariance.pose;
    marker.scale = autoware_utils::createMarkerScale(3.0, 1.5, 1.5);
    marker.color = object.lost_count == 0 ? normal_color : disappearing_color;
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createPathMarkerArray(
  const PathWithLaneId & path, const std::string & ns, const int64_t lane_id, const double r,
  const double g, const double b)
{
  const auto arclength = calcPathArcLengthArray(path);
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;
  int32_t uid = bitShift(lane_id);
  int32_t i = 0;
  int32_t idx = 0;
  for (const auto & p : path.points) {
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.pose = p.point.pose;
    marker.scale = autoware_utils::createMarkerScale(0.2, 0.1, 0.3);
    marker.color = autoware_utils::createMarkerColor(r, g, b, 0.999);
    msg.markers.push_back(marker);
    if (idx % 10 == 0) {
      auto marker_text = marker;
      marker_text.id = uid + i++;
      marker_text.type = Marker::TEXT_VIEW_FACING;
      std::stringstream ss;
      ss << std::fixed << std::setprecision(1) << "i=" << idx << "\ns=" << arclength.at(idx);
      marker_text.text = ss.str();
      marker_text.color = autoware_utils::createMarkerColor(1, 1, 1, 0.999);
      msg.markers.push_back(marker_text);
    }
    ++idx;
  }

  return msg;
}

MarkerArray createVirtualWallMarkerArray(
  const Pose & pose, const int64_t lane_id, const std::string & stop_factor)
{
  MarkerArray msg;

  Marker marker_virtual_wall{};
  marker_virtual_wall.header.frame_id = "map";
  marker_virtual_wall.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  marker_virtual_wall.ns = "stop_virtual_wall";
  marker_virtual_wall.id = lane_id;
  marker_virtual_wall.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_virtual_wall.type = Marker::CUBE;
  marker_virtual_wall.action = Marker::ADD;
  marker_virtual_wall.pose = pose;
  marker_virtual_wall.pose.position.z += 1.0;
  marker_virtual_wall.scale = autoware_utils::createMarkerScale(0.1, 5.0, 2.0);
  marker_virtual_wall.color = autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.5);
  msg.markers.push_back(marker_virtual_wall);

  Marker marker_factor_text{};
  marker_factor_text.header.frame_id = "map";
  marker_factor_text.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  marker_factor_text.ns = "factor_text";
  marker_factor_text.id = lane_id;
  marker_factor_text.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_factor_text.type = Marker::TEXT_VIEW_FACING;
  marker_factor_text.action = Marker::ADD;
  marker_factor_text.pose = pose;
  marker_factor_text.pose.position.z += 2.0;
  marker_factor_text.scale = autoware_utils::createMarkerScale(0.0, 0.0, 1.0);
  marker_factor_text.color = autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 0.999);
  marker_factor_text.text = stop_factor;
  msg.markers.push_back(marker_factor_text);

  return msg;
}

MarkerArray createPoseLineMarkerArray(
  const Pose & pose, const std::string & ns, const int64_t id, const double r, const double g,
  const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker_line{};
  marker_line.header.frame_id = "map";
  marker_line.header.stamp = current_time;
  marker_line.ns = ns + "_line";
  marker_line.id = id;
  marker_line.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_line.type = Marker::LINE_STRIP;
  marker_line.action = Marker::ADD;
  marker_line.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  marker_line.scale = autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
  marker_line.color = autoware_utils::createMarkerColor(r, g, b, 0.999);

  const double yaw = tf2::getYaw(pose.orientation);

  const double a = 3.0;
  Point p0;
  p0.x = pose.position.x - a * std::sin(yaw);
  p0.y = pose.position.y + a * std::cos(yaw);
  p0.z = pose.position.z;
  marker_line.points.push_back(p0);

  Point p1;
  p1.x = pose.position.x + a * std::sin(yaw);
  p1.y = pose.position.y - a * std::cos(yaw);
  p1.z = pose.position.z;
  marker_line.points.push_back(p1);

  msg.markers.push_back(marker_line);

  return msg;
}

MarkerArray createPoseMarkerArray(
  const Pose & pose, const std::string & ns, const int64_t id, const double r, const double g,
  const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;
  marker.id = id;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.type = Marker::ARROW;
  marker.action = Marker::ADD;
  marker.pose = pose;
  marker.scale = autoware_utils::createMarkerScale(0.7, 0.3, 0.3);
  marker.color = autoware_utils::createMarkerColor(r, g, b, 0.999);
  msg.markers.push_back(marker);

  return msg;
}

}  // namespace marker_utils

std::string toStrInfo(const behavior_path_planner::ShiftPointArray & sp_arr)
{
  if (sp_arr.empty()) {
    return "point is empty";
  }
  std::stringstream ss;
  for (const auto & sp : sp_arr) {
    ss << std::endl << toStrInfo(sp);
  }
  return ss.str();
}

std::string toStrInfo(const behavior_path_planner::ShiftPoint & sp)
{
  const auto & ps = sp.start.position;
  const auto & pe = sp.end.position;
  std::stringstream ss;
  ss << "shift length: " << sp.length << ", start_idx: " << sp.start_idx
     << ", end_idx: " << sp.end_idx << ", start: (" << ps.x << ", " << ps.y << "), end: (" << pe.x
     << ", " << pe.y << ")";
  return ss.str();
}

std::string toStrInfo(const behavior_path_planner::AvoidPointArray & ap_arr)
{
  if (ap_arr.empty()) {
    return "point is empty";
  }
  std::stringstream ss;
  for (const auto & ap : ap_arr) {
    ss << std::endl << toStrInfo(ap);
  }
  return ss.str();
}

std::string toStrInfo(const behavior_path_planner::AvoidPoint & ap)
{
  std::stringstream pids;
  for (const auto pid : ap.parent_ids) {
    pids << pid << ", ";
  }
  const auto & ps = ap.start.position;
  const auto & pe = ap.end.position;
  std::stringstream ss;
  ss << "id = " << ap.id << ", shift length: " << ap.length << ", start_idx: " << ap.start_idx
     << ", end_idx: " << ap.end_idx << ", start_dist = " << ap.start_longitudinal
     << ", end_dist = " << ap.end_longitudinal << ", start_length: " << ap.start_length
     << ", start: (" << ps.x << ", " << ps.y << "), end: (" << pe.x << ", " << pe.y
     << "), relative_length: " << ap.getRelativeLength() << ", grad = " << ap.getGradient()
     << ", parent_ids = [" << pids.str() << "]";
  return ss.str();
}
