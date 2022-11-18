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

namespace marker_utils::avoidance_marker
{
using behavior_path_planner::AvoidLine;
using behavior_path_planner::util::shiftPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;
using visualization_msgs::msg::Marker;

MarkerArray createAvoidLineMarkerArray(
  const AvoidLineArray & shift_lines, std::string && ns, const float & r, const float & g,
  const float & b, const double & w)
{
  AvoidLineArray shift_lines_local = shift_lines;
  if (shift_lines_local.empty()) {
    shift_lines_local.push_back(AvoidLine());
  }

  int32_t id{0};
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  for (const auto & sl : shift_lines_local) {
    // ROS_ERROR("sl: s = (%f, %f), g = (%f, %f)", sl.start.x, sl.start.y, sl.end.x, sl.end.y);
    Marker basic_marker = createDefaultMarker(
      "map", current_time, ns, 0L, Marker::CUBE, createMarkerScale(0.5, 0.5, 0.5),
      createMarkerColor(r, g, b, 0.9));
    basic_marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    {
      // start point
      auto marker_s = basic_marker;
      marker_s.id = id++;
      marker_s.pose = sl.start;
      // shiftPose(&marker_s.pose, current_shift);  // old
      shiftPose(&marker_s.pose, sl.start_shift_length);
      msg.markers.push_back(marker_s);

      // end point
      auto marker_e = basic_marker;
      marker_e.id = id++;
      marker_e.pose = sl.end;
      shiftPose(&marker_e.pose, sl.end_shift_length);
      msg.markers.push_back(marker_e);

      // start-to-end line
      auto marker_l = basic_marker;
      marker_l.id = id++;
      marker_l.type = Marker::LINE_STRIP;
      marker_l.scale = tier4_autoware_utils::createMarkerScale(w, 0.0, 0.0);
      marker_l.points.push_back(marker_s.pose.position);
      marker_l.points.push_back(marker_e.pose.position);
      msg.markers.push_back(marker_l);
    }
    // current_shift = sp.length;
  }

  return msg;
}

MarkerArray createTargetObjectsMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects, std::string && ns)
{
  const auto normal_color = tier4_autoware_utils::createMarkerColor(0.9, 0.0, 0.0, 0.8);
  const auto disappearing_color = tier4_autoware_utils::createMarkerColor(0.9, 0.5, 0.9, 0.6);

  const auto uuid_to_id = [](const unique_identifier_msgs::msg::UUID & object_id) {
    int32_t ret = 0;

    for (size_t i = 0; i < sizeof(int32_t) / sizeof(int8_t); ++i) {
      ret <<= sizeof(int8_t);
      ret |= object_id.uuid.at(i);
    }

    return ret;
  };

  MarkerArray msg;
  msg.markers.reserve(objects.size() * 2);

  {
    auto marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::CUBE,
      createMarkerScale(3.0, 1.5, 1.5), normal_color);
    for (const auto & object : objects) {
      marker.id = uuid_to_id(object.object.object_id);
      marker.pose = object.object.kinematics.initial_pose_with_covariance.pose;
      marker.scale = tier4_autoware_utils::createMarkerScale(3.0, 1.5, 1.5);
      marker.color = std::fabs(object.lost_time) < 1e-2 ? normal_color : disappearing_color;
      msg.markers.push_back(marker);
    }
  }

  {
    for (const auto & object : objects) {
      const auto pos = object.object.kinematics.initial_pose_with_covariance.pose.position;

      {
        auto marker = createDefaultMarker(
          "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns + "_envelope_polygon", 0L,
          Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0),
          createMarkerColor(1.0, 1.0, 1.0, 0.999));

        for (const auto & p : object.envelope_poly.outer()) {
          marker.points.push_back(createPoint(p.x(), p.y(), pos.z));
        }

        marker.points.push_back(marker.points.front());
        marker.id = uuid_to_id(object.object.object_id);
        msg.markers.push_back(marker);
      }
    }
  }

  return msg;
}

MarkerArray createOtherObjectsMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects, std::string && ns)
{
  const auto normal_color = tier4_autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.8);

  const auto uuid_to_id = [](const unique_identifier_msgs::msg::UUID & object_id) {
    int32_t ret = 0;

    for (size_t i = 0; i < sizeof(int32_t) / sizeof(int8_t); ++i) {
      ret <<= sizeof(int8_t);
      ret |= object_id.uuid.at(i);
    }

    return ret;
  };

  MarkerArray msg;
  msg.markers.reserve(objects.size());

  {
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::CUBE,
      createMarkerScale(3.0, 1.5, 1.5), normal_color);
    for (const auto & object : objects) {
      marker.id = uuid_to_id(object.object.object_id);
      marker.pose = object.object.kinematics.initial_pose_with_covariance.pose;
      marker.scale = tier4_autoware_utils::createMarkerScale(3.0, 1.5, 1.5);
      marker.color = normal_color;
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

MarkerArray makeOverhangToRoadShoulderMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects, std::string && ns)
{
  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::TEXT_VIEW_FACING,
    createMarkerScale(1.0, 1.0, 1.0), createMarkerColor(1.0, 1.0, 0.0, 1.0));

  int32_t i{0};
  MarkerArray msg;
  for (const auto & object : objects) {
    marker.id = ++i;
    marker.pose = object.overhang_pose;
    std::ostringstream string_stream;
    string_stream << "(to_road_shoulder_distance = " << object.to_road_shoulder_distance << " [m])";
    marker.text = string_stream.str();
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createOverhangFurthestLineStringMarkerArray(
  const lanelet::ConstLineStrings3d & linestrings, std::string && ns, const float & r,
  const float & g, const float & b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  for (const auto & linestring : linestrings) {
    const auto id = static_cast<int>(linestring.id());
    Marker marker = createDefaultMarker(
      "map", current_time, ns, id, Marker::LINE_STRIP, createMarkerScale(0.4, 0.0, 0.0),
      createMarkerColor(r, g, b, 0.999));

    marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    for (const auto & p : linestring.basicLineString()) {
      marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
    }
    msg.markers.push_back(marker);

    Marker marker_linestring_id = createDefaultMarker(
      "map", current_time, "linestring_id", id, Marker::TEXT_VIEW_FACING,
      createMarkerScale(1.5, 1.5, 1.5), createMarkerColor(1.0, 1.0, 1.0, 0.8));
    Pose text_id_pose;
    text_id_pose.position.x = linestring.front().x();
    text_id_pose.position.y = linestring.front().y();
    text_id_pose.position.z = linestring.front().z();
    marker_linestring_id.pose = text_id_pose;
    std::ostringstream ss;
    ss << "(ID : " << id << ") ";
    marker_linestring_id.text = ss.str();
    msg.markers.push_back(marker_linestring_id);
  }

  return msg;
}
}  // namespace marker_utils::avoidance_marker

std::string toStrInfo(const behavior_path_planner::ShiftLineArray & sl_arr)
{
  if (sl_arr.empty()) {
    return "point is empty";
  }
  std::stringstream ss;
  for (const auto & sl : sl_arr) {
    ss << std::endl << toStrInfo(sl);
  }
  return ss.str();
}

std::string toStrInfo(const behavior_path_planner::ShiftLine & sl)
{
  const auto & ps = sl.start.position;
  const auto & pe = sl.end.position;
  std::stringstream ss;
  ss << "shift length: " << sl.end_shift_length << ", start_idx: " << sl.start_idx
     << ", end_idx: " << sl.end_idx << ", start: (" << ps.x << ", " << ps.y << "), end: (" << pe.x
     << ", " << pe.y << ")";
  return ss.str();
}

std::string toStrInfo(const behavior_path_planner::AvoidLineArray & ap_arr)
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
std::string toStrInfo(const behavior_path_planner::AvoidLine & ap)
{
  std::stringstream pids;
  for (const auto pid : ap.parent_ids) {
    pids << pid << ", ";
  }
  const auto & ps = ap.start.position;
  const auto & pe = ap.end.position;
  std::stringstream ss;
  ss << "id = " << ap.id << ", shift length: " << ap.end_shift_length
     << ", start_idx: " << ap.start_idx << ", end_idx: " << ap.end_idx
     << ", start_dist = " << ap.start_longitudinal << ", end_dist = " << ap.end_longitudinal
     << ", start_shift_length: " << ap.start_shift_length << ", start: (" << ps.x << ", " << ps.y
     << "), end: (" << pe.x << ", " << pe.y << "), relative_length: " << ap.getRelativeLength()
     << ", grad = " << ap.getGradient() << ", parent_ids = [" << pids.str() << "]";
  return ss.str();
}
