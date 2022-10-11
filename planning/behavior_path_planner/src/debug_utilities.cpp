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

#include "behavior_path_planner/debug_utilities.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <tier4_autoware_utils/ros/marker_helper.hpp>

namespace marker_utils
{
using behavior_path_planner::ShiftLine;
using behavior_path_planner::util::calcPathArcLengthArray;
using behavior_path_planner::util::shiftPose;
using std_msgs::msg::ColorRGBA;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerOrientation;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;
using visualization_msgs::msg::Marker;

MarkerArray createPoseMarkerArray(
  const Pose & pose, std::string && ns, const int32_t & id, const float & r, const float & g,
  const float & b)
{
  MarkerArray msg;

  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, id, Marker::ARROW,
    createMarkerScale(0.7, 0.3, 0.3), createMarkerColor(r, g, b, 0.999));
  marker.pose = pose;
  msg.markers.push_back(marker);

  return msg;
}

MarkerArray createPathMarkerArray(
  const PathWithLaneId & path, std::string && ns, const int64_t & lane_id, const float & r,
  const float & g, const float & b)
{
  const auto arclength = calcPathArcLengthArray(path);
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  const int32_t uid = bitShift(lane_id);
  int32_t i{0};
  int32_t idx{0};

  MarkerArray msg;
  const auto reserve_size = path.points.size() + static_cast<size_t>(path.points.size() / 10);
  msg.markers.reserve(reserve_size);

  Marker marker = createDefaultMarker(
    "map", current_time, ns, 0L, Marker::ARROW, createMarkerScale(0.2, 0.1, 0.3),
    createMarkerColor(r, g, b, 0.999));

  Marker marker_text = createDefaultMarker(
    "map", current_time, ns, 0L, Marker::TEXT_VIEW_FACING, createMarkerScale(0.2, 0.1, 0.3),
    createMarkerColor(1, 1, 1, 0.999));

  for (const auto & p : path.points) {
    marker.id = uid + i++;
    marker.pose = p.point.pose;
    msg.markers.push_back(marker);
    if (idx % 10 == 0) {
      marker_text.id = uid + i++;
      std::stringstream ss;
      ss << std::fixed << std::setprecision(1) << "i=" << idx << "\ns=" << arclength.at(idx);
      marker_text.text = ss.str();
      msg.markers.push_back(marker_text);
    }
    ++idx;
  }

  return msg;
}
MarkerArray createShiftLineMarkerArray(
  const ShiftLineArray & shift_lines, const double & base_shift, std::string && ns, const float & r,
  const float & g, const float & b, const float & w)
{
  ShiftLineArray shift_lines_local = shift_lines;
  if (shift_lines.empty()) {
    shift_lines_local.push_back(ShiftLine());
  }

  MarkerArray msg;
  msg.markers.reserve(shift_lines_local.size() * 3);
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  int id{0};

  // TODO(Horibe) now assuming the shift point is aligned in longitudinal distance order
  double current_shift = base_shift;
  for (const auto & sp : shift_lines_local) {
    // ROS_ERROR("sp: s = (%f, %f), g = (%f, %f)", sp.start.x, sp.start.y, sp.end.x, sp.end.y);
    Marker basic_marker = createDefaultMarker(
      "map", current_time, ns, 0L, Marker::CUBE, createMarkerScale(0.5, 0.5, 0.5),
      createMarkerColor(r, g, b, 0.5));
    basic_marker.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
    {
      // start point
      auto marker_s = basic_marker;
      marker_s.id = ++id;
      marker_s.pose = sp.start;
      shiftPose(&marker_s.pose, current_shift);  // old
      msg.markers.push_back(marker_s);

      // end point
      auto marker_e = basic_marker;
      marker_e.id = ++id;
      marker_e.pose = sp.end;
      shiftPose(&marker_e.pose, sp.end_shift_length);
      msg.markers.push_back(marker_e);

      // start-to-end line
      auto marker_l = basic_marker;
      marker_l.id = ++id;
      marker_l.type = Marker::LINE_STRIP;
      marker_l.scale = createMarkerScale(w, 0.0, 0.0);
      marker_l.points.reserve(2);
      marker_l.points.push_back(marker_s.pose.position);
      marker_l.points.push_back(marker_e.pose.position);
      msg.markers.push_back(marker_l);
    }
    current_shift = sp.end_shift_length;
  }

  return msg;
}

MarkerArray createShiftLengthMarkerArray(
  const std::vector<double> & shift_distance,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & reference, std::string && ns,
  const float & r, const float & g, const float & b)
{
  if (shift_distance.size() != reference.points.size()) {
    return MarkerArray{};
  }

  MarkerArray msg;

  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::LINE_STRIP,
    createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(r, g, b, 0.9));
  marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  marker.points.reserve(shift_distance.size());

  for (size_t i = 0; i < shift_distance.size(); ++i) {
    auto p = reference.points.at(i).point.pose;
    shiftPose(&p, shift_distance.at(i));
    marker.points.push_back(p.position);
  }

  msg.markers.push_back(marker);
  return msg;
}

MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, std::string && ns, const float & r,
  const float & g, const float & b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  for (const auto & lanelet : lanelets) {
    Marker marker = createDefaultMarker(

      "map", current_time, ns, static_cast<int32_t>(lanelet.id()), Marker::LINE_STRIP,
      createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(r, g, b, 0.999));
    marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);

    if (!lanelet.polygon3d().empty()) {
      marker.points.reserve(lanelet.polygon3d().size() + 1);
    }

    for (const auto & p : lanelet.polygon3d()) {
      marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
    }

    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }

    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createFurthestLineStringMarkerArray(const lanelet::ConstLineStrings3d & linestrings)
{
  if (linestrings.empty()) {
    return MarkerArray();
  }

  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "shared_linestring_lanelets", 0L, Marker::LINE_STRIP,
    createMarkerScale(0.3, 0.0, 0.0), createMarkerColor(0.996, 0.658, 0.466, 0.999));
  marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);

  const auto reserve_size = linestrings.size() / 2;
  lanelet::ConstLineStrings3d lefts;
  lanelet::ConstLineStrings3d rights;
  lefts.reserve(reserve_size);
  rights.reserve(reserve_size);

  size_t total_marker_reserve_size{0};
  for (size_t idx = 1; idx < linestrings.size(); idx += 2) {
    rights.emplace_back(linestrings.at(idx - 1));
    lefts.emplace_back(linestrings.at(idx));

    for (const auto & ls : linestrings.at(idx - 1).basicLineString()) {
      total_marker_reserve_size += ls.size();
    }
    for (const auto & ls : linestrings.at(idx).basicLineString()) {
      total_marker_reserve_size += ls.size();
    }
  }

  if (!total_marker_reserve_size) {
    marker.points.reserve(total_marker_reserve_size);
  }

  const auto & first_ls = lefts.front().basicLineString();
  for (const auto & ls : first_ls) {
    marker.points.push_back(createPoint(ls.x(), ls.y(), ls.z()));
  }

  for (auto idx = lefts.cbegin() + 1; idx != lefts.cend(); ++idx) {
    Point front = createPoint(
      idx->basicLineString().front().x(), idx->basicLineString().front().y(),
      idx->basicLineString().front().z());
    Point front_inverted = createPoint(
      idx->invert().basicLineString().front().x(), idx->invert().basicLineString().front().y(),
      idx->invert().basicLineString().front().z());

    const auto & marker_back = marker.points.back();
    const bool isFrontNear = tier4_autoware_utils::calcDistance2d(marker_back, front) <
                             tier4_autoware_utils::calcDistance2d(marker_back, front_inverted);
    const auto & left_ls = (isFrontNear) ? idx->basicLineString() : idx->invert().basicLineString();
    for (auto ls = left_ls.cbegin(); ls != left_ls.cend(); ++ls) {
      marker.points.push_back(createPoint(ls->x(), ls->y(), ls->z()));
    }
  }

  for (auto idx = rights.crbegin(); idx != rights.crend(); ++idx) {
    Point front = createPoint(
      idx->basicLineString().front().x(), idx->basicLineString().front().y(),
      idx->basicLineString().front().z());
    Point front_inverted = createPoint(
      idx->invert().basicLineString().front().x(), idx->invert().basicLineString().front().y(),
      idx->invert().basicLineString().front().z());

    const auto & marker_back = marker.points.back();
    const bool isFrontFurther = tier4_autoware_utils::calcDistance2d(marker_back, front) >
                                tier4_autoware_utils::calcDistance2d(marker_back, front_inverted);
    const auto & right_ls =
      (isFrontFurther) ? idx->basicLineString() : idx->invert().basicLineString();
    for (auto ls = right_ls.crbegin(); ls != right_ls.crend(); ++ls) {
      marker.points.push_back(createPoint(ls->x(), ls->y(), ls->z()));
    }
  }

  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }

  MarkerArray msg;

  msg.markers.push_back(marker);
  return msg;
}

MarkerArray createPolygonMarkerArray(
  const Polygon & polygon, std::string && ns, const int64_t & lane_id, const float & r,
  const float & g, const float & b)
{
  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, static_cast<int32_t>(lane_id), Marker::LINE_STRIP,
    createMarkerScale(0.3, 0.0, 0.0), createMarkerColor(r, g, b, 0.8));

  marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);

  if (!polygon.points.empty()) {
    marker.points.reserve(polygon.points.size() + 1);
  }

  for (const auto & p : polygon.points) {
    marker.points.push_back(createPoint(p.x, p.y, p.z));
  }
  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }

  MarkerArray msg;
  msg.markers.push_back(marker);

  return msg;
}

MarkerArray createObjectsMarkerArray(
  const PredictedObjects & objects, std::string && ns, const int64_t & lane_id, const float & r,
  const float & g, const float & b)
{
  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::CUBE,
    createMarkerScale(3.0, 1.0, 1.0), createMarkerColor(r, g, b, 0.8));

  int32_t uid = bitShift(lane_id);
  int32_t i{0};

  MarkerArray msg;
  msg.markers.reserve(objects.objects.size());

  for (const auto & object : objects.objects) {
    marker.id = uid + i++;
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace marker_utils
