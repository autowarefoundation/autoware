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

#include "behavior_path_avoidance_module/debug.hpp"

#include "behavior_path_planner_common/marker_utils/utils.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"

#include <lanelet2_extension/visualization/visualization.hpp>
#include <magic_enum.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <tier4_planning_msgs/msg/avoidance_debug_factor.hpp>

#include <string>
#include <vector>

namespace marker_utils::avoidance_marker
{

using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::getPose;
using visualization_msgs::msg::Marker;

namespace
{

int32_t uuidToInt32(const unique_identifier_msgs::msg::UUID & uuid)
{
  int32_t ret = 0;

  for (size_t i = 0; i < sizeof(int32_t) / sizeof(int8_t); ++i) {
    ret <<= sizeof(int8_t);
    ret |= uuid.uuid.at(i);
  }

  return ret;
}

MarkerArray createObjectsCubeMarkerArray(
  const ObjectDataArray & objects, std::string && ns, const Vector3 & scale,
  const ColorRGBA & color)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;

  MarkerArray msg;

  const auto is_small_object = [](const auto & o) {
    const auto t = behavior_path_planner::utils::getHighestProbLabel(o.classification);
    return t == ObjectClassification::PEDESTRIAN || t == ObjectClassification::BICYCLE ||
           t == ObjectClassification::MOTORCYCLE || t == ObjectClassification::UNKNOWN;
  };

  for (const auto & object : objects) {
    auto marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::CUBE, scale, color);

    if (is_small_object(object.object)) {
      marker.scale = createMarkerScale(0.5, 0.5, 1.5);
      marker.type = Marker::CYLINDER;
    }

    marker.id = uuidToInt32(object.object.object_id);
    marker.pose = object.object.kinematics.initial_pose_with_covariance.pose;
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createObjectPolygonMarkerArray(const ObjectDataArray & objects, std::string && ns)
{
  MarkerArray msg;

  for (const auto & object : objects) {
    auto marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::LINE_STRIP,
      createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));

    const auto pos = object.object.kinematics.initial_pose_with_covariance.pose.position;

    for (const auto & p : object.envelope_poly.outer()) {
      marker.points.push_back(createPoint(p.x(), p.y(), pos.z));
    }

    marker.points.push_back(marker.points.front());
    marker.id = uuidToInt32(object.object.object_id);
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createToDrivableBoundDistance(const ObjectDataArray & objects, std::string && ns)
{
  MarkerArray msg;

  for (const auto & object : objects) {
    if (!object.nearest_bound_point.has_value()) {
      continue;
    }

    {
      auto marker = createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::LINE_STRIP,
        createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(1.0, 0.0, 0.42, 0.999));

      marker.points.push_back(object.overhang_pose.position);
      marker.points.push_back(object.nearest_bound_point.value());
      marker.id = uuidToInt32(object.object.object_id);
      msg.markers.push_back(marker);
    }

    {
      auto marker = createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::TEXT_VIEW_FACING,
        createMarkerScale(0.5, 0.5, 0.5), createMarkerColor(1.0, 1.0, 0.0, 1.0));

      marker.pose.position = object.nearest_bound_point.value();
      std::ostringstream string_stream;
      string_stream << object.to_road_shoulder_distance << "[m]";
      marker.text = string_stream.str();
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

MarkerArray createObjectInfoMarkerArray(const ObjectDataArray & objects, std::string && ns)
{
  MarkerArray msg;

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.5, 0.5, 0.5), createMarkerColor(1.0, 1.0, 0.0, 1.0));

  for (const auto & object : objects) {
    {
      marker.id = uuidToInt32(object.object.object_id);
      marker.pose = object.object.kinematics.initial_pose_with_covariance.pose;
      std::ostringstream string_stream;
      string_stream << std::fixed << std::setprecision(2) << std::boolalpha;
      string_stream << "ratio:" << object.shiftable_ratio << " [-]\n"
                    << "lateral:" << object.to_centerline << " [m]\n"
                    << "necessity:" << object.avoid_required << " [-]\n"
                    << "stoppable:" << object.is_stoppable << " [-]\n"
                    << "stop_factor:" << object.to_stop_factor_distance << " [m]\n"
                    << "move_time:" << object.move_time << " [s]\n"
                    << "stop_time:" << object.stop_time << " [s]\n";
      marker.text = string_stream.str();
      marker.color = createMarkerColor(1.0, 1.0, 0.0, 0.999);
      marker.scale = createMarkerScale(0.5, 0.5, 0.5);
      marker.ns = ns;
      msg.markers.push_back(marker);
    }

    {
      marker.id = uuidToInt32(object.object.object_id);
      marker.pose.position.z += 2.0;
      std::ostringstream string_stream;
      string_stream << object.reason;
      marker.text = string_stream.str();
      marker.color = createMarkerColor(1.0, 1.0, 1.0, 0.999);
      marker.scale = createMarkerScale(0.6, 0.6, 0.6);
      marker.ns = ns + "_reason";
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

MarkerArray avoidableObjectsMarkerArray(const ObjectDataArray & objects, std::string && ns)
{
  MarkerArray msg;
  msg.markers.reserve(objects.size() * 4);

  appendMarkerArray(
    createObjectsCubeMarkerArray(
      objects, ns + "_cube", createMarkerScale(3.0, 1.5, 1.5),
      createMarkerColor(1.0, 1.0, 0.0, 0.8)),
    &msg);

  appendMarkerArray(createObjectInfoMarkerArray(objects, ns + "_info"), &msg);
  appendMarkerArray(createObjectPolygonMarkerArray(objects, ns + "_envelope_polygon"), &msg);
  appendMarkerArray(createToDrivableBoundDistance(objects, ns + "_to_drivable_bound"), &msg);

  return msg;
}

MarkerArray unAvoidableObjectsMarkerArray(const ObjectDataArray & objects, std::string && ns)
{
  MarkerArray msg;
  msg.markers.reserve(objects.size() * 4);

  appendMarkerArray(
    createObjectsCubeMarkerArray(
      objects, ns + "_cube", createMarkerScale(3.0, 1.5, 1.5),
      createMarkerColor(1.0, 0.0, 0.0, 0.8)),
    &msg);

  appendMarkerArray(createObjectInfoMarkerArray(objects, ns + "_info"), &msg);
  appendMarkerArray(createObjectPolygonMarkerArray(objects, ns + "_envelope_polygon"), &msg);
  appendMarkerArray(createToDrivableBoundDistance(objects, ns + "_to_drivable_bound"), &msg);

  return msg;
}

}  // namespace

MarkerArray createEgoStatusMarkerArray(
  const AvoidancePlanningData & data, const Pose & p_ego, std::string && ns)
{
  MarkerArray msg;

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.5, 0.5, 0.5), createMarkerColor(1.0, 1.0, 1.0, 0.999));
  marker.pose = p_ego;

  {
    std::ostringstream string_stream;
    string_stream << std::fixed << std::setprecision(2) << std::boolalpha;
    string_stream << "avoid_req:" << data.avoid_required << ","
                  << "yield_req:" << data.yield_required << ","
                  << "safe:" << data.safe;
    marker.text = string_stream.str();

    msg.markers.push_back(marker);
  }

  {
    std::ostringstream string_stream;
    string_stream << "ego_state:";
    string_stream << magic_enum::enum_name(data.state);
    marker.text = string_stream.str();
    marker.pose.position.z += 2.0;
    marker.id++;

    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createAvoidLineMarkerArray(
  const AvoidLineArray & shift_lines, std::string && ns, const float & r, const float & g,
  const float & b, const double & w)
{
  MarkerArray msg;

  if (shift_lines.empty()) {
    return msg;
  }

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::CUBE,
    createMarkerScale(0.1, 0.1, 0.1), createMarkerColor(r, g, b, 0.9));

  int32_t shift_line_id{0};
  int32_t marker_id{0};
  for (const auto & s : shift_lines) {
    // shift line
    {
      auto m = marker;
      m.id = marker_id++;
      m.type = Marker::LINE_STRIP;
      m.scale = createMarkerScale(w, 0.0, 0.0);
      m.points.push_back(calcOffsetPose(s.start, 0.0, s.start_shift_length, 0.0).position);
      m.points.push_back(calcOffsetPose(s.end, 0.0, s.end_shift_length, 0.0).position);
      msg.markers.push_back(m);
    }

    // start point
    {
      auto m = marker;
      m.id = marker_id++;
      m.type = Marker::CUBE;
      m.scale = createMarkerScale(0.2, 0.2, 0.2);
      m.pose = calcOffsetPose(s.start, 0.0, s.start_shift_length, 0.0);
      msg.markers.push_back(m);
    }

    // end point
    {
      auto m = marker;
      m.id = marker_id++;
      m.type = Marker::CUBE;
      m.scale = createMarkerScale(0.2, 0.2, 0.2);
      m.pose = calcOffsetPose(s.end, 0.0, s.end_shift_length, 0.0);
      msg.markers.push_back(m);
    }

    // start text
    {
      auto m = marker;
      std::ostringstream string_stream;
      string_stream << "(S):" << shift_line_id;
      m.id = marker_id++;
      m.type = Marker::TEXT_VIEW_FACING;
      m.scale = createMarkerScale(0.3, 0.3, 0.3);
      m.pose = calcOffsetPose(s.start, 0.0, s.start_shift_length + 0.3, 0.0);
      m.text = string_stream.str();
      msg.markers.push_back(m);
    }

    // end text
    {
      auto m = marker;
      std::ostringstream string_stream;
      string_stream << "(E):" << shift_line_id;
      m.id = marker_id++;
      m.type = Marker::TEXT_VIEW_FACING;
      m.scale = createMarkerScale(0.3, 0.3, 0.3);
      m.pose = calcOffsetPose(s.end, 0.0, s.end_shift_length - 0.3, 0.0);
      m.text = string_stream.str();
      msg.markers.push_back(m);
    }

    shift_line_id++;
  }

  return msg;
}

MarkerArray createPredictedVehiclePositions(const PathWithLaneId & path, std::string && ns)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  auto p_marker = createDefaultMarker(
    "map", current_time, ns, 0L, Marker::POINTS, createMarkerScale(0.4, 0.4, 0.0),
    createMarkerColor(1.0, 0.0, 0.0, 0.999));

  const auto pushPointMarker = [&](const Pose & p, const double t) {
    const auto r = t > 10.0 ? 1.0 : t / 10.0;
    p_marker.points.push_back(p.position);
    p_marker.colors.push_back(createMarkerColor(r, 1.0 - r, 0.0, 0.999));
  };

  auto t_marker = createDefaultMarker(
    "map", current_time, ns + "_text", 0L, Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.3, 0.3, 0.3), createMarkerColor(1.0, 1.0, 0.0, 1.0));

  const auto pushTextMarker = [&](const Pose & p, const double t, const double d, const double v) {
    t_marker.id++;
    t_marker.pose = p;
    std::ostringstream string_stream;
    string_stream << std::fixed << std::setprecision(2);
    string_stream << "t[s]: " << t << "\n"
                  << "d[m]: " << d << "\n"
                  << "v[m/s]: " << v;
    t_marker.text = string_stream.str();
    msg.markers.push_back(t_marker);
  };

  constexpr double dt_save = 1.0;
  double t_save = 0.0;
  double t_sum = 0.0;
  double d_sum = 0.0;

  if (path.points.empty()) {
    return msg;
  }

  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & p1 = path.points.at(i - 1);
    const auto & p2 = path.points.at(i);
    const auto ds = calcDistance2d(p1, p2);

    if (t_save < t_sum + 1e-3) {
      pushPointMarker(getPose(p1), t_sum);
      pushTextMarker(getPose(p1), t_sum, d_sum, p1.point.longitudinal_velocity_mps);
      t_save += dt_save;
    }

    const auto v = std::max(p1.point.longitudinal_velocity_mps, float{1.0});

    t_sum += ds / v;
    d_sum += ds;
  }

  msg.markers.push_back(p_marker);

  return msg;
}

MarkerArray createTargetObjectsMarkerArray(const ObjectDataArray & objects, const std::string & ns)
{
  ObjectDataArray avoidable;
  ObjectDataArray unavoidable;
  for (const auto & o : objects) {
    if (o.is_avoidable) {
      avoidable.push_back(o);
    } else {
      unavoidable.push_back(o);
    }
  }

  MarkerArray msg;
  msg.markers.reserve(objects.size() * 4);

  appendMarkerArray(avoidableObjectsMarkerArray(avoidable, "avoidable_" + ns), &msg);
  appendMarkerArray(unAvoidableObjectsMarkerArray(unavoidable, "unavoidable_" + ns), &msg);

  return msg;
}

MarkerArray createOtherObjectsMarkerArray(const ObjectDataArray & objects, const std::string & ns)
{
  using behavior_path_planner::utils::convertToSnakeCase;

  const auto filtered_objects = [&objects, &ns]() {
    ObjectDataArray ret{};
    for (const auto & o : objects) {
      if (o.reason != ns) {
        continue;
      }
      ret.push_back(o);
    }

    return ret;
  }();

  MarkerArray msg;
  msg.markers.reserve(filtered_objects.size() * 2);

  appendMarkerArray(
    createObjectsCubeMarkerArray(
      filtered_objects, "others_" + convertToSnakeCase(ns) + "_cube",
      createMarkerScale(3.0, 1.5, 1.5), createMarkerColor(0.0, 1.0, 0.0, 0.8)),
    &msg);
  appendMarkerArray(
    createObjectInfoMarkerArray(filtered_objects, "others_" + convertToSnakeCase(ns) + "_info"),
    &msg);

  return msg;
}

MarkerArray createDrivableBounds(
  const AvoidancePlanningData & data, std::string && ns, const float & r, const float & g,
  const float & b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  // right bound
  {
    auto marker = createDefaultMarker(
      "map", current_time, ns + "_right", 0L, Marker::LINE_STRIP, createMarkerScale(0.4, 0.0, 0.0),
      createMarkerColor(r, g, b, 0.999));

    for (const auto & p : data.right_bound) {
      marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
    }

    msg.markers.push_back(marker);
  }

  // left bound
  {
    auto marker = createDefaultMarker(
      "map", current_time, ns + "_left", 0L, Marker::LINE_STRIP, createMarkerScale(0.4, 0.0, 0.0),
      createMarkerColor(r, g, b, 0.999));

    for (const auto & p : data.left_bound) {
      marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
    }

    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createDebugMarkerArray(
  const AvoidancePlanningData & data, const PathShifter & shifter, const DebugData & debug)
{
  using behavior_path_planner::utils::transformToLanelets;
  using lanelet::visualization::laneletsAsTriangleMarkerArray;
  using marker_utils::createLaneletsAreaMarkerArray;
  using marker_utils::createObjectsMarkerArray;
  using marker_utils::createPathMarkerArray;
  using marker_utils::createPolygonMarkerArray;
  using marker_utils::createPoseMarkerArray;
  using marker_utils::createShiftGradMarkerArray;
  using marker_utils::createShiftLengthMarkerArray;
  using marker_utils::createShiftLineMarkerArray;
  using marker_utils::showPolygon;
  using marker_utils::showPredictedPath;
  using marker_utils::showSafetyCheckInfo;
  using tier4_planning_msgs::msg::AvoidanceDebugFactor;

  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  const auto & path = data.reference_path;

  const auto add = [&msg](const MarkerArray & added) { appendMarkerArray(added, &msg); };

  const auto addAvoidLine =
    [&](const AvoidLineArray & al_arr, const auto & ns, auto r, auto g, auto b, double w = 0.05) {
      add(createAvoidLineMarkerArray(al_arr, ns, r, g, b, w));
    };

  const auto addShiftLine =
    [&](const ShiftLineArray & sl_arr, const auto & ns, auto r, auto g, auto b, double w = 0.1) {
      add(createShiftLineMarkerArray(sl_arr, shifter.getBaseOffset(), ns, r, g, b, w));
    };

  const auto addObjects = [&](const ObjectDataArray & objects, const auto & ns) {
    add(createOtherObjectsMarkerArray(objects, ns));
  };

  const auto addShiftLength =
    [&](const auto & shift_length, const auto & ns, auto r, auto g, auto b) {
      add(createShiftLengthMarkerArray(shift_length, path, ns, r, g, b));
    };

  const auto addShiftGrad = [&](
                              const auto & shift_grad, const auto & shift_length, const auto & ns,
                              auto r, auto g, auto b) {
    add(createShiftGradMarkerArray(shift_grad, shift_length, path, ns, r, g, b));
  };

  // ignore objects
  {
    addObjects(data.other_objects, AvoidanceDebugFactor::OBJECT_IS_BEHIND_THRESHOLD);
    addObjects(data.other_objects, AvoidanceDebugFactor::OBJECT_IS_IN_FRONT_THRESHOLD);
    addObjects(data.other_objects, AvoidanceDebugFactor::OBJECT_IS_NOT_TYPE);
    addObjects(data.other_objects, AvoidanceDebugFactor::OBJECT_BEHIND_PATH_GOAL);
    addObjects(data.other_objects, AvoidanceDebugFactor::TOO_NEAR_TO_CENTERLINE);
    addObjects(data.other_objects, AvoidanceDebugFactor::NOT_PARKING_OBJECT);
    addObjects(data.other_objects, std::string("MovingObject"));
    addObjects(data.other_objects, std::string("CrosswalkUser"));
    addObjects(data.other_objects, std::string("OutOfTargetArea"));
    addObjects(data.other_objects, std::string("NotNeedAvoidance"));
    addObjects(data.other_objects, std::string("LessThanExecutionThreshold"));
    addObjects(data.other_objects, std::string("TooNearToGoal"));
    addObjects(data.other_objects, std::string("ParallelToEgoLane"));
    addObjects(data.other_objects, std::string("MergingToEgoLane"));
  }

  // shift line pre-process
  {
    addAvoidLine(debug.step1_registered_shift_line, "step1_registered_shift_line", 0.2, 0.2, 1.0);
    addAvoidLine(debug.step1_current_shift_line, "step1_current_shift_line", 0.2, 0.4, 0.8, 0.3);
    addAvoidLine(debug.step1_merged_shift_line, "step1_merged_shift_line", 0.2, 0.6, 0.6, 0.3);
    addAvoidLine(debug.step1_filled_shift_line, "step1_filled_shift_line", 0.2, 0.8, 0.4, 0.3);
    addAvoidLine(debug.step1_return_shift_line, "step1_return_shift_line", 0.2, 1.0, 0.2, 0.3);
  }

  // merge process
  {
    addAvoidLine(debug.step2_merged_shift_line, "step2_merged_shift_line", 0.2, 1.0, 0.0, 0.3);
  }

  // trimming process
  {
    addAvoidLine(debug.step3_grad_filtered_1st, "step3_grad_filtered_1st", 0.2, 0.8, 0.0, 0.3);
    addAvoidLine(debug.step3_grad_filtered_2nd, "step3_grad_filtered_2nd", 0.4, 0.6, 0.0, 0.3);
    addAvoidLine(debug.step3_grad_filtered_3rd, "step3_grad_filtered_3rd", 0.6, 0.4, 0.0, 0.3);
  }

  // registering process
  {
    addShiftLine(shifter.getShiftLines(), "step4_old_shift_line", 1.0, 1.0, 0.0, 0.3);
    addAvoidLine(data.new_shift_line, "step4_new_shift_line", 1.0, 0.0, 0.0, 0.3);
  }

  // safety check
  {
    add(showSafetyCheckInfo(debug.collision_check, "object_debug_info"));
    add(showPredictedPath(debug.collision_check, "ego_predicted_path"));
    add(showPolygon(debug.collision_check, "ego_and_target_polygon_relation"));
  }

  // shift length
  {
    addShiftLength(debug.pos_shift, "merged_length_pos", 0.0, 0.7, 0.5);
    addShiftLength(debug.neg_shift, "merged_length_neg", 0.0, 0.5, 0.7);
    addShiftLength(debug.total_shift, "merged_length_total", 0.99, 0.4, 0.2);
  }

  // shift grad
  {
    addShiftGrad(debug.pos_shift_grad, debug.pos_shift, "merged_grad_pos", 0.0, 0.7, 0.5);
    addShiftGrad(debug.neg_shift_grad, debug.neg_shift, "merged_grad_neg", 0.0, 0.5, 0.7);
    addShiftGrad(debug.total_forward_grad, debug.total_shift, "grad_forward", 0.99, 0.4, 0.2);
    addShiftGrad(debug.total_backward_grad, debug.total_shift, "grad_backward", 0.4, 0.2, 0.9);
  }

  // misc
  {
    add(createPathMarkerArray(path, "centerline_resampled", 0, 0.0, 0.9, 0.5));
    add(createPolygonMarkerArray(debug.detection_area, "detection_area", 0L, 0.16, 1.0, 0.69, 0.1));
    add(createDrivableBounds(data, "drivable_bound", 1.0, 0.0, 0.42));
    add(laneletsAsTriangleMarkerArray(
      "drivable_lanes", transformToLanelets(data.drivable_lanes),
      createMarkerColor(0.16, 1.0, 0.69, 0.2)));
    add(laneletsAsTriangleMarkerArray(
      "current_lanes", data.current_lanelets, createMarkerColor(1.0, 1.0, 1.0, 0.2)));
    add(laneletsAsTriangleMarkerArray(
      "safety_check_lanes", debug.safety_check_lanes, createMarkerColor(1.0, 0.0, 0.42, 0.2)));
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
  using tier4_autoware_utils::toHexString;

  std::stringstream pids;
  for (const auto pid : ap.parent_ids) {
    pids << toHexString(pid) << ", ";
  }
  const auto & ps = ap.start.position;
  const auto & pe = ap.end.position;
  std::stringstream ss;
  ss << "id = " << toHexString(ap.id) << ", shift length: " << ap.end_shift_length
     << ", start_idx: " << ap.start_idx << ", end_idx: " << ap.end_idx
     << ", start_dist = " << ap.start_longitudinal << ", end_dist = " << ap.end_longitudinal
     << ", start_shift_length: " << ap.start_shift_length << ", start: (" << ps.x << ", " << ps.y
     << "), end: (" << pe.x << ", " << pe.y << "), relative_length: " << ap.getRelativeLength()
     << ", grad = " << ap.getGradient() << ", parent_ids = [" << pids.str() << "]";
  return ss.str();
}
