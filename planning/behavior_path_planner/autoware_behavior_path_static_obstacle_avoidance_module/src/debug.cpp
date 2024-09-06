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

#include "autoware/behavior_path_static_obstacle_avoidance_module/debug.hpp"

#include "autoware/behavior_path_planner_common/marker_utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <magic_enum.hpp>

#include <string>
#include <vector>

namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance
{
namespace
{

int32_t uuidToInt32(const unique_identifier_msgs::msg::UUID & uuid)
{
  int32_t ret = 0;

  for (size_t i = 0; i < sizeof(int32_t) / sizeof(int8_t); ++i) {
    ret <<= sizeof(int8_t) * 8;
    ret |= uuid.uuid.at(i);
  }

  return ret;
}

MarkerArray createObjectsCubeMarkerArray(
  const ObjectDataArray & objects, std::string && ns, const Vector3 & scale,
  const ColorRGBA & color)
{
  using autoware_perception_msgs::msg::ObjectClassification;

  MarkerArray msg;

  const auto is_small_object = [](const auto & o) {
    const auto t = autoware::behavior_path_planner::utils::getHighestProbLabel(o.classification);
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
    marker.pose = object.getPose();
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

    for (const auto & p : object.envelope_poly.outer()) {
      marker.points.push_back(createPoint(p.x(), p.y(), object.getPosition().z));
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
    if (!object.narrowest_place.has_value()) {
      continue;
    }

    {
      auto marker = createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::LINE_STRIP,
        createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(1.0, 0.0, 0.42, 0.999));

      marker.points.push_back(object.narrowest_place.value().first);
      marker.points.push_back(object.narrowest_place.value().second);
      marker.id = uuidToInt32(object.object.object_id);
      msg.markers.push_back(marker);
    }

    {
      auto marker = createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::TEXT_VIEW_FACING,
        createMarkerScale(0.5, 0.5, 0.5), createMarkerColor(1.0, 1.0, 0.0, 1.0));

      marker.pose.position = object.narrowest_place.value().second;
      std::ostringstream string_stream;
      string_stream << object.to_road_shoulder_distance << "[m]";
      marker.text = string_stream.str();
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

MarkerArray createObjectInfoMarkerArray(
  const ObjectDataArray & objects, std::string && ns, const bool verbose)
{
  MarkerArray msg;

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.5, 0.5, 0.5), createMarkerColor(1.0, 1.0, 0.0, 1.0));

  for (const auto & object : objects) {
    if (verbose) {
      marker.id = uuidToInt32(object.object.object_id);
      marker.pose = object.getPose();
      std::ostringstream string_stream;
      string_stream << std::fixed << std::setprecision(2) << std::boolalpha;
      string_stream << "ratio:" << object.shiftable_ratio << " [-]\n"
                    << "lateral:" << object.to_centerline << " [m]\n"
                    << "clip:" << object.is_clip_target << " [-]\n"
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
      marker.pose = object.getPose();
      marker.pose.position.z += 2.0;
      std::ostringstream string_stream;
      string_stream << magic_enum::enum_name(object.info) << (object.is_parked ? "(PARKED)" : "");
      string_stream << (object.is_ambiguous ? "(WAIT AND SEE)" : "");
      marker.text = string_stream.str();
      marker.color = createMarkerColor(1.0, 1.0, 1.0, 0.999);
      marker.scale = createMarkerScale(0.6, 0.6, 0.6);
      marker.ns = ns + "_reason";
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

MarkerArray createOverhangLaneletMarkerArray(const ObjectDataArray & objects, std::string && ns)
{
  MarkerArray msg;
  msg.markers.reserve(objects.size());

  for (const auto & object : objects) {
    appendMarkerArray(
      marker_utils::createLaneletsAreaMarkerArray(
        {object.overhang_lanelet}, std::string(ns), 0.0, 0.0, 1.0),
      &msg);
  }

  return msg;
}

MarkerArray avoidableObjectsMarkerArray(const ObjectDataArray & objects, std::string && ns)
{
  MarkerArray msg;
  msg.markers.reserve(objects.size() * 5);

  appendMarkerArray(
    createObjectsCubeMarkerArray(
      objects, ns + "_cube", createMarkerScale(3.0, 1.5, 1.5),
      createMarkerColor(1.0, 1.0, 0.0, 0.8)),
    &msg);

  appendMarkerArray(createObjectInfoMarkerArray(objects, ns + "_info", true), &msg);
  appendMarkerArray(createObjectPolygonMarkerArray(objects, ns + "_envelope_polygon"), &msg);
  appendMarkerArray(createToDrivableBoundDistance(objects, ns + "_to_drivable_bound"), &msg);
  appendMarkerArray(createOverhangLaneletMarkerArray(objects, ns + "_overhang_lanelet"), &msg);

  return msg;
}

MarkerArray unAvoidableObjectsMarkerArray(const ObjectDataArray & objects, std::string && ns)
{
  MarkerArray msg;
  msg.markers.reserve(objects.size() * 5);

  appendMarkerArray(
    createObjectsCubeMarkerArray(
      objects, ns + "_cube", createMarkerScale(3.0, 1.5, 1.5),
      createMarkerColor(1.0, 0.0, 0.0, 0.8)),
    &msg);

  appendMarkerArray(createObjectInfoMarkerArray(objects, ns + "_info", true), &msg);
  appendMarkerArray(createObjectPolygonMarkerArray(objects, ns + "_envelope_polygon"), &msg);
  appendMarkerArray(createToDrivableBoundDistance(objects, ns + "_to_drivable_bound"), &msg);
  appendMarkerArray(createOverhangLaneletMarkerArray(objects, ns + "_overhang_lanelet"), &msg);

  return msg;
}

MarkerArray createTurnSignalMarkerArray(const TurnSignalInfo & turn_signal_info, std::string && ns)
{
  MarkerArray msg;

  if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::NO_COMMAND) {
    return msg;
  }

  const auto yaw_offset = [&turn_signal_info]() {
    if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
      return -1.0 * M_PI_2;
    }

    if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
      return M_PI_2;
    }

    return 0.0;
  }();

  size_t i = 0;
  {
    auto marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i++, Marker::ARROW,
      createMarkerScale(0.6, 0.3, 0.3), createMarkerColor(0.0, 0.0, 1.0, 0.999));
    marker.pose = turn_signal_info.desired_start_point;
    marker.pose = calcOffsetPose(marker.pose, 0.0, 0.0, 0.0, yaw_offset);

    msg.markers.push_back(marker);
  }
  {
    auto marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i++, Marker::ARROW,
      createMarkerScale(0.6, 0.3, 0.3), createMarkerColor(0.0, 0.0, 1.0, 0.999));
    marker.pose = turn_signal_info.desired_end_point;
    marker.pose = calcOffsetPose(marker.pose, 0.0, 0.0, 0.0, yaw_offset);

    msg.markers.push_back(marker);
  }
  {
    auto marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i++, Marker::ARROW,
      createMarkerScale(0.8, 0.4, 0.4), createMarkerColor(1.0, 0.0, 0.0, 0.999));
    marker.pose = turn_signal_info.required_start_point;
    marker.pose = calcOffsetPose(marker.pose, 0.0, 0.0, 0.0, yaw_offset);

    msg.markers.push_back(marker);
  }
  {
    auto marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i++, Marker::ARROW,
      createMarkerScale(0.8, 0.4, 0.4), createMarkerColor(1.0, 0.0, 0.0, 0.999));
    marker.pose = turn_signal_info.required_end_point;
    marker.pose = calcOffsetPose(marker.pose, 0.0, 0.0, 0.0, yaw_offset);

    msg.markers.push_back(marker);
  }

  return msg;
}

}  // namespace

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

MarkerArray createOtherObjectsMarkerArray(
  const ObjectDataArray & objects, const ObjectInfo & info, const bool verbose)
{
  const auto filtered_objects = [&objects, &info]() {
    ObjectDataArray ret{};
    for (const auto & o : objects) {
      if (o.info != info) {
        continue;
      }
      ret.push_back(o);
    }

    return ret;
  }();

  MarkerArray msg;
  msg.markers.reserve(filtered_objects.size() * 2);

  std::ostringstream string_stream;
  string_stream << magic_enum::enum_name(info);

  std::string ns = string_stream.str();
  transform(ns.begin(), ns.end(), ns.begin(), tolower);

  appendMarkerArray(
    createObjectsCubeMarkerArray(
      filtered_objects, "others_" + ns + "_cube", createMarkerScale(3.0, 1.5, 1.5),
      createMarkerColor(0.5, 0.5, 0.5, 0.8)),
    &msg);
  appendMarkerArray(
    createObjectInfoMarkerArray(filtered_objects, "others_" + ns + "_info", verbose), &msg);
  appendMarkerArray(
    createOverhangLaneletMarkerArray(filtered_objects, "others_" + ns + "_overhang_lanelet"), &msg);

  return msg;
}

MarkerArray createAmbiguousObjectsMarkerArray(
  const ObjectDataArray & objects, const Pose & ego_pose, const std::string & policy)
{
  MarkerArray msg;

  if (policy != "manual") {
    return msg;
  }

  for (const auto & object : objects) {
    if (!object.is_ambiguous || !object.is_avoidable) {
      continue;
    }

    {
      auto marker = createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "ambiguous_target", 0L, Marker::ARROW,
        createMarkerScale(0.5, 1.0, 1.0), createMarkerColor(1.0, 1.0, 0.0, 0.999));

      Point src, dst;
      src = object.getPosition();
      src.z += 4.0;
      dst = object.getPosition();
      dst.z += 2.0;

      marker.points.push_back(src);
      marker.points.push_back(dst);
      marker.id = uuidToInt32(object.object.object_id);

      msg.markers.push_back(marker);
    }

    {
      auto marker = createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "ambiguous_target_text", 0L,
        Marker::TEXT_VIEW_FACING, createMarkerScale(0.5, 0.5, 0.5),
        createMarkerColor(1.0, 1.0, 0.0, 1.0));

      marker.id = uuidToInt32(object.object.object_id);
      marker.pose = object.getPose();
      marker.pose.position.z += 4.5;
      std::ostringstream string_stream;
      string_stream << "SHOULD AVOID?";
      marker.text = string_stream.str();
      marker.color = createMarkerColor(1.0, 1.0, 0.0, 0.999);
      marker.scale = createMarkerScale(0.8, 0.8, 0.8);
      msg.markers.push_back(marker);
    }

    {
      auto marker = createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "request_text", 0L, Marker::TEXT_VIEW_FACING,
        createMarkerScale(0.5, 0.5, 0.5), createMarkerColor(1.0, 1.0, 0.0, 1.0));

      marker.id = uuidToInt32(object.object.object_id);
      marker.pose = ego_pose;
      marker.pose.position.z += 2.0;
      std::ostringstream string_stream;
      string_stream << "SYSTEM REQUESTS OPERATOR SUPPORT.";
      marker.text = string_stream.str();
      marker.color = createMarkerColor(1.0, 1.0, 0.0, 0.999);
      marker.scale = createMarkerScale(0.8, 0.8, 0.8);
      msg.markers.push_back(marker);
    }

    return msg;
  }

  return msg;
}

MarkerArray createStopTargetObjectMarkerArray(const AvoidancePlanningData & data)
{
  MarkerArray msg;

  if (!data.stop_target_object.has_value()) {
    return msg;
  }

  appendMarkerArray(
    createObjectsCubeMarkerArray(
      {data.stop_target_object.value()}, "stop_target", createMarkerScale(3.4, 1.9, 1.9),
      createMarkerColor(1.0, 0.0, 0.42, 0.5)),
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
      marker.points.push_back(p);
    }

    msg.markers.push_back(marker);
  }

  // left bound
  {
    auto marker = createDefaultMarker(
      "map", current_time, ns + "_left", 0L, Marker::LINE_STRIP, createMarkerScale(0.4, 0.0, 0.0),
      createMarkerColor(r, g, b, 0.999));

    for (const auto & p : data.left_bound) {
      marker.points.push_back(p);
    }

    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createDebugMarkerArray(
  const BehaviorModuleOutput & output, const AvoidancePlanningData & data,
  const PathShifter & shifter, const DebugData & debug,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  using autoware::behavior_path_planner::utils::transformToLanelets;
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
    add(createOtherObjectsMarkerArray(objects, ns, parameters->enable_other_objects_info));
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
  if (parameters->enable_other_objects_marker) {
    addObjects(data.other_objects, ObjectInfo::FURTHER_THAN_THRESHOLD);
    addObjects(data.other_objects, ObjectInfo::IS_NOT_TARGET_OBJECT);
    addObjects(data.other_objects, ObjectInfo::FURTHER_THAN_GOAL);
    addObjects(data.other_objects, ObjectInfo::TOO_NEAR_TO_CENTERLINE);
    addObjects(data.other_objects, ObjectInfo::IS_NOT_PARKING_OBJECT);
    addObjects(data.other_objects, ObjectInfo::MOVING_OBJECT);
    addObjects(data.other_objects, ObjectInfo::CROSSWALK_USER);
    addObjects(data.other_objects, ObjectInfo::OUT_OF_TARGET_AREA);
    addObjects(data.other_objects, ObjectInfo::ENOUGH_LATERAL_DISTANCE);
    addObjects(data.other_objects, ObjectInfo::LESS_THAN_EXECUTION_THRESHOLD);
    addObjects(data.other_objects, ObjectInfo::TOO_NEAR_TO_GOAL);
    addObjects(data.other_objects, ObjectInfo::PARALLEL_TO_EGO_LANE);
    addObjects(data.other_objects, ObjectInfo::MERGING_TO_EGO_LANE);
    addObjects(data.other_objects, ObjectInfo::DEVIATING_FROM_EGO_LANE);
    addObjects(data.other_objects, ObjectInfo::UNSTABLE_OBJECT);
    addObjects(data.other_objects, ObjectInfo::AMBIGUOUS_STOPPED_VEHICLE);
    addObjects(data.other_objects, ObjectInfo::INVALID_SHIFT_LINE);
  }

  if (parameters->enable_shift_line_marker) {
    // shift line pre-process
    addAvoidLine(debug.step1_registered_shift_line, "step1_registered_shift_line", 0.2, 0.2, 1.0);
    addAvoidLine(debug.step1_current_shift_line, "step1_current_shift_line", 0.2, 0.4, 0.8, 0.3);
    addAvoidLine(debug.step1_merged_shift_line, "step1_merged_shift_line", 0.2, 0.6, 0.6, 0.3);
    addAvoidLine(debug.step1_filled_shift_line, "step1_filled_shift_line", 0.2, 0.8, 0.4, 0.3);
    addAvoidLine(debug.step1_return_shift_line, "step1_return_shift_line", 0.2, 1.0, 0.2, 0.3);

    // merge process
    addAvoidLine(debug.step2_merged_shift_line, "step2_merged_shift_line", 0.2, 1.0, 0.0, 0.3);

    // trimming process
    addAvoidLine(debug.step3_grad_filtered_1st, "step3_grad_filtered_1st", 0.2, 0.8, 0.0, 0.3);
    addAvoidLine(debug.step3_grad_filtered_2nd, "step3_grad_filtered_2nd", 0.4, 0.6, 0.0, 0.3);
    addAvoidLine(debug.step3_grad_filtered_3rd, "step3_grad_filtered_3rd", 0.6, 0.4, 0.0, 0.3);

    // registering process
    addShiftLine(shifter.getShiftLines(), "step4_old_shift_line", 1.0, 1.0, 0.0, 0.3);
    addAvoidLine(data.new_shift_line, "step4_new_shift_line", 1.0, 0.0, 0.0, 0.3);

    // shift length
    addShiftLength(debug.pos_shift, "merged_length_pos", 0.0, 0.7, 0.5);
    addShiftLength(debug.neg_shift, "merged_length_neg", 0.0, 0.5, 0.7);
    addShiftLength(debug.total_shift, "merged_length_total", 0.99, 0.4, 0.2);

    // shift grad
    addShiftGrad(debug.pos_shift_grad, debug.pos_shift, "merged_grad_pos", 0.0, 0.7, 0.5);
    addShiftGrad(debug.neg_shift_grad, debug.neg_shift, "merged_grad_neg", 0.0, 0.5, 0.7);
    addShiftGrad(debug.total_forward_grad, debug.total_shift, "grad_forward", 0.99, 0.4, 0.2);
    addShiftGrad(debug.total_backward_grad, debug.total_shift, "grad_backward", 0.4, 0.2, 0.9);
  }

  // safety check
  if (parameters->enable_safety_check_marker) {
    add(showSafetyCheckInfo(debug.collision_check, "object_debug_info"));
    add(showPredictedPath(debug.collision_check, "ego_predicted_path"));
    add(showPolygon(debug.collision_check, "ego_and_target_polygon_relation"));
  }

  // detection area
  if (parameters->enable_detection_area_marker) {
    size_t i = 0;
    for (const auto & detection_area : debug.detection_areas) {
      add(createPolygonMarkerArray(detection_area, "detection_area", i++, 0.16, 1.0, 0.69, 0.1));
    }
  }

  // drivable bound
  if (parameters->enable_drivable_bound_marker) {
    add(createDrivableBounds(data, "drivable_bound", 1.0, 0.0, 0.42));
  }

  // lane
  if (parameters->enable_lane_marker) {
    add(laneletsAsTriangleMarkerArray(
      "drivable_lanes", transformToLanelets(data.drivable_lanes),
      createMarkerColor(0.16, 1.0, 0.69, 0.2)));
    add(laneletsAsTriangleMarkerArray(
      "current_lanes", data.current_lanelets, createMarkerColor(1.0, 1.0, 1.0, 0.2)));
    add(laneletsAsTriangleMarkerArray(
      "safety_check_lanes", debug.safety_check_lanes, createMarkerColor(1.0, 0.0, 0.42, 0.2)));
  }

  // misc
  if (parameters->enable_misc_marker) {
    add(createPathMarkerArray(path, "centerline_resampled", 0, 0.0, 0.9, 0.5));
    add(createTurnSignalMarkerArray(output.turn_signal_info, "turn_signal_info"));
  }

  return msg;
}
}  // namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance

std::string toStrInfo(const autoware::behavior_path_planner::ShiftLineArray & sl_arr)
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

std::string toStrInfo(const autoware::behavior_path_planner::ShiftLine & sl)
{
  const auto & ps = sl.start.position;
  const auto & pe = sl.end.position;
  std::stringstream ss;
  ss << "shift length: " << sl.end_shift_length << ", start_idx: " << sl.start_idx
     << ", end_idx: " << sl.end_idx << ", start: (" << ps.x << ", " << ps.y << "), end: (" << pe.x
     << ", " << pe.y << ")";
  return ss.str();
}

std::string toStrInfo(const autoware::behavior_path_planner::AvoidLineArray & ap_arr)
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
std::string toStrInfo(const autoware::behavior_path_planner::AvoidLine & ap)
{
  using autoware::universe_utils::toHexString;

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
