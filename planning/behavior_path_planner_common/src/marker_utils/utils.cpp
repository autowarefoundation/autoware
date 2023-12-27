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

#include "behavior_path_planner_common/marker_utils/utils.hpp"

#include "behavior_path_planner_common/marker_utils/colors.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "behavior_path_planner_common/utils/path_utils.hpp"

#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <cstdint>

namespace marker_utils
{
using behavior_path_planner::utils::calcPathArcLengthArray;
using std_msgs::msg::ColorRGBA;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerOrientation;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;
using visualization_msgs::msg::Marker;

void addFootprintMarker(
  visualization_msgs::msg::Marker & marker, const geometry_msgs::msg::Pose & pose,
  const vehicle_info_util::VehicleInfo & vehicle_info)
{
  const double half_width = vehicle_info.vehicle_width_m / 2.0;
  const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  marker.points.push_back(
    tier4_autoware_utils::calcOffsetPose(pose, base_to_front, -half_width, 0.0).position);
  marker.points.push_back(
    tier4_autoware_utils::calcOffsetPose(pose, base_to_front, half_width, 0.0).position);
  marker.points.push_back(
    tier4_autoware_utils::calcOffsetPose(pose, -base_to_rear, half_width, 0.0).position);
  marker.points.push_back(
    tier4_autoware_utils::calcOffsetPose(pose, -base_to_rear, -half_width, 0.0).position);
  marker.points.push_back(marker.points.front());
}

MarkerArray createFootprintMarkerArray(
  const Pose & base_link_pose, const vehicle_info_util::VehicleInfo vehicle_info,
  const std::string && ns, const int32_t & id, const float & r, const float & g, const float & b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker = createDefaultMarker(
    "map", current_time, ns, id, Marker::LINE_STRIP, createMarkerScale(0.2, 0.2, 0.2),
    createMarkerColor(r, g, b, 0.999));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  MarkerArray marker_array;

  addFootprintMarker(marker, base_link_pose, vehicle_info);

  msg.markers.push_back(marker);
  return msg;
}

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
    shift_lines_local.emplace_back();
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
      marker_s.pose = calcOffsetPose(marker_s.pose, 0.0, current_shift, 0.0);
      msg.markers.push_back(marker_s);

      // end point
      auto marker_e = basic_marker;
      marker_e.id = ++id;
      marker_e.pose = sp.end;
      marker_e.pose = calcOffsetPose(marker_e.pose, 0.0, sp.end_shift_length, 0.0);
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
  const std::vector<double> & shift_distance, const PathWithLaneId & reference, std::string && ns,
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
    const auto p =
      calcOffsetPose(reference.points.at(i).point.pose, 0.0, shift_distance.at(i), 0.0);
    marker.points.push_back(p.position);
  }

  msg.markers.push_back(marker);
  return msg;
}

MarkerArray createShiftGradMarkerArray(
  const std::vector<double> & grad, const std::vector<double> & shift_distance,
  const PathWithLaneId & reference, std::string && ns, const float & r, const float & g,
  const float & b)
{
  if (grad.size() != reference.points.size()) {
    return MarkerArray{};
  }

  if (shift_distance.size() != reference.points.size()) {
    return MarkerArray{};
  }

  MarkerArray msg;

  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.1, 0.1, 0.1), createMarkerColor(r, g, b, 0.9));

  for (size_t i = 0; i < grad.size(); ++i) {
    std::ostringstream string_stream;
    string_stream << "Grad:" << grad.at(i);
    marker.text = string_stream.str();
    marker.pose = calcOffsetPose(reference.points.at(i).point.pose, 0.0, shift_distance.at(i), 0.0);
    marker.id = i;

    msg.markers.push_back(marker);
  }

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

MarkerArray createPolygonMarkerArray(
  const Polygon & polygon, std::string && ns, const int64_t & lane_id, const float & r,
  const float & g, const float & b, const float & w)
{
  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, static_cast<int32_t>(lane_id), Marker::LINE_STRIP,
    createMarkerScale(w, 0.0, 0.0), createMarkerColor(r, g, b, 0.8));

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

MarkerArray createDrivableLanesMarkerArray(
  const std::vector<DrivableLanes> & drivable_lanes, std::string && ns)
{
  MarkerArray msg;

  int32_t i{0};

  const auto get_lanelet_marker =
    [&ns, &i](const auto & lanelet, const auto r, const auto g, const auto b) {
      auto marker = createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, bitShift(lanelet.id()) + i++,
        Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(r, g, b, 0.999));

      if (lanelet.polygon3d().empty()) {
        return marker;
      }

      marker.points.reserve(lanelet.polygon3d().size() + 1);

      for (const auto & p : lanelet.polygon3d()) {
        marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
      }

      marker.points.push_back(marker.points.front());

      return marker;
    };

  const auto get_drivable_lanes = [&msg, &get_lanelet_marker](const DrivableLanes & drivable_lane) {
    {
      msg.markers.push_back(get_lanelet_marker(drivable_lane.right_lane, 1.0, 0.0, 0.0));
    }

    {
      msg.markers.push_back(get_lanelet_marker(drivable_lane.left_lane, 0.0, 1.0, 0.0));
    }

    std::for_each(
      drivable_lane.middle_lanes.begin(), drivable_lane.middle_lanes.end(),
      [&](const auto & lane) { msg.markers.push_back(get_lanelet_marker(lane, 0.0, 0.0, 1.0)); });
  };

  std::for_each(drivable_lanes.begin(), drivable_lanes.end(), get_drivable_lanes);

  return msg;
}
MarkerArray createPredictedPathMarkerArray(
  const PredictedPath & predicted_path, const vehicle_info_util::VehicleInfo & vehicle_info,
  std::string && ns, const int32_t & id, const float & r, const float & g, const float & b)
{
  if (predicted_path.path.empty()) {
    return MarkerArray{};
  }

  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  const auto & path = predicted_path.path;

  Marker marker = createDefaultMarker(
    "map", current_time, ns, id, Marker::LINE_STRIP, createMarkerScale(0.1, 0.1, 0.1),
    createMarkerColor(r, g, b, 0.999));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  MarkerArray marker_array;
  for (size_t i = 0; i < path.size(); ++i) {
    marker.id = i + id;
    marker.points.clear();

    const auto & predicted_path_pose = path.at(i);
    addFootprintMarker(marker, predicted_path_pose, vehicle_info);

    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

MarkerArray showPolygon(const CollisionCheckDebugMap & obj_debug_vec, std::string && ns)
{
  if (obj_debug_vec.empty()) {
    return MarkerArray{};
  }

  int32_t id{0};
  const auto now = rclcpp::Clock{RCL_ROS_TIME}.now();

  constexpr float line_scale_val{0.2};
  const auto line_marker_scale = createMarkerScale(line_scale_val, line_scale_val, line_scale_val);

  auto default_line_marker = [&](const auto & color = colors::green()) {
    return createDefaultMarker("map", now, ns, ++id, Marker::LINE_STRIP, line_marker_scale, color);
  };

  constexpr float text_scale_val{1.5};
  const auto text_marker_scale = createMarkerScale(text_scale_val, text_scale_val, text_scale_val);

  auto default_text_marker = [&]() {
    return createDefaultMarker(
      "map", now, ns + "_text", ++id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      text_marker_scale, colors::white());
  };

  auto default_cube_marker =
    [&](const auto & width, const auto & depth, const auto & color = colors::green()) {
      return createDefaultMarker(
        "map", now, ns + "_cube", ++id, visualization_msgs::msg::Marker::CUBE,
        createMarkerScale(width, depth, 1.0), color);
    };

  MarkerArray marker_array;
  marker_array.markers.reserve(
    obj_debug_vec.size() * 5);  // poly ego, text ego, poly obj, text obj, cube obj

  int32_t idx = {0};
  for (const auto & [uuid, info] : obj_debug_vec) {
    const auto color = info.is_safe ? colors::green() : colors::red();
    const auto poly_z = info.current_obj_pose.position.z;  // temporally

    const auto insert_polygon_marker = [&](const auto & polygon) {
      marker_array.markers.emplace_back();
      auto & polygon_marker = marker_array.markers.back();
      polygon_marker = default_line_marker(color);
      polygon_marker.points.reserve(polygon.outer().size());
      for (const auto & p : polygon.outer()) {
        polygon_marker.points.push_back(createPoint(p.x(), p.y(), poly_z));
      }
    };

    insert_polygon_marker(info.extended_ego_polygon);
    insert_polygon_marker(info.extended_obj_polygon);

    const auto str_idx = std::to_string(++idx);
    const auto insert_text_marker = [&](const auto & pose) {
      marker_array.markers.emplace_back();
      auto & text_marker = marker_array.markers.back();
      text_marker = default_text_marker();
      text_marker.text = str_idx;
      text_marker.pose = pose;
    };

    insert_text_marker(info.expected_ego_pose);
    insert_text_marker(info.expected_obj_pose);

    const auto insert_cube_marker = [&](const auto & pose) {
      marker_array.markers.emplace_back();
      auto & cube_marker = marker_array.markers.back();
      cube_marker = default_cube_marker(1.0, 1.0, color);
      cube_marker.pose = pose;
    };
    insert_cube_marker(info.current_obj_pose);
  }
  return marker_array;
}

MarkerArray showPredictedPath(const CollisionCheckDebugMap & obj_debug_vec, std::string && ns)
{
  int32_t id{0};
  const auto current_time{rclcpp::Clock{RCL_ROS_TIME}.now()};
  const auto arrow_marker_scale = createMarkerScale(1.0, 0.3, 0.3);
  const auto default_arrow_marker = [&](const auto & color) {
    return createDefaultMarker(
      "map", current_time, ns, ++id, Marker::ARROW, arrow_marker_scale, color);
  };

  MarkerArray marker_array;
  marker_array.markers.reserve(std::accumulate(
    obj_debug_vec.cbegin(), obj_debug_vec.cend(), 0UL,
    [&](const auto current_sum, const auto & obj_debug) {
      const auto & [uuid, info] = obj_debug;
      return current_sum + info.ego_predicted_path.size() + info.obj_predicted_path.size() + 2;
    }));

  for (const auto & [uuid, info] : obj_debug_vec) {
    const auto insert_marker = [&](const auto & path, const auto & color) {
      for (const auto & pose : path) {
        marker_array.markers.emplace_back();
        auto & marker = marker_array.markers.back();
        marker = default_arrow_marker(color);
        marker.pose = pose.pose;
      }
    };

    insert_marker(info.ego_predicted_path, colors::aqua());
    insert_marker(info.obj_predicted_path, colors::yellow());
    const auto insert_expected_pose_marker = [&](const auto & pose, const auto & color) {
      // instead of checking for distance, inserting a new marker might be more efficient
      marker_array.markers.emplace_back();
      auto & marker = marker_array.markers.back();
      marker = default_arrow_marker(color);
      marker.pose = pose;
      marker.pose.position.z += 0.05;
    };

    insert_expected_pose_marker(info.expected_ego_pose, colors::red());
    insert_expected_pose_marker(info.expected_obj_pose, colors::red());
  }
  return marker_array;
}

MarkerArray showSafetyCheckInfo(const CollisionCheckDebugMap & obj_debug_vec, std::string && ns)
{
  int32_t id{0};
  auto default_text_marker = [&]() {
    return createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, ++id, Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.5, 0.5, 0.5), colors::aqua());
  };

  MarkerArray marker_array;

  marker_array.markers.reserve(obj_debug_vec.size());

  int idx{0};

  for (const auto & [uuid, info] : obj_debug_vec) {
    auto safety_check_info_text = default_text_marker();
    safety_check_info_text.pose = info.current_obj_pose;

    std::ostringstream ss;

    ss << "Idx: " << ++idx << "\nUnsafe reason: " << info.unsafe_reason
       << "\nRSS dist: " << std::setprecision(4) << info.rss_longitudinal
       << "\nEgo to obj: " << info.inter_vehicle_distance
       << "\nExtended polygon: " << (info.is_front ? "ego" : "object")
       << "\nExtended polygon lateral offset: " << info.lat_offset
       << "\nExtended polygon forward longitudinal offset: " << info.forward_lon_offset
       << "\nExtended polygon backward longitudinal offset: " << info.backward_lon_offset
       << "\nLast checked position: " << (info.is_front ? "obj in front ego" : "obj at back ego")
       << "\nSafe: " << (info.is_safe ? "Yes" : "No");

    safety_check_info_text.text = ss.str();
    marker_array.markers.push_back(safety_check_info_text);
  }
  return marker_array;
}

MarkerArray showFilteredObjects(
  const ExtendedPredictedObjects & predicted_objects, const std::string & ns,
  const ColorRGBA & color, int32_t id)
{
  using behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObject;
  if (predicted_objects.empty()) {
    return MarkerArray{};
  }

  const auto now = rclcpp::Clock{RCL_ROS_TIME}.now();

  auto default_cube_marker =
    [&](const auto & width, const auto & depth, const auto & color = colors::green()) {
      return createDefaultMarker(
        "map", now, ns + "_cube", ++id, visualization_msgs::msg::Marker::CUBE,
        createMarkerScale(width, depth, 1.0), color);
    };

  MarkerArray marker_array;
  marker_array.markers.reserve(
    predicted_objects.size());  // poly ego, text ego, poly obj, text obj, cube obj

  std::for_each(
    predicted_objects.begin(), predicted_objects.end(), [&](const ExtendedPredictedObject & obj) {
      const auto insert_cube_marker = [&](const auto & pose) {
        marker_array.markers.emplace_back();
        auto & cube_marker = marker_array.markers.back();
        cube_marker = default_cube_marker(1.0, 1.0, color);
        cube_marker.pose = pose;
      };
      insert_cube_marker(obj.initial_pose.pose);
    });

  return marker_array;
}

}  // namespace marker_utils
