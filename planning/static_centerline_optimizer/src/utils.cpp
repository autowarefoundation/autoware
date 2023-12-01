// Copyright 2022 Tier IV, Inc.
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

#include "static_centerline_optimizer/utils.hpp"

#include "behavior_path_planner_common/data_manager.hpp"
#include "behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/ros/marker_helper.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
namespace static_centerline_optimizer
{
namespace
{
nav_msgs::msg::Odometry::ConstSharedPtr convert_to_odometry(const geometry_msgs::msg::Pose & pose)
{
  auto odometry_ptr = std::make_shared<nav_msgs::msg::Odometry>();
  odometry_ptr->pose.pose = pose;
  return odometry_ptr;
}

lanelet::Point3d createPoint3d(const double x, const double y, const double z = 19.0)
{
  lanelet::Point3d point(lanelet::utils::getId());
  point.setAttribute("local_x", x);
  point.setAttribute("local_y", y);
  point.setAttribute("ele", z);

  return point;
}
}  // namespace

namespace utils
{
geometry_msgs::msg::Pose get_center_pose(
  const RouteHandler & route_handler, const size_t lanelet_id)
{
  // get middle idx of the lanelet
  const auto lanelet = route_handler.getLaneletsFromId(lanelet_id);
  const auto center_line = lanelet.centerline();
  const size_t middle_point_idx = std::floor(center_line.size() / 2.0);

  // get middle position of the lanelet
  geometry_msgs::msg::Point middle_pos;
  middle_pos.x = center_line[middle_point_idx].x();
  middle_pos.y = center_line[middle_point_idx].y();

  // get next middle position of the lanelet
  geometry_msgs::msg::Point next_middle_pos;
  next_middle_pos.x = center_line[middle_point_idx + 1].x();
  next_middle_pos.y = center_line[middle_point_idx + 1].y();

  // calculate middle pose
  geometry_msgs::msg::Pose middle_pose;
  middle_pose.position = middle_pos;
  const double yaw = tier4_autoware_utils::calcAzimuthAngle(middle_pos, next_middle_pos);
  middle_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);

  return middle_pose;
}

PathWithLaneId get_path_with_lane_id(
  const RouteHandler & route_handler, const lanelet::ConstLanelets lanelets,
  const geometry_msgs::msg::Pose & start_pose, const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold)
{
  // get center line
  constexpr double s_start = 0.0;
  constexpr double s_end = std::numeric_limits<double>::max();
  auto path_with_lane_id = route_handler.getCenterLinePath(lanelets, s_start, s_end);
  path_with_lane_id.header.frame_id = "map";

  // create planner data
  auto planner_data = std::make_shared<behavior_path_planner::PlannerData>();
  planner_data->route_handler = std::make_shared<RouteHandler>(route_handler);
  planner_data->self_odometry = convert_to_odometry(start_pose);
  planner_data->parameters.ego_nearest_dist_threshold = ego_nearest_dist_threshold;
  planner_data->parameters.ego_nearest_yaw_threshold = ego_nearest_yaw_threshold;

  // generate drivable area and store it in path with lane id
  constexpr double vehicle_length = 0.0;
  const auto drivable_lanes = behavior_path_planner::utils::generateDrivableLanes(lanelets);
  behavior_path_planner::utils::generateDrivableArea(
    path_with_lane_id, drivable_lanes, false, false, vehicle_length, planner_data);

  return path_with_lane_id;
}

void update_centerline(
  RouteHandler & route_handler, const lanelet::ConstLanelets & lanelets,
  const std::vector<TrajectoryPoint> & new_centerline)
{
  // get lanelet as reference to update centerline
  lanelet::Lanelets lanelets_ref;
  for (const auto & lanelet : lanelets) {
    for (auto & lanelet_ref : route_handler.getLaneletMapPtr()->laneletLayer) {
      if (lanelet_ref.id() == lanelet.id()) {
        lanelets_ref.push_back(lanelet_ref);
      }
    }
  }

  // store new centerline in lanelets
  size_t lanelet_idx = 0;
  lanelet::LineString3d centerline(lanelet::utils::getId());
  for (size_t traj_idx = 0; traj_idx < new_centerline.size(); ++traj_idx) {
    const auto & traj_pos = new_centerline.at(traj_idx).pose.position;

    for (; lanelet_idx < lanelets_ref.size(); ++lanelet_idx) {
      auto & lanelet_ref = lanelets_ref.at(lanelet_idx);

      const lanelet::BasicPoint2d point(traj_pos.x, traj_pos.y);
      const bool is_inside = lanelet::geometry::inside(lanelet_ref, point);
      if (is_inside) {
        const auto center_point = createPoint3d(traj_pos.x, traj_pos.y, traj_pos.z);

        // set center point
        centerline.push_back(center_point);
        route_handler.getLaneletMapPtr()->add(center_point);
        break;
      }

      if (!centerline.empty()) {
        // set centerline
        route_handler.getLaneletMapPtr()->add(centerline);
        lanelet_ref.setCenterline(centerline);

        // prepare new centerline
        centerline = lanelet::LineString3d(lanelet::utils::getId());
      }
    }

    if (traj_idx == new_centerline.size() - 1 && !centerline.empty()) {
      auto & lanelet_ref = lanelets_ref.at(lanelet_idx);

      // set centerline
      route_handler.getLaneletMapPtr()->add(centerline);
      lanelet_ref.setCenterline(centerline);
    }
  }
}

MarkerArray create_footprint_marker(
  const LinearRing2d & footprint_poly, const std::array<double, 3> & marker_color,
  const rclcpp::Time & now, const size_t idx)
{
  const double r = marker_color.at(0);
  const double g = marker_color.at(1);
  const double b = marker_color.at(2);

  auto marker = tier4_autoware_utils::createDefaultMarker(
    "map", rclcpp::Clock().now(), "unsafe_footprints", idx,
    visualization_msgs::msg::Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(0.1, 0.0, 0.0),
    tier4_autoware_utils::createMarkerColor(r, g, b, 0.999));
  marker.header.stamp = now;
  marker.lifetime = rclcpp::Duration(0, 0);

  for (const auto & point : footprint_poly) {
    geometry_msgs::msg::Point geom_point;
    geom_point.x = point.x();
    geom_point.y = point.y();
    // geom_point.z = point.z();

    marker.points.push_back(geom_point);
  }
  marker.points.push_back(marker.points.front());

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(marker);

  return marker_array;
}

MarkerArray create_distance_text_marker(
  const geometry_msgs::msg::Pose & pose, const double dist,
  const std::array<double, 3> & marker_color, const rclcpp::Time & now, const size_t idx)
{
  const double r = marker_color.at(0);
  const double g = marker_color.at(1);
  const double b = marker_color.at(2);

  auto marker = tier4_autoware_utils::createDefaultMarker(
    "map", rclcpp::Clock().now(), "unsafe_footprints_distance", idx,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    tier4_autoware_utils::createMarkerScale(0.5, 0.5, 0.5),
    tier4_autoware_utils::createMarkerColor(r, g, b, 0.999));
  marker.pose = pose;
  marker.header.stamp = now;
  marker.lifetime = rclcpp::Duration(0, 0);

  std::stringstream ss;
  ss << std::setprecision(2) << dist;
  marker.text = ss.str();

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(marker);

  return marker_array;
}
}  // namespace utils
}  // namespace static_centerline_optimizer
