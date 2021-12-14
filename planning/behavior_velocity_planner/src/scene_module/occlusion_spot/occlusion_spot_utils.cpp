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

#include <interpolation/spline_interpolation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <utilization/interpolate.hpp>
#include <utilization/path_utilization.hpp>
#include <utilization/util.hpp>

#include <functional>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace occlusion_spot_utils
{
bool splineInterpolate(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const double interval,
  autoware_auto_planning_msgs::msg::PathWithLaneId * output, const rclcpp::Logger logger)
{
  *output = input;

  if (input.points.size() <= 1) {
    RCLCPP_WARN(logger, "Do not interpolate because path size is 1.");
    return false;
  }

  static constexpr double ep = 1.0e-8;

  // calc arclength for path
  std::vector<double> base_x;
  std::vector<double> base_y;
  std::vector<double> base_z;
  for (const auto & p : input.points) {
    base_x.push_back(p.point.pose.position.x);
    base_y.push_back(p.point.pose.position.y);
    base_z.push_back(p.point.pose.position.z);
  }
  std::vector<double> base_s = interpolation::calcEuclidDist(base_x, base_y);

  // remove duplicating sample points
  {
    size_t Ns = base_s.size();
    size_t i = 1;
    while (i < Ns) {
      if (std::fabs(base_s[i - 1] - base_s[i]) < ep) {
        base_s.erase(base_s.begin() + i);
        base_x.erase(base_x.begin() + i);
        base_y.erase(base_y.begin() + i);
        base_z.erase(base_z.begin() + i);
        Ns -= 1;
        i -= 1;
      }
      ++i;
    }
  }

  std::vector<double> resampled_s;
  for (double d = 0.0; d < base_s.back() - ep; d += interval) {
    resampled_s.push_back(d);
  }

  // do spline for xy
  const std::vector<double> resampled_x = ::interpolation::slerp(base_s, base_x, resampled_s);
  const std::vector<double> resampled_y = ::interpolation::slerp(base_s, base_y, resampled_s);
  const std::vector<double> resampled_z = ::interpolation::slerp(base_s, base_z, resampled_s);

  // set xy
  output->points.clear();
  for (size_t i = 0; i < resampled_s.size(); i++) {
    autoware_auto_planning_msgs::msg::PathPointWithLaneId p;
    p.point.pose.position.x = resampled_x.at(i);
    p.point.pose.position.y = resampled_y.at(i);
    p.point.pose.position.z = resampled_z.at(i);
    output->points.push_back(p);
  }

  // set yaw
  for (int i = 1; i < static_cast<int>(resampled_s.size()) - 1; i++) {
    auto p = output->points.at(i - 1).point.pose.position;
    auto n = output->points.at(i + 1).point.pose.position;
    double yaw = std::atan2(n.y - p.y, n.x - p.x);
    output->points.at(i).point.pose.orientation = planning_utils::getQuaternionFromYaw(yaw);
  }
  if (output->points.size() > 1) {
    size_t l = resampled_s.size();
    output->points.front().point.pose.orientation = output->points.at(1).point.pose.orientation;
    output->points.back().point.pose.orientation = output->points.at(l - 1).point.pose.orientation;
  }
  return true;
}

ROAD_TYPE getCurrentRoadType(
  const lanelet::ConstLanelet & current_lanelet,
  [[maybe_unused]] const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  const auto logger{rclcpp::get_logger("behavior_velocity_planner").get_child("occlusion_spot")};
  rclcpp::Clock clock{RCL_ROS_TIME};
  occlusion_spot_utils::ROAD_TYPE road_type;
  std::string location;
  if (
    current_lanelet.hasAttribute(lanelet::AttributeNamesString::Subtype) &&
    current_lanelet.attribute(lanelet::AttributeNamesString::Subtype) ==
      lanelet::AttributeValueString::Highway) {
    location = "highway";
  } else {
    location = current_lanelet.attributeOr("location", "else");
  }
  RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "location: " << location);
  if (location == "urban" || location == "public") {
    road_type = occlusion_spot_utils::ROAD_TYPE::PUBLIC;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "public road: " << location);
  } else if (location == "private") {
    road_type = occlusion_spot_utils::ROAD_TYPE::PRIVATE;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "private road");
  } else if (location == "highway") {
    road_type = occlusion_spot_utils::ROAD_TYPE::HIGHWAY;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "highway road");
  } else {
    road_type = occlusion_spot_utils::ROAD_TYPE::UNKNOWN;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "unknown road");
  }
  return road_type;
}

geometry_msgs::msg::Point setPoint(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

void calcSlowDownPointsForPossibleCollision(
  const int closest_idx, const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const double offset_from_ego_to_target, std::vector<PossibleCollisionInfo> & possible_collisions)
{
  if (possible_collisions.empty()) {
    return;
  }
  std::sort(
    possible_collisions.begin(), possible_collisions.end(),
    [](PossibleCollisionInfo pc1, PossibleCollisionInfo pc2) {
      return pc1.arc_lane_dist_at_collision.length < pc2.arc_lane_dist_at_collision.length;
    });

  // get interpolated value between (s0,v0) - (s1,value) - (s2,v2)
  auto getInterpolatedValue = [](double s0, double v0, double s1, double s2, double v2) {
    if (s2 - s0 < std::numeric_limits<float>::min()) {
      return v0;
    }
    return v0 + (v2 - v0) * (s1 - s0) / (s2 - s0);
  };
  // insert path point orientation to possible collision
  size_t collision_index = 0;
  double dist_along_path_point = offset_from_ego_to_target;
  double dist_along_next_path_point = dist_along_path_point;
  for (size_t idx = closest_idx; idx < path.points.size() - 1; idx++) {
    auto p_prev = path.points[idx].point;
    auto p_next = path.points[idx + 1].point;
    const double dist_to_col =
      possible_collisions.at(collision_index).arc_lane_dist_at_collision.length;
    dist_along_next_path_point +=
      tier4_autoware_utils::calcDistance2d(p_prev.pose.position, p_next.pose.position);
    // process if nearest possible collision is between current and next path point
    if (dist_along_path_point < dist_to_col) {
      for (; collision_index < possible_collisions.size(); collision_index++) {
        const double d0 = dist_along_path_point;  // distance at arc coordinate
        const double d1 = dist_along_next_path_point;
        const auto p0 = p_prev.pose.position;
        const auto p1 = p_next.pose.position;
        auto & col = possible_collisions[collision_index];
        auto & v = col.collision_path_point.longitudinal_velocity_mps;
        const double current_dist2col = col.arc_lane_dist_at_collision.length;
        v = getInterpolatedValue(
          d0, p_prev.longitudinal_velocity_mps, dist_to_col, d1, p_next.longitudinal_velocity_mps);
        const double x = getInterpolatedValue(d0, p0.x, dist_to_col, d1, p1.x);
        const double y = getInterpolatedValue(d0, p0.y, dist_to_col, d1, p1.y);
        const double z = getInterpolatedValue(d0, p0.z, dist_to_col, d1, p1.z);
        // height is used to visualize marker correctly

        possible_collisions[collision_index].collision_path_point.pose.position = setPoint(x, y, z);
        possible_collisions[collision_index].intersection_pose.position.z = z;
        possible_collisions[collision_index].obstacle_info.position.z = z;
        // break searching if dist to collision is farther than next path point
        if (dist_along_next_path_point < current_dist2col) {
          break;
        }
      }
      if (collision_index == possible_collisions.size()) {
        break;
      }
    }
    dist_along_path_point = dist_along_next_path_point;
  }
}

autoware_auto_planning_msgs::msg::Path toPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path_with_id)
{
  autoware_auto_planning_msgs::msg::Path path;
  for (const auto & p : path_with_id.points) {
    path.points.push_back(p.point);
  }
  return path;
}

lanelet::ConstLanelet buildPathLanelet(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  autoware_auto_planning_msgs::msg::Path converted_path = filterLitterPathPoint(toPath(path));
  if (converted_path.points.empty()) {
    return lanelet::ConstLanelet();
  }
  // interpolation is done previously
  lanelet::BasicLineString2d path_line;
  path_line.reserve(converted_path.points.size());
  // skip last point that will creates extra ordinary path
  for (size_t i = 0; i < converted_path.points.size() - 1; ++i) {
    const auto & pose_i = converted_path.points[i].pose;
    path_line.emplace_back(pose_i.position.x, pose_i.position.y);
  }
  // set simplified line to lanelet
  lanelet::BasicLineString2d simplified_line;
  boost::geometry::simplify(path_line, simplified_line, 0.1);
  lanelet::Points3d path_points;
  path_points.reserve(simplified_line.size());
  for (const auto & p : simplified_line) {
    path_points.emplace_back(lanelet::InvalId, p.x(), p.y(), 0.0);
  }
  lanelet::LineString3d centerline(lanelet::InvalId, path_points);
  lanelet::Lanelet path_lanelet(lanelet::InvalId);
  path_lanelet.setCenterline(centerline);
  return lanelet::ConstLanelet(path_lanelet);
}

void calculateCollisionPathPointFromOcclusionSpot(
  PossibleCollisionInfo & pc, const lanelet::BasicPoint2d & obstacle_point,
  const double offset_from_ego_to_target, const lanelet::ConstLanelet & path_lanelet,
  const PlannerParam & param)
{
  auto calcSignedArcDistance = [](const double lateral_distance, const double offset) {
    if (lateral_distance < 0) {
      return lateral_distance + offset;
    }
    return lateral_distance - offset;
  };
  lanelet::ArcCoordinates arc_lane_occlusion_point =
    lanelet::geometry::toArcCoordinates(path_lanelet.centerline2d(), obstacle_point);
  lanelet::BasicPoint2d intersection_point = lanelet::geometry::fromArcCoordinates(
    path_lanelet.centerline2d(), {arc_lane_occlusion_point.length, 0.0});
  lanelet::BasicPoint2d col_point = lanelet::geometry::fromArcCoordinates(
    path_lanelet.centerline2d(),
    {arc_lane_occlusion_point.length - param.vehicle_info.baselink_to_front, 0.0});
  geometry_msgs::msg::Point search_point;
  search_point.x = col_point[0];
  search_point.y = col_point[1];
  search_point.z = 0;
  double path_angle = lanelet::utils::getLaneletAngle(path_lanelet, search_point);
  tf2::Quaternion quat;
  quat.setRPY(0, 0, path_angle);
  autoware_auto_planning_msgs::msg::PathPoint collision_path_point;
  pc.collision_path_point.pose.position.x = col_point[0];
  pc.collision_path_point.pose.position.y = col_point[1];
  pc.collision_path_point.pose.orientation = tf2::toMsg(quat);
  ObstacleInfo obstacle_info;
  pc.obstacle_info.position.x = obstacle_point[0];
  pc.obstacle_info.position.y = obstacle_point[1];
  pc.obstacle_info.max_velocity = param.pedestrian_vel;
  geometry_msgs::msg::Pose intersection_pose;
  pc.intersection_pose.position.x = intersection_point[0];
  pc.intersection_pose.position.y = intersection_point[1];
  pc.intersection_pose.orientation = tf2::toMsg(quat);
  double signed_lateral_distance = calcSignedArcDistance(
    arc_lane_occlusion_point.distance, param.vehicle_info.vehicle_width * 0.5);
  pc.arc_lane_dist_at_collision = {
    arc_lane_occlusion_point.length + offset_from_ego_to_target -
      param.vehicle_info.baselink_to_front,
    std::abs(signed_lateral_distance)};
}

void createPossibleCollisionBehindParkedVehicle(
  std::vector<PossibleCollisionInfo> & possible_collisions,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const PlannerParam & param,
  const double offset_from_ego_to_target,
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr & dyn_obj_arr)
{
  lanelet::ConstLanelet path_lanelet = buildPathLanelet(path);
  if (path_lanelet.centerline2d().empty()) {
    return;
  }
  std::vector<lanelet::ArcCoordinates> arcs;
  auto calcSignedArcDistance = [](const double lateral_distance, const double offset) {
    if (lateral_distance < 0) {
      return lateral_distance + offset;
    }
    return lateral_distance - offset;
  };
  const double half_vehicle_width = 0.5 * param.vehicle_info.vehicle_width;
  for (const auto & dyn : dyn_obj_arr->objects) {
    // consider if dynamic object is a car or bus or truck
    if (
      dyn.classification.at(0).label !=
        autoware_auto_perception_msgs::msg::ObjectClassification::CAR &&
      dyn.classification.at(0).label !=
        autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK &&
      dyn.classification.at(0).label !=
        autoware_auto_perception_msgs::msg::ObjectClassification::BUS) {
      continue;
    }
    const auto & state = dyn.kinematics;
    // ignore if vehicle is moving
    if (state.initial_twist_with_covariance.twist.linear.x > param.stuck_vehicle_vel) {
      continue;
    }
    const geometry_msgs::msg::Point & p = dyn.kinematics.initial_pose_with_covariance.pose.position;
    const geometry_msgs::msg::Quaternion & q =
      dyn.kinematics.initial_pose_with_covariance.pose.orientation;
    lanelet::BasicPoint2d obj_point;
    obj_point[0] = p.x;
    obj_point[1] = p.y;
    lanelet::ArcCoordinates arc_lane_point_at_occlusion =
      lanelet::geometry::toArcCoordinates(path_lanelet.centerline2d(), obj_point);
    // add a half size of car to arc length as behind car
    arc_lane_point_at_occlusion.length += dyn.shape.dimensions.x * 0.5;
    // signed lateral distance used to convert obstacle position
    double signed_lateral_distance =
      calcSignedArcDistance(arc_lane_point_at_occlusion.distance, half_vehicle_width);
    arc_lane_point_at_occlusion.distance =
      std::abs(arc_lane_point_at_occlusion.distance) - 0.5 * dyn.shape.dimensions.y;
    // ignore if obstacle is not within attention area
    if (
      offset_from_ego_to_target + arc_lane_point_at_occlusion.length <
        param.vehicle_info.baselink_to_front ||
      arc_lane_point_at_occlusion.distance < half_vehicle_width ||
      param.lateral_distance_thr + half_vehicle_width < arc_lane_point_at_occlusion.distance) {
      continue;
    }
    // subtract  baselink_to_front from longitudinal distance to occlusion point
    double longitudinal_dist_at_collision_point =
      arc_lane_point_at_occlusion.length - param.vehicle_info.baselink_to_front;
    // collision point at baselink
    lanelet::BasicPoint2d collision_point = lanelet::geometry::fromArcCoordinates(
      path_lanelet.centerline2d(), {longitudinal_dist_at_collision_point, 0.0});
    geometry_msgs::msg::Point search_point;
    search_point.x = collision_point[0];
    search_point.y = collision_point[1];
    search_point.z = 0;
    double path_angle = lanelet::utils::getLaneletAngle(path_lanelet, search_point);
    // ignore if angle is more different than 10[degree]
    double obj_angle = tf2::getYaw(q);
    const double diff_angle = tier4_autoware_utils::normalizeRadian(path_angle - obj_angle);
    if (std::abs(diff_angle) > param.angle_thr) {
      continue;
    }
    lanelet::BasicPoint2d obstacle_point = lanelet::geometry::fromArcCoordinates(
      path_lanelet.centerline2d(), {arc_lane_point_at_occlusion.length, signed_lateral_distance});
    PossibleCollisionInfo pc;
    calculateCollisionPathPointFromOcclusionSpot(
      pc, obstacle_point, offset_from_ego_to_target, path_lanelet, param);
    possible_collisions.emplace_back(pc);
  }
}

bool extractTargetRoad(
  const int closest_idx, const lanelet::LaneletMapPtr lanelet_map_ptr, const double max_range,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & src_path,
  double & offset_from_closest_to_target,
  autoware_auto_planning_msgs::msg::PathWithLaneId & tar_path, const ROAD_TYPE & target_road_type)
{
  bool found_target = false;
  // search lanelet that includes target_road_type only
  for (size_t i = closest_idx; i < src_path.points.size(); i++) {
    occlusion_spot_utils::ROAD_TYPE search_road_type = occlusion_spot_utils::getCurrentRoadType(
      lanelet_map_ptr->laneletLayer.get(src_path.points[i].lane_ids[0]), lanelet_map_ptr);
    if (found_target && search_road_type != target_road_type) {
      break;
    }
    // ignore path farther than max range
    if (offset_from_closest_to_target > max_range) {
      break;
    }
    if (search_road_type == target_road_type) {
      tar_path.points.emplace_back(src_path.points[i]);
      found_target = true;
    }
    if (!found_target && i < src_path.points.size() - 1) {
      const auto & curr_p = src_path.points[i].point.pose.position;
      const auto & next_p = src_path.points[i + 1].point.pose.position;
      offset_from_closest_to_target += tier4_autoware_utils::calcDistance2d(curr_p, next_p);
    }
  }
  return found_target;
}

void generatePossibleCollisions(
  std::vector<PossibleCollisionInfo> & possible_collisions,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const grid_map::GridMap & grid,
  const double offset_from_ego_to_closest, const double offset_from_closest_to_target,
  const PlannerParam & param, std::vector<lanelet::BasicPolygon2d> & debug)
{
  // NOTE : buildPathLanelet first index should always be zero because path is already limited
  lanelet::ConstLanelet path_lanelet = buildPathLanelet(path);
  if (path_lanelet.centerline2d().empty()) {
    return;
  }
  // generate sidewalk possible collision
  generateSidewalkPossibleCollisions(
    possible_collisions, grid, offset_from_ego_to_closest, offset_from_closest_to_target,
    path_lanelet, param, debug);
  possible_collisions.insert(
    possible_collisions.end(), possible_collisions.begin(), possible_collisions.end());
}

void generateSidewalkPossibleCollisions(
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const double offset_from_ego_to_closest, const double offset_from_closest_to_target,
  const lanelet::ConstLanelet & path_lanelet, const PlannerParam & param,
  std::vector<lanelet::BasicPolygon2d> & debug)
{
  const double offset_form_ego_to_target =
    offset_from_ego_to_closest + offset_from_closest_to_target;
  double min_range = 0;
  // if ego is inside the target path_lanelet, set ignore_length
  if (offset_form_ego_to_target > param.vehicle_info.baselink_to_front) {
    min_range = 0;
  } else {
    min_range = param.vehicle_info.baselink_to_front - offset_form_ego_to_target;
  }
  std::vector<geometry::Slice> sidewalk_slices = geometry::buildSidewalkSlices(
    path_lanelet, min_range, param.vehicle_info.vehicle_width * 0.5, param.sidewalk.slice_size,
    param.sidewalk.focus_range);
  double length_lower_bound = std::numeric_limits<double>::max();
  double distance_lower_bound = std::numeric_limits<double>::max();
  std::sort(
    sidewalk_slices.begin(), sidewalk_slices.end(),
    [](const geometry::Slice & s1, const geometry::Slice & s2) {
      return s1.range.min_length < s2.range.min_length;
    });

  for (const geometry::Slice & sidewalk_slice : sidewalk_slices) {
    debug.push_back(sidewalk_slice.polygon);
    if ((sidewalk_slice.range.min_length < length_lower_bound ||
         std::abs(sidewalk_slice.range.min_distance) < distance_lower_bound)) {
      std::vector<grid_map::Position> occlusion_spot_positions;
      grid_utils::findOcclusionSpots(
        occlusion_spot_positions, grid, sidewalk_slice.polygon,
        param.sidewalk.min_occlusion_spot_size);
      generateSidewalkPossibleCollisionFromOcclusionSpot(
        possible_collisions, grid, occlusion_spot_positions, offset_form_ego_to_target,
        path_lanelet, param);
      if (!possible_collisions.empty()) {
        length_lower_bound = sidewalk_slice.range.min_length;
        distance_lower_bound = std::abs(sidewalk_slice.range.min_distance);
        possible_collisions.insert(
          possible_collisions.end(), possible_collisions.begin(), possible_collisions.end());
        debug.push_back(sidewalk_slice.polygon);
      }
    }
  }
}

void generateSidewalkPossibleCollisionFromOcclusionSpot(
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const std::vector<grid_map::Position> & occlusion_spot_positions,
  const double offset_form_ego_to_target, const lanelet::ConstLanelet & path_lanelet,
  const PlannerParam & param)
{
  for (grid_map::Position occlusion_spot_position : occlusion_spot_positions) {
    // arc intersection
    lanelet::BasicPoint2d obstacle_point = {occlusion_spot_position[0], occlusion_spot_position[1]};
    lanelet::ArcCoordinates arc_lane_point_at_occlusion =
      lanelet::geometry::toArcCoordinates(path_lanelet.centerline2d(), obstacle_point);
    lanelet::BasicPoint2d intersection_point = lanelet::geometry::fromArcCoordinates(
      path_lanelet.centerline2d(), {arc_lane_point_at_occlusion.length, 0.0});
    bool collision_free =
      grid_utils::isCollisionFree(grid, occlusion_spot_position, intersection_point);
    if (collision_free) {
      PossibleCollisionInfo pc;
      calculateCollisionPathPointFromOcclusionSpot(
        pc, obstacle_point, offset_form_ego_to_target, path_lanelet, param);
      if (pc.arc_lane_dist_at_collision.length < param.vehicle_info.baselink_to_front) {
        continue;
      }
      possible_collisions.emplace_back(pc);
      break;
    }
  }
}

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner
