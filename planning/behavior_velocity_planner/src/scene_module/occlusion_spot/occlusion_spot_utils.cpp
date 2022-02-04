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

#include <deque>
#include <functional>
#include <limits>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace occlusion_spot_utils
{
lanelet::ConstLanelet toPathLanelet(const PathWithLaneId & path)
{
  lanelet::Points3d path_points;
  path_points.reserve(path.points.size());
  for (const auto & path_point : path.points) {
    const auto & p = path_point.point.pose.position;
    path_points.emplace_back(lanelet::InvalId, p.x, p.y, p.z);
  }
  lanelet::LineString3d centerline(lanelet::InvalId, path_points);
  lanelet::Lanelet path_lanelet(lanelet::InvalId);
  path_lanelet.setCenterline(centerline);
  return lanelet::ConstLanelet(path_lanelet);
}

bool splineInterpolate(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const double interval,
  autoware_auto_planning_msgs::msg::PathWithLaneId * output, const rclcpp::Logger logger)
{
  *output = input;

  if (input.points.size() <= 1) {
    RCLCPP_DEBUG(logger, "Do not interpolate because path size is 1.");
    return false;
  }

  static constexpr double ep = 1.0e-8;

  // calc arclength for path
  std::vector<double> base_x;
  std::vector<double> base_y;
  std::vector<double> base_z;
  std::vector<double> base_v;
  for (const auto & p : input.points) {
    base_x.push_back(p.point.pose.position.x);
    base_y.push_back(p.point.pose.position.y);
    base_z.push_back(p.point.pose.position.z);
    base_v.push_back(p.point.longitudinal_velocity_mps);
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
        base_v.erase(base_v.begin() + i);
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
  const std::vector<double> resampled_v = ::interpolation::slerp(base_s, base_v, resampled_s);

  // set xy
  output->points.clear();
  for (size_t i = 0; i < resampled_s.size(); i++) {
    PathPointWithLaneId p;
    p.point.pose.position.x = resampled_x.at(i);
    p.point.pose.position.y = resampled_y.at(i);
    p.point.pose.position.z = resampled_z.at(i);
    p.point.longitudinal_velocity_mps = resampled_v.at(i);
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

void calcSlowDownPointsForPossibleCollision(
  const int closest_idx, const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const double offset, std::vector<PossibleCollisionInfo> & possible_collisions)
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
  double dist_along_path_point = offset;
  double dist_along_next_path_point = dist_along_path_point;
  for (size_t idx = closest_idx; idx < path.points.size() - 1; idx++) {
    auto p_prev = path.points.at(idx).point;
    auto p_next = path.points.at(idx + 1).point;
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
        const double v0 = p_prev.longitudinal_velocity_mps;
        const double v1 = p_next.longitudinal_velocity_mps;
        const double v = getInterpolatedValue(d0, v0, dist_to_col, d1, v1);
        const double z = getInterpolatedValue(d0, p0.z, dist_to_col, d1, p1.z);
        // height is used to visualize marker correctly
        auto & col = possible_collisions.at(collision_index);
        col.collision_path_point.longitudinal_velocity_mps = v;
        col.collision_path_point.pose.position.z = z;
        col.intersection_pose.position.z = z;
        col.obstacle_info.position.z = z;
        const double current_dist2col = col.arc_lane_dist_at_collision.length + offset;
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

std::vector<PredictedObject> getParkedVehicles(
  const PredictedObjects & dyn_objects, const PlannerParam & param,
  std::vector<Point> & debug_point)
{
  std::vector<PredictedObject> parked_vehicles;
  std::vector<Point> points;
  for (const auto & obj : dyn_objects.objects) {
    bool is_parked_vehicle = true;
    if (!occlusion_spot_utils::isStuckVehicle(obj, param.stuck_vehicle_vel)) {
      continue;
    }
    const geometry_msgs::msg::Point & p = obj.kinematics.initial_pose_with_covariance.pose.position;
    BasicPoint2d obj_point(p.x, p.y);
    if (is_parked_vehicle) {
      parked_vehicles.emplace_back(obj);
      points.emplace_back(p);
    }
  }
  debug_point = points;
  return parked_vehicles;
}

ArcCoordinates getOcclusionPoint(const PredictedObject & obj, const ConstLineString2d & ll_string)
{
  Polygon2d poly = planning_utils::toFootprintPolygon(obj);
  std::deque<lanelet::ArcCoordinates> arcs;
  for (const auto & p : poly.outer()) {
    lanelet::BasicPoint2d obj_p = {p.x(), p.y()};
    arcs.emplace_back(lanelet::geometry::toArcCoordinates(ll_string, obj_p));
  }
  /** remove
   *  x--------*
   *  |        |
   *  x--------* <- return
   * Ego===============> path
   **/
  // sort by arc length
  std::sort(arcs.begin(), arcs.end(), [](ArcCoordinates arc1, ArcCoordinates arc2) {
    return arc1.length < arc2.length;
  });
  // remove closest 2 polygon point
  arcs.pop_front();
  arcs.pop_front();
  // sort by arc distance
  std::sort(arcs.begin(), arcs.end(), [](ArcCoordinates arc1, ArcCoordinates arc2) {
    return std::abs(arc1.distance) < std::abs(arc2.distance);
  });
  // return closest to path point which is choosen by farthest 2 points.
  return arcs.at(0);
}

// calculate value removing offset distance or 0
double calcSignedLateralDistanceWithOffset(const double lateral, const double offset)
{
  // if distance is lower than offset return 0;
  if (std::abs(lateral) < offset) {
    return 0;
  } else if (lateral < 0) {
    return lateral + offset;
  } else {
    return lateral - offset;
  }
  // error case
  return -1.0;
}

PossibleCollisionInfo calculateCollisionPathPointFromOcclusionSpot(
  const ArcCoordinates & arc_coord_occlusion,
  const ArcCoordinates & arc_coord_occlusion_with_offset,
  const lanelet::ConstLanelet & path_lanelet, const PlannerParam & param)
{
  auto setPose = [](const lanelet::ConstLanelet & pl, const ArcCoordinates & arc) {
    const auto & ll = pl.centerline2d();
    BasicPoint2d bp = fromArcCoordinates(ll, arc);
    Pose pose;
    pose.position.x = bp[0];
    pose.position.y = bp[1];
    pose.position.z = 0.0;
    pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(
      lanelet::utils::getLaneletAngle(pl, pose.position));
    return pose;
  };
  /**
   * @brief calculate obstacle collision intersection from arc coordinate info.
   *                                      ^
   *                                      |
   * Ego ---------collision----------intersection-------> path
   *                                      |
   *             ------------------       |
   *            |     Vehicle      |   obstacle
   *             ------------------
   */
  PossibleCollisionInfo pc;
  pc.arc_lane_dist_at_collision = arc_coord_occlusion_with_offset;
  pc.obstacle_info.position = setPose(path_lanelet, arc_coord_occlusion).position;
  pc.obstacle_info.max_velocity = param.pedestrian_vel;
  pc.collision_path_point.pose =
    setPose(path_lanelet, {arc_coord_occlusion_with_offset.length, 0.0});
  pc.intersection_pose = setPose(path_lanelet, {arc_coord_occlusion.length, 0.0});
  return pc;
}

std::vector<PossibleCollisionInfo> generatePossibleCollisionBehindParkedVehicle(
  const PathWithLaneId & path, const PlannerParam & param, const double offset_from_start_to_ego,
  const std::vector<PredictedObject> & dyn_objects)
{
  lanelet::ConstLanelet path_lanelet = toPathLanelet(path);
  std::vector<PossibleCollisionInfo> possible_collisions;
  const double half_vehicle_width = 0.5 * param.vehicle_info.vehicle_width;
  const double baselink_to_front = param.vehicle_info.baselink_to_front;
  auto ll = path_lanelet.centerline2d();
  for (const auto & dyn : dyn_objects) {
    ArcCoordinates arc_coord_occlusion = getOcclusionPoint(dyn, ll);
    ArcCoordinates arc_coord_occlusion_with_offset = {
      arc_coord_occlusion.length - baselink_to_front,
      calcSignedLateralDistanceWithOffset(arc_coord_occlusion.distance, half_vehicle_width)};
    // ignore if collision is not avoidable by velocity control.
    if (
      arc_coord_occlusion_with_offset.length < offset_from_start_to_ego ||
      arc_coord_occlusion_with_offset.length > param.detection_area_length ||
      arc_coord_occlusion_with_offset.length > lanelet::geometry::length2d(path_lanelet) ||
      std::abs(arc_coord_occlusion_with_offset.distance) <= 1e-3 ||
      std::abs(arc_coord_occlusion_with_offset.distance) > param.lateral_distance_thr) {
      continue;
    }
    PossibleCollisionInfo pc = calculateCollisionPathPointFromOcclusionSpot(
      arc_coord_occlusion, arc_coord_occlusion_with_offset, path_lanelet, param);
    possible_collisions.emplace_back(pc);
  }
  // sort by arc length
  std::sort(
    possible_collisions.begin(), possible_collisions.end(),
    [](PossibleCollisionInfo pc1, PossibleCollisionInfo pc2) {
      return pc1.arc_lane_dist_at_collision.length < pc2.arc_lane_dist_at_collision.length;
    });
  return possible_collisions;
}

DetectionAreaIdx extractTargetRoadArcLength(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const double max_range, const PathWithLaneId & path,
  const ROAD_TYPE & target_road_type)
{
  bool found_target = false;
  double start_dist = 0.0;
  double dist_sum = 0.0;
  // search lanelet that includes target_road_type only
  for (size_t i = 0; i < path.points.size() - 1; i++) {
    ROAD_TYPE search_road_type = occlusion_spot_utils::getCurrentRoadType(
      lanelet_map_ptr->laneletLayer.get(path.points[i].lane_ids[0]), lanelet_map_ptr);
    if (found_target && search_road_type != target_road_type) {
      break;
    }
    // ignore path farther than max range
    if (dist_sum > max_range) {
      break;
    }
    if (!found_target && search_road_type == target_road_type) {
      start_dist = dist_sum;
      found_target = true;
    }
    const auto & curr_p = path.points[i].point.pose.position;
    const auto & next_p = path.points[i + 1].point.pose.position;
    dist_sum += tier4_autoware_utils::calcDistance2d(curr_p, next_p);
  }
  if (!found_target) return {};
  return DetectionAreaIdx(std::make_pair(start_dist, dist_sum));
}

void filterCollisionByRoadType(
  std::vector<PossibleCollisionInfo> & possible_collisions, const DetectionAreaIdx area)
{
  std::pair<int, int> focus_length = area.get();
  for (auto it = possible_collisions.begin(); it != possible_collisions.end();) {
    const auto & pc_len = it->arc_lane_dist_at_collision.length;
    if (focus_length.first < pc_len && pc_len < focus_length.second) {
      it++;
    } else {
      // -----erase-----|start------target-------end|----erase---
      it = possible_collisions.erase(it);
    }
  }
}

void generateSidewalkPossibleCollisions(
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const PathWithLaneId & path, const double offset_from_start_to_ego, const PlannerParam & param,
  std::vector<lanelet::BasicPolygon2d> & debug)
{
  lanelet::ConstLanelet path_lanelet = toPathLanelet(path);
  if (path_lanelet.centerline2d().empty()) {
    return;
  }
  std::vector<geometry::Slice> sidewalk_slices;
  geometry::buildSidewalkSlices(
    sidewalk_slices, path_lanelet, 0.0, param.vehicle_info.vehicle_width * 0.5,
    param.sidewalk.slice_size, param.sidewalk.focus_range);
  double length_lower_bound = std::numeric_limits<double>::max();
  double distance_lower_bound = std::numeric_limits<double>::max();
  // sort distance closest first to skip inferior collision
  std::sort(
    sidewalk_slices.begin(), sidewalk_slices.end(),
    [](const geometry::Slice & s1, const geometry::Slice & s2) {
      return std::abs(s1.range.min_distance) < std::abs(s2.range.min_distance);
    });

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
        possible_collisions, grid, occlusion_spot_positions, offset_from_start_to_ego, path_lanelet,
        param);
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
  const double offset_from_start_to_ego, const lanelet::ConstLanelet & path_lanelet,
  const PlannerParam & param)
{
  const double baselink_to_front = param.vehicle_info.baselink_to_front;
  const double half_vehicle_width = param.vehicle_info.vehicle_width * 0.5;
  double distance_lower_bound = std::numeric_limits<double>::max();
  PossibleCollisionInfo candidate;
  bool has_collision = false;
  for (grid_map::Position occlusion_spot_position : occlusion_spot_positions) {
    // arc intersection
    lanelet::BasicPoint2d obstacle_point = {occlusion_spot_position[0], occlusion_spot_position[1]};
    lanelet::ArcCoordinates arc_coord_occlusion_point =
      lanelet::geometry::toArcCoordinates(path_lanelet.centerline2d(), obstacle_point);
    const double length_to_col = arc_coord_occlusion_point.length - baselink_to_front;
    ArcCoordinates arc_coord_collision_point = {
      length_to_col,
      calcSignedLateralDistanceWithOffset(arc_coord_occlusion_point.distance, half_vehicle_width)};
    if (length_to_col < offset_from_start_to_ego) {
      continue;
    }
    PossibleCollisionInfo pc = calculateCollisionPathPointFromOcclusionSpot(
      arc_coord_occlusion_point, arc_coord_collision_point, path_lanelet, param);
    const auto & ip = pc.intersection_pose.position;
    bool collision_free_at_intersection =
      grid_utils::isCollisionFree(grid, occlusion_spot_position, grid_map::Position(ip.x, ip.y));
    // this is going to extract collision that is nearest to path
    if (
      collision_free_at_intersection && distance_lower_bound > arc_coord_collision_point.distance) {
      distance_lower_bound = arc_coord_collision_point.distance;
      candidate = {pc};
      has_collision = true;
    }
  }
  if (has_collision) possible_collisions.emplace_back(candidate);
}

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner
