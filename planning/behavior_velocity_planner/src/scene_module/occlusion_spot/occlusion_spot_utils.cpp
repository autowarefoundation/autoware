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
#include <scene_module/occlusion_spot/geometry.hpp>
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>
#include <scene_module/occlusion_spot/risk_predictive_braking.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
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
namespace bg = boost::geometry;
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
        col.collision_with_margin.longitudinal_velocity_mps = v;
        col.collision_with_margin.pose.position.z = z;
        col.collision_pose.position.z = z;
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

void handleCollisionOffset(std::vector<PossibleCollisionInfo> & possible_collisions, double offset)
{
  for (auto & pc : possible_collisions) {
    pc.arc_lane_dist_at_collision.length -= offset;
  }
}

void clipPathByLength(
  const PathWithLaneId & path, PathWithLaneId & clipped, const double max_length)
{
  double length_sum = 0;
  for (int i = 0; i < static_cast<int>(path.points.size()) - 1; i++) {
    length_sum += tier4_autoware_utils::calcDistance2d(path.points.at(i), path.points.at(i + 1));
    if (length_sum > max_length) return;
    clipped.points.emplace_back(path.points.at(i));
  }
}

bool isStuckVehicle(PredictedObject obj, const double min_vel)
{
  if (
    obj.classification.at(0).label == ObjectClassification::CAR ||
    obj.classification.at(0).label == ObjectClassification::TRUCK ||
    obj.classification.at(0).label == ObjectClassification::BUS) {
    if (std::abs(obj.kinematics.initial_twist_with_covariance.twist.linear.x) < min_vel) {
      return true;
    }
  }
  return false;
}

double offsetFromStartToEgo(
  const PathWithLaneId & path, const Pose & ego_pose, const int closest_idx)
{
  double offset_from_ego_to_closest = 0;
  for (int i = 0; i < closest_idx; i++) {
    const auto & curr_p = path.points.at(i).point.pose.position;
    const auto & next_p = path.points.at(i + 1).point.pose.position;
    offset_from_ego_to_closest += tier4_autoware_utils::calcDistance2d(curr_p, next_p);
  }
  const double offset_from_closest_to_target =
    -planning_utils::transformRelCoordinate2D(ego_pose, path.points[closest_idx].point.pose)
       .position.x;
  return offset_from_ego_to_closest + offset_from_closest_to_target;
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
  const double ttc = std::abs(arc_coord_occlusion_with_offset.distance / param.pedestrian_vel);
  SafeMotion sm = calculateSafeMotion(param.v, ttc);
  double distance_to_stop = arc_coord_occlusion_with_offset.length - sm.stop_dist;
  const double eps = 0.1;
  // avoid inserting path point behind original path
  if (distance_to_stop < eps) distance_to_stop = eps;
  pc.arc_lane_dist_at_collision = {distance_to_stop, arc_coord_occlusion_with_offset.distance};
  pc.obstacle_info.safe_motion = sm;
  pc.obstacle_info.ttc = ttc;
  pc.obstacle_info.position = setPose(path_lanelet, arc_coord_occlusion).position;
  pc.obstacle_info.max_velocity = param.pedestrian_vel;
  pc.collision_pose = setPose(path_lanelet, {arc_coord_occlusion_with_offset.length, 0.0});
  pc.collision_with_margin.pose = setPose(path_lanelet, {distance_to_stop, 0.0});
  pc.intersection_pose = setPose(path_lanelet, {arc_coord_occlusion.length, 0.0});
  return pc;
}

std::vector<PossibleCollisionInfo> generatePossibleCollisionBehindParkedVehicle(
  const PathWithLaneId & path, const PlannerParam & param, const double offset_from_start_to_ego,
  const std::vector<PredictedObject> & dyn_objects)
{
  lanelet::ConstLanelet path_lanelet = toPathLanelet(path);
  std::vector<PossibleCollisionInfo> possible_collisions;
  auto ll = path_lanelet.centerline2d();
  for (const auto & dyn : dyn_objects) {
    ArcCoordinates arc_coord_occlusion = getOcclusionPoint(dyn, ll);
    ArcCoordinates arc_coord_occlusion_with_offset = {
      arc_coord_occlusion.length - param.baselink_to_front,
      calcSignedLateralDistanceWithOffset(arc_coord_occlusion.distance, param.half_vehicle_width)};
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

std::vector<PredictedObject> filterDynamicObjectByDetectionArea(
  std::vector<PredictedObject> & objs, const std::vector<Slice> polys)
{
  std::vector<PredictedObject> filtered_obj;
  // stuck points by predicted objects
  for (const auto & object : objs) {
    // check if the footprint is in the stuck detect area
    const Polygon2d obj_footprint = planning_utils::toFootprintPolygon(object);
    for (const auto & p : polys) {
      if (!bg::disjoint(obj_footprint, p.polygon)) {
        filtered_obj.emplace_back(object);
      }
    }
  }
  return filtered_obj;
}

void createPossibleCollisionsInDetectionArea(
  const std::vector<Slice> & detection_area_polygons,
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const PathWithLaneId & path, const double offset_from_start_to_ego, const PlannerParam & param,
  std::vector<Point> & debug_points)
{
  lanelet::ConstLanelet path_lanelet = toPathLanelet(path);
  if (path_lanelet.centerline2d().empty()) {
    return;
  }
  double distance_lower_bound = std::numeric_limits<double>::max();
  for (const Slice & detection_area_slice : detection_area_polygons) {
    std::vector<grid_map::Position> occlusion_spot_positions;
    grid_utils::findOcclusionSpots(
      occlusion_spot_positions, grid, detection_area_slice.polygon,
      param.detection_area.min_occlusion_spot_size);
    Point p;
    for (const auto & op : occlusion_spot_positions) {
      p.x = op[0];
      p.y = op[1];
      debug_points.emplace_back(p);
    }
    if (occlusion_spot_positions.empty()) continue;
    // for each partition find nearest occlusion spot from polygon's origin
    BasicPoint2d base_point = detection_area_slice.polygon.at(0);
    const auto pc = generateOneNotableCollisionFromOcclusionSpot(
      grid, occlusion_spot_positions, offset_from_start_to_ego, base_point, path_lanelet, param);
    if (!pc) continue;
    const double lateral_distance = std::abs(pc.get().arc_lane_dist_at_collision.distance);
    if (lateral_distance > distance_lower_bound) continue;
    distance_lower_bound = lateral_distance;
    possible_collisions.emplace_back(pc.get());
  }
}

boost::optional<PossibleCollisionInfo> generateOneNotableCollisionFromOcclusionSpot(
  const grid_map::GridMap & grid, const std::vector<grid_map::Position> & occlusion_spot_positions,
  const double offset_from_start_to_ego, const BasicPoint2d base_point,
  const lanelet::ConstLanelet & path_lanelet, const PlannerParam & param)
{
  const double baselink_to_front = param.baselink_to_front;
  const double half_vehicle_width = param.half_vehicle_width;
  double distance_lower_bound = std::numeric_limits<double>::max();
  PossibleCollisionInfo candidate;
  bool has_collision = false;
  for (grid_map::Position occlusion_spot_position : occlusion_spot_positions) {
    // arc intersection
    lanelet::BasicPoint2d obstacle_point = {occlusion_spot_position[0], occlusion_spot_position[1]};
    lanelet::ArcCoordinates arc_coord_occlusion_point =
      lanelet::geometry::toArcCoordinates(path_lanelet.centerline2d(), obstacle_point);
    const double dist =
      std::hypot(base_point[0] - obstacle_point[0], base_point[1] - obstacle_point[1]);
    // skip if absolute distance is larger
    if (distance_lower_bound < dist) continue;
    const double length_to_col = arc_coord_occlusion_point.length - baselink_to_front;
    // skip if occlusion is behind ego bumper
    if (length_to_col < offset_from_start_to_ego) {
      continue;
    }
    ArcCoordinates arc_coord_collision_point = {
      length_to_col,
      calcSignedLateralDistanceWithOffset(arc_coord_occlusion_point.distance, half_vehicle_width)};
    PossibleCollisionInfo pc = calculateCollisionPathPointFromOcclusionSpot(
      arc_coord_occlusion_point, arc_coord_collision_point, path_lanelet, param);
    const auto & ip = pc.intersection_pose.position;
    bool collision_free_at_intersection =
      grid_utils::isCollisionFree(grid, occlusion_spot_position, grid_map::Position(ip.x, ip.y));
    if (collision_free_at_intersection) {
      distance_lower_bound = dist;
      candidate = pc;
      has_collision = true;
    }
  }
  if (has_collision) {
    return candidate;
  } else {
    return {};
  }
}

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner
