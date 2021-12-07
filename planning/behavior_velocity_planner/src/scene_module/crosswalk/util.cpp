// Copyright 2020 Tier IV, Inc.
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

#include <scene_module/crosswalk/util.hpp>
#include <utilization/util.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;

bool getBackwardPointFromBasePoint(
  const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
  const Eigen::Vector2d & base_point, const double backward_length, Eigen::Vector2d & output_point)
{
  Eigen::Vector2d line_vec = line_point2 - line_point1;
  Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
  output_point = base_point + backward_vec;
  return true;
}

bool insertTargetVelocityPoint(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const Polygon & polygon,
  const double & margin, const double & velocity, const PlannerData & planner_data,
  autoware_auto_planning_msgs::msg::PathWithLaneId & output, DebugData & debug_data,
  boost::optional<int> & first_stop_path_point_index)
{
  output = input;
  for (size_t i = 0; i < output.points.size() - 1; ++i) {
    const auto p0 = output.points.at(i).point.pose.position;
    const auto p1 = output.points.at(i + 1).point.pose.position;
    const Line line{{p0.x, p0.y}, {p1.x, p1.y}};
    std::vector<Point> collision_points;
    bg::intersection(polygon, line, collision_points);

    if (collision_points.empty()) {
      continue;
    }
    // -- debug code --
    for (const auto & cp : collision_points) {
      Eigen::Vector3d point3d(cp.x(), cp.y(), planner_data.current_pose.pose.position.z);
      debug_data.collision_points.push_back(point3d);
    }
    std::vector<Eigen::Vector3d> line3d;
    line3d.emplace_back(p0.x, p0.y, p0.z);
    line3d.emplace_back(p1.x, p1.y, p1.z);
    debug_data.collision_lines.push_back(line3d);
    // ----------------

    // check nearest collision point
    Point nearest_collision_point{};
    double min_dist = 0.0;
    for (size_t j = 0; j < collision_points.size(); ++j) {
      double dist = bg::distance(Point(p0.x, p0.y), collision_points.at(j));
      if (j == 0 || dist < min_dist) {
        min_dist = dist;
        nearest_collision_point = collision_points.at(j);
        debug_data.nearest_collision_point =
          planning_utils::toRosPoint(collision_points.at(j), p0.z);
      }
    }

    // search target point index
    size_t insert_target_point_idx = 0;
    const double base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;
    double length_sum = 0;

    const double target_length = margin + base_link2front;
    Eigen::Vector2d point1, point2;
    point1 << nearest_collision_point.x(), nearest_collision_point.y();
    point2 << p0.x, p0.y;
    length_sum += (point2 - point1).norm();
    for (size_t j = i; 0 < j; --j) {
      if (target_length < length_sum) {
        insert_target_point_idx = j + 1;
        break;
      }
      const auto pj1 = output.points.at(j).point.pose.position;
      const auto pj2 = output.points.at(j - 1).point.pose.position;
      point1 << pj1.x, pj1.y;
      point2 << pj2.x, pj2.y;
      length_sum += (point2 - point1).norm();
    }

    // create target point
    Eigen::Vector2d target_point;
    autoware_auto_planning_msgs::msg::PathPointWithLaneId target_point_with_lane_id;
    getBackwardPointFromBasePoint(point2, point1, point2, length_sum - target_length, target_point);
    const int target_velocity_point_idx =
      std::max(static_cast<int>(insert_target_point_idx) - 1, 0);
    target_point_with_lane_id = output.points.at(target_velocity_point_idx);
    target_point_with_lane_id.point.pose.position.x = target_point.x();
    target_point_with_lane_id.point.pose.position.y = target_point.y();
    if (insert_target_point_idx > 0) {
      // calculate z-position of the target point (Internal division point of point1/point2)
      // if insert_target_point_idx is zero, use z-position of target_velocity_point_idx
      const double internal_div_ratio =
        (point1 - target_point).norm() /
        ((point1 - target_point).norm() + (point2 - target_point).norm());
      target_point_with_lane_id.point.pose.position.z =
        output.points.at(insert_target_point_idx).point.pose.position.z * (1 - internal_div_ratio) +
        output.points.at(insert_target_point_idx - 1).point.pose.position.z * internal_div_ratio;
    }

    if ((point1 - point2).norm() > 1.0E-3) {
      const double yaw = std::atan2(point1.y() - point2.y(), point1.x() - point2.x());
      target_point_with_lane_id.point.pose.orientation = planning_utils::getQuaternionFromYaw(yaw);
    }
    target_point_with_lane_id.point.longitudinal_velocity_mps = velocity;
    if (velocity == 0.0 && target_velocity_point_idx < first_stop_path_point_index) {
      first_stop_path_point_index = target_velocity_point_idx;
      // -- debug code --
      debug_data.first_stop_pose = target_point_with_lane_id.point.pose;
      // ----------------
    }
    // -- debug code --
    if (velocity == 0.0) {
      debug_data.stop_poses.push_back(target_point_with_lane_id.point.pose);
    } else {
      debug_data.slow_poses.push_back(target_point_with_lane_id.point.pose);
    }
    // ----------------

    // insert target point
    output.points.insert(
      output.points.begin() + insert_target_point_idx, target_point_with_lane_id);

    // insert 0 velocity after target point
    for (size_t j = insert_target_point_idx; j < output.points.size(); ++j) {
      output.points.at(j).point.longitudinal_velocity_mps =
        std::min(static_cast<float>(velocity), output.points.at(j).point.longitudinal_velocity_mps);
    }
    return true;
  }
  return false;
}

lanelet::Optional<lanelet::ConstLineString3d> getStopLineFromMap(
  const int lane_id, const std::shared_ptr<const PlannerData> & planner_data,
  const std::string & attribute_name)
{
  lanelet::ConstLanelet lanelet = planner_data->lanelet_map->laneletLayer.get(lane_id);
  const auto road_markings = lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
  lanelet::ConstLineStrings3d stop_line;
  for (const auto & road_marking : road_markings) {
    // TODO(someone): Create regulatory element for crosswalk
    const std::string type =
      road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");
    const int target_id = road_marking->roadMarking().attributeOr(attribute_name, 0);
    if (type == lanelet::AttributeValueString::StopLine && target_id == lane_id) {
      stop_line.push_back(road_marking->roadMarking());
      break;  // only one stop_line exists.
    }
  }
  if (stop_line.empty()) {
    return {};
  }

  return stop_line.front();
}

bool insertTargetVelocityPoint(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input,
  const lanelet::ConstLineString3d & stop_line, const double & margin, const double & velocity,
  const PlannerData & planner_data, autoware_auto_planning_msgs::msg::PathWithLaneId & output,
  DebugData & debug_data, boost::optional<int> & first_stop_path_point_index)
{
  using lanelet::utils::to2D;
  using lanelet::utils::toHybrid;
  output = input;
  for (size_t i = 0; i < output.points.size() - 1; ++i) {
    const auto p0 = output.points.at(i).point.pose.position;
    const auto p1 = output.points.at(i + 1).point.pose.position;
    const Line line{{p0.x, p0.y}, {p1.x, p1.y}};
    std::vector<Point> collision_points;
    bg::intersection(toHybrid(to2D(stop_line)), line, collision_points);

    if (collision_points.empty()) {
      continue;
    }
    // -- debug code --
    for (const auto & cp : collision_points) {
      Eigen::Vector3d point3d(cp.x(), cp.y(), planner_data.current_pose.pose.position.z);
      debug_data.collision_points.push_back(point3d);
    }
    std::vector<Eigen::Vector3d> line3d;
    line3d.emplace_back(p0.x, p0.y, p0.z);
    line3d.emplace_back(p1.x, p1.y, p1.z);
    debug_data.collision_lines.push_back(line3d);
    // ----------------

    // check nearest collision point
    Point nearest_collision_point{};
    double min_dist = 0.0;
    for (size_t j = 0; j < collision_points.size(); ++j) {
      double dist = bg::distance(Point(p0.x, p0.y), collision_points.at(j));
      if (j == 0 || dist < min_dist) {
        min_dist = dist;
        nearest_collision_point = collision_points.at(j);
        debug_data.nearest_collision_point =
          planning_utils::toRosPoint(collision_points.at(j), p0.z);
      }
    }

    // search target point index
    size_t insert_target_point_idx = 0;
    const double base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;
    double length_sum = 0;

    const double target_length = margin + base_link2front;
    Eigen::Vector2d point1, point2;
    point1 << nearest_collision_point.x(), nearest_collision_point.y();
    point2 << p0.x, p0.y;
    length_sum += (point2 - point1).norm();
    for (size_t j = i; 0 < j; --j) {
      if (target_length < length_sum) {
        insert_target_point_idx = j + 1;
        break;
      }
      const auto pj1 = output.points.at(j).point.pose.position;
      const auto pj2 = output.points.at(j - 1).point.pose.position;
      point1 << pj1.x, pj1.y;
      point2 << pj2.x, pj2.y;
      length_sum += (point2 - point1).norm();
    }

    // create target point
    Eigen::Vector2d target_point;
    autoware_auto_planning_msgs::msg::PathPointWithLaneId target_point_with_lane_id;
    getBackwardPointFromBasePoint(point2, point1, point2, length_sum - target_length, target_point);
    const int target_velocity_point_idx =
      std::max(static_cast<int>(insert_target_point_idx) - 1, 0);
    target_point_with_lane_id = output.points.at(target_velocity_point_idx);
    target_point_with_lane_id.point.pose.position.x = target_point.x();
    target_point_with_lane_id.point.pose.position.y = target_point.y();
    if (insert_target_point_idx > 0) {
      // calculate z-position of the target point (Internal division point of point1/point2)
      // if insert_target_point_idx is zero, use z-position of target_velocity_point_idx
      const double internal_div_ratio =
        (point1 - target_point).norm() /
        ((point1 - target_point).norm() + (point2 - target_point).norm());
      target_point_with_lane_id.point.pose.position.z =
        output.points.at(insert_target_point_idx).point.pose.position.z * (1 - internal_div_ratio) +
        output.points.at(insert_target_point_idx - 1).point.pose.position.z * internal_div_ratio;
    }

    if ((point1 - point2).norm() > 1.0E-3) {
      const double yaw = std::atan2(point1.y() - point2.y(), point1.x() - point2.x());
      target_point_with_lane_id.point.pose.orientation = planning_utils::getQuaternionFromYaw(yaw);
    }
    target_point_with_lane_id.point.longitudinal_velocity_mps = velocity;
    if (velocity == 0.0 && target_velocity_point_idx < first_stop_path_point_index) {
      first_stop_path_point_index = target_velocity_point_idx;
      // -- debug code --
      debug_data.first_stop_pose = target_point_with_lane_id.point.pose;
      // ----------------
    }
    // -- debug code --
    if (velocity == 0.0) {
      debug_data.stop_poses.push_back(target_point_with_lane_id.point.pose);
    } else {
      debug_data.slow_poses.push_back(target_point_with_lane_id.point.pose);
    }
    // ----------------

    // insert target point
    output.points.insert(
      output.points.begin() + insert_target_point_idx, target_point_with_lane_id);

    // insert 0 velocity after target point
    for (size_t j = insert_target_point_idx; j < output.points.size(); ++j) {
      output.points.at(j).point.longitudinal_velocity_mps =
        std::min(static_cast<float>(velocity), output.points.at(j).point.longitudinal_velocity_mps);
    }
    return true;
  }
  return false;
}

bool isClockWise(const Polygon & polygon)
{
  const auto n = polygon.outer().size();

  const double x_offset = polygon.outer().at(0).x();
  const double y_offset = polygon.outer().at(0).y();
  double sum = 0.0;
  for (std::size_t i = 0; i < polygon.outer().size(); ++i) {
    sum +=
      (polygon.outer().at(i).x() - x_offset) * (polygon.outer().at((i + 1) % n).y() - y_offset) -
      (polygon.outer().at(i).y() - y_offset) * (polygon.outer().at((i + 1) % n).x() - x_offset);
  }
  return sum < 0.0;
}

Polygon inverseClockWise(const Polygon & polygon)
{
  const auto & poly = polygon.outer();
  Polygon inverted_polygon;
  inverted_polygon.outer().reserve(poly.size());
  std::reverse_copy(poly.begin(), poly.end(), std::back_inserter(inverted_polygon.outer()));
  return inverted_polygon;
}
}  // namespace behavior_velocity_planner
