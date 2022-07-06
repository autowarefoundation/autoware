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

#ifndef SCENE_MODULE__CROSSWALK__UTIL_HPP_
#define SCENE_MODULE__CROSSWALK__UTIL_HPP_

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <memory>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "behavior_velocity_planner/planner_data.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

namespace behavior_velocity_planner
{

using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;

enum class CollisionPointState { YIELD, EGO_PASS_FIRST, EGO_PASS_LATER, IGNORE };

struct CollisionPoint
{
  geometry_msgs::msg::Point collision_point{};
  double time_to_collision;
  double time_to_vehicle;
  CollisionPointState state{CollisionPointState::EGO_PASS_FIRST};
};

struct DebugData
{
  bool ignore_crosswalk{false};
  double base_link2front;
  double stop_judge_range;

  geometry_msgs::msg::Pose first_stop_pose;
  geometry_msgs::msg::Point nearest_collision_point;

  boost::optional<geometry_msgs::msg::Point> range_near_point{boost::none};
  boost::optional<geometry_msgs::msg::Point> range_far_point{boost::none};

  std::vector<CollisionPoint> collision_points;

  std::vector<geometry_msgs::msg::Pose> stop_poses;
  std::vector<geometry_msgs::msg::Pose> slow_poses;
  std::vector<geometry_msgs::msg::Point> stop_factor_points;
  std::vector<geometry_msgs::msg::Point> crosswalk_polygon;
  std::vector<geometry_msgs::msg::Polygon> ego_polygons;
  std::vector<geometry_msgs::msg::Polygon> obj_polygons;
};

bool insertTargetVelocityPoint(
  const PathWithLaneId & input,
  const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> & polygon,
  const double & margin, const double & velocity, const PlannerData & planner_data,
  PathWithLaneId & output, DebugData & debug_data,
  boost::optional<int> & first_stop_path_point_index);

lanelet::Optional<lanelet::ConstLineString3d> getStopLineFromMap(
  const int lane_id, const std::shared_ptr<const PlannerData> & planner_data,
  const std::string & attribute_name);

bool insertTargetVelocityPoint(
  const PathWithLaneId & input, const lanelet::ConstLineString3d & stop_line, const double & margin,
  const double & velocity, const PlannerData & planner_data, PathWithLaneId & output,
  DebugData & debug_data, boost::optional<int> & first_stop_path_point_index);

bool insertTargetVelocityPoint(
  const PathWithLaneId & input,
  const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &
    stop_line,
  const double & margin, const double & velocity, const PlannerData & planner_data,
  PathWithLaneId & output, DebugData & debug_data,
  boost::optional<int> & first_stop_path_point_index);

bool insertTargetVelocityPoint(
  const PathWithLaneId & input, const geometry_msgs::msg::Point & stop_point, const double & margin,
  const double & velocity, const PlannerData & planner_data, PathWithLaneId & output,
  DebugData & debug_data, boost::optional<int> & first_stop_path_point_index);

bool isClockWise(
  const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> & polygon);

boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> inverseClockWise(
  const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> & polygon);
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__CROSSWALK__UTIL_HPP_
