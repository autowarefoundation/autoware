// Copyright 2023 TIER IV, Inc.
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

#include "footprint.hpp"

#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/algorithms/envelope.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <tf2/utils.h>

#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
{
autoware::universe_utils::MultiPolygon2d make_forward_footprints(
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & obstacles,
  const PlannerParam & params, const double hysteresis)
{
  autoware::universe_utils::MultiPolygon2d forward_footprints;
  for (const auto & obstacle : obstacles)
    forward_footprints.push_back(project_to_pose(
      make_forward_footprint(obstacle, params, hysteresis),
      obstacle.kinematics.initial_pose_with_covariance.pose));
  return forward_footprints;
}

autoware::universe_utils::Polygon2d make_forward_footprint(
  const autoware_perception_msgs::msg::PredictedObject & obstacle, const PlannerParam & params,
  const double hysteresis)
{
  const auto & shape = obstacle.shape.dimensions;
  const auto longitudinal_offset =
    shape.x / 2.0 +
    obstacle.kinematics.initial_twist_with_covariance.twist.linear.x * params.time_horizon;
  const auto lateral_offset = (shape.y + params.extra_object_width) / 2.0 + hysteresis;
  return autoware::universe_utils::Polygon2d{
    {{longitudinal_offset, -lateral_offset},
     {longitudinal_offset, lateral_offset},
     {shape.x / 2.0, lateral_offset},
     {shape.x / 2.0, -lateral_offset},
     {longitudinal_offset, -lateral_offset}},
    {}};
}

autoware::universe_utils::Polygon2d project_to_pose(
  const autoware::universe_utils::Polygon2d & base_footprint, const geometry_msgs::msg::Pose & pose)
{
  const auto angle = tf2::getYaw(pose.orientation);
  const auto rotated_footprint = autoware::universe_utils::rotatePolygon(base_footprint, angle);
  autoware::universe_utils::Polygon2d footprint;
  for (const auto & p : rotated_footprint.outer())
    footprint.outer().emplace_back(p.x() + pose.position.x, p.y() + pose.position.y);
  return footprint;
}

void make_ego_footprint_rtree(EgoData & ego_data, const PlannerParam & params)
{
  for (const auto & p : ego_data.trajectory)
    ego_data.trajectory_footprints.push_back(autoware::universe_utils::toFootprint(
      p.pose, params.ego_longitudinal_offset, 0.0, params.ego_lateral_offset * 2.0));
  std::vector<BoxIndexPair> rtree_nodes;
  rtree_nodes.reserve(ego_data.trajectory_footprints.size());
  for (auto i = 0UL; i < ego_data.trajectory_footprints.size(); ++i) {
    const auto box = boost::geometry::return_envelope<autoware::universe_utils::Box2d>(
      ego_data.trajectory_footprints[i]);
    rtree_nodes.emplace_back(box, i);
  }
  ego_data.rtree = Rtree(rtree_nodes);
}

}  // namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
