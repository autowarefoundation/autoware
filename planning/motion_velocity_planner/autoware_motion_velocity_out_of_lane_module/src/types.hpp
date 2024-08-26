// Copyright 2024 TIER IV, Inc.
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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::out_of_lane
{
using Polygons = boost::geometry::model::multi_polygon<lanelet::BasicPolygonWithHoles2d>;

/// @brief parameters for the out_of_lane module
struct PlannerParam
{
  std::string mode;                  // mode used to consider a conflict with an object
  bool skip_if_already_overlapping;  // if true, do not run the module when ego already overlaps
                                     // another lane
  double max_arc_length;  // [m] maximum arc length along the trajectory to check for collision
  bool ignore_lane_changeable_lanelets;  // if true, ignore overlaps on lane changeable lanelets

  double time_threshold;  // [s](mode="threshold") objects time threshold
  double ttc_threshold;  // [s](mode="ttc") threshold on time to collision between ego and an object

  bool objects_cut_predicted_paths_beyond_red_lights;  // whether to cut predicted paths beyond red
                                                       // lights' stop lines
  double objects_min_vel;          // [m/s] objects lower than this velocity will be ignored
  double objects_min_confidence;   // minimum confidence to consider a predicted path
  bool objects_ignore_behind_ego;  // if true, objects behind the ego vehicle are ignored

  // action to insert in the trajectory if an object causes a collision at an overlap
  double lon_dist_buffer;      // [m] safety distance buffer to keep in front of the ego vehicle
  double lat_dist_buffer;      // [m] safety distance buffer to keep on the side of the ego vehicle
  double slow_velocity;        // [m/s] slowdown velocity
  double stop_dist_threshold;  // [m] if a collision is detected bellow this distance ahead of ego,
                               // try to insert a stop point
  double precision;            // [m] precision when inserting a stop pose in the trajectory
  double
    min_decision_duration;  // [s] duration needed before a stop or slowdown point can be removed

  // ego dimensions used to create its polygon footprint
  double front_offset;        // [m]  front offset (from vehicle info)
  double rear_offset;         // [m]  rear offset (from vehicle info)
  double right_offset;        // [m]  right offset (from vehicle info)
  double left_offset;         // [m]  left offset (from vehicle info)
  double extra_front_offset;  // [m] extra front distance
  double extra_rear_offset;   // [m] extra rear distance
  double extra_right_offset;  // [m] extra right distance
  double extra_left_offset;   // [m] extra left distance
};

namespace bgi = boost::geometry::index;
struct StopLine
{
  universe_utils::LineString2d stop_line;
  lanelet::ConstLanelets lanelets;
};
using StopLineNode = std::pair<universe_utils::Box2d, StopLine>;
using StopLinesRtree = bgi::rtree<StopLineNode, bgi::rstar<16>>;
using OutAreaNode = std::pair<universe_utils::Box2d, size_t>;
using OutAreaRtree = bgi::rtree<OutAreaNode, bgi::rstar<16>>;

/// @brief data related to the ego vehicle
struct EgoData
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory_points;
  geometry_msgs::msg::Pose pose;
  size_t first_trajectory_idx{};
  double longitudinal_offset_to_first_trajectory_index{};
  double min_stop_distance{};
  double min_slowdown_distance{};
  double min_stop_arc_length{};

  Polygons drivable_lane_polygons;

  lanelet::BasicPolygon2d current_footprint;
  std::vector<lanelet::BasicPolygon2d> trajectory_footprints;

  StopLinesRtree stop_lines_rtree;
};

struct OutOfLanePoint
{
  size_t trajectory_index;
  lanelet::BasicPolygon2d outside_ring;
  std::set<double> collision_times;
  std::optional<double> min_object_arrival_time;
  std::optional<double> max_object_arrival_time;
  std::optional<double> ttc;
  lanelet::ConstLanelets overlapped_lanelets;
  bool to_avoid = false;
};
struct OutOfLaneData
{
  std::vector<OutOfLanePoint> outside_points;
  OutAreaRtree outside_areas_rtree;
};

/// @brief debug data
struct DebugData
{
  size_t prev_out_of_lane_areas = 0;
  size_t prev_ttcs = 0;
  size_t prev_objects = 0;
  size_t prev_stop_line = 0;
};

}  // namespace autoware::motion_velocity_planner::out_of_lane

#endif  // TYPES_HPP_
