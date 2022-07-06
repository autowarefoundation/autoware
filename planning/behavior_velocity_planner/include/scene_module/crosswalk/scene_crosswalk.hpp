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

#ifndef SCENE_MODULE__CROSSWALK__SCENE_CROSSWALK_HPP_
#define SCENE_MODULE__CROSSWALK__SCENE_CROSSWALK_HPP_

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scene_module/crosswalk/util.hpp>
#include <scene_module/scene_module_interface.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::TrafficLight;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using tier4_api_msgs::msg::CrosswalkStatus;
using tier4_autoware_utils::StopWatch;
using tier4_planning_msgs::msg::StopFactor;
using tier4_planning_msgs::msg::StopReason;

class CrosswalkModule : public SceneModuleInterface
{
public:
  struct PlannerParam
  {
    bool show_processing_time;
    // param for stop position
    double stop_margin;
    double stop_line_distance;
    double stop_line_margin;
    double stop_position_threshold;
    // param for ego velocity
    double slow_velocity;
    double max_slow_down_jerk;
    double max_slow_down_accel;
    double no_relax_velocity;
    // param for stuck vehicle
    double stuck_vehicle_velocity;
    double max_lateral_offset;
    double stuck_vehicle_attention_range;
    // param for pass judge logic
    double ego_pass_first_margin;
    double ego_pass_later_margin;
    double stop_object_velocity;
    double min_object_velocity;
    double max_yield_timeout;
    // param for input data
    double external_input_timeout;
    double tl_state_timeout;
    // param for target area & object
    double crosswalk_attention_range;
    bool look_unknown;
    bool look_bicycle;
    bool look_motorcycle;
    bool look_pedestrian;
  };

  CrosswalkModule(
    const int64_t module_id, const lanelet::ConstLanelet & crosswalk,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray() override;

private:
  int64_t module_id_;

  boost::optional<std::pair<size_t, PathPointWithLaneId>> findNearestStopPoint(
    const PathWithLaneId & ego_path, StopReason & stop_reason);

  boost::optional<std::pair<double, geometry_msgs::msg::Point>> getStopLine(
    const PathWithLaneId & ego_path) const;

  std::vector<CollisionPoint> getCollisionPoints(
    const PathWithLaneId & ego_path, const PredictedObject & object,
    const std::pair<double, double> & crosswalk_attention_range);

  std::pair<double, double> getAttentionRange(const PathWithLaneId & ego_path);

  void insertDecelPoint(
    const std::pair<size_t, PathPointWithLaneId> & stop_point, PathWithLaneId & output);

  void clampAttentionRangeByNeighborCrosswalks(
    const PathWithLaneId & ego_path, double & near_attention_range, double & far_attention_range);

  CollisionPoint createCollisionPoint(
    const geometry_msgs::msg::Point & nearest_collision_point, const double dist_ego2cp,
    const double dist_obj2cp, const geometry_msgs::msg::Vector3 & ego_vel,
    const geometry_msgs::msg::Vector3 & obj_vel) const;

  CollisionPointState getCollisionPointState(const double ttc, const double ttv) const;

  float calcTargetVelocity(
    const PathPointWithLaneId & stop_point, const PathWithLaneId & ego_path) const;

  bool isStuckVehicle(const PathWithLaneId & ego_path, const PredictedObject & object) const;

  bool isRedSignalForPedestrians() const;

  bool isTargetType(const PredictedObject & object) const;

  bool isTargetExternalInputStatus(const int target_status) const;

  geometry_msgs::msg::Polygon createObjectPolygon(
    const double width_m, const double length_m) const;

  geometry_msgs::msg::Polygon createVehiclePolygon(
    const vehicle_info_util::VehicleInfo & vehicle_info) const;

  lanelet::ConstLanelet crosswalk_;

  std::vector<geometry_msgs::msg::Point> path_intersects_;

  // Parameter
  PlannerParam planner_param_;

  // Ignore objects
  std::unordered_map<std::string, rclcpp::Time> stopped_objects_;
  std::unordered_map<std::string, rclcpp::Time> ignore_objects_;

  // Debug
  DebugData debug_data_;

  // Stop watch
  StopWatch<std::chrono::milliseconds> stop_watch_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__CROSSWALK__SCENE_CROSSWALK_HPP_
