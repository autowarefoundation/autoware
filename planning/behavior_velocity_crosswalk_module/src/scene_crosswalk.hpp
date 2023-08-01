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

#ifndef SCENE_CROSSWALK_HPP_
#define SCENE_CROSSWALK_HPP_

#include "behavior_velocity_crosswalk_module/util.hpp"

#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <optional>
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
using lanelet::autoware::Crosswalk;
using tier4_api_msgs::msg::CrosswalkStatus;
using tier4_autoware_utils::Polygon2d;
using tier4_autoware_utils::StopWatch;

class CrosswalkModule : public SceneModuleInterface
{
public:
  struct PlannerParam
  {
    bool show_processing_time;
    // param for stop position
    double stop_distance_from_object;
    double stop_distance_from_crosswalk;
    double far_object_threshold;
    double stop_position_threshold;
    // param for ego velocity
    float min_slow_down_velocity;
    double max_slow_down_jerk;
    double max_slow_down_accel;
    double no_relax_velocity;
    // param for stuck vehicle
    double stuck_vehicle_velocity;
    double max_stuck_vehicle_lateral_offset;
    double stuck_vehicle_attention_range;
    double min_acc_for_stuck_vehicle;
    double max_jerk_for_stuck_vehicle;
    double min_jerk_for_stuck_vehicle;
    // param for pass judge logic
    double ego_pass_first_margin;
    double ego_pass_later_margin;
    double stop_object_velocity;
    double min_object_velocity;
    bool disable_stop_for_yield_cancel;
    bool disable_yield_for_new_stopped_object;
    double timeout_no_intention_to_walk;
    double timeout_ego_stop_for_yield;
    // param for input data
    double traffic_light_state_timeout;
    // param for target area & object
    double crosswalk_attention_range;
    bool look_unknown;
    bool look_bicycle;
    bool look_motorcycle;
    bool look_pedestrian;
  };

  struct ObjectInfo
  {
    // NOTE: FULLY_STOPPED means stopped object which can be ignored.
    enum class State { STOPPED = 0, FULLY_STOPPED, OTHER };
    State state;
    std::optional<rclcpp::Time> time_to_start_stopped{std::nullopt};

    void updateState(
      const rclcpp::Time & now, const double obj_vel, const bool is_ego_yielding,
      const bool has_traffic_light, const PlannerParam & planner_param)
    {
      const bool is_stopped = obj_vel < planner_param.stop_object_velocity;

      if (is_stopped) {
        if (state == State::FULLY_STOPPED) {
          return;
        }

        if (!time_to_start_stopped) {
          time_to_start_stopped = now;
        }
        const bool intent_to_cross =
          (now - *time_to_start_stopped).seconds() < planner_param.timeout_no_intention_to_walk;
        if (
          (is_ego_yielding || (has_traffic_light && planner_param.disable_stop_for_yield_cancel)) &&
          !intent_to_cross) {
          state = State::FULLY_STOPPED;
        } else {
          // NOTE: Object may start moving
          state = State::STOPPED;
        }
      } else {
        time_to_start_stopped = std::nullopt;
        state = State::OTHER;
      }
    }
  };
  struct ObjectInfoManager
  {
    void init() { current_uuids_.clear(); }
    void update(
      const std::string & uuid, const double obj_vel, const rclcpp::Time & now,
      const bool is_ego_yielding, const bool has_traffic_light, const PlannerParam & planner_param)
    {
      // update current uuids
      current_uuids_.push_back(uuid);

      // add new object
      if (objects.count(uuid) == 0) {
        if (
          has_traffic_light && planner_param.disable_stop_for_yield_cancel &&
          planner_param.disable_yield_for_new_stopped_object) {
          objects.emplace(uuid, ObjectInfo{ObjectInfo::State::FULLY_STOPPED});
        } else {
          objects.emplace(uuid, ObjectInfo{ObjectInfo::State::OTHER});
        }
      }

      // update object state
      objects.at(uuid).updateState(now, obj_vel, is_ego_yielding, has_traffic_light, planner_param);
    }
    void finalize()
    {
      // remove objects not set in current_uuids_
      std::vector<std::string> obsolete_uuids;
      for (const auto & object : objects) {
        if (
          std::find(current_uuids_.begin(), current_uuids_.end(), object.first) ==
          current_uuids_.end()) {
          obsolete_uuids.push_back(object.first);
        }
      }
      for (const auto & obsolete_uuid : obsolete_uuids) {
        objects.erase(obsolete_uuid);
      }
    }
    ObjectInfo::State getState(const std::string & uuid) const { return objects.at(uuid).state; }

    std::unordered_map<std::string, ObjectInfo> objects;
    std::vector<std::string> current_uuids_;
  };

  CrosswalkModule(
    const int64_t module_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
    const PlannerParam & planner_param, const bool use_regulatory_element,
    const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  motion_utils::VirtualWalls createVirtualWalls() override;

private:
  // main functions
  void applySafetySlowDownSpeed(
    PathWithLaneId & output, const std::vector<geometry_msgs::msg::Point> & path_intersects);

  std::optional<std::pair<geometry_msgs::msg::Point, double>> getStopPointWithMargin(
    const PathWithLaneId & ego_path,
    const std::vector<geometry_msgs::msg::Point> & path_intersects) const;

  std::optional<StopFactor> checkStopForCrosswalkUsers(
    const PathWithLaneId & ego_path, const PathWithLaneId & sparse_resample_path,
    const std::optional<std::pair<geometry_msgs::msg::Point, double>> & p_stop_line,
    const std::vector<geometry_msgs::msg::Point> & path_intersects,
    const std::optional<geometry_msgs::msg::Pose> & default_stop_pose);

  std::optional<StopFactor> checkStopForStuckVehicles(
    const PathWithLaneId & ego_path, const std::vector<PredictedObject> & objects,
    const std::vector<geometry_msgs::msg::Point> & path_intersects,
    const std::optional<geometry_msgs::msg::Pose> & stop_pose) const;

  std::vector<CollisionPoint> getCollisionPoints(
    const PathWithLaneId & ego_path, const PredictedObject & object,
    const Polygon2d & attention_area, const std::pair<double, double> & crosswalk_attention_range);

  std::optional<StopFactor> getNearestStopFactor(
    const PathWithLaneId & ego_path,
    const std::optional<StopFactor> & stop_factor_for_crosswalk_users,
    const std::optional<StopFactor> & stop_factor_for_stuck_vehicles);

  void planGo(PathWithLaneId & ego_path, const std::optional<StopFactor> & stop_factor);

  void planStop(
    PathWithLaneId & ego_path, const std::optional<StopFactor> & nearest_stop_factor,
    const std::optional<geometry_msgs::msg::Pose> & default_stop_pose, StopReason * stop_reason);

  // minor functions
  std::pair<double, double> getAttentionRange(
    const PathWithLaneId & ego_path,
    const std::vector<geometry_msgs::msg::Point> & path_intersects);

  void insertDecelPointWithDebugInfo(
    const geometry_msgs::msg::Point & stop_point, const float target_velocity,
    PathWithLaneId & output);

  std::pair<double, double> clampAttentionRangeByNeighborCrosswalks(
    const PathWithLaneId & ego_path, const double near_attention_range,
    const double far_attention_range);

  CollisionPoint createCollisionPoint(
    const geometry_msgs::msg::Point & nearest_collision_point, const double dist_ego2cp,
    const double dist_obj2cp, const geometry_msgs::msg::Vector3 & ego_vel,
    const geometry_msgs::msg::Vector3 & obj_vel) const;

  CollisionState getCollisionState(
    const std::string & obj_uuid, const double ttc, const double ttv) const;

  float calcTargetVelocity(
    const geometry_msgs::msg::Point & stop_point, const PathWithLaneId & ego_path) const;

  Polygon2d getAttentionArea(
    const PathWithLaneId & sparse_resample_path,
    const std::pair<double, double> & crosswalk_attention_range) const;

  bool isStuckVehicle(
    const PathWithLaneId & ego_path, const std::vector<PredictedObject> & objects,
    const std::vector<geometry_msgs::msg::Point> & path_intersects) const;

  void updateObjectState(const double dist_ego_to_stop);

  bool isRedSignalForPedestrians() const;

  static bool isVehicle(const PredictedObject & object);

  bool isCrosswalkUserType(const PredictedObject & object) const;

  static geometry_msgs::msg::Polygon createObjectPolygon(
    const double width_m, const double length_m);

  static geometry_msgs::msg::Polygon createVehiclePolygon(
    const vehicle_info_util::VehicleInfo & vehicle_info);

  void recordTime(const int step_num)
  {
    RCLCPP_INFO_EXPRESSION(
      logger_, planner_param_.show_processing_time, "- step%d: %f ms", step_num,
      stop_watch_.toc("total_processing_time", false));
  }

  const int64_t module_id_;

  lanelet::ConstLanelet crosswalk_;

  lanelet::ConstLineStrings3d stop_lines_;

  // Parameter
  const PlannerParam planner_param_;

  ObjectInfoManager object_info_manager_;

  // Debug
  mutable DebugData debug_data_;

  // Stop watch
  StopWatch<std::chrono::milliseconds> stop_watch_;

  // whether ego passed safety_slow_point
  bool passed_safety_slow_point_;

  // whether use regulatory element
  const bool use_regulatory_element_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_CROSSWALK_HPP_
