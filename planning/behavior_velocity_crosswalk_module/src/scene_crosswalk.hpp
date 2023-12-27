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
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_debug_msgs/msg/string_stamped.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_perception_msgs::msg::TrafficSignalElement;
using lanelet::autoware::Crosswalk;
using tier4_api_msgs::msg::CrosswalkStatus;
using tier4_autoware_utils::Polygon2d;
using tier4_autoware_utils::StopWatch;

namespace
{
double interpolateEgoPassMargin(
  const std::vector<double> & x_vec, const std::vector<double> & y_vec, const double target_x)
{
  for (size_t i = 0; i < x_vec.size(); ++i) {
    if (target_x < x_vec.at(i)) {
      if (i == 0) {
        return y_vec.at(i);
      }
      const double ratio = (target_x - x_vec.at(i - 1)) / (x_vec.at(i) - x_vec.at(i - 1));
      return y_vec.at(i - 1) + ratio * (y_vec.at(i) - y_vec.at(i - 1));
    }
  }
  return y_vec.back();
}

double InterpolateMap(
  const std::vector<double> & key_map, const std::vector<double> & value_map, const double query)
{
  // If the speed is out of range of the key_map, apply zero-order hold.
  if (query <= key_map.front()) {
    return value_map.front();
  }
  if (query >= key_map.back()) {
    return value_map.back();
  }

  // Apply linear interpolation
  for (size_t i = 0; i < key_map.size() - 1; ++i) {
    if (key_map.at(i) <= query && query <= key_map.at(i + 1)) {
      auto ratio = (query - key_map.at(i)) / std::max(key_map.at(i + 1) - key_map.at(i), 1.0e-5);
      ratio = std::clamp(ratio, 0.0, 1.0);
      const auto interp = value_map.at(i) + ratio * (value_map.at(i + 1) - value_map.at(i));
      return interp;
    }
  }

  std::cerr << "Crosswalk's InterpolateMap interpolation logic is broken."
               "Please check the code."
            << std::endl;
  return value_map.back();
}
}  // namespace

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
    bool enable_stuck_check_in_intersection{false};
    double stuck_vehicle_velocity;
    double max_stuck_vehicle_lateral_offset;
    double stuck_vehicle_attention_range;
    double min_acc_for_stuck_vehicle;
    double max_jerk_for_stuck_vehicle;
    double min_jerk_for_stuck_vehicle;
    // param for pass judge logic
    std::vector<double> ego_pass_first_margin_x;
    std::vector<double> ego_pass_first_margin_y;
    double ego_pass_first_additional_margin;
    std::vector<double> ego_pass_later_margin_x;
    std::vector<double> ego_pass_later_margin_y;
    double ego_pass_later_additional_margin;
    double max_offset_to_crosswalk_for_yield;
    double min_acc_for_no_stop_decision;
    double max_jerk_for_no_stop_decision;
    double min_jerk_for_no_stop_decision;
    double stop_object_velocity;
    double min_object_velocity;
    bool disable_stop_for_yield_cancel;
    bool disable_yield_for_new_stopped_object;
    std::vector<double> distance_map_for_no_intention_to_walk;
    std::vector<double> timeout_map_for_no_intention_to_walk;
    double timeout_ego_stop_for_yield;
    // param for input data
    double traffic_light_state_timeout;
    // param for target area & object
    double crosswalk_attention_range;
    double vehicle_object_cross_angle_threshold;
    bool look_unknown;
    bool look_bicycle;
    bool look_motorcycle;
    bool look_pedestrian;
  };

  struct ObjectInfo
  {
    CollisionState collision_state{};
    std::optional<rclcpp::Time> time_to_start_stopped{std::nullopt};
    uint8_t classification{ObjectClassification::UNKNOWN};

    geometry_msgs::msg::Point position{};
    std::optional<CollisionPoint> collision_point{};

    void transitState(
      const rclcpp::Time & now, const geometry_msgs::msg::Point & position, const double vel,
      const bool is_ego_yielding, const bool has_traffic_light,
      const std::optional<CollisionPoint> & collision_point, const PlannerParam & planner_param,
      const lanelet::BasicPolygon2d & crosswalk_polygon)
    {
      const bool is_stopped = vel < planner_param.stop_object_velocity;

      // Check if the object can be ignored
      if (is_stopped) {
        if (collision_state == CollisionState::IGNORE) {
          return;
        }

        if (!time_to_start_stopped) {
          time_to_start_stopped = now;
        }
        const double distance_to_crosswalk =
          bg::distance(crosswalk_polygon, lanelet::BasicPoint2d(position.x, position.y));
        const double timeout_no_intention_to_walk = InterpolateMap(
          planner_param.distance_map_for_no_intention_to_walk,
          planner_param.timeout_map_for_no_intention_to_walk, distance_to_crosswalk);
        const bool intent_to_cross =
          (now - *time_to_start_stopped).seconds() < timeout_no_intention_to_walk;
        if (
          (is_ego_yielding || (has_traffic_light && planner_param.disable_stop_for_yield_cancel)) &&
          !intent_to_cross) {
          collision_state = CollisionState::IGNORE;
          return;
        }
      } else {
        time_to_start_stopped = std::nullopt;
      }

      // Compare time to collision and vehicle
      if (collision_point) {
        // Check if ego will pass first
        const double ego_pass_first_additional_margin =
          collision_state == CollisionState::EGO_PASS_FIRST
            ? 0.0
            : planner_param.ego_pass_first_additional_margin;
        const double ego_pass_first_margin = interpolateEgoPassMargin(
          planner_param.ego_pass_first_margin_x, planner_param.ego_pass_first_margin_y,
          collision_point->time_to_collision);
        if (
          collision_point->time_to_collision + ego_pass_first_margin +
            ego_pass_first_additional_margin <
          collision_point->time_to_vehicle) {
          collision_state = CollisionState::EGO_PASS_FIRST;
          return;
        }

        // Check if ego will pass later
        const double ego_pass_later_additional_margin =
          collision_state == CollisionState::EGO_PASS_LATER
            ? 0.0
            : planner_param.ego_pass_later_additional_margin;
        const double ego_pass_later_margin = interpolateEgoPassMargin(
          planner_param.ego_pass_later_margin_x, planner_param.ego_pass_later_margin_y,
          collision_point->time_to_vehicle);
        if (
          collision_point->time_to_vehicle + ego_pass_later_margin +
            ego_pass_later_additional_margin <
          collision_point->time_to_collision) {
          collision_state = CollisionState::EGO_PASS_LATER;
          return;
        }
        collision_state = CollisionState::YIELD;
        return;
      }
    }
  };
  struct ObjectInfoManager
  {
    void init() { current_uuids_.clear(); }
    void update(
      const std::string & uuid, const geometry_msgs::msg::Point & position, const double vel,
      const rclcpp::Time & now, const bool is_ego_yielding, const bool has_traffic_light,
      const std::optional<CollisionPoint> & collision_point, const uint8_t classification,
      const PlannerParam & planner_param, const lanelet::BasicPolygon2d & crosswalk_polygon)
    {
      // update current uuids
      current_uuids_.push_back(uuid);

      // add new object
      if (objects.count(uuid) == 0) {
        if (
          has_traffic_light && planner_param.disable_stop_for_yield_cancel &&
          planner_param.disable_yield_for_new_stopped_object) {
          objects.emplace(uuid, ObjectInfo{CollisionState::IGNORE});
        } else {
          objects.emplace(uuid, ObjectInfo{CollisionState::YIELD});
        }
      }

      // update object state
      objects.at(uuid).transitState(
        now, position, vel, is_ego_yielding, has_traffic_light, collision_point, planner_param,
        crosswalk_polygon);
      objects.at(uuid).collision_point = collision_point;
      objects.at(uuid).position = position;
      objects.at(uuid).classification = classification;
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

    std::vector<ObjectInfo> getObject() const
    {
      std::vector<ObjectInfo> object_info_vec;
      for (auto object : objects) {
        object_info_vec.push_back(object.second);
      }
      return object_info_vec;
    }
    CollisionState getCollisionState(const std::string & uuid) const
    {
      return objects.at(uuid).collision_state;
    }

    std::unordered_map<std::string, ObjectInfo> objects;
    std::vector<std::string> current_uuids_;
  };

  CrosswalkModule(
    rclcpp::Node & node, const int64_t lane_id, const int64_t module_id,
    const std::optional<int64_t> & reg_elem_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
    const PlannerParam & planner_param, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock);

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
    const std::optional<geometry_msgs::msg::Pose> & stop_pose);

  std::optional<double> findEgoPassageDirectionAlongPath(
    const PathWithLaneId & sparse_resample_path) const;
  std::optional<double> findObjectPassageDirectionAlongVehicleLane(
    const autoware_auto_perception_msgs::msg::PredictedPath & path) const;

  std::optional<CollisionPoint> getCollisionPoint(
    const PathWithLaneId & ego_path, const PredictedObject & object,
    const std::pair<double, double> & crosswalk_attention_range, const Polygon2d & attention_area);

  std::optional<StopFactor> getNearestStopFactor(
    const PathWithLaneId & ego_path,
    const std::optional<StopFactor> & stop_factor_for_crosswalk_users,
    const std::optional<StopFactor> & stop_factor_for_stuck_vehicles);

  void setDistanceToStop(
    const PathWithLaneId & ego_path,
    const std::optional<geometry_msgs::msg::Pose> & default_stop_pose,
    const std::optional<StopFactor> & stop_factor);

  void planGo(PathWithLaneId & ego_path, const std::optional<StopFactor> & stop_factor) const;

  void planStop(
    PathWithLaneId & ego_path, const std::optional<StopFactor> & nearest_stop_factor,
    const std::optional<geometry_msgs::msg::Pose> & default_stop_pose, StopReason * stop_reason);

  // minor functions
  std::pair<double, double> getAttentionRange(
    const PathWithLaneId & ego_path,
    const std::vector<geometry_msgs::msg::Point> & path_intersects);

  void insertDecelPointWithDebugInfo(
    const geometry_msgs::msg::Point & stop_point, const float target_velocity,
    PathWithLaneId & output) const;

  std::pair<double, double> clampAttentionRangeByNeighborCrosswalks(
    const PathWithLaneId & ego_path, const double near_attention_range,
    const double far_attention_range);

  CollisionPoint createCollisionPoint(
    const geometry_msgs::msg::Point & nearest_collision_point, const double dist_ego2cp,
    const double dist_obj2cp, const geometry_msgs::msg::Vector3 & ego_vel,
    const geometry_msgs::msg::Vector3 & obj_vel,
    const std::optional<double> object_crosswalk_passage_direction) const;

  float calcTargetVelocity(
    const geometry_msgs::msg::Point & stop_point, const PathWithLaneId & ego_path) const;

  Polygon2d getAttentionArea(
    const PathWithLaneId & sparse_resample_path,
    const std::pair<double, double> & crosswalk_attention_range) const;

  bool isStuckVehicle(
    const PathWithLaneId & ego_path, const std::vector<PredictedObject> & objects,
    const std::vector<geometry_msgs::msg::Point> & path_intersects) const;

  void updateObjectState(
    const double dist_ego_to_stop, const PathWithLaneId & sparse_resample_path,
    const std::pair<double, double> & crosswalk_attention_range, const Polygon2d & attention_area);

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

  rclcpp::Publisher<tier4_debug_msgs::msg::StringStamped>::SharedPtr collision_info_pub_;

  lanelet::ConstLanelet crosswalk_;

  lanelet::ConstLanelet road_;

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
