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

#ifndef AUTOWARE__BEHAVIOR_PATH_DYNAMIC_OBSTACLE_AVOIDANCE_MODULE__SCENE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_DYNAMIC_OBSTACLE_AVOIDANCE_MODULE__SCENE_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
template <typename T>
bool isInVector(const T & val, const std::vector<T> & vec)
{
  return std::find(vec.begin(), vec.end(), val) != vec.end();
}

template <typename T, typename S>
std::vector<T> getAllKeys(const std::unordered_map<T, S> & map)
{
  std::vector<T> keys;
  for (const auto & pair : map) {
    keys.push_back(pair.first);
  }
  return keys;
}
}  // namespace

namespace autoware::behavior_path_planner
{
using autoware::universe_utils::Polygon2d;
using autoware_perception_msgs::msg::PredictedPath;
using tier4_planning_msgs::msg::PathWithLaneId;

struct MinMaxValue
{
  double min_value{0.0};
  double max_value{0.0};
  MinMaxValue operator+(const double scalar) const
  {
    MinMaxValue ret;
    ret.min_value = min_value + scalar;
    ret.max_value = max_value + scalar;
    return ret;
  };
  void swap() { std::swap(min_value, max_value); }
};

enum class PolygonGenerationMethod {
  EGO_PATH_BASE = 0,
  OBJECT_PATH_BASE,
};

enum class ObjectType {
  OUT_OF_SCOPE = 0,  // The module do not care about this type of objects.
  REGULATED,    // The module assumes this type of objects move in parallel against lanes. Drivable
                // areas are divided proportionately with the ego. Typically, cars, bus and trucks
                // are classified to this type.
  UNREGULATED,  // The module does not assume the objects move in parallel against lanes and
                // assigns drivable area with priority to ego. Typically, pedestrians should be
                // classified to this type.
};

struct DynamicAvoidanceParameters
{
  // common
  bool enable_debug_info{true};
  bool use_hatched_road_markings{true};

  std::string use_lane_type{"opposite_direction_lane"};

  // obstacle types to avoid
  bool avoid_car{true};
  bool avoid_truck{true};
  bool avoid_bus{true};
  bool avoid_trailer{true};
  bool avoid_unknown{false};
  bool avoid_bicycle{false};
  bool avoid_motorcycle{false};
  bool avoid_pedestrian{false};
  double max_obstacle_vel{0.0};
  double min_obstacle_vel{0.0};
  int successive_num_to_entry_dynamic_avoidance_condition{0};
  int successive_num_to_exit_dynamic_avoidance_condition{0};

  double min_obj_lat_offset_to_ego_path{0.0};
  double max_obj_lat_offset_to_ego_path{0.0};

  double min_time_to_start_cut_in{0.0};
  double min_lon_offset_ego_to_cut_in_object{0.0};
  double min_cut_in_object_vel{0.0};
  double max_time_from_outside_ego_path_for_cut_out{0.0};
  double min_cut_out_object_lat_vel{0.0};
  double min_cut_out_object_vel{0.0};
  double max_front_object_angle{0.0};
  double min_front_object_vel{0.0};
  double max_front_object_ego_path_lat_cover_ratio{0.0};
  double min_overtaking_crossing_object_vel{0.0};
  double max_overtaking_crossing_object_angle{0.0};
  double min_oncoming_crossing_object_vel{0.0};
  double max_oncoming_crossing_object_angle{0.0};
  double max_pedestrian_crossing_vel{0.0};
  double max_stopped_object_vel{0.0};

  // drivable area generation
  PolygonGenerationMethod polygon_generation_method{};
  bool expand_drivable_area;
  double min_obj_path_based_lon_polygon_margin{0.0};
  double lat_offset_from_obstacle{0.0};
  double margin_distance_around_pedestrian{0.0};

  double end_time_to_consider{0.0};
  double threshold_confidence{0.0};

  double max_lat_offset_to_avoid{0.0};
  double max_time_for_lat_shift{0.0};
  double lpf_gain_for_lat_avoid_to_offset{0.0};

  double max_ego_lat_acc{0.0};
  double max_ego_lat_jerk{0.0};
  double delay_time_ego_shift{0.0};

  double max_time_to_collision_overtaking_object{0.0};
  double start_duration_to_avoid_overtaking_object{0.0};
  double end_duration_to_avoid_overtaking_object{0.0};
  double duration_to_hold_avoidance_overtaking_object{0.0};

  double max_time_to_collision_oncoming_object{0.0};
  double start_duration_to_avoid_oncoming_object{0.0};
  double end_duration_to_avoid_oncoming_object{0.0};
};

struct TimeWhileCollision
{
  double time_to_start_collision;
  double time_to_end_collision;
};

struct LatFeasiblePaths
{
  std::vector<geometry_msgs::msg::Point> left_path;
  std::vector<geometry_msgs::msg::Point> right_path;
};
class DynamicObstacleAvoidanceModule : public SceneModuleInterface
{
public:
  static constexpr const char * logger_namespace =
    "planning.scenario_planning.lane_driving.behavior_planning.behavior_path_planner.dynamic_"
    "obstacle_avoidance";

  struct DynamicAvoidanceObject
  {
    DynamicAvoidanceObject(
      const PredictedObject & predicted_object, const double arg_vel, const double arg_lat_vel,
      const bool arg_is_object_on_ego_path,
      const std::optional<rclcpp::Time> & arg_latest_time_inside_ego_path)
    : uuid(autoware::universe_utils::toHexString(predicted_object.object_id)),
      label(predicted_object.classification.front().label),
      pose(predicted_object.kinematics.initial_pose_with_covariance.pose),
      shape(predicted_object.shape),
      vel(arg_vel),
      lat_vel(arg_lat_vel),
      is_object_on_ego_path(arg_is_object_on_ego_path),
      latest_time_inside_ego_path(arg_latest_time_inside_ego_path)
    {
      for (const auto & path : predicted_object.kinematics.predicted_paths) {
        predicted_paths.push_back(path);
      }
    }

    std::string uuid{};
    uint8_t label{};
    geometry_msgs::msg::Pose pose{};
    autoware_perception_msgs::msg::Shape shape;
    double vel{0.0};
    double lat_vel{0.0};
    bool is_object_on_ego_path{false};
    std::optional<rclcpp::Time> latest_time_inside_ego_path{std::nullopt};
    std::vector<autoware_perception_msgs::msg::PredictedPath> predicted_paths{};

    // NOTE: Previous values of the following are used for low-pass filtering.
    //       Therefore, they has to be initialized as nullopt.
    std::optional<MinMaxValue> lon_offset_to_avoid{std::nullopt};
    std::optional<MinMaxValue> lat_offset_to_avoid{std::nullopt};
    bool is_collision_left{false};
    bool should_be_avoided{false};
    std::vector<geometry_msgs::msg::Pose> ref_points_for_obj_poly;
    LatFeasiblePaths ego_lat_feasible_paths;

    // add additional information (not update to the latest data)
    void update(
      const MinMaxValue & arg_lon_offset_to_avoid, const MinMaxValue & arg_lat_offset_to_avoid,
      const bool arg_is_collision_left, const bool arg_should_be_avoided,
      const std::vector<geometry_msgs::msg::Pose> & arg_ref_points_for_obj_poly)
    {
      lon_offset_to_avoid = arg_lon_offset_to_avoid;
      lat_offset_to_avoid = arg_lat_offset_to_avoid;
      is_collision_left = arg_is_collision_left;
      should_be_avoided = arg_should_be_avoided;
      ref_points_for_obj_poly = arg_ref_points_for_obj_poly;
    }
  };

  struct TargetObjectsManager
  {
    TargetObjectsManager(const int arg_max_count, const int arg_min_count)
    : max_count_(arg_max_count), min_count_(arg_min_count)
    {
    }
    int max_count_{0};
    int min_count_{0};  // TODO(murooka): The sign needs to be opposite?

    void initialize() { current_uuids_.clear(); }
    void updateObject(const std::string & uuid, const DynamicAvoidanceObject & object)
    {
      // add/update object
      if (object_map_.count(uuid) != 0) {
        const auto prev_object = object_map_.at(uuid);
        object_map_.at(uuid) = object;
        // TODO(murooka) refactor this. Counter can be moved to DynamicObject,
        //               and TargetObjectsManager can be removed.
        object_map_.at(uuid).ego_lat_feasible_paths = prev_object.ego_lat_feasible_paths;
      } else {
        object_map_.emplace(uuid, object);
      }

      // increase counter
      if (counter_map_.count(uuid) != 0) {
        counter_map_.at(uuid) = std::min(max_count_, std::max(1, counter_map_.at(uuid) + 1));
      } else {
        counter_map_.emplace(uuid, 1);
      }

      // memorize uuid
      current_uuids_.push_back(uuid);
    }

    void finalize(const LatFeasiblePaths & ego_lat_feasible_paths)
    {
      // decrease counter for not updated uuids
      std::vector<std::string> not_updated_uuids;
      for (const auto & object : object_map_) {
        if (!isInVector(object.first, current_uuids_)) {
          not_updated_uuids.push_back(object.first);
        }
      }
      for (const auto & uuid : not_updated_uuids) {
        if (counter_map_.count(uuid) != 0) {
          counter_map_.at(uuid) = std::max(0, counter_map_.at(uuid) - 1);
        } else {
          counter_map_.emplace(uuid, -1);
        }
      }

      // update valid object uuids and its variable
      for (const auto & counter : counter_map_) {
        if (!isInVector(counter.first, valid_object_uuids_) && max_count_ <= counter.second) {
          valid_object_uuids_.push_back(counter.first);
          object_map_.at(counter.first).ego_lat_feasible_paths = ego_lat_feasible_paths;
        }
      }
      valid_object_uuids_.erase(
        std::remove_if(
          valid_object_uuids_.begin(), valid_object_uuids_.end(),
          [&](const auto & uuid) {
            return counter_map_.count(uuid) == 0 || counter_map_.at(uuid) < min_count_;
          }),
        valid_object_uuids_.end());

      // remove objects whose counter is lower than threshold
      const auto counter_map_keys = getAllKeys(counter_map_);
      for (const auto & key : counter_map_keys) {
        if (counter_map_.at(key) < min_count_) {
          counter_map_.erase(key);
          object_map_.erase(key);
        }
      }
    }
    std::vector<DynamicAvoidanceObject> getValidObjects() const
    {
      std::vector<DynamicAvoidanceObject> valid_objects;
      for (const auto & valid_object_uuid : valid_object_uuids_) {
        if (object_map_.count(valid_object_uuid) == 0) {
          std::cerr
            << "[DynamicAvoidance] Internal calculation has an error when getting valid objects."
            << std::endl;
          continue;
        }
        valid_objects.push_back(object_map_.at(valid_object_uuid));
      }

      return valid_objects;
    }
    void updateObjectVariables(
      const std::string & uuid, const MinMaxValue & lon_offset_to_avoid,
      const MinMaxValue & lat_offset_to_avoid, const bool is_collision_left,
      const bool should_be_avoided,
      const std::vector<geometry_msgs::msg::Pose> & ref_points_for_obj_poly)
    {
      if (object_map_.count(uuid) != 0) {
        object_map_.at(uuid).update(
          lon_offset_to_avoid, lat_offset_to_avoid, is_collision_left, should_be_avoided,
          ref_points_for_obj_poly);
      }
    }

    std::vector<std::string> current_uuids_;
    // NOTE: positive is for meeting entry condition, and negative is for exiting.
    std::unordered_map<std::string, int> counter_map_;
    std::unordered_map<std::string, DynamicAvoidanceObject> object_map_;
    std::vector<std::string> valid_object_uuids_{};
  };

  struct DecisionWithReason
  {
    bool decision;
    std::string reason{""};
  };

  DynamicObstacleAvoidanceModule(
    const std::string & name, rclcpp::Node & node,
    std::shared_ptr<DynamicAvoidanceParameters> parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    std::shared_ptr<SteeringFactorInterface> & steering_factor_interface_ptr);

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<DynamicAvoidanceParameters>>(parameters);
  }

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  // TODO(someone): remove this, and use base class function
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void updateData() override;
  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  struct LatLonOffset
  {
    const size_t nearest_idx;
    const double max_lat_offset;
    const double min_lat_offset;
    const double max_lon_offset;
    const double min_lon_offset;
  };
  struct EgoPathReservePoly
  {
    const autoware::universe_utils::Polygon2d left_avoid;
    const autoware::universe_utils::Polygon2d right_avoid;
  };

  bool canTransitSuccessState() override;

  bool canTransitFailureState() override { return false; }

  ObjectType getObjectType(const uint8_t label) const;
  void registerRegulatedObjects(const std::vector<DynamicAvoidanceObject> & prev_objects);
  void registerUnregulatedObjects(const std::vector<DynamicAvoidanceObject> & prev_objects);
  void determineWhetherToAvoidAgainstRegulatedObjects(
    const std::vector<DynamicAvoidanceObject> & prev_objects);
  void determineWhetherToAvoidAgainstUnregulatedObjects(
    const std::vector<DynamicAvoidanceObject> & prev_objects);
  LatFeasiblePaths generateLateralFeasiblePaths(
    const geometry_msgs::msg::Pose & ego_pose, const double ego_vel) const;
  void updateRefPathBeforeLaneChange(const std::vector<PathPointWithLaneId> & ego_ref_path_points);
  bool willObjectCutIn(
    const std::vector<PathPointWithLaneId> & ego_path, const PredictedPath & predicted_path,
    const double obj_tangent_vel, const LatLonOffset & lat_lon_offset) const;
  DecisionWithReason willObjectCutOut(
    const double obj_tangent_vel, const double obj_normal_vel, const bool is_object_left,
    const std::optional<DynamicAvoidanceObject> & prev_object) const;
  bool willObjectBeOutsideEgoChangingPath(
    const geometry_msgs::msg::Pose & obj_pose,
    const autoware_perception_msgs::msg::Shape & obj_shape, const double obj_vel) const;
  bool isObjectFarFromPath(
    const PredictedObject & predicted_object, const double obj_dist_to_path) const;
  TimeWhileCollision calcTimeWhileCollision(
    const std::vector<PathPointWithLaneId> & ego_path, const double obj_tangent_vel,
    const LatLonOffset & lat_lon_offset) const;
  std::optional<std::pair<size_t, size_t>> calcCollisionSection(
    const std::vector<PathPointWithLaneId> & ego_path, const PredictedPath & obj_path) const;
  LatLonOffset getLateralLongitudinalOffset(
    const std::vector<geometry_msgs::msg::Pose> & ego_points,
    const geometry_msgs::msg::Pose & obj_pose, const size_t obj_seg_idx,
    const autoware_perception_msgs::msg::Shape & obj_shape) const;
  double calcValidLengthToAvoid(
    const PredictedPath & obj_path, const geometry_msgs::msg::Pose & obj_pose,
    const autoware_perception_msgs::msg::Shape & obj_shape,
    const bool is_object_same_direction) const;
  MinMaxValue calcMinMaxLongitudinalOffsetToAvoid(
    const std::vector<geometry_msgs::msg::Pose> & ref_points_for_obj_poly,
    const geometry_msgs::msg::Pose & obj_pose, const Polygon2d & obj_points, const double obj_vel,
    const PredictedPath & obj_path, const autoware_perception_msgs::msg::Shape & obj_shape,
    const TimeWhileCollision & time_while_collision) const;
  std::optional<MinMaxValue> calcMinMaxLateralOffsetToAvoidRegulatedObject(
    const std::vector<geometry_msgs::msg::Pose> & ref_points_for_obj_poly,
    const Polygon2d & obj_points, const geometry_msgs::msg::Point & obj_pos, const double obj_vel,
    const bool is_collision_left, const double obj_normal_vel,
    const std::optional<DynamicAvoidanceObject> & prev_object) const;
  std::optional<MinMaxValue> calcMinMaxLateralOffsetToAvoidUnregulatedObject(
    const std::vector<geometry_msgs::msg::Pose> & ref_points_for_obj_poly,
    const std::optional<DynamicAvoidanceObject> & prev_object,
    const DynamicAvoidanceObject & object) const;
  std::pair<lanelet::ConstLanelets, lanelet::ConstLanelets> getAdjacentLanes(
    const double forward_distance, const double backward_distance) const;
  std::optional<autoware::universe_utils::Polygon2d> calcEgoPathBasedDynamicObstaclePolygon(
    const DynamicAvoidanceObject & object) const;
  std::optional<autoware::universe_utils::Polygon2d> calcObjectPathBasedDynamicObstaclePolygon(
    const DynamicAvoidanceObject & object) const;
  std::optional<autoware::universe_utils::Polygon2d> calcPredictedPathBasedDynamicObstaclePolygon(
    const DynamicAvoidanceObject & object, const EgoPathReservePoly & ego_path_poly) const;
  EgoPathReservePoly calcEgoPathReservePoly(const PathWithLaneId & ego_path) const;
  lanelet::ConstLanelets getCurrentLanesFromPath(
    const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data);
  DrivableLanes generateExpandedDrivableLanes(
    const lanelet::ConstLanelet & lanelet, const std::shared_ptr<const PlannerData> & planner_data,
    const std::shared_ptr<DynamicAvoidanceParameters> & parameters);

  void printIgnoreReason(const std::string & obj_uuid, const std::string & reason)
  {
    const auto reason_text =
      "[DynamicAvoidance] Ignore obstacle (%s)" + (reason == "" ? "." : " since " + reason + ".");
    RCLCPP_INFO_EXPRESSION(
      getLogger(), parameters_->enable_debug_info, reason_text.c_str(), obj_uuid.c_str());
  }

  std::vector<DynamicObstacleAvoidanceModule::DynamicAvoidanceObject> target_objects_;
  // std::vector<DynamicObstacleAvoidanceModule::DynamicAvoidanceObject> prev_target_objects_;
  std::optional<std::vector<PathPointWithLaneId>> prev_input_ref_path_points_{std::nullopt};
  std::optional<std::vector<PathPointWithLaneId>> ref_path_before_lane_change_{std::nullopt};
  std::shared_ptr<DynamicAvoidanceParameters> parameters_;

  TargetObjectsManager target_objects_manager_;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_DYNAMIC_OBSTACLE_AVOIDANCE_MODULE__SCENE_HPP_
