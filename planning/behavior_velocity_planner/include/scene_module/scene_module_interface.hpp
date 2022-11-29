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

#ifndef SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_
#define SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_

#include "behavior_velocity_planner/planner_data.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rtc_interface/rtc_interface.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <utilization/util.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>
#include <tier4_planning_msgs/msg/stop_reason.hpp>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>
#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

// Debug
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

namespace behavior_velocity_planner
{

using autoware_auto_planning_msgs::msg::PathWithLaneId;
using builtin_interfaces::msg::Time;
using rtc_interface::RTCInterface;
using tier4_autoware_utils::DebugPublisher;
using tier4_autoware_utils::StopWatch;
using tier4_debug_msgs::msg::Float64Stamped;
using tier4_planning_msgs::msg::StopFactor;
using tier4_planning_msgs::msg::StopReason;
using unique_identifier_msgs::msg::UUID;

class SceneModuleInterface
{
public:
  explicit SceneModuleInterface(
    const int64_t module_id, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
  : module_id_(module_id),
    safe_(false),
    distance_(std::numeric_limits<double>::lowest()),
    logger_(logger),
    clock_(clock)
  {
  }
  virtual ~SceneModuleInterface() = default;

  virtual bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) = 0;

  virtual visualization_msgs::msg::MarkerArray createDebugMarkerArray() = 0;
  virtual visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray() = 0;

  int64_t getModuleId() const { return module_id_; }
  void setPlannerData(const std::shared_ptr<const PlannerData> & planner_data)
  {
    planner_data_ = planner_data;
  }

  boost::optional<tier4_v2x_msgs::msg::InfrastructureCommand> getInfrastructureCommand()
  {
    return infrastructure_command_;
  }

  void setInfrastructureCommand(
    const boost::optional<tier4_v2x_msgs::msg::InfrastructureCommand> & command)
  {
    infrastructure_command_ = command;
  }

  boost::optional<int> getFirstStopPathPointIndex() { return first_stop_path_point_index_; }

  void setActivation(const bool activated) { activated_ = activated; }
  bool isActivated() const { return activated_; }
  bool isSafe() const { return safe_; }
  double getDistance() const { return distance_; }

protected:
  const int64_t module_id_;
  bool activated_;
  bool safe_;
  double distance_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<const PlannerData> planner_data_;
  boost::optional<tier4_v2x_msgs::msg::InfrastructureCommand> infrastructure_command_;
  boost::optional<int> first_stop_path_point_index_;

  void setSafe(const bool safe) { safe_ = safe; }
  void setDistance(const double distance) { distance_ = distance; }

  template <class T>
  size_t findEgoSegmentIndex(const std::vector<T> & points) const
  {
    const auto & p = planner_data_;
    return motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      points, p->current_pose.pose, p->ego_nearest_dist_threshold, p->ego_nearest_yaw_threshold);
  }

  template <class T>
  size_t findEgoIndex(const std::vector<T> & points) const
  {
    const auto & p = planner_data_;
    return motion_utils::findFirstNearestIndexWithSoftConstraints(
      points, p->current_pose.pose, p->ego_nearest_dist_threshold, p->ego_nearest_yaw_threshold);
  }
};

class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(rclcpp::Node & node, [[maybe_unused]] const char * module_name)
  : clock_(node.get_clock()), logger_(node.get_logger())
  {
    const auto ns = std::string("~/debug/") + module_name;
    pub_debug_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(ns, 1);
    if (!node.has_parameter("is_publish_debug_path")) {
      is_publish_debug_path_ = node.declare_parameter("is_publish_debug_path", false);
    } else {
      is_publish_debug_path_ = node.get_parameter("is_publish_debug_path").as_bool();
    }
    if (is_publish_debug_path_) {
      pub_debug_path_ = node.create_publisher<autoware_auto_planning_msgs::msg::PathWithLaneId>(
        std::string("~/debug/path_with_lane_id/") + module_name, 1);
    }
    pub_virtual_wall_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(
      std::string("~/virtual_wall/") + module_name, 5);
    pub_stop_reason_ =
      node.create_publisher<tier4_planning_msgs::msg::StopReasonArray>("~/output/stop_reasons", 1);
    pub_infrastructure_commands_ =
      node.create_publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>(
        "~/output/infrastructure_commands", 1);

    processing_time_publisher_ = std::make_shared<DebugPublisher>(&node, "~/debug");
  }

  virtual ~SceneModuleManagerInterface() = default;

  virtual const char * getModuleName() = 0;

  boost::optional<int> getFirstStopPathPointIndex() { return first_stop_path_point_index_; }

  void updateSceneModuleInstances(
    const std::shared_ptr<const PlannerData> & planner_data,
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
  {
    planner_data_ = planner_data;

    launchNewModules(path);
    deleteExpiredModules(path);
  }

  virtual void plan(autoware_auto_planning_msgs::msg::PathWithLaneId * path)
  {
    modifyPathVelocity(path);
  }

protected:
  virtual void modifyPathVelocity(autoware_auto_planning_msgs::msg::PathWithLaneId * path)
  {
    StopWatch<std::chrono::milliseconds> stop_watch;
    stop_watch.tic("Total");
    visualization_msgs::msg::MarkerArray debug_marker_array;
    visualization_msgs::msg::MarkerArray virtual_wall_marker_array;
    tier4_planning_msgs::msg::StopReasonArray stop_reason_array;
    stop_reason_array.header.frame_id = "map";
    stop_reason_array.header.stamp = clock_->now();

    tier4_v2x_msgs::msg::InfrastructureCommandArray infrastructure_command_array;
    infrastructure_command_array.stamp = clock_->now();

    first_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;
    for (const auto & scene_module : scene_modules_) {
      tier4_planning_msgs::msg::StopReason stop_reason;
      scene_module->setPlannerData(planner_data_);
      scene_module->modifyPathVelocity(path, &stop_reason);

      if (stop_reason.reason != "") {
        stop_reason_array.stop_reasons.emplace_back(stop_reason);
      }

      if (const auto command = scene_module->getInfrastructureCommand()) {
        infrastructure_command_array.commands.push_back(*command);
      }

      if (scene_module->getFirstStopPathPointIndex() < first_stop_path_point_index_) {
        first_stop_path_point_index_ = scene_module->getFirstStopPathPointIndex();
      }

      for (const auto & marker : scene_module->createDebugMarkerArray().markers) {
        debug_marker_array.markers.push_back(marker);
      }

      for (const auto & marker : scene_module->createVirtualWallMarkerArray().markers) {
        virtual_wall_marker_array.markers.push_back(marker);
      }
    }

    if (!stop_reason_array.stop_reasons.empty()) {
      pub_stop_reason_->publish(stop_reason_array);
    }
    pub_infrastructure_commands_->publish(infrastructure_command_array);
    pub_debug_->publish(debug_marker_array);
    if (is_publish_debug_path_) {
      autoware_auto_planning_msgs::msg::PathWithLaneId debug_path;
      debug_path.header = path->header;
      debug_path.points = path->points;
      pub_debug_path_->publish(debug_path);
    }
    pub_virtual_wall_->publish(virtual_wall_marker_array);
    processing_time_publisher_->publish<Float64Stamped>(
      std::string(getModuleName()) + "/processing_time_ms", stop_watch.toc("Total"));
  }

  virtual void launchNewModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) = 0;

  virtual std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
  getModuleExpiredFunction(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) = 0;

  virtual void deleteExpiredModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
  {
    const auto isModuleExpired = getModuleExpiredFunction(path);

    // Copy container to avoid iterator corruption
    // due to scene_modules_.erase() in unregisterModule()
    const auto copied_scene_modules = scene_modules_;

    for (const auto & scene_module : copied_scene_modules) {
      if (isModuleExpired(scene_module)) {
        unregisterModule(scene_module);
      }
    }
  }

  bool isModuleRegistered(const int64_t module_id)
  {
    return registered_module_id_set_.count(module_id) != 0;
  }

  void registerModule(const std::shared_ptr<SceneModuleInterface> & scene_module)
  {
    RCLCPP_INFO(
      logger_, "register task: module = %s, id = %lu", getModuleName(),
      scene_module->getModuleId());
    registered_module_id_set_.emplace(scene_module->getModuleId());
    scene_modules_.insert(scene_module);
  }

  void unregisterModule(const std::shared_ptr<SceneModuleInterface> & scene_module)
  {
    RCLCPP_INFO(
      logger_, "unregister task: module = %s, id = %lu", getModuleName(),
      scene_module->getModuleId());
    registered_module_id_set_.erase(scene_module->getModuleId());
    scene_modules_.erase(scene_module);
  }

  template <class T>
  size_t findEgoSegmentIndex(const std::vector<T> & points) const
  {
    const auto & p = planner_data_;
    return motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      points, p->current_pose.pose, p->ego_nearest_dist_threshold, p->ego_nearest_yaw_threshold);
  }

  template <class T>
  size_t findEgoIndex(
    const std::vector<T> & points, const geometry_msgs::msg::Pose & ego_pose) const
  {
    const auto & p = planner_data_;
    return motion_utils::findFirstNearestIndexWithSoftConstraints(
      points, p->current_pose.pose, p->ego_nearest_dist_threshold, p->ego_nearest_yaw_threshold);
  }

  std::set<std::shared_ptr<SceneModuleInterface>> scene_modules_;
  std::set<int64_t> registered_module_id_set_;

  std::shared_ptr<const PlannerData> planner_data_;

  boost::optional<int> first_stop_path_point_index_;
  rclcpp::Clock::SharedPtr clock_;
  // Debug
  bool is_publish_debug_path_ = {false};  // note : this is very heavy debug topic option
  rclcpp::Logger logger_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_virtual_wall_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::PathWithLaneId>::SharedPtr pub_debug_path_;
  rclcpp::Publisher<tier4_planning_msgs::msg::StopReasonArray>::SharedPtr pub_stop_reason_;
  rclcpp::Publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>::SharedPtr
    pub_infrastructure_commands_;

  std::shared_ptr<DebugPublisher> processing_time_publisher_;
};

class SceneModuleManagerInterfaceWithRTC : public SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterfaceWithRTC(rclcpp::Node & node, const char * module_name)
  : SceneModuleManagerInterface(node, module_name), rtc_interface_(&node, module_name)
  {
  }

  void plan(autoware_auto_planning_msgs::msg::PathWithLaneId * path) override
  {
    setActivation();
    modifyPathVelocity(path);
    sendRTC(path->header.stamp);
  }

protected:
  RTCInterface rtc_interface_;
  std::unordered_map<int64_t, UUID> map_uuid_;

  void sendRTC(const Time & stamp)
  {
    for (const auto & scene_module : scene_modules_) {
      const UUID uuid = getUUID(scene_module->getModuleId());
      updateRTCStatus(uuid, scene_module->isSafe(), scene_module->getDistance(), stamp);
    }
    publishRTCStatus(stamp);
  }

  void setActivation()
  {
    for (const auto & scene_module : scene_modules_) {
      const UUID uuid = getUUID(scene_module->getModuleId());
      scene_module->setActivation(rtc_interface_.isActivated(uuid));
    }
  }

  void updateRTCStatus(
    const UUID & uuid, const bool safe, const double distance, const Time & stamp)
  {
    rtc_interface_.updateCooperateStatus(uuid, safe, distance, distance, stamp);
  }

  void removeRTCStatus(const UUID & uuid) { rtc_interface_.removeCooperateStatus(uuid); }

  void publishRTCStatus(const Time & stamp) { rtc_interface_.publishCooperateStatus(stamp); }

  UUID getUUID(const int64_t & module_id) const
  {
    if (map_uuid_.count(module_id) == 0) {
      const UUID uuid;
      return uuid;
    }
    return map_uuid_.at(module_id);
  }

  void generateUUID(const int64_t & module_id)
  {
    // Generate random number
    UUID uuid;
    std::mt19937 gen(std::random_device{}());
    std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
    std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);
    map_uuid_.insert({module_id, uuid});
  }

  void removeUUID(const int64_t & module_id)
  {
    const auto result = map_uuid_.erase(module_id);
    if (result == 0) {
      RCLCPP_WARN_STREAM(
        logger_, "[removeUUID] module_id = " << module_id << " is not registered.");
    }
  }

  void deleteExpiredModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) override
  {
    const auto isModuleExpired = getModuleExpiredFunction(path);

    // Copy container to avoid iterator corruption
    // due to scene_modules_.erase() in unregisterModule()
    const auto copied_scene_modules = scene_modules_;

    for (const auto & scene_module : copied_scene_modules) {
      if (isModuleExpired(scene_module)) {
        removeRTCStatus(getUUID(scene_module->getModuleId()));
        removeUUID(scene_module->getModuleId());
        unregisterModule(scene_module);
      }
    }
  }
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_
