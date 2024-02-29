// Copyright 2019 Autoware Foundation
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

#ifndef BEHAVIOR_VELOCITY_PLANNER_COMMON__PLANNER_DATA_HPP_
#define BEHAVIOR_VELOCITY_PLANNER_COMMON__PLANNER_DATA_HPP_

#include "route_handler/route_handler.hpp"

#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_velocity_smoother/smoother/smoother_base.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_api_msgs/msg/crosswalk_status.hpp>
#include <tier4_api_msgs/msg/intersection_status.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <vector>

namespace behavior_velocity_planner
{
class BehaviorVelocityPlannerNode;
struct PlannerData
{
  explicit PlannerData(rclcpp::Node & node)
  : vehicle_info_(vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo())
  {
    max_stop_acceleration_threshold = node.declare_parameter<double>(
      "max_accel");  // TODO(someone): read min_acc in velocity_controller.param.yaml?
    max_stop_jerk_threshold = node.declare_parameter<double>("max_jerk");
    system_delay = node.declare_parameter<double>("system_delay");
    delay_response_time = node.declare_parameter<double>("delay_response_time");
  }

  // msgs from callbacks that are used for data-ready
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_odometry;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr current_acceleration;
  static constexpr double velocity_buffer_time_sec = 10.0;
  std::deque<geometry_msgs::msg::TwistStamped> velocity_buffer;
  autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr predicted_objects;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr no_ground_pointcloud;
  // occupancy grid
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid;

  // nearest search
  double ego_nearest_dist_threshold;
  double ego_nearest_yaw_threshold;

  // other internal data
  // traffic_light_id_map_raw is the raw observation, while traffic_light_id_map_keep_last keeps the
  // last observed infomation for UNKNOWN
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_raw_;
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_last_observed_;
  std::optional<tier4_planning_msgs::msg::VelocityLimit> external_velocity_limit;
  tier4_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr virtual_traffic_light_states;

  // This variable is used when the Autoware's behavior has to depend on whether it's simulation or
  // not.
  bool is_simulation = false;

  // velocity smoother
  std::shared_ptr<motion_velocity_smoother::SmootherBase> velocity_smoother_;
  // route handler
  std::shared_ptr<route_handler::RouteHandler> route_handler_;
  // parameters
  vehicle_info_util::VehicleInfo vehicle_info_;

  // additional parameters
  double max_stop_acceleration_threshold;
  double max_stop_jerk_threshold;
  double system_delay;
  double delay_response_time;
  double stop_line_extend_length;

  bool isVehicleStopped(const double stop_duration = 0.0) const
  {
    if (velocity_buffer.empty()) {
      return false;
    }

    // Get velocities within stop_duration
    const auto now = rclcpp::Clock{RCL_ROS_TIME}.now();
    std::vector<double> vs;
    for (const auto & velocity : velocity_buffer) {
      vs.push_back(velocity.twist.linear.x);

      const auto & s = velocity.header.stamp;
      const auto time_diff =
        now >= s ? now - s : rclcpp::Duration(0, 0);  // Note: negative time throws an exception.
      if (time_diff.seconds() >= stop_duration) {
        break;
      }
    }

    // Check all velocities
    constexpr double stop_velocity = 1e-3;
    for (const auto & v : vs) {
      if (v >= stop_velocity) {
        return false;
      }
    }

    return true;
  }

  /**
   *@fn
   *@brief queries the traffic signal information of given Id. if keep_last_observation = true,
   *recent UNKNOWN observation is overwritten as the last non-UNKNOWN observation
   */
  std::optional<TrafficSignalStamped> getTrafficSignal(
    const lanelet::Id id, const bool keep_last_observation = false) const
  {
    const auto & traffic_light_id_map =
      keep_last_observation ? traffic_light_id_map_last_observed_ : traffic_light_id_map_raw_;
    if (traffic_light_id_map.count(id) == 0) {
      return std::nullopt;
    }
    return std::make_optional<TrafficSignalStamped>(traffic_light_id_map.at(id));
  }
};
}  // namespace behavior_velocity_planner

#endif  // BEHAVIOR_VELOCITY_PLANNER_COMMON__PLANNER_DATA_HPP_
