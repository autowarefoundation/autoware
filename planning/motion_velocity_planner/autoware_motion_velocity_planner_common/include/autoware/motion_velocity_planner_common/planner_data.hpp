// Copyright 2024 Autoware Foundation
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

#ifndef AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__PLANNER_DATA_HPP_
#define AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__PLANNER_DATA_HPP_

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_velocity_planner_common/collision_checker.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/velocity_smoother/smoother/smoother_base.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_api_msgs/msg/crosswalk_status.hpp>
#include <tier4_api_msgs/msg/intersection_status.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>

#include <lanelet2_core/Forward.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <memory>
#include <optional>

namespace autoware::motion_velocity_planner
{
struct TrafficSignalStamped
{
  builtin_interfaces::msg::Time stamp;
  autoware_perception_msgs::msg::TrafficLightGroup signal;
};
struct PlannerData
{
  explicit PlannerData(rclcpp::Node & node)
  : vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo())
  {
  }

  // msgs from callbacks that are used for data-ready
  nav_msgs::msg::Odometry current_odometry;
  geometry_msgs::msg::AccelWithCovarianceStamped current_acceleration;
  autoware_perception_msgs::msg::PredictedObjects predicted_objects;
  pcl::PointCloud<pcl::PointXYZ> no_ground_pointcloud;
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  std::shared_ptr<route_handler::RouteHandler> route_handler;

  // nearest search
  double ego_nearest_dist_threshold{};
  double ego_nearest_yaw_threshold{};

  // other internal data
  // traffic_light_id_map_raw is the raw observation, while traffic_light_id_map_keep_last keeps the
  // last observed infomation for UNKNOWN
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_raw_;
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_last_observed_;
  std::optional<tier4_planning_msgs::msg::VelocityLimit> external_velocity_limit;
  tier4_v2x_msgs::msg::VirtualTrafficLightStateArray virtual_traffic_light_states;

  // velocity smoother
  std::shared_ptr<autoware::velocity_smoother::SmootherBase> velocity_smoother_;
  // parameters
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  /**
   *@fn
   *@brief queries the traffic signal information of given Id. if keep_last_observation = true,
   *recent UNKNOWN observation is overwritten as the last non-UNKNOWN observation
   */
  [[nodiscard]] std::optional<TrafficSignalStamped> get_traffic_signal(
    const lanelet::Id id, const bool keep_last_observation = false) const
  {
    const auto & traffic_light_id_map =
      keep_last_observation ? traffic_light_id_map_last_observed_ : traffic_light_id_map_raw_;
    if (traffic_light_id_map.count(id) == 0) {
      return std::nullopt;
    }
    return std::make_optional<TrafficSignalStamped>(traffic_light_id_map.at(id));
  }

  [[nodiscard]] std::optional<double> calculate_min_deceleration_distance(
    const double target_velocity) const
  {
    return motion_utils::calcDecelDistWithJerkAndAccConstraints(
      current_odometry.twist.twist.linear.x, target_velocity,
      current_acceleration.accel.accel.linear.x, velocity_smoother_->getMinDecel(),
      std::abs(velocity_smoother_->getMinJerk()), velocity_smoother_->getMinJerk());
  }
};
}  // namespace autoware::motion_velocity_planner

#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__PLANNER_DATA_HPP_
