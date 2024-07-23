// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__TYPE_ALIAS_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__TYPE_ALIAS_HPP_

#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include "autoware_adapi_v1_msgs/msg/planning_behavior.hpp"
#include "autoware_adapi_v1_msgs/msg/velocity_factor_array.hpp"
#include "autoware_perception_msgs/msg/predicted_object.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"
#include "tier4_debug_msgs/msg/float64_stamped.hpp"
#include "tier4_planning_msgs/msg/stop_factor.hpp"
#include "tier4_planning_msgs/msg/stop_reason_array.hpp"
#include "tier4_planning_msgs/msg/stop_speed_exceeded.hpp"
#include "tier4_planning_msgs/msg/velocity_limit.hpp"
#include "tier4_planning_msgs/msg/velocity_limit_clear_command.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using autoware::vehicle_info_utils::VehicleInfo;
using autoware_adapi_v1_msgs::msg::PlanningBehavior;
using autoware_adapi_v1_msgs::msg::VelocityFactor;
using autoware_adapi_v1_msgs::msg::VelocityFactorArray;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelStamped;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using tier4_debug_msgs::msg::Float32Stamped;
using tier4_debug_msgs::msg::Float64Stamped;
using tier4_planning_msgs::msg::StopFactor;
using tier4_planning_msgs::msg::StopReason;
using tier4_planning_msgs::msg::StopReasonArray;
using tier4_planning_msgs::msg::StopSpeedExceeded;
using tier4_planning_msgs::msg::VelocityLimit;
using tier4_planning_msgs::msg::VelocityLimitClearCommand;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
namespace bg = boost::geometry;
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__TYPE_ALIAS_HPP_
