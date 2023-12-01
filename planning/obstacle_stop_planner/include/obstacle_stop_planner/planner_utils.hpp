
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

#ifndef OBSTACLE_STOP_PLANNER__PLANNER_UTILS_HPP_
#define OBSTACLE_STOP_PLANNER__PLANNER_UTILS_HPP_

#include "obstacle_stop_planner/planner_data.hpp"

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace motion_planning
{

namespace bg = boost::geometry;

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using std_msgs::msg::Header;

using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;
using vehicle_info_util::VehicleInfo;

using TrajectoryPoints = std::vector<TrajectoryPoint>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using PointVariant = std::variant<float, double>;

std::optional<std::pair<double, double>> calcFeasibleMarginAndVelocity(
  const SlowDownParam & slow_down_param, const double dist_baselink_to_obstacle,
  const double current_vel, const double current_acc);

std::optional<std::pair<size_t, TrajectoryPoint>> getForwardInsertPointFromBasePoint(
  const size_t base_idx, const TrajectoryPoints & trajectory, const double margin);

std::optional<std::pair<size_t, TrajectoryPoint>> getBackwardInsertPointFromBasePoint(
  const size_t base_idx, const TrajectoryPoints & trajectory, const double margin);

std::optional<std::pair<size_t, double>> findNearestFrontIndex(
  const size_t start_idx, const TrajectoryPoints & trajectory, const Point & point);

void insertStopPoint(
  const StopPoint & stop_point, TrajectoryPoints & output, DiagnosticStatus & stop_reason_diag);

bool isInFrontOfTargetPoint(const Pose & pose, const Point & point);

bool checkValidIndex(const Pose & p_base, const Pose & p_next, const Pose & p_target);

bool withinPolygon(
  const Polygon2d & boost_polygon, const double radius, const Point2d & prev_point,
  const Point2d & next_point, PointCloud::Ptr candidate_points_ptr,
  PointCloud::Ptr within_points_ptr);

bool withinPolyhedron(
  const Polygon2d & boost_polygon, const double radius, const Point2d & prev_point,
  const Point2d & next_point, PointCloud::Ptr candidate_points_ptr,
  PointCloud::Ptr within_points_ptr, double z_min, double z_max);

void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point);

void createOneStepPolygon(
  const Pose & base_step_pose, const Pose & next_step_pose, Polygon2d & hull_polygon,
  const VehicleInfo & vehicle_info, const double expand_width = 0.0);

void getNearestPoint(
  const PointCloud & pointcloud, const Pose & base_pose, pcl::PointXYZ * nearest_collision_point,
  rclcpp::Time * nearest_collision_point_time);

void getLateralNearestPoint(
  const PointCloud & pointcloud, const Pose & base_pose, pcl::PointXYZ * lateral_nearest_point,
  double * deviation);

double getNearestPointAndDistanceForPredictedObject(
  const geometry_msgs::msg::PoseArray & points, const Pose & base_pose,
  geometry_msgs::msg::Point * nearest_collision_point);

void getLateralNearestPointForPredictedObject(
  const PoseArray & object, const Pose & base_pose, pcl::PointXYZ * lateral_nearest_point,
  double * deviation);

Pose getVehicleCenterFromBase(const Pose & base_pose, const VehicleInfo & vehicle_info);

Polygon2d convertPolygonObjectToGeometryPolygon(
  const Pose & current_pose, const autoware_auto_perception_msgs::msg::Shape & obj_shape);

Polygon2d convertCylindricalObjectToGeometryPolygon(
  const Pose & current_pose, const autoware_auto_perception_msgs::msg::Shape & obj_shape);

Polygon2d convertBoundingBoxObjectToGeometryPolygon(
  const Pose & current_pose, const double & base_to_front, const double & base_to_rear,
  const double & base_to_width);

std::string jsonDumpsPose(const Pose & pose);

DiagnosticStatus makeStopReasonDiag(const std::string stop_reason, const Pose & stop_pose);

TrajectoryPoint getBackwardPointFromBasePoint(
  const TrajectoryPoint & p_from, const TrajectoryPoint & p_to, const TrajectoryPoint & p_base,
  const double backward_length);

TrajectoryPoints decimateTrajectory(
  const TrajectoryPoints & input, const double step_length, std::map<size_t, size_t> & index_map);

TrajectoryPoints extendTrajectory(const TrajectoryPoints & input, const double extend_distance);

TrajectoryPoint getExtendTrajectoryPoint(
  double extend_distance, const TrajectoryPoint & goal_point);

bool intersectsInZAxis(const PredictedObject & object, const double z_min, const double z_max);

pcl::PointXYZ pointToPcl(const double x, const double y, const double z);

std::optional<PredictedObject> getObstacleFromUuid(
  const PredictedObjects & obstacles, const unique_identifier_msgs::msg::UUID & target_object_id);

bool isFrontObstacle(const Pose & ego_pose, const geometry_msgs::msg::Point & obstacle_pos);

double calcObstacleMaxLength(const autoware_auto_perception_msgs::msg::Shape & shape);

rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node * node_ptr);

}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__PLANNER_UTILS_HPP_
