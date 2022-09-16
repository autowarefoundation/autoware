
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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/optional.hpp>

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
using geometry_msgs::msg::TransformStamped;
using std_msgs::msg::Header;

using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;
using vehicle_info_util::VehicleInfo;

using TrajectoryPoints = std::vector<TrajectoryPoint>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

bool validCheckDecelPlan(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin);

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE1)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (am) minimum deceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @param (jd) minimum jerk [m/sss]
 * @param (t_min) duration of constant deceleration [s]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType1(
  const double v0, const double vt, const double a0, const double am, const double ja,
  const double jd, const double t_min);

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE2)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (am) minimum deceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @param (jd) minimum jerk [m/sss]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType2(
  const double v0, const double vt, const double a0, const double ja, const double jd);

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE3)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType3(
  const double v0, const double vt, const double a0, const double ja);

boost::optional<double> calcDecelDistWithJerkAndAccConstraints(
  const double current_vel, const double target_vel, const double current_acc, const double acc_min,
  const double jerk_acc, const double jerk_dec);

boost::optional<std::pair<double, double>> calcFeasibleMarginAndVelocity(
  const SlowDownParam & slow_down_param, const double dist_baselink_to_obstacle,
  const double current_vel, const double current_acc);

boost::optional<std::pair<size_t, TrajectoryPoint>> getForwardInsertPointFromBasePoint(
  const size_t base_idx, const TrajectoryPoints & trajectory, const double margin);

boost::optional<std::pair<size_t, TrajectoryPoint>> getBackwardInsertPointFromBasePoint(
  const size_t base_idx, const TrajectoryPoints & trajectory, const double margin);

boost::optional<std::pair<size_t, double>> findNearestFrontIndex(
  const size_t start_idx, const TrajectoryPoints & trajectory, const Point & point);

void insertStopPoint(
  const StopPoint & stop_point, TrajectoryPoints & output, DiagnosticStatus & stop_reason_diag);

bool isInFrontOfTargetPoint(const Pose & pose, const Point & point);

bool checkValidIndex(const Pose & p_base, const Pose & p_next, const Pose & p_target);

bool withinPolygon(
  const std::vector<cv::Point2d> & cv_polygon, const double radius, const Point2d & prev_point,
  const Point2d & next_point, PointCloud::Ptr candidate_points_ptr,
  PointCloud::Ptr within_points_ptr);

bool convexHull(
  const std::vector<cv::Point2d> & pointcloud, std::vector<cv::Point2d> & polygon_points);

void createOneStepPolygon(
  const Pose & base_step_pose, const Pose & next_step_pose, std::vector<cv::Point2d> & polygon,
  const VehicleInfo & vehicle_info, const double expand_width = 0.0);

bool getSelfPose(const Header & header, const tf2_ros::Buffer & tf_buffer, Pose & self_pose);

void getNearestPoint(
  const PointCloud & pointcloud, const Pose & base_pose, pcl::PointXYZ * nearest_collision_point,
  rclcpp::Time * nearest_collision_point_time);

void getLateralNearestPoint(
  const PointCloud & pointcloud, const Pose & base_pose, pcl::PointXYZ * lateral_nearest_point,
  double * deviation);

Pose getVehicleCenterFromBase(const Pose & base_pose, const VehicleInfo & vehicle_info);

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

rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node * node_ptr);

}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__PLANNER_UTILS_HPP_
