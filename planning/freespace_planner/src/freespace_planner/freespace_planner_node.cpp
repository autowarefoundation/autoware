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

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "freespace_planner/freespace_planner_node.hpp"

#include "freespace_planning_algorithms/abstract_algorithm.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;
using freespace_planning_algorithms::AstarSearch;
using freespace_planning_algorithms::PlannerWaypoint;
using freespace_planning_algorithms::PlannerWaypoints;
using freespace_planning_algorithms::RRTStar;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using tier4_planning_msgs::msg::Scenario;

bool isActive(const Scenario::ConstSharedPtr & scenario)
{
  if (!scenario) {
    return false;
  }

  const auto & s = scenario->activating_scenarios;
  if (std::find(std::begin(s), std::end(s), Scenario::PARKING) != std::end(s)) {
    return true;
  }

  return false;
}

PoseArray trajectory2PoseArray(const Trajectory & trajectory)
{
  PoseArray pose_array;
  pose_array.header = trajectory.header;

  for (const auto & point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  return pose_array;
}

std::vector<size_t> getReversingIndices(const Trajectory & trajectory)
{
  std::vector<size_t> indices;

  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    if (
      trajectory.points.at(i).longitudinal_velocity_mps *
        trajectory.points.at(i + 1).longitudinal_velocity_mps <
      0) {
      indices.push_back(i);
    }
  }

  return indices;
}

size_t getNextTargetIndex(
  const size_t trajectory_size, const std::vector<size_t> & reversing_indices,
  const size_t current_target_index)
{
  if (!reversing_indices.empty()) {
    for (const auto reversing_index : reversing_indices) {
      if (reversing_index > current_target_index) {
        return reversing_index;
      }
    }
  }

  return trajectory_size - 1;
}

Trajectory getPartialTrajectory(
  const Trajectory & trajectory, const size_t start_index, const size_t end_index)
{
  Trajectory partial_trajectory;
  partial_trajectory.header = trajectory.header;
  partial_trajectory.header.stamp = rclcpp::Clock().now();

  partial_trajectory.points.reserve(trajectory.points.size());
  for (size_t i = start_index; i <= end_index; ++i) {
    partial_trajectory.points.push_back(trajectory.points.at(i));
  }

  // Modify velocity at start/end point
  if (partial_trajectory.points.size() >= 2) {
    partial_trajectory.points.front().longitudinal_velocity_mps =
      partial_trajectory.points.at(1).longitudinal_velocity_mps;
  }
  if (!partial_trajectory.points.empty()) {
    partial_trajectory.points.back().longitudinal_velocity_mps = 0;
  }

  return partial_trajectory;
}

double calcDistance2d(const Trajectory & trajectory, const Pose & pose)
{
  const auto idx = motion_utils::findNearestIndex(trajectory.points, pose.position);
  return tier4_autoware_utils::calcDistance2d(trajectory.points.at(idx), pose);
}

Pose transformPose(const Pose & pose, const TransformStamped & transform)
{
  PoseStamped transformed_pose;
  PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

Trajectory createTrajectory(
  const PoseStamped & current_pose, const PlannerWaypoints & planner_waypoints,
  const double & velocity)
{
  Trajectory trajectory;
  trajectory.header = planner_waypoints.header;

  for (const auto & awp : planner_waypoints.waypoints) {
    TrajectoryPoint point;

    point.pose = awp.pose.pose;

    point.pose.position.z = current_pose.pose.position.z;  // height = const
    point.longitudinal_velocity_mps = velocity / 3.6;      // velocity = const

    // switch sign by forward/backward
    point.longitudinal_velocity_mps = (awp.is_back ? -1 : 1) * point.longitudinal_velocity_mps;

    trajectory.points.push_back(point);
  }

  return trajectory;
}

Trajectory createStopTrajectory(const PoseStamped & current_pose)
{
  PlannerWaypoints waypoints;
  PlannerWaypoint waypoint;

  waypoints.header.stamp = rclcpp::Clock().now();
  waypoints.header.frame_id = current_pose.header.frame_id;
  waypoint.pose.header = waypoints.header;
  waypoint.pose.pose = current_pose.pose;
  waypoint.is_back = false;
  waypoints.waypoints.push_back(waypoint);

  return createTrajectory(current_pose, waypoints, 0.0);
}

Trajectory createStopTrajectory(const Trajectory & trajectory)
{
  Trajectory stop_trajectory = trajectory;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    stop_trajectory.points.at(i).longitudinal_velocity_mps = 0.0;
  }
  return stop_trajectory;
}

bool isStopped(
  const std::deque<Odometry::ConstSharedPtr> & odom_buffer, const double th_stopped_velocity_mps)
{
  for (const auto & odom : odom_buffer) {
    if (std::abs(odom->twist.twist.linear.x) > th_stopped_velocity_mps) {
      return false;
    }
  }
  return true;
}

}  // namespace

namespace freespace_planner
{
FreespacePlannerNode::FreespacePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("freespace_planner", node_options)
{
  using std::placeholders::_1;

  // NodeParam
  {
    auto & p = node_param_;
    p.planning_algorithm = declare_parameter<std::string>("planning_algorithm");
    p.waypoints_velocity = declare_parameter<double>("waypoints_velocity");
    p.update_rate = declare_parameter<double>("update_rate");
    p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m");
    p.th_stopped_time_sec = declare_parameter<double>("th_stopped_time_sec");
    p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps");
    p.th_course_out_distance_m = declare_parameter<double>("th_course_out_distance_m");
    p.vehicle_shape_margin_m = declare_parameter<double>("vehicle_shape_margin_m");
    p.replan_when_obstacle_found = declare_parameter<bool>("replan_when_obstacle_found");
    p.replan_when_course_out = declare_parameter<bool>("replan_when_course_out");
  }

  // set vehicle_info
  {
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
    vehicle_shape_.length = vehicle_info.vehicle_length_m;
    vehicle_shape_.width = vehicle_info.vehicle_width_m;
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
  }

  // Planning
  initializePlanningAlgorithm();

  // Subscribers
  {
    route_sub_ = create_subscription<LaneletRoute>(
      "~/input/route", rclcpp::QoS{1}.transient_local(),
      std::bind(&FreespacePlannerNode::onRoute, this, _1));
    occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
      "~/input/occupancy_grid", 1, std::bind(&FreespacePlannerNode::onOccupancyGrid, this, _1));
    scenario_sub_ = create_subscription<Scenario>(
      "~/input/scenario", 1, std::bind(&FreespacePlannerNode::onScenario, this, _1));
    odom_sub_ = create_subscription<Odometry>(
      "~/input/odometry", 100, std::bind(&FreespacePlannerNode::onOdometry, this, _1));
  }

  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);
    debug_pose_array_pub_ = create_publisher<PoseArray>("~/debug/pose_array", qos);
    debug_partial_pose_array_pub_ = create_publisher<PoseArray>("~/debug/partial_pose_array", qos);
    parking_state_pub_ = create_publisher<std_msgs::msg::Bool>("is_completed", qos);
  }

  // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Timer
  {
    const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&FreespacePlannerNode::onTimer, this));
  }

  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
}

PlannerCommonParam FreespacePlannerNode::getPlannerCommonParam()
{
  PlannerCommonParam p;

  // search configs
  p.time_limit = declare_parameter<double>("time_limit");
  p.minimum_turning_radius = declare_parameter<double>("minimum_turning_radius");
  p.maximum_turning_radius = declare_parameter<double>("maximum_turning_radius");
  p.turning_radius_size = declare_parameter<int>("turning_radius_size");
  p.maximum_turning_radius = std::max(p.maximum_turning_radius, p.minimum_turning_radius);
  p.turning_radius_size = std::max(p.turning_radius_size, 1);

  p.theta_size = declare_parameter<int>("theta_size");
  p.angle_goal_range = declare_parameter<double>("angle_goal_range");
  p.curve_weight = declare_parameter<double>("curve_weight");
  p.reverse_weight = declare_parameter<double>("reverse_weight");
  p.lateral_goal_range = declare_parameter<double>("lateral_goal_range");
  p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range");

  // costmap configs
  p.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

  return p;
}

void FreespacePlannerNode::onRoute(const LaneletRoute::ConstSharedPtr msg)
{
  route_ = msg;

  goal_pose_.header = msg->header;
  goal_pose_.pose = msg->goal_pose;

  reset();
}

void FreespacePlannerNode::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
{
  occupancy_grid_ = msg;
}

void FreespacePlannerNode::onScenario(const Scenario::ConstSharedPtr msg)
{
  scenario_ = msg;
}

void FreespacePlannerNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;

  odom_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_buffer_.front()->header.stamp);

    if (time_diff.seconds() < node_param_.th_stopped_time_sec) {
      break;
    }

    odom_buffer_.pop_front();
  }
}

bool FreespacePlannerNode::isPlanRequired()
{
  if (trajectory_.points.empty()) {
    return true;
  }

  if (node_param_.replan_when_obstacle_found) {
    algo_->setMap(*occupancy_grid_);

    const size_t nearest_index_partial =
      motion_utils::findNearestIndex(partial_trajectory_.points, current_pose_.pose.position);
    const size_t end_index_partial = partial_trajectory_.points.size() - 1;

    const auto forward_trajectory =
      getPartialTrajectory(partial_trajectory_, nearest_index_partial, end_index_partial);

    const bool is_obstacle_found =
      algo_->hasObstacleOnTrajectory(trajectory2PoseArray(forward_trajectory));
    if (is_obstacle_found) {
      RCLCPP_INFO(get_logger(), "Found obstacle");
      return true;
    }
  }

  if (node_param_.replan_when_course_out) {
    const bool is_course_out =
      calcDistance2d(trajectory_, current_pose_.pose) > node_param_.th_course_out_distance_m;
    if (is_course_out) {
      RCLCPP_INFO(get_logger(), "Course out");
      return true;
    }
  }

  return false;
}

void FreespacePlannerNode::updateTargetIndex()
{
  const auto is_near_target =
    tier4_autoware_utils::calcDistance2d(trajectory_.points.at(target_index_), current_pose_) <
    node_param_.th_arrived_distance_m;

  const auto is_stopped = isStopped(odom_buffer_, node_param_.th_stopped_velocity_mps);

  if (is_near_target && is_stopped) {
    const auto new_target_index =
      getNextTargetIndex(trajectory_.points.size(), reversing_indices_, target_index_);

    if (new_target_index == target_index_) {
      // Finished publishing all partial trajectories
      is_completed_ = true;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Freespace planning completed");
      std_msgs::msg::Bool is_completed_msg;
      is_completed_msg.data = is_completed_;
      parking_state_pub_->publish(is_completed_msg);
    } else {
      // Switch to next partial trajectory
      prev_target_index_ = target_index_;
      target_index_ =
        getNextTargetIndex(trajectory_.points.size(), reversing_indices_, target_index_);
    }
  }
}

void FreespacePlannerNode::onTimer()
{
  // Check all inputs are ready
  if (!occupancy_grid_ || !route_ || !scenario_ || !odom_) {
    return;
  }

  if (!isActive(scenario_)) {
    reset();
    return;
  }

  if (is_completed_) {
    return;
  }

  // Get current pose
  current_pose_.pose = odom_->pose.pose;
  current_pose_.header = odom_->header;

  if (current_pose_.header.frame_id == "") {
    return;
  }

  if (isPlanRequired()) {
    // Stop before planning new trajectory
    const auto stop_trajectory = partial_trajectory_.points.empty()
                                   ? createStopTrajectory(current_pose_)
                                   : createStopTrajectory(partial_trajectory_);
    trajectory_pub_->publish(stop_trajectory);
    debug_pose_array_pub_->publish(trajectory2PoseArray(stop_trajectory));
    debug_partial_pose_array_pub_->publish(trajectory2PoseArray(stop_trajectory));

    reset();

    // Plan new trajectory
    planTrajectory();
  }

  // StopTrajectory
  if (trajectory_.points.size() <= 1) {
    return;
  }

  // Update partial trajectory
  updateTargetIndex();
  partial_trajectory_ = getPartialTrajectory(trajectory_, prev_target_index_, target_index_);

  // Publish messages
  trajectory_pub_->publish(partial_trajectory_);
  debug_pose_array_pub_->publish(trajectory2PoseArray(trajectory_));
  debug_partial_pose_array_pub_->publish(trajectory2PoseArray(partial_trajectory_));
}

void FreespacePlannerNode::planTrajectory()
{
  if (occupancy_grid_ == nullptr) {
    return;
  }

  // Provide robot shape and map for the planner
  algo_->setMap(*occupancy_grid_);

  // Calculate poses in costmap frame
  const auto current_pose_in_costmap_frame = transformPose(
    current_pose_.pose,
    getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

  const auto goal_pose_in_costmap_frame = transformPose(
    goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

  // execute planning
  const rclcpp::Time start = get_clock()->now();
  const bool result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  const rclcpp::Time end = get_clock()->now();

  RCLCPP_INFO(get_logger(), "Freespace planning: %f [s]", (end - start).seconds());

  if (result) {
    RCLCPP_INFO(get_logger(), "Found goal!");
    trajectory_ =
      createTrajectory(current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);
    reversing_indices_ = getReversingIndices(trajectory_);
    prev_target_index_ = 0;
    target_index_ =
      getNextTargetIndex(trajectory_.points.size(), reversing_indices_, prev_target_index_);

  } else {
    RCLCPP_INFO(get_logger(), "Can't find goal...");
    reset();
  }
}

void FreespacePlannerNode::reset()
{
  trajectory_ = Trajectory();
  partial_trajectory_ = Trajectory();
  is_completed_ = false;
  std_msgs::msg::Bool is_completed_msg;
  is_completed_msg.data = is_completed_;
  parking_state_pub_->publish(is_completed_msg);
}

TransformStamped FreespacePlannerNode::getTransform(
  const std::string & from, const std::string & to)
{
  TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

void FreespacePlannerNode::initializePlanningAlgorithm()
{
  // Extend robot shape
  freespace_planning_algorithms::VehicleShape extended_vehicle_shape = vehicle_shape_;
  const double margin = node_param_.vehicle_shape_margin_m;
  extended_vehicle_shape.length += margin;
  extended_vehicle_shape.width += margin;
  extended_vehicle_shape.base2back += margin / 2;

  const auto planner_common_param = getPlannerCommonParam();

  const auto algo_name = node_param_.planning_algorithm;

  // initialize specified algorithm
  if (algo_name == "astar") {
    algo_ = std::make_unique<AstarSearch>(planner_common_param, extended_vehicle_shape, *this);
  } else if (algo_name == "rrtstar") {
    algo_ = std::make_unique<RRTStar>(planner_common_param, extended_vehicle_shape, *this);
  } else {
    throw std::runtime_error("No such algorithm named " + algo_name + " exists.");
  }
  RCLCPP_INFO_STREAM(get_logger(), "initialize planning algorithm: " << algo_name);
}
}  // namespace freespace_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(freespace_planner::FreespacePlannerNode)
