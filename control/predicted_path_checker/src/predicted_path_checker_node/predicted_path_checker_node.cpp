// Copyright 2023 LeoDrive A.Ş. All rights reserved.
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

#include "predicted_path_checker/predicted_path_checker_node.hpp"

#include <motion_utils/marker/marker_helper.hpp>
#include <motion_utils/resample/resample.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::predicted_path_checker
{

PredictedPathCheckerNode::PredictedPathCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("predicted_path_checker_node", node_options), updater_(this)
{
  using std::placeholders::_1;

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_cli(cli_set_stop_, group_cli_);
  adaptor.init_sub(sub_stop_state_, this, &PredictedPathCheckerNode::onIsStopped);
  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  // Node Parameter
  node_param_.update_rate = declare_parameter("update_rate", 10.0);
  node_param_.ego_nearest_dist_threshold = declare_parameter("ego_nearest_dist_threshold", 3.0);
  node_param_.ego_nearest_yaw_threshold = declare_parameter("ego_nearest_yaw_threshold", 1.046);
  node_param_.max_deceleration = declare_parameter("max_deceleration", 1.5);
  node_param_.delay_time = declare_parameter("delay_time", 0.17);
  node_param_.stop_margin = declare_parameter("stop_margin", 0.5);
  node_param_.min_trajectory_check_length = declare_parameter("min_trajectory_check_length", 1.5);
  node_param_.trajectory_check_time = declare_parameter("trajectory_check_time", 3.0);
  node_param_.resample_interval = declare_parameter("resample_interval", 0.5);
  node_param_.distinct_point_distance_threshold =
    declare_parameter("distinct_point_distance_threshold", 0.3);
  node_param_.distinct_point_yaw_threshold = declare_parameter("distinct_point_yaw_threshold", 5.0);
  node_param_.filtering_distance_threshold = declare_parameter("filtering_distance_threshold", 1.5);
  node_param_.use_object_prediction = declare_parameter("use_object_prediction", true);

  // Collision Checker Parameter
  collision_checker_param_.width_margin =
    declare_parameter("collision_checker_params.width_margin", 0.2);
  collision_checker_param_.enable_z_axis_obstacle_filtering =
    declare_parameter("collision_checker_params.enable_z_axis_obstacle_filtering", false);
  collision_checker_param_.z_axis_filtering_buffer =
    declare_parameter("collision_checker_params.z_axis_filtering_buffer", 0.3);
  collision_checker_param_.chattering_threshold =
    declare_parameter("collision_checker_params.chattering_threshold", 0.2);

  // Subscriber
  self_pose_listener_ = std::make_shared<tier4_autoware_utils::SelfPoseListener>(this);

  sub_dynamic_objects_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::SensorDataQoS(),
    std::bind(&PredictedPathCheckerNode::onDynamicObjects, this, _1));
  sub_reference_trajectory_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/input/reference_trajectory", 1,
    std::bind(&PredictedPathCheckerNode::onReferenceTrajectory, this, _1));
  sub_predicted_trajectory_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/input/predicted_trajectory", 1,
    std::bind(&PredictedPathCheckerNode::onPredictedTrajectory, this, _1));
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1, std::bind(&PredictedPathCheckerNode::onOdom, this, _1));
  sub_accel_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "~/input/current_accel", rclcpp::QoS{1},
    std::bind(&PredictedPathCheckerNode::onAccel, this, _1));

  debug_ptr_ =
    std::make_shared<PredictedPathCheckerDebugNode>(this, vehicle_info_.max_longitudinal_offset_m);

  // Core

  collision_checker_ = std::make_unique<CollisionChecker>(this, debug_ptr_);
  collision_checker_->setParam(collision_checker_param_);

  // Diagnostic Updater
  updater_.setHardwareID("predicted_path_checker");
  updater_.add("predicted_path_checker", this, &PredictedPathCheckerNode::checkVehicleState);

  // Wait for first self pose
  self_pose_listener_->waitForFirstPose();

  // Timer
  initTimer(1.0 / node_param_.update_rate);
}

void PredictedPathCheckerNode::onDynamicObjects(const PredictedObjects::ConstSharedPtr msg)
{
  object_ptr_ = msg;
}

void PredictedPathCheckerNode::onReferenceTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  reference_trajectory_ = msg;
}

void PredictedPathCheckerNode::onPredictedTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  predicted_trajectory_ = msg;
}

void PredictedPathCheckerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_twist_ = std::make_shared<geometry_msgs::msg::Twist>(msg->twist.twist);
}

void PredictedPathCheckerNode::onAccel(
  const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  current_accel_ = msg;
}

void PredictedPathCheckerNode::onIsStopped(
  const control_interface::IsStopped::Message::ConstSharedPtr msg)
{
  is_stopped_ptr_ = msg;

  is_stopped_by_node_ =
    is_stopped_ptr_->data &&
    std::find(
      is_stopped_ptr_->requested_sources.begin(), is_stopped_ptr_->requested_sources.end(),
      "predicted_path_checker") != is_stopped_ptr_->requested_sources.end();
}

void PredictedPathCheckerNode::initTimer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&PredictedPathCheckerNode::onTimer, this));
}

bool PredictedPathCheckerNode::isDataReady()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current_pose...");
    return false;
  }

  if (!object_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for dynamic objects msg...");
    return false;
  }

  if (!reference_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "waiting for reference_trajectory msg...");
    return false;
  }

  if (!predicted_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "waiting for predicted_trajectory msg...");
    return false;
  }

  if (!current_twist_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current_twist msg...");
    return false;
  }

  if (!current_accel_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current_accel msg...");
    return false;
  }

  if (!is_stopped_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for is_stopped msg...");
    return false;
  }

  if (!cli_set_stop_->service_is_ready()) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for stop service...");
    return false;
  }

  return true;
}

bool PredictedPathCheckerNode::isDataTimeout()
{
  const auto now = this->now();

  constexpr double th_pose_timeout = 1.0;
  const auto pose_time_diff = rclcpp::Time(current_pose_->header.stamp).seconds() - now.seconds();
  if (pose_time_diff > th_pose_timeout) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "pose is timeout...");
    return true;
  }

  return false;
}

void PredictedPathCheckerNode::onTimer()
{
  current_pose_ = self_pose_listener_->getCurrentPose();

  if (!isDataReady()) {
    return;
  }

  if (isDataTimeout()) {
    return;
  }

  // Cut trajectory
  const auto cut_trajectory = cutTrajectory(
    *predicted_trajectory_, std::max(
                              node_param_.min_trajectory_check_length,
                              current_twist_->linear.x * node_param_.trajectory_check_time));

  // Convert to trajectory array

  TrajectoryPoints predicted_trajectory_array = motion_utils::convertToTrajectoryPointArray(
    motion_utils::resampleTrajectory(cut_trajectory, node_param_.resample_interval));

  // Filter the objects

  PredictedObjects filtered_objects;
  filterObstacles(
    current_pose_.get()->pose, predicted_trajectory_array, node_param_.filtering_distance_threshold,
    node_param_.use_object_prediction, filtered_objects);

  PredictedObjects::ConstSharedPtr filtered_obj_ptr =
    std::make_shared<PredictedObjects>(filtered_objects);

  // Check collision

  const auto collision_checker_output =
    collision_checker_->checkTrajectoryForCollision(predicted_trajectory_array, filtered_obj_ptr);

  if (!collision_checker_output) {
    // There is no need to stop
    if (is_stopped_by_node_) {
      sendRequest(false);
    }
    current_state_ = State::DRIVE;
    updater_.force_update();
    debug_ptr_->publish();

    return;
  }

  // Extend trajectory

  extendTrajectoryPointsArray(predicted_trajectory_array);

  // Insert collision and stop points

  const auto stop_idx =
    insertStopPoint(predicted_trajectory_array, collision_checker_output.get().first);

  // Check ego vehicle is stopped or not
  constexpr double th_stopped_velocity = 0.001;
  const bool is_ego_vehicle_stopped = current_twist_->linear.x < th_stopped_velocity;

  // If ego vehicle is not stopped, check obstacle is in the brake distance
  if (!is_ego_vehicle_stopped) {
    // Calculate projected velocity and acceleration of the object on the trajectory point

    const auto projected_obj_vel_acc = calculateProjectedVelAndAcc(
      collision_checker_output->second, predicted_trajectory_array.at(stop_idx));

    // Calculate relative velocity and acceleration wrt the object

    const double relative_velocity = current_twist_->linear.x - projected_obj_vel_acc.first;
    const double relative_acceleration =
      current_accel_->accel.accel.linear.x - projected_obj_vel_acc.second;

    // Check if the vehicle is in the brake distance

    const bool is_in_brake_distance = utils::isInBrakeDistance(
      predicted_trajectory_array, stop_idx, relative_velocity, relative_acceleration,
      node_param_.max_deceleration, node_param_.delay_time);

    if (is_in_brake_distance) {
      // Send emergency and stop request
      current_state_ = State::EMERGENCY;
      updater_.force_update();
      if (!is_stopped_by_node_) {
        sendRequest(true);
      }
      debug_ptr_->publish();
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000 /* ms */,
        "There is an obstacle in the brake distance. Sending emergency and stop request...");
      return;
    }
  }

  // If it is not in the brake distance, check if the collision point is discrete from the reference
  // trajectory or not

  const auto reference_trajectory_array =
    motion_utils::convertToTrajectoryPointArray(*reference_trajectory_);

  const auto is_discrete_point =
    isItDiscretePoint(reference_trajectory_array, predicted_trajectory_array.at(stop_idx));

  if (is_discrete_point) {
    // Check reference trajectory has stop point or not

    const auto is_there_stop_in_ref_trajectory = isThereStopPointOnReferenceTrajectory(
      predicted_trajectory_array.at(stop_idx).pose, reference_trajectory_array);

    if (!is_there_stop_in_ref_trajectory) {
      // Send stop
      if (!is_stopped_by_node_) {
        sendRequest(true);
      }
      current_state_ = State::STOP;
      updater_.force_update();

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000 /* ms */,
        "There is an obstacle on predicted path. Sending stop request...");

      debug_ptr_->publish();
      return;
    }
  }

  // If it is not discrete point, planning should handle it. Send drive.
  current_state_ = State::DRIVE;
  updater_.force_update();

  if (is_stopped_by_node_) {
    sendRequest(false);
  }

  debug_ptr_->publish();
}

TrajectoryPoints PredictedPathCheckerNode::trimTrajectoryFromSelfPose(
  const TrajectoryPoints & input, const Pose & self_pose) const
{
  TrajectoryPoints output{};

  const size_t min_distance_index = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
                                      input, self_pose, node_param_.ego_nearest_dist_threshold,
                                      node_param_.ego_nearest_yaw_threshold) +
                                    1;

  for (size_t i = min_distance_index; i < input.size(); ++i) {
    output.push_back(input.at(i));
  }

  return output;
}

bool PredictedPathCheckerNode::isThereStopPointOnReferenceTrajectory(
  const geometry_msgs::msg::Pose & pose, const TrajectoryPoints & reference_trajectory_array)
{
  const auto trimmed_reference_trajectory_array =
    trimTrajectoryFromSelfPose(reference_trajectory_array, current_pose_.get()->pose);

  const auto nearest_stop_point_on_ref_trajectory =
    motion_utils::findNearestIndex(trimmed_reference_trajectory_array, pose);

  const auto stop_point_on_trajectory = motion_utils::searchZeroVelocityIndex(
    trimmed_reference_trajectory_array, 0, *nearest_stop_point_on_ref_trajectory);

  return !!stop_point_on_trajectory;
}

void PredictedPathCheckerNode::checkVehicleState(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (current_state_ == State::EMERGENCY) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "vehicle will collide with obstacles";
  }
  if (current_state_ == State::STOP) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "vehicle will stop due to obstacle";
  }

  stat.summary(level, msg);
}

void PredictedPathCheckerNode::sendRequest(bool make_stop_vehicle)
{
  if (!is_calling_set_stop_ && cli_set_stop_->service_is_ready()) {
    const auto req = std::make_shared<control_interface::SetStop::Service::Request>();
    req->stop = make_stop_vehicle;
    req->request_source = "predicted_path_checker";
    is_calling_set_stop_ = true;
    cli_set_stop_->async_send_request(req, [this](auto) { is_calling_set_stop_ = false; });
  }
}

bool PredictedPathCheckerNode::isItDiscretePoint(
  const TrajectoryPoints & reference_trajectory, const TrajectoryPoint & collision_point) const
{
  const auto nearest_segment =
    motion_utils::findNearestSegmentIndex(reference_trajectory, collision_point.pose);

  const auto nearest_point = utils::calcInterpolatedPoint(
    reference_trajectory, collision_point.pose.position, *nearest_segment, false);

  const auto distance = tier4_autoware_utils::calcDistance2d(
    nearest_point.pose.position, collision_point.pose.position);

  const auto yaw_diff =
    std::abs(tier4_autoware_utils::calcYawDeviation(nearest_point.pose, collision_point.pose));
  return distance >= node_param_.distinct_point_distance_threshold ||
         yaw_diff >= tier4_autoware_utils::deg2rad(node_param_.distinct_point_yaw_threshold);
}

Trajectory PredictedPathCheckerNode::cutTrajectory(
  const Trajectory & trajectory, const double length)
{
  Trajectory cut;
  cut.header = trajectory.header;
  if (trajectory.points.empty()) {
    return cut;
  }
  double total_length = 0.0;
  cut.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points.at(i);

    const auto p1 = tier4_autoware_utils::fromMsg(cut.points.back().pose.position);
    const auto p2 = tier4_autoware_utils::fromMsg(point.pose.position);
    const auto points_distance = boost::geometry::distance(p1.to_2d(), p2.to_2d());

    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0.001) {
      break;
    }

    // Require interpolation
    if (remain_distance <= points_distance) {
      const Eigen::Vector3d p_interpolated = p1 + remain_distance * (p2 - p1).normalized();

      TrajectoryPoint p;
      p.pose.position.x = p_interpolated.x();
      p.pose.position.y = p_interpolated.y();
      p.pose.position.z = p_interpolated.z();
      p.pose.orientation = point.pose.orientation;

      cut.points.push_back(p);
      break;
    }

    cut.points.push_back(point);
    total_length += points_distance;
  }
  motion_utils::removeOverlapPoints(cut.points);

  return cut;
}

void PredictedPathCheckerNode::extendTrajectoryPointsArray(TrajectoryPoints & trajectory)
{
  // It extends the trajectory to the end of the footprint of the vehicle to get better distance to
  // collision_point.
  const double extend_distance = vehicle_info_.max_longitudinal_offset_m + node_param_.stop_margin;
  const auto & goal_point = trajectory.back();
  const auto trajectory_point_extend = utils::getExtendTrajectoryPoint(extend_distance, goal_point);
  trajectory.push_back(trajectory_point_extend);
}

size_t PredictedPathCheckerNode::insertStopPoint(
  TrajectoryPoints & trajectory, const geometry_msgs::msg::Point collision_point)
{
  const auto nearest_collision_segment =
    motion_utils::findNearestSegmentIndex(trajectory, collision_point);

  const auto nearest_collision_point =
    utils::calcInterpolatedPoint(trajectory, collision_point, nearest_collision_segment, true);

  const size_t collision_idx = nearest_collision_segment + 1;

  trajectory.insert(trajectory.begin() + static_cast<int>(collision_idx), nearest_collision_point);

  const auto stop_point =
    utils::findStopPoint(trajectory, collision_idx, node_param_.stop_margin, vehicle_info_);

  const size_t stop_idx = stop_point.first + 1;
  trajectory.insert(trajectory.begin() + static_cast<int>(stop_idx), stop_point.second);

  debug_ptr_->pushPose(stop_point.second.pose, PoseType::Stop);
  return stop_idx;
}

std::pair<double, double> PredictedPathCheckerNode::calculateProjectedVelAndAcc(
  const PredictedObject & object, const TrajectoryPoint & trajectory_point)
{
  const auto & orientation_obj = object.kinematics.initial_pose_with_covariance.pose.orientation;
  const auto & orientation_stop_point = trajectory_point.pose.orientation;
  const auto velocity_obj = object.kinematics.initial_twist_with_covariance.twist.linear.x;
  const auto acceleration_obj =
    object.kinematics.initial_acceleration_with_covariance.accel.linear.x;
  const auto k = std::cos(tier4_autoware_utils::normalizeRadian(
    tf2::getYaw(orientation_obj) - tf2::getYaw(orientation_stop_point)));
  const auto projected_velocity = velocity_obj * k;
  const auto projected_acceleration = acceleration_obj * k;
  return std::make_pair(projected_velocity, projected_acceleration);
}

void PredictedPathCheckerNode::filterObstacles(
  const Pose & ego_pose, const TrajectoryPoints & traj, const double dist_threshold,
  const bool use_prediction, PredictedObjects & filtered_objects)
{
  filtered_objects.header.frame_id = object_ptr_.get()->header.frame_id;
  filtered_objects.header.stamp = this->now();

  for (auto & object : object_ptr_.get()->objects) {
    // Check is it in front of ego vehicle
    if (!utils::isFrontObstacle(
          ego_pose, object.kinematics.initial_pose_with_covariance.pose.position)) {
      continue;
    }

    // Check is it near to trajectory
    const double max_length = utils::calcObstacleMaxLength(object.shape);
    const size_t seg_idx = motion_utils::findNearestSegmentIndex(
      traj, object.kinematics.initial_pose_with_covariance.pose.position);
    const auto p_front = tier4_autoware_utils::getPoint(traj.at(seg_idx));
    const auto p_back = tier4_autoware_utils::getPoint(traj.at(seg_idx + 1));
    const auto & p_target = object.kinematics.initial_pose_with_covariance.pose.position;
    const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0.0};
    const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0.0};

    if (seg_idx == traj.size() - 2) {
      // Calculate longitudinal offset
      const auto longitudinal_dist = std::abs(segment_vec.dot(target_vec) / segment_vec.norm());
      if (
        longitudinal_dist - max_length - vehicle_info_.max_longitudinal_offset_m - dist_threshold >
        0.0) {
        continue;
      }
    }
    const auto lateral_dist = std::abs(segment_vec.cross(target_vec)(2) / segment_vec.norm());
    if (lateral_dist - max_length - vehicle_info_.max_lateral_offset_m - dist_threshold > 0.0) {
      continue;
    }
    PredictedObject filtered_object = object;
    if (use_prediction) {
      utils::getCurrentObjectPose(filtered_object, object_ptr_.get()->header.stamp, this->now());
    }
    filtered_objects.objects.push_back(filtered_object);
  }
}

}  // namespace autoware::motion::control::predicted_path_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::motion::control::predicted_path_checker::PredictedPathCheckerNode)
