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

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "obstacle_stop_planner/node.hpp"
#include "obstacle_stop_planner/planner_utils.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace motion_planning
{
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcLongitudinalOffsetToSegment;
using motion_utils::calcSignedArcLength;
using motion_utils::findFirstNearestIndexWithSoftConstraints;
using motion_utils::findFirstNearestSegmentIndexWithSoftConstraints;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::getPoint;
using tier4_autoware_utils::getPose;
using tier4_autoware_utils::getRPY;

ObstacleStopPlannerNode::ObstacleStopPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_stop_planner", node_options)
{
  // Vehicle Parameters
  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  const auto & i = vehicle_info_;

  // Parameters
  {
    auto & p = node_param_;
    p.enable_slow_down = declare_parameter<bool>("enable_slow_down");
    p.max_velocity = declare_parameter<double>("max_velocity");
    p.hunting_threshold = declare_parameter<double>("hunting_threshold");
    p.ego_nearest_dist_threshold = declare_parameter<double>("ego_nearest_dist_threshold");
    p.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");
  }

  {
    auto & p = stop_param_;
    const std::string ns = "stop_planner.";

    // params for stop position
    p.max_longitudinal_margin =
      declare_parameter<double>(ns + "stop_position.max_longitudinal_margin");
    p.min_longitudinal_margin =
      declare_parameter<double>(ns + "stop_position.min_longitudinal_margin");
    p.hold_stop_margin_distance =
      declare_parameter<double>(ns + "stop_position.hold_stop_margin_distance");

    // params for detection area
    p.lateral_margin = declare_parameter<double>(ns + "detection_area.lateral_margin");
    p.extend_distance = declare_parameter<double>(ns + "detection_area.extend_distance");
    p.step_length = declare_parameter<double>(ns + "detection_area.step_length");

    // apply offset
    p.max_longitudinal_margin += i.max_longitudinal_offset_m;
    p.min_longitudinal_margin += i.max_longitudinal_offset_m;
    p.stop_search_radius =
      p.step_length +
      std::hypot(i.vehicle_width_m / 2.0 + p.lateral_margin, i.vehicle_length_m / 2.0);
  }

  {
    auto & p = slow_down_param_;
    const std::string ns = "slow_down_planner.";

    // common param
    p.normal_min_jerk = declare_parameter<double>("normal.min_jerk");
    p.normal_min_acc = declare_parameter<double>("normal.min_acc");
    p.limit_min_jerk = declare_parameter<double>("limit.min_jerk");
    p.limit_min_acc = declare_parameter<double>("limit.min_acc");

    // params for slow down section
    p.longitudinal_forward_margin =
      declare_parameter<double>(ns + "slow_down_section.longitudinal_forward_margin");
    p.longitudinal_backward_margin =
      declare_parameter<double>(ns + "slow_down_section.longitudinal_backward_margin");
    p.min_longitudinal_forward_margin =
      declare_parameter<double>(ns + "slow_down_section.min_longitudinal_forward_margin");
    p.longitudinal_margin_span =
      declare_parameter<double>(ns + "slow_down_section.longitudinal_margin_span");

    // params for detection area
    p.lateral_margin = declare_parameter<double>(ns + "detection_area.lateral_margin");

    // params for target velocity
    p.max_slow_down_velocity =
      declare_parameter<double>(ns + "target_velocity.max_slow_down_velocity");
    p.min_slow_down_velocity =
      declare_parameter<double>(ns + "target_velocity.min_slow_down_velocity");
    p.slow_down_velocity = declare_parameter<double>(ns + "target_velocity.slow_down_velocity");

    // consider jerk/dec constraints in slow down
    p.consider_constraints = declare_parameter<bool>(ns + "consider_constraints");
    p.slow_down_min_jerk = declare_parameter<double>(ns + "constraints.jerk_min_slow_down");
    p.jerk_start = declare_parameter<double>(ns + "constraints.jerk_start");
    p.jerk_span = declare_parameter<double>(ns + "constraints.jerk_span");

    p.velocity_threshold_decel_complete =
      declare_parameter<double>(ns + "velocity_threshold_decel_complete");
    p.acceleration_threshold_decel_complete =
      declare_parameter<double>(ns + "acceleration_threshold_decel_complete");

    // apply offset
    p.longitudinal_forward_margin += i.max_longitudinal_offset_m;
    p.min_longitudinal_forward_margin += i.wheel_base_m + i.front_overhang_m;
    p.longitudinal_backward_margin += i.rear_overhang_m;
    p.slow_down_search_radius =
      stop_param_.step_length +
      std::hypot(i.vehicle_width_m / 2.0 + p.lateral_margin, i.vehicle_length_m / 2.0);
  }

  // Initializer
  acc_controller_ = std::make_unique<AdaptiveCruiseController>(
    this, i.vehicle_width_m, i.vehicle_length_m, i.max_longitudinal_offset_m);
  debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(this, i.max_longitudinal_offset_m);
  last_detect_time_slowdown_point_ = this->now();
  last_detect_time_collision_point_ = this->now();

  // Publishers
  pub_trajectory_ = this->create_publisher<Trajectory>("~/output/trajectory", 1);

  pub_stop_reason_ = this->create_publisher<DiagnosticStatus>("~/output/stop_reason", 1);

  pub_clear_velocity_limit_ = this->create_publisher<VelocityLimitClearCommand>(
    "~/output/velocity_limit_clear_command", rclcpp::QoS{1}.transient_local());

  pub_velocity_limit_ = this->create_publisher<VelocityLimit>(
    "~/output/max_velocity", rclcpp::QoS{1}.transient_local());

  // Subscribers
  sub_point_cloud_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ObstacleStopPlannerNode::onPointCloud, this, std::placeholders::_1),
    createSubscriptionOptions(this));

  sub_trajectory_ = this->create_subscription<Trajectory>(
    "~/input/trajectory", 1,
    std::bind(&ObstacleStopPlannerNode::onTrigger, this, std::placeholders::_1),
    createSubscriptionOptions(this));

  sub_odometry_ = this->create_subscription<Odometry>(
    "~/input/odometry", 1,
    std::bind(&ObstacleStopPlannerNode::onOdometry, this, std::placeholders::_1),
    createSubscriptionOptions(this));

  sub_acceleration_ = this->create_subscription<AccelWithCovarianceStamped>(
    "~/input/acceleration", 1,
    std::bind(&ObstacleStopPlannerNode::onAcceleration, this, std::placeholders::_1),
    createSubscriptionOptions(this));

  sub_dynamic_objects_ = this->create_subscription<PredictedObjects>(
    "~/input/objects", 1,
    std::bind(&ObstacleStopPlannerNode::onDynamicObjects, this, std::placeholders::_1),
    createSubscriptionOptions(this));

  sub_expand_stop_range_ = this->create_subscription<ExpandStopRange>(
    "~/input/expand_stop_range", 1,
    std::bind(&ObstacleStopPlannerNode::onExpandStopRange, this, std::placeholders::_1),
    createSubscriptionOptions(this));
}

void ObstacleStopPlannerNode::onPointCloud(const PointCloud2::ConstSharedPtr input_msg)
{
  // mutex for obstacle_ros_pointcloud_ptr_
  // NOTE: *obstacle_ros_pointcloud_ptr_ is used
  std::lock_guard<std::mutex> lock(mutex_);

  obstacle_ros_pointcloud_ptr_ = std::make_shared<PointCloud2>();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  PointCloud::Ptr pointcloud_ptr(new PointCloud);
  PointCloud::Ptr no_height_pointcloud_ptr(new PointCloud);
  PointCloud::Ptr no_height_filtered_pointcloud_ptr(new PointCloud);

  pcl::fromROSMsg(*input_msg, *pointcloud_ptr);

  for (const auto & point : pointcloud_ptr->points) {
    no_height_pointcloud_ptr->push_back(pcl::PointXYZ(point.x, point.y, 0.0));
  }
  filter.setInputCloud(no_height_pointcloud_ptr);
  filter.setLeafSize(0.05f, 0.05f, 100000.0f);
  filter.filter(*no_height_filtered_pointcloud_ptr);
  pcl::toROSMsg(*no_height_filtered_pointcloud_ptr, *obstacle_ros_pointcloud_ptr_);
  obstacle_ros_pointcloud_ptr_->header = input_msg->header;
}

void ObstacleStopPlannerNode::onTrigger(const Trajectory::ConstSharedPtr input_msg)
{
  mutex_.lock();
  // NOTE: these variables must not be referenced for multithreading
  const auto vehicle_info = vehicle_info_;
  const auto stop_param = stop_param_;
  const auto obstacle_ros_pointcloud_ptr = obstacle_ros_pointcloud_ptr_;
  const auto object_ptr = object_ptr_;
  const auto current_velocity_ptr = current_velocity_ptr_;
  const auto current_acceleration_ptr = current_acceleration_ptr_;
  mutex_.unlock();

  {
    const auto waiting = [this](const auto & str) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(), "waiting for %s ...",
        str);
    };

    if (!object_ptr) {
      waiting("perception object");
      return;
    }

    if (!obstacle_ros_pointcloud_ptr) {
      waiting("obstacle pointcloud");
      return;
    }

    if (!current_velocity_ptr) {
      waiting("current velocity");
      return;
    }

    if (!current_acceleration_ptr) {
      waiting("current acceleration");
      return;
    }

    if (input_msg->points.empty()) {
      return;
    }
  }

  const auto current_vel = current_velocity_ptr->twist.twist.linear.x;
  const auto current_acc = current_acceleration_ptr->accel.accel.linear.x;

  // TODO(someone): support backward path
  const auto is_driving_forward = motion_utils::isDrivingForwardWithTwist(input_msg->points);
  is_driving_forward_ = is_driving_forward ? is_driving_forward.get() : is_driving_forward_;
  if (!is_driving_forward_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000, "Backward path is NOT supported. publish input as it is.");
    pub_trajectory_->publish(*input_msg);
    return;
  }

  PlannerData planner_data{};

  getSelfPose(input_msg->header, tf_buffer_, planner_data.current_pose);

  Trajectory output_trajectory = *input_msg;
  TrajectoryPoints output_trajectory_points =
    motion_utils::convertToTrajectoryPointArray(*input_msg);

  // trim trajectory from self pose
  const auto base_trajectory = trimTrajectoryWithIndexFromSelfPose(
    motion_utils::convertToTrajectoryPointArray(*input_msg), planner_data.current_pose,
    planner_data.trajectory_trim_index);
  // extend trajectory to consider obstacles after the goal
  const auto extend_trajectory = extendTrajectory(base_trajectory, stop_param.extend_distance);
  // decimate trajectory for calculation cost
  const auto decimate_trajectory = decimateTrajectory(
    extend_trajectory, stop_param.step_length, planner_data.decimate_trajectory_index_map);

  // search obstacles within slow-down/collision area
  searchObstacle(
    decimate_trajectory, output_trajectory_points, planner_data, input_msg->header, vehicle_info,
    stop_param, obstacle_ros_pointcloud_ptr);
  // insert slow-down-section/stop-point
  insertVelocity(
    output_trajectory_points, planner_data, input_msg->header, vehicle_info, current_acc,
    current_vel, stop_param);

  const auto no_slow_down_section = !planner_data.slow_down_require && !latest_slow_down_section_;
  const auto no_hunting =
    (rclcpp::Time(input_msg->header.stamp) - last_detect_time_slowdown_point_).seconds() >
    node_param_.hunting_threshold;
  if (node_param_.enable_slow_down && no_slow_down_section && set_velocity_limit_ && no_hunting) {
    resetExternalVelocityLimit(current_acc, current_vel);
  }

  auto trajectory = motion_utils::convertToTrajectory(output_trajectory_points);
  publishDebugData(planner_data, current_acc, current_vel);

  trajectory.header = input_msg->header;
  pub_trajectory_->publish(trajectory);
}

void ObstacleStopPlannerNode::searchObstacle(
  const TrajectoryPoints & decimate_trajectory, TrajectoryPoints & output,
  PlannerData & planner_data, const Header & trajectory_header, const VehicleInfo & vehicle_info,
  const StopParam & stop_param, const PointCloud2::SharedPtr obstacle_ros_pointcloud_ptr)
{
  // search candidate obstacle pointcloud
  PointCloud::Ptr slow_down_pointcloud_ptr(new PointCloud);
  PointCloud::Ptr obstacle_candidate_pointcloud_ptr(new PointCloud);
  if (!searchPointcloudNearTrajectory(
        decimate_trajectory, obstacle_ros_pointcloud_ptr, obstacle_candidate_pointcloud_ptr,
        trajectory_header, vehicle_info, stop_param)) {
    return;
  }

  for (size_t i = 0; i < decimate_trajectory.size() - 1; ++i) {
    // create one step circle center for vehicle
    const auto & p_front = decimate_trajectory.at(i).pose;
    const auto & p_back = decimate_trajectory.at(i + 1).pose;
    const auto prev_center_pose = getVehicleCenterFromBase(p_front, vehicle_info);
    const Point2d prev_center_point(prev_center_pose.position.x, prev_center_pose.position.y);
    const auto next_center_pose = getVehicleCenterFromBase(p_back, vehicle_info);
    const Point2d next_center_point(next_center_pose.position.x, next_center_pose.position.y);

    if (node_param_.enable_slow_down) {
      std::vector<cv::Point2d> one_step_move_slow_down_range_polygon;
      // create one step polygon for slow_down range
      createOneStepPolygon(
        p_front, p_back, one_step_move_slow_down_range_polygon, vehicle_info,
        slow_down_param_.lateral_margin);
      debug_ptr_->pushPolygon(
        one_step_move_slow_down_range_polygon, p_front.position.z, PolygonType::SlowDownRange);

      planner_data.found_slow_down_points = withinPolygon(
        one_step_move_slow_down_range_polygon, slow_down_param_.slow_down_search_radius,
        prev_center_point, next_center_point, obstacle_candidate_pointcloud_ptr,
        slow_down_pointcloud_ptr);

      const auto found_first_slow_down_points =
        planner_data.found_slow_down_points && !planner_data.slow_down_require;

      if (found_first_slow_down_points) {
        // found nearest slow down obstacle
        planner_data.decimate_trajectory_slow_down_index = i;
        planner_data.slow_down_require = true;
        getNearestPoint(
          *slow_down_pointcloud_ptr, p_front, &planner_data.nearest_slow_down_point,
          &planner_data.nearest_collision_point_time);
        getLateralNearestPoint(
          *slow_down_pointcloud_ptr, p_front, &planner_data.lateral_nearest_slow_down_point,
          &planner_data.lateral_deviation);

        debug_ptr_->pushObstaclePoint(planner_data.nearest_slow_down_point, PointType::SlowDown);
        debug_ptr_->pushPolygon(
          one_step_move_slow_down_range_polygon, p_front.position.z, PolygonType::SlowDown);

        last_detect_time_slowdown_point_ = trajectory_header.stamp;
      }

    } else {
      slow_down_pointcloud_ptr = obstacle_candidate_pointcloud_ptr;
    }

    {
      std::vector<cv::Point2d> one_step_move_vehicle_polygon;
      // create one step polygon for vehicle
      createOneStepPolygon(
        p_front, p_back, one_step_move_vehicle_polygon, vehicle_info, stop_param.lateral_margin);
      debug_ptr_->pushPolygon(
        one_step_move_vehicle_polygon, decimate_trajectory.at(i).pose.position.z,
        PolygonType::Vehicle);

      PointCloud::Ptr collision_pointcloud_ptr(new PointCloud);
      collision_pointcloud_ptr->header = obstacle_candidate_pointcloud_ptr->header;

      planner_data.found_collision_points = withinPolygon(
        one_step_move_vehicle_polygon, stop_param.stop_search_radius, prev_center_point,
        next_center_point, slow_down_pointcloud_ptr, collision_pointcloud_ptr);

      if (planner_data.found_collision_points) {
        planner_data.decimate_trajectory_collision_index = i;
        getNearestPoint(
          *collision_pointcloud_ptr, p_front, &planner_data.nearest_collision_point,
          &planner_data.nearest_collision_point_time);

        debug_ptr_->pushObstaclePoint(planner_data.nearest_collision_point, PointType::Stop);
        debug_ptr_->pushPolygon(
          one_step_move_vehicle_polygon, p_front.position.z, PolygonType::Collision);

        planner_data.stop_require = planner_data.found_collision_points;

        mutex_.lock();
        const auto object_ptr = object_ptr_;
        const auto current_velocity_ptr = current_velocity_ptr_;
        mutex_.unlock();

        acc_controller_->insertAdaptiveCruiseVelocity(
          decimate_trajectory, planner_data.decimate_trajectory_collision_index,
          planner_data.current_pose, planner_data.nearest_collision_point,
          planner_data.nearest_collision_point_time, object_ptr, current_velocity_ptr,
          &planner_data.stop_require, &output, trajectory_header);

        if (planner_data.stop_require) {
          last_detect_time_collision_point_ = trajectory_header.stamp;
        }

        break;
      }
    }
  }
}

void ObstacleStopPlannerNode::insertVelocity(
  TrajectoryPoints & output, PlannerData & planner_data, const Header & trajectory_header,
  const VehicleInfo & vehicle_info, const double current_acc, const double current_vel,
  const StopParam & stop_param)
{
  const auto & base_link2front = vehicle_info.max_longitudinal_offset_m;
  const auto no_hunting_collision_point =
    (rclcpp::Time(trajectory_header.stamp) - last_detect_time_collision_point_).seconds() >
    node_param_.hunting_threshold;

  if (planner_data.stop_require) {
    // insert stop point
    const auto traj_end_idx = output.size() - 1;
    const auto idx = planner_data.decimate_trajectory_index_map.at(
                       planner_data.decimate_trajectory_collision_index) +
                     planner_data.trajectory_trim_index;
    const auto index_with_dist_remain = findNearestFrontIndex(
      std::min(idx, traj_end_idx), output,
      createPoint(
        planner_data.nearest_collision_point.x, planner_data.nearest_collision_point.y, 0));

    if (index_with_dist_remain) {
      const auto vehicle_idx = std::min(planner_data.trajectory_trim_index, traj_end_idx);
      const auto dist_baselink_to_obstacle =
        calcSignedArcLength(output, vehicle_idx, index_with_dist_remain.get().first);

      debug_ptr_->setDebugValues(
        DebugValues::TYPE::COLLISION_OBSTACLE_DISTANCE,
        dist_baselink_to_obstacle + index_with_dist_remain.get().second - base_link2front);

      const auto stop_point = searchInsertPoint(
        index_with_dist_remain.get().first, output, index_with_dist_remain.get().second,
        stop_param);

      const auto & ego_pose = planner_data.current_pose;
      const size_t ego_seg_idx = findFirstNearestIndexWithSoftConstraints(
        output, ego_pose, node_param_.ego_nearest_dist_threshold,
        node_param_.ego_nearest_yaw_threshold);

      const double stop_point_distance = [&]() {
        if (output.size() < 2) {
          return 0.0;
        }

        size_t stop_seg_idx = 0;
        const double lon_offset =
          calcLongitudinalOffsetToSegment(output, stop_point.index, getPoint(stop_point.point));
        if (lon_offset < 0) {
          stop_seg_idx = std::max(static_cast<size_t>(0), stop_point.index - 1);
        } else {
          stop_seg_idx = std::min(output.size() - 2, stop_point.index);
        }

        return calcSignedArcLength(
          output, ego_pose.position, ego_seg_idx, getPoint(stop_point.point), stop_seg_idx);
      }();
      const auto is_stopped = current_vel < 0.01;

      const auto & ego_pos = planner_data.current_pose.position;
      if (stop_point_distance < stop_param_.hold_stop_margin_distance && is_stopped) {
        const auto ego_pos_on_path = calcLongitudinalOffsetPose(output, ego_pos, 0.0);

        if (ego_pos_on_path) {
          StopPoint current_stop_pos{};
          current_stop_pos.index = findFirstNearestSegmentIndexWithSoftConstraints(
            output, ego_pose, node_param_.ego_nearest_dist_threshold,
            node_param_.ego_nearest_yaw_threshold);
          current_stop_pos.point.pose = ego_pos_on_path.get();

          insertStopPoint(current_stop_pos, output, planner_data.stop_reason_diag);
          latest_stop_point_ = current_stop_pos;

          debug_ptr_->pushPose(getPose(stop_point.point), PoseType::TargetStop);
          debug_ptr_->pushPose(getPose(current_stop_pos.point), PoseType::Stop);
        }

      } else {
        insertStopPoint(stop_point, output, planner_data.stop_reason_diag);
        latest_stop_point_ = stop_point;

        debug_ptr_->pushPose(getPose(stop_point.point), PoseType::TargetStop);
        debug_ptr_->pushPose(getPose(stop_point.point), PoseType::Stop);
      }
    }
  } else if (!no_hunting_collision_point) {
    if (latest_stop_point_) {
      // update stop point index with the current trajectory
      latest_stop_point_.get().index = findFirstNearestSegmentIndexWithSoftConstraints(
        output, getPose(latest_stop_point_.get().point), node_param_.ego_nearest_dist_threshold,
        node_param_.ego_nearest_yaw_threshold);
      insertStopPoint(latest_stop_point_.get(), output, planner_data.stop_reason_diag);
      debug_ptr_->pushPose(getPose(latest_stop_point_.get().point), PoseType::TargetStop);
      debug_ptr_->pushPose(getPose(latest_stop_point_.get().point), PoseType::Stop);
    }
  }

  const auto no_hunting_slowdown_point =
    (rclcpp::Time(trajectory_header.stamp) - last_detect_time_slowdown_point_).seconds() >
    node_param_.hunting_threshold;

  if (planner_data.slow_down_require) {
    // insert slow down point
    const auto traj_end_idx = output.size() - 1;
    const auto idx = planner_data.decimate_trajectory_index_map.at(
      planner_data.decimate_trajectory_slow_down_index);
    const auto index_with_dist_remain = findNearestFrontIndex(
      std::min(idx, traj_end_idx), output,
      createPoint(
        planner_data.nearest_slow_down_point.x, planner_data.nearest_slow_down_point.y, 0));

    if (index_with_dist_remain) {
      const auto vehicle_idx = std::min(planner_data.trajectory_trim_index, traj_end_idx);
      const auto dist_baselink_to_obstacle =
        calcSignedArcLength(output, vehicle_idx, index_with_dist_remain.get().first);

      debug_ptr_->setDebugValues(
        DebugValues::TYPE::SLOWDOWN_OBSTACLE_DISTANCE,
        dist_baselink_to_obstacle + index_with_dist_remain.get().second - base_link2front);
      const auto slow_down_section = createSlowDownSection(
        index_with_dist_remain.get().first, output, planner_data.lateral_deviation,
        index_with_dist_remain.get().second, dist_baselink_to_obstacle, vehicle_info, current_acc,
        current_vel);

      if (
        (!latest_slow_down_section_ &&
         dist_baselink_to_obstacle + index_with_dist_remain.get().second <
           vehicle_info.max_longitudinal_offset_m) ||
        !no_hunting_slowdown_point) {
        latest_slow_down_section_ = slow_down_section;
      }

      insertSlowDownSection(slow_down_section, output);
    }
  } else if (!no_hunting_slowdown_point) {
    if (latest_slow_down_section_) {
      insertSlowDownSection(latest_slow_down_section_.get(), output);
    }
  }

  if (node_param_.enable_slow_down && latest_slow_down_section_) {
    // check whether ego is in slow down section or not
    const auto & p_start = latest_slow_down_section_.get().start_point.pose.position;
    const auto & p_end = latest_slow_down_section_.get().end_point.pose.position;
    const auto reach_slow_down_start_point =
      isInFrontOfTargetPoint(planner_data.current_pose, p_start);
    const auto reach_slow_down_end_point = isInFrontOfTargetPoint(planner_data.current_pose, p_end);
    const auto is_in_slow_down_section = reach_slow_down_start_point && !reach_slow_down_end_point;
    const auto index_with_dist_remain = findNearestFrontIndex(0, output, p_end);

    if (is_in_slow_down_section && index_with_dist_remain) {
      const auto end_insert_point_with_idx = getBackwardInsertPointFromBasePoint(
        index_with_dist_remain.get().first, output, -index_with_dist_remain.get().second);

      const auto slow_down_velocity =
        slow_down_param_.min_slow_down_velocity +
        (slow_down_param_.max_slow_down_velocity - slow_down_param_.min_slow_down_velocity) *
          std::max(planner_data.lateral_deviation - vehicle_info.vehicle_width_m / 2, 0.0) /
          slow_down_param_.lateral_margin;

      const auto target_velocity = slow_down_param_.consider_constraints
                                     ? slow_down_param_.slow_down_velocity
                                     : slow_down_velocity;

      SlowDownSection slow_down_section{};
      slow_down_section.slow_down_start_idx = 0;
      slow_down_section.start_point = output.front();
      slow_down_section.slow_down_end_idx = end_insert_point_with_idx.get().first;
      slow_down_section.end_point = end_insert_point_with_idx.get().second;
      slow_down_section.velocity =
        set_velocity_limit_ ? std::numeric_limits<double>::max() : target_velocity;

      insertSlowDownSection(slow_down_section, output);
    } else if (no_hunting_slowdown_point) {
      latest_slow_down_section_ = {};
    }
  }

  if (output.size() >= 2) {
    for (size_t i = 0; i < output.size() - 2; ++i) {
      const auto & p_base = output.at(i).pose;
      const auto & p_target = output.at(i + 1).pose;
      const auto & p_next = output.at(i + 2).pose;
      if (!checkValidIndex(p_base, p_next, p_target)) {
        RCLCPP_ERROR(get_logger(), "detect bad index");
      }
    }
  }

  pub_stop_reason_->publish(planner_data.stop_reason_diag);
}

void ObstacleStopPlannerNode::onExpandStopRange(const ExpandStopRange::ConstSharedPtr input_msg)
{
  // mutex for vehicle_info_, stop_param_
  std::lock_guard<std::mutex> lock(mutex_);

  const auto & i = vehicle_info_;
  stop_param_.lateral_margin = input_msg->expand_stop_range;
  stop_param_.stop_search_radius =
    stop_param_.step_length +
    std::hypot(i.vehicle_width_m / 2.0 + stop_param_.lateral_margin, i.vehicle_length_m / 2.0);
}

StopPoint ObstacleStopPlannerNode::searchInsertPoint(
  const int idx, const TrajectoryPoints & base_trajectory, const double dist_remain,
  const StopParam & stop_param)
{
  const auto max_dist_stop_point =
    createTargetPoint(idx, stop_param.max_longitudinal_margin, base_trajectory, dist_remain);
  const auto min_dist_stop_point =
    createTargetPoint(idx, stop_param.min_longitudinal_margin, base_trajectory, dist_remain);

  // check if stop point is already inserted by behavior planner
  bool is_inserted_already_stop_point = false;
  const double epsilon = 1e-3;
  for (int j = max_dist_stop_point.index - 1; j < static_cast<int>(idx); ++j) {
    if (std::abs(base_trajectory.at(std::max(j, 0)).longitudinal_velocity_mps) < epsilon) {
      is_inserted_already_stop_point = true;
      break;
    }
  }
  // insert stop point
  StopPoint stop_point{};
  stop_point.index =
    !is_inserted_already_stop_point ? max_dist_stop_point.index : min_dist_stop_point.index;
  stop_point.point =
    !is_inserted_already_stop_point ? max_dist_stop_point.point : min_dist_stop_point.point;
  return stop_point;
}

StopPoint ObstacleStopPlannerNode::createTargetPoint(
  const int idx, const double margin, const TrajectoryPoints & base_trajectory,
  const double dist_remain)
{
  const auto update_margin_from_vehicle = margin - dist_remain;
  const auto insert_point_with_idx =
    getBackwardInsertPointFromBasePoint(idx, base_trajectory, update_margin_from_vehicle);

  if (!insert_point_with_idx) {
    // TODO(Satoshi Ota)
    return StopPoint{};
  }

  StopPoint stop_point{};
  stop_point.index = insert_point_with_idx.get().first;
  stop_point.point = insert_point_with_idx.get().second;

  return stop_point;
}

SlowDownSection ObstacleStopPlannerNode::createSlowDownSection(
  const int idx, const TrajectoryPoints & base_trajectory, const double lateral_deviation,
  const double dist_remain, const double dist_baselink_to_obstacle,
  const VehicleInfo & vehicle_info, const double current_acc, const double current_vel)
{
  if (slow_down_param_.consider_constraints) {
    const auto margin_with_vel = calcFeasibleMarginAndVelocity(
      slow_down_param_, dist_baselink_to_obstacle + dist_remain, current_vel, current_acc);

    const auto relax_target_vel = margin_with_vel == boost::none;
    if (relax_target_vel && !set_velocity_limit_) {
      setExternalVelocityLimit();
    }

    const auto no_need_velocity_limit =
      dist_baselink_to_obstacle + dist_remain > slow_down_param_.longitudinal_forward_margin;
    if (set_velocity_limit_ && no_need_velocity_limit) {
      resetExternalVelocityLimit(current_acc, current_vel);
    }

    const auto use_velocity_limit = relax_target_vel || set_velocity_limit_;

    const auto update_forward_margin_from_vehicle =
      use_velocity_limit ? slow_down_param_.min_longitudinal_forward_margin - dist_remain
                         : margin_with_vel.get().first - dist_remain;
    const auto update_backward_margin_from_vehicle =
      slow_down_param_.longitudinal_backward_margin + dist_remain;

    const auto velocity =
      use_velocity_limit ? std::numeric_limits<double>::max() : margin_with_vel.get().second;

    return createSlowDownSectionFromMargin(
      idx, base_trajectory, update_forward_margin_from_vehicle, update_backward_margin_from_vehicle,
      velocity);
  } else {
    const auto update_forward_margin_from_vehicle =
      slow_down_param_.longitudinal_forward_margin - dist_remain;
    const auto update_backward_margin_from_vehicle =
      slow_down_param_.longitudinal_backward_margin + dist_remain;

    const auto velocity =
      slow_down_param_.min_slow_down_velocity +
      (slow_down_param_.max_slow_down_velocity - slow_down_param_.min_slow_down_velocity) *
        std::max(lateral_deviation - vehicle_info.vehicle_width_m / 2, 0.0) /
        slow_down_param_.lateral_margin;

    return createSlowDownSectionFromMargin(
      idx, base_trajectory, update_forward_margin_from_vehicle, update_backward_margin_from_vehicle,
      velocity);
  }

  return SlowDownSection{};
}

SlowDownSection ObstacleStopPlannerNode::createSlowDownSectionFromMargin(
  const int idx, const TrajectoryPoints & base_trajectory, const double forward_margin,
  const double backward_margin, const double velocity)
{
  // calc slow down start point
  const auto start_insert_point_with_idx =
    getBackwardInsertPointFromBasePoint(idx, base_trajectory, forward_margin);
  // calc slow down end point
  const auto end_insert_point_with_idx =
    getForwardInsertPointFromBasePoint(idx, base_trajectory, backward_margin);

  if (!start_insert_point_with_idx || !end_insert_point_with_idx) {
    // TODO(Satoshi Ota)
    return SlowDownSection{};
  }

  SlowDownSection slow_down_section{};
  slow_down_section.slow_down_start_idx = start_insert_point_with_idx.get().first;
  slow_down_section.start_point = start_insert_point_with_idx.get().second;
  slow_down_section.slow_down_end_idx = end_insert_point_with_idx.get().first;
  slow_down_section.end_point = end_insert_point_with_idx.get().second;
  slow_down_section.velocity = velocity;

  return slow_down_section;
}

void ObstacleStopPlannerNode::insertSlowDownSection(
  const SlowDownSection & slow_down_section, TrajectoryPoints & output)
{
  const auto traj_end_idx = output.size() - 1;
  const auto & start_idx = slow_down_section.slow_down_start_idx;
  const auto & end_idx = slow_down_section.slow_down_end_idx;

  const auto p_base_start = output.at(start_idx);
  const auto p_next_start = output.at(std::min(start_idx + 1, traj_end_idx));
  const auto & p_insert_start = slow_down_section.start_point;

  const auto p_base_end = output.at(end_idx);
  const auto p_next_end = output.at(std::min(end_idx + 1, traj_end_idx));
  const auto & p_insert_end = slow_down_section.end_point;

  constexpr double min_dist = 1e-3;

  const auto is_valid_index_start =
    checkValidIndex(p_base_start.pose, p_next_start.pose, p_insert_start.pose);
  const auto is_start_p_base_and_p_insert_overlap =
    calcDistance2d(p_base_start, p_insert_start) < min_dist;
  const auto is_start_p_next_and_p_insert_overlap =
    calcDistance2d(p_next_start, p_insert_start) < min_dist;

  auto update_start_idx = start_idx;
  auto update_end_idx = end_idx;

  if (
    !is_start_p_base_and_p_insert_overlap && !is_start_p_next_and_p_insert_overlap &&
    is_valid_index_start) {
    // insert: start_idx and end_idx are shifted by one
    output.insert(output.begin() + start_idx + 1, p_insert_start);
    update_start_idx = std::min(update_start_idx + 1, traj_end_idx);
    update_end_idx = std::min(update_end_idx + 1, traj_end_idx);
  } else if (is_start_p_next_and_p_insert_overlap) {
    // not insert: p_insert is merged into p_next
    update_start_idx = std::min(update_start_idx + 1, traj_end_idx);
  }

  const auto is_end_p_base_and_p_insert_overlap =
    calcDistance2d(p_base_end, p_insert_end) < min_dist;
  const auto is_end_p_next_and_p_insert_overlap =
    calcDistance2d(p_next_end, p_insert_end) < min_dist;
  const auto is_valid_index_end =
    checkValidIndex(p_base_end.pose, p_next_end.pose, p_insert_end.pose);

  if (
    !is_end_p_base_and_p_insert_overlap && !is_end_p_next_and_p_insert_overlap &&
    is_valid_index_end) {
    // insert: end_idx is shifted by one
    output.insert(output.begin() + update_end_idx + 1, p_insert_end);
    update_end_idx = std::min(update_end_idx + 1, traj_end_idx);
  } else if (is_end_p_next_and_p_insert_overlap) {
    // not insert: p_insert is merged into p_next
    update_end_idx = std::min(update_end_idx + 1, traj_end_idx);
  }

  for (size_t i = update_start_idx; i <= update_end_idx; ++i) {
    output.at(i).longitudinal_velocity_mps = std::min(
      slow_down_section.velocity, static_cast<double>(output.at(i).longitudinal_velocity_mps));
  }

  debug_ptr_->pushPose(p_base_start.pose, PoseType::SlowDownStart);
  debug_ptr_->pushPose(p_base_end.pose, PoseType::SlowDownEnd);
}

void ObstacleStopPlannerNode::onDynamicObjects(const PredictedObjects::ConstSharedPtr input_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  object_ptr_ = input_msg;
}

void ObstacleStopPlannerNode::onOdometry(const Odometry::ConstSharedPtr input_msg)
{
  // mutex for current_acc_, lpf_acc_
  std::lock_guard<std::mutex> lock(mutex_);
  current_velocity_ptr_ = input_msg;
}

void ObstacleStopPlannerNode::onAcceleration(
  const AccelWithCovarianceStamped::ConstSharedPtr input_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_acceleration_ptr_ = input_msg;
}

TrajectoryPoints ObstacleStopPlannerNode::trimTrajectoryWithIndexFromSelfPose(
  const TrajectoryPoints & input, const Pose & self_pose, size_t & index)
{
  TrajectoryPoints output{};

  const size_t min_distance_index = findFirstNearestIndexWithSoftConstraints(
    input, self_pose, node_param_.ego_nearest_dist_threshold,
    node_param_.ego_nearest_yaw_threshold);

  for (size_t i = min_distance_index; i < input.size(); ++i) {
    output.push_back(input.at(i));
  }
  index = min_distance_index;

  return output;
}

bool ObstacleStopPlannerNode::searchPointcloudNearTrajectory(
  const TrajectoryPoints & trajectory, const PointCloud2::ConstSharedPtr & input_points_ptr,
  PointCloud::Ptr output_points_ptr, const Header & trajectory_header,
  const VehicleInfo & vehicle_info, const StopParam & stop_param)
{
  // transform pointcloud
  TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      trajectory_header.frame_id, input_points_ptr->header.frame_id, input_points_ptr->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Failed to look up transform from " << trajectory_header.frame_id << " to "
                                                        << input_points_ptr->header.frame_id);
    return false;
  }

  PointCloud2 transformed_points{};
  const Eigen::Matrix4f affine_matrix =
    tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(affine_matrix, *input_points_ptr, transformed_points);
  PointCloud::Ptr transformed_points_ptr(new PointCloud);
  pcl::fromROSMsg(transformed_points, *transformed_points_ptr);

  output_points_ptr->header = transformed_points_ptr->header;

  // search obstacle candidate pointcloud to reduce calculation cost
  const double search_radius = node_param_.enable_slow_down
                                 ? slow_down_param_.slow_down_search_radius
                                 : stop_param.stop_search_radius;
  const double squared_radius = search_radius * search_radius;
  std::vector<geometry_msgs::msg::Point> center_points;
  center_points.reserve(trajectory.size());
  for (const auto & trajectory_point : trajectory)
    center_points.push_back(getVehicleCenterFromBase(trajectory_point.pose, vehicle_info).position);
  for (const auto & point : transformed_points_ptr->points) {
    for (const auto & center_point : center_points) {
      const double x = center_point.x - point.x;
      const double y = center_point.y - point.y;
      const double squared_distance = x * x + y * y;
      if (squared_distance < squared_radius) {
        output_points_ptr->points.push_back(point);
        break;
      }
    }
  }
  return true;
}

void ObstacleStopPlannerNode::setExternalVelocityLimit()
{
  const auto & p = slow_down_param_;
  auto slow_down_limit_vel = std::make_shared<VelocityLimit>();
  slow_down_limit_vel->stamp = this->now();
  slow_down_limit_vel->max_velocity = p.slow_down_velocity;
  slow_down_limit_vel->constraints.min_acceleration =
    p.slow_down_min_jerk < p.normal_min_jerk ? p.limit_min_acc : p.normal_min_acc;
  slow_down_limit_vel->constraints.max_jerk = std::abs(p.slow_down_min_jerk);
  slow_down_limit_vel->constraints.min_jerk = p.slow_down_min_jerk;
  slow_down_limit_vel->use_constraints = true;
  slow_down_limit_vel->sender = "obstacle_stop_planner";

  pub_velocity_limit_->publish(*slow_down_limit_vel);
  set_velocity_limit_ = true;

  RCLCPP_INFO(
    get_logger(), "set velocity limit. jerk:%-6.2f dec:%-6.2f",
    slow_down_limit_vel->constraints.min_jerk, slow_down_limit_vel->constraints.min_acceleration);
}

void ObstacleStopPlannerNode::resetExternalVelocityLimit(
  const double current_acc, const double current_vel)
{
  const auto reach_target_vel = current_vel < slow_down_param_.slow_down_velocity +
                                                slow_down_param_.velocity_threshold_decel_complete;
  const auto constant_vel =
    std::abs(current_acc) < slow_down_param_.acceleration_threshold_decel_complete;
  const auto no_undershoot = reach_target_vel && constant_vel;

  if (!no_undershoot) {
    return;
  }

  auto velocity_limit_clear_command = std::make_shared<VelocityLimitClearCommand>();
  velocity_limit_clear_command->stamp = this->now();
  velocity_limit_clear_command->command = true;
  velocity_limit_clear_command->sender = "obstacle_stop_planner";

  pub_clear_velocity_limit_->publish(*velocity_limit_clear_command);
  set_velocity_limit_ = false;

  RCLCPP_INFO(get_logger(), "reset velocity limit");
}

void ObstacleStopPlannerNode::publishDebugData(
  const PlannerData & planner_data, const double current_acc, const double current_vel)
{
  debug_ptr_->setDebugValues(DebugValues::TYPE::CURRENT_VEL, current_vel);
  debug_ptr_->setDebugValues(DebugValues::TYPE::CURRENT_ACC, current_acc);
  debug_ptr_->setDebugValues(
    DebugValues::TYPE::FLAG_FIND_SLOW_DOWN_OBSTACLE, planner_data.slow_down_require);
  debug_ptr_->setDebugValues(
    DebugValues::TYPE::FLAG_FIND_COLLISION_OBSTACLE, planner_data.stop_require);
  debug_ptr_->setDebugValues(DebugValues::TYPE::FLAG_EXTERNAL, set_velocity_limit_);

  const auto now_adaptive_cruise =
    !planner_data.stop_require && planner_data.found_collision_points;
  debug_ptr_->setDebugValues(DebugValues::TYPE::FLAG_ADAPTIVE_CRUISE, now_adaptive_cruise);

  debug_ptr_->publish();
}

}  // namespace motion_planning

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::ObstacleStopPlannerNode)
