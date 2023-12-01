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
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "obstacle_stop_planner/node.hpp"
#include "obstacle_stop_planner/planner_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

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
using autoware_auto_perception_msgs::msg::PredictedObject;
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
    p.enable_z_axis_obstacle_filtering =
      declare_parameter<bool>("enable_z_axis_obstacle_filtering");
    p.z_axis_filtering_buffer = declare_parameter<double>("z_axis_filtering_buffer");
    p.max_velocity = declare_parameter<double>("max_velocity");
    p.chattering_threshold = declare_parameter<double>("chattering_threshold");
    p.ego_nearest_dist_threshold = declare_parameter<double>("ego_nearest_dist_threshold");
    p.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");
    p.voxel_grid_x = declare_parameter<double>("voxel_grid_x");
    p.voxel_grid_y = declare_parameter<double>("voxel_grid_y");
    p.voxel_grid_z = declare_parameter<double>("voxel_grid_z");
    p.use_predicted_objects = declare_parameter<bool>("use_predicted_objects");
    p.publish_obstacle_polygon = declare_parameter<bool>("publish_obstacle_polygon");
    p.predicted_object_filtering_threshold =
      declare_parameter<double>("predicted_object_filtering_threshold");
  }

  {
    auto & p = stop_param_;
    const std::string ns = "stop_planner.";

    // params for stop position
    p.max_longitudinal_margin =
      declare_parameter<double>(ns + "stop_position.max_longitudinal_margin");
    p.max_longitudinal_margin_behind_goal =
      declare_parameter<double>(ns + "stop_position.max_longitudinal_margin_behind_goal");
    p.min_longitudinal_margin =
      declare_parameter<double>(ns + "stop_position.min_longitudinal_margin");
    p.hold_stop_margin_distance =
      declare_parameter<double>(ns + "stop_position.hold_stop_margin_distance");

    // params for detection area
    p.lateral_margin = declare_parameter<double>(ns + "detection_area.lateral_margin");
    p.vehicle_lateral_margin =
      declare_parameter<double>(ns + "detection_area.vehicle_lateral_margin");
    p.pedestrian_lateral_margin =
      declare_parameter<double>(ns + "detection_area.pedestrian_lateral_margin");
    p.unknown_lateral_margin =
      declare_parameter<double>(ns + "detection_area.unknown_lateral_margin");
    p.enable_stop_behind_goal_for_obstacle =
      declare_parameter<bool>(ns + "detection_area.enable_stop_behind_goal_for_obstacle");
    p.step_length = declare_parameter<double>(ns + "detection_area.step_length");

    // apply offset
    p.max_longitudinal_margin += i.max_longitudinal_offset_m;
    p.max_longitudinal_margin_behind_goal += i.max_longitudinal_offset_m;
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
    p.vehicle_lateral_margin =
      declare_parameter<double>(ns + "detection_area.vehicle_lateral_margin");
    p.pedestrian_lateral_margin =
      declare_parameter<double>(ns + "detection_area.pedestrian_lateral_margin");
    p.unknown_lateral_margin =
      declare_parameter<double>(ns + "detection_area.unknown_lateral_margin");

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

  if (node_param_.use_predicted_objects) {
    // Search the maximum lateral margin
    std::vector<double> lateral_margins{
      stop_param_.pedestrian_lateral_margin, stop_param_.vehicle_lateral_margin,
      stop_param_.unknown_lateral_margin};
    if (node_param_.enable_slow_down) {
      lateral_margins.push_back(slow_down_param_.pedestrian_lateral_margin);
      lateral_margins.push_back(slow_down_param_.vehicle_lateral_margin);
      lateral_margins.push_back(slow_down_param_.unknown_lateral_margin);
    }
    const double max_lateral_margin =
      *std::max_element(lateral_margins.begin(), lateral_margins.end());
    object_filtering_margin_ =
      max_lateral_margin + node_param_.predicted_object_filtering_threshold;
  }

  // Initializer
  acc_controller_ = std::make_unique<AdaptiveCruiseController>(
    this, i.vehicle_width_m, i.vehicle_length_m, i.max_longitudinal_offset_m);
  debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(this, i.max_longitudinal_offset_m);

  // Publishers
  pub_trajectory_ = this->create_publisher<Trajectory>("~/output/trajectory", 1);

  pub_stop_reason_ = this->create_publisher<DiagnosticStatus>("~/output/stop_reason", 1);

  pub_clear_velocity_limit_ = this->create_publisher<VelocityLimitClearCommand>(
    "~/output/velocity_limit_clear_command", rclcpp::QoS{1}.transient_local());

  pub_velocity_limit_ = this->create_publisher<VelocityLimit>(
    "~/output/max_velocity", rclcpp::QoS{1}.transient_local());

  pub_obstacle_pointcloud_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/obstacle_pointcloud", 1);

  pub_collision_pointcloud_debug_ =
    this->create_publisher<PointCloud2>("~/debug/collision_pointcloud", 1);

  pub_processing_time_ms_ = this->create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);

  // Subscribers
  if (!node_param_.use_predicted_objects) {
    // No need to point cloud while using predicted objects
    sub_point_cloud_ = this->create_subscription<PointCloud2>(
      "~/input/pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleStopPlannerNode::onPointCloud, this, std::placeholders::_1),
      createSubscriptionOptions(this));
  }

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

  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
}

void ObstacleStopPlannerNode::onPointCloud(const PointCloud2::ConstSharedPtr input_msg)
{
  // mutex for obstacle_ros_pointcloud_ptr_
  // NOTE: *obstacle_ros_pointcloud_ptr_ is used
  std::lock_guard<std::mutex> lock(mutex_);

  obstacle_ros_pointcloud_ptr_ = std::make_shared<PointCloud2>();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  PointCloud::Ptr pointcloud_ptr(new PointCloud);
  PointCloud::Ptr no_height_filtered_pointcloud_ptr(new PointCloud);

  pcl::fromROSMsg(*input_msg, *pointcloud_ptr);
  if (!node_param_.enable_z_axis_obstacle_filtering) {
    filter.setInputCloud(pointcloud_ptr);
    filter.setLeafSize(
      node_param_.voxel_grid_x, node_param_.voxel_grid_y, node_param_.voxel_grid_z);
    filter.filter(*no_height_filtered_pointcloud_ptr);
    pcl::toROSMsg(*no_height_filtered_pointcloud_ptr, *obstacle_ros_pointcloud_ptr_);
  } else {
    pcl::toROSMsg(*pointcloud_ptr, *obstacle_ros_pointcloud_ptr_);
  }

  obstacle_ros_pointcloud_ptr_->header = input_msg->header;
  pub_obstacle_pointcloud_->publish(*obstacle_ros_pointcloud_ptr_);
}

void ObstacleStopPlannerNode::onTrigger(const Trajectory::ConstSharedPtr input_msg)
{
  stop_watch_.tic(__func__);

  mutex_.lock();
  // NOTE: these variables must not be referenced for multithreading
  const auto vehicle_info = vehicle_info_;
  const auto stop_param = stop_param_;
  const auto obstacle_ros_pointcloud_ptr = obstacle_ros_pointcloud_ptr_;
  const auto object_ptr = object_ptr_;
  const auto current_odometry_ptr = current_odometry_ptr_;
  const auto current_acceleration_ptr = current_acceleration_ptr_;
  mutex_.unlock();

  {
    const auto waiting = [this](const auto & str) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(), "waiting for %s ...",
        str);
    };

    if (!object_ptr && node_param_.use_predicted_objects) {
      waiting("perception object");
      return;
    }

    if (!obstacle_ros_pointcloud_ptr && !node_param_.use_predicted_objects) {
      waiting("obstacle pointcloud");
      return;
    }

    if (!current_odometry_ptr) {
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

  const auto current_vel = current_odometry_ptr->twist.twist.linear.x;
  const auto current_acc = current_acceleration_ptr->accel.accel.linear.x;

  // TODO(someone): support backward path
  const auto is_driving_forward = motion_utils::isDrivingForwardWithTwist(input_msg->points);
  is_driving_forward_ = is_driving_forward ? is_driving_forward.value() : is_driving_forward_;
  if (!is_driving_forward_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000, "Backward path is NOT supported. publish input as it is.");
    pub_trajectory_->publish(*input_msg);
    return;
  }

  PlannerData planner_data{};

  planner_data.current_pose = current_odometry_ptr_->pose.pose;

  Trajectory output_trajectory = *input_msg;
  TrajectoryPoints output_trajectory_points =
    motion_utils::convertToTrajectoryPointArray(*input_msg);

  // trim trajectory from self pose
  TrajectoryPoints base_trajectory = trimTrajectoryWithIndexFromSelfPose(
    motion_utils::convertToTrajectoryPointArray(*input_msg), planner_data.current_pose,
    planner_data.trajectory_trim_index);

  // extend trajectory to consider obstacles after the goal
  if (stop_param.enable_stop_behind_goal_for_obstacle) {
    base_trajectory = extendTrajectory(base_trajectory, stop_param.max_longitudinal_margin);
  }
  // decimate trajectory for calculation cost
  const auto decimate_trajectory = decimateTrajectory(
    base_trajectory, stop_param.step_length, planner_data.decimate_trajectory_index_map);

  if (node_param_.use_predicted_objects) {
    searchPredictedObject(
      decimate_trajectory, output_trajectory_points, planner_data, input_msg->header, vehicle_info,
      stop_param);
  } else {
    // search obstacles within slow-down/collision area
    searchObstacle(
      decimate_trajectory, output_trajectory_points, planner_data, input_msg->header, vehicle_info,
      stop_param, obstacle_ros_pointcloud_ptr);
  }

  // insert slow-down-section/stop-point
  insertVelocity(
    output_trajectory_points, planner_data, input_msg->header, vehicle_info, current_acc,
    current_vel, stop_param);

  const auto no_slow_down_section = !planner_data.slow_down_require && !latest_slow_down_section_;
  if (node_param_.enable_slow_down && no_slow_down_section && set_velocity_limit_) {
    resetExternalVelocityLimit(current_acc, current_vel);
  }

  auto trajectory = motion_utils::convertToTrajectory(output_trajectory_points);
  publishDebugData(planner_data, current_acc, current_vel);

  trajectory.header = input_msg->header;
  pub_trajectory_->publish(trajectory);

  Float64Stamped processing_time_ms;
  processing_time_ms.stamp = now();
  processing_time_ms.data = stop_watch_.toc(__func__);
  pub_processing_time_ms_->publish(processing_time_ms);
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

  const auto now = this->now();

  updateObstacleHistory(now);

  for (size_t i = 0; i < decimate_trajectory.size() - 1; ++i) {
    // create one step circle center for vehicle
    const auto & p_front = decimate_trajectory.at(i).pose;
    const auto & p_back = decimate_trajectory.at(i + 1).pose;
    const auto z_axis_min = p_front.position.z;
    const auto z_axis_max =
      p_front.position.z + vehicle_info.vehicle_height_m + node_param_.z_axis_filtering_buffer;
    const auto prev_center_pose = getVehicleCenterFromBase(p_front, vehicle_info);
    const Point2d prev_center_point(prev_center_pose.position.x, prev_center_pose.position.y);
    const auto next_center_pose = getVehicleCenterFromBase(p_back, vehicle_info);
    const Point2d next_center_point(next_center_pose.position.x, next_center_pose.position.y);

    if (node_param_.enable_slow_down) {
      Polygon2d one_step_move_slow_down_range_polygon;
      // create one step polygon for slow_down range
      createOneStepPolygon(
        p_front, p_back, one_step_move_slow_down_range_polygon, vehicle_info,
        slow_down_param_.lateral_margin);
      debug_ptr_->pushPolygon(
        one_step_move_slow_down_range_polygon, p_front.position.z, PolygonType::SlowDownRange);

      if (node_param_.enable_z_axis_obstacle_filtering) {
        planner_data.found_slow_down_points = withinPolyhedron(
          one_step_move_slow_down_range_polygon, slow_down_param_.slow_down_search_radius,
          prev_center_point, next_center_point, obstacle_candidate_pointcloud_ptr,
          slow_down_pointcloud_ptr, z_axis_min, z_axis_max);
      } else {
        planner_data.found_slow_down_points = withinPolygon(
          one_step_move_slow_down_range_polygon, slow_down_param_.slow_down_search_radius,
          prev_center_point, next_center_point, obstacle_candidate_pointcloud_ptr,
          slow_down_pointcloud_ptr);
      }
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
      }

    } else {
      slow_down_pointcloud_ptr = obstacle_candidate_pointcloud_ptr;
    }

    {
      Polygon2d one_step_move_vehicle_polygon;
      // create one step polygon for vehicle
      createOneStepPolygon(
        p_front, p_back, one_step_move_vehicle_polygon, vehicle_info, stop_param.lateral_margin);
      if (node_param_.enable_z_axis_obstacle_filtering) {
        debug_ptr_->pushPolyhedron(
          one_step_move_vehicle_polygon, z_axis_min, z_axis_max, PolygonType::Vehicle);
      } else {
        debug_ptr_->pushPolygon(
          one_step_move_vehicle_polygon, p_front.position.z, PolygonType::Vehicle);
      }

      PointCloud::Ptr collision_pointcloud_ptr(new PointCloud);
      collision_pointcloud_ptr->header = obstacle_candidate_pointcloud_ptr->header;

      const auto found_collision_points =
        node_param_.enable_z_axis_obstacle_filtering
          ? withinPolyhedron(
              one_step_move_vehicle_polygon, stop_param.stop_search_radius, prev_center_point,
              next_center_point, slow_down_pointcloud_ptr, collision_pointcloud_ptr, z_axis_min,
              z_axis_max)
          : withinPolygon(
              one_step_move_vehicle_polygon, stop_param.stop_search_radius, prev_center_point,
              next_center_point, slow_down_pointcloud_ptr, collision_pointcloud_ptr);

      if (found_collision_points) {
        pcl::PointXYZ nearest_collision_point;
        rclcpp::Time nearest_collision_point_time;

        getNearestPoint(
          *collision_pointcloud_ptr, p_front, &nearest_collision_point,
          &nearest_collision_point_time);

        obstacle_history_.emplace_back(now, nearest_collision_point);

        break;
      }
    }
  }

  for (size_t i = 0; i < decimate_trajectory.size() - 1; ++i) {
    const auto old_point_cloud_ptr = getOldPointCloudPtr();
    if (old_point_cloud_ptr->empty()) {
      // no need to check collision
      break;
    }

    // create one step circle center for vehicle
    const auto & p_front = decimate_trajectory.at(i).pose;
    const auto & p_back = decimate_trajectory.at(i + 1).pose;
    const auto z_axis_min = p_front.position.z;
    const auto z_axis_max =
      p_front.position.z + vehicle_info.vehicle_height_m + node_param_.z_axis_filtering_buffer;
    const auto prev_center_pose = getVehicleCenterFromBase(p_front, vehicle_info);
    const Point2d prev_center_point(prev_center_pose.position.x, prev_center_pose.position.y);
    const auto next_center_pose = getVehicleCenterFromBase(p_back, vehicle_info);
    const Point2d next_center_point(next_center_pose.position.x, next_center_pose.position.y);

    Polygon2d one_step_move_vehicle_polygon;
    // create one step polygon for vehicle
    createOneStepPolygon(
      p_front, p_back, one_step_move_vehicle_polygon, vehicle_info, stop_param.lateral_margin);

    PointCloud::Ptr collision_pointcloud_ptr(new PointCloud);
    collision_pointcloud_ptr->header = obstacle_candidate_pointcloud_ptr->header;

    // check new collision points
    if (node_param_.enable_z_axis_obstacle_filtering) {
      planner_data.found_collision_points = withinPolyhedron(
        one_step_move_vehicle_polygon, stop_param.stop_search_radius, prev_center_point,
        next_center_point, old_point_cloud_ptr, collision_pointcloud_ptr, z_axis_min, z_axis_max);
    } else {
      planner_data.found_collision_points = withinPolygon(
        one_step_move_vehicle_polygon, stop_param.stop_search_radius, prev_center_point,
        next_center_point, old_point_cloud_ptr, collision_pointcloud_ptr);
    }

    if (planner_data.found_collision_points) {
      planner_data.decimate_trajectory_collision_index = i;
      getNearestPoint(
        *collision_pointcloud_ptr, p_front, &planner_data.nearest_collision_point,
        &planner_data.nearest_collision_point_time);

      if (node_param_.enable_z_axis_obstacle_filtering) {
        debug_ptr_->pushPolyhedron(
          one_step_move_vehicle_polygon, z_axis_min, z_axis_max, PolygonType::Collision);
        if (
          (pub_collision_pointcloud_debug_->get_subscription_count() +
           pub_collision_pointcloud_debug_->get_intra_process_subscription_count()) > 0) {
          auto obstacle_ros_pointcloud_debug_ptr = std::make_shared<PointCloud2>();
          pcl::toROSMsg(*collision_pointcloud_ptr, *obstacle_ros_pointcloud_debug_ptr);
          obstacle_ros_pointcloud_debug_ptr->header.frame_id = trajectory_header.frame_id;
          pub_collision_pointcloud_debug_->publish(*obstacle_ros_pointcloud_debug_ptr);
        }
      } else {
        debug_ptr_->pushObstaclePoint(planner_data.nearest_collision_point, PointType::Stop);
        debug_ptr_->pushPolygon(
          one_step_move_vehicle_polygon, p_front.position.z, PolygonType::Collision);
      }

      planner_data.stop_require = planner_data.found_collision_points;

      mutex_.lock();
      const auto current_odometry_ptr = current_odometry_ptr_;
      const auto object_ptr = object_ptr_;
      mutex_.unlock();

      acc_controller_->insertAdaptiveCruiseVelocity(
        decimate_trajectory, planner_data.decimate_trajectory_collision_index,
        planner_data.current_pose, planner_data.nearest_collision_point,
        planner_data.nearest_collision_point_time, object_ptr, current_odometry_ptr,
        &planner_data.stop_require, &output, trajectory_header);

      if (!planner_data.stop_require) {
        obstacle_history_.clear();
      }
      break;
    }
  }
}

void ObstacleStopPlannerNode::searchPredictedObject(
  const TrajectoryPoints & decimate_trajectory, TrajectoryPoints & output,
  PlannerData & planner_data, const Header & trajectory_header, const VehicleInfo & vehicle_info,
  const StopParam & stop_param)
{
  mutex_.lock();
  const auto object_ptr = object_ptr_;
  const auto current_odometry_pointer = current_odometry_ptr_;
  mutex_.unlock();

  const auto ego_pose = current_odometry_pointer->pose.pose;
  PredictedObjects filtered_objects;
  filterObstacles(
    *object_ptr.get(), ego_pose, decimate_trajectory, object_filtering_margin_, filtered_objects);

  const auto now = this->now();

  updatePredictedObstacleHistory(now);
  for (size_t i = 0; i < decimate_trajectory.size() - 1; ++i) {
    // create one step circle center for vehicle
    const auto & p_front = decimate_trajectory.at(i).pose;
    const auto & p_back = decimate_trajectory.at(i + 1).pose;
    const auto z_axis_min = p_front.position.z;
    const auto z_axis_max =
      p_front.position.z + vehicle_info.vehicle_height_m + node_param_.z_axis_filtering_buffer;

    if (node_param_.enable_slow_down) {
      double min_slow_down_norm = 0.0;
      bool is_init = false;
      size_t nearest_slow_down_object_index = 0;
      geometry_msgs::msg::Point nearest_slow_down_point;
      geometry_msgs::msg::PoseArray slow_down_points;

      for (size_t j = 0; j < filtered_objects.objects.size(); ++j) {
        const auto & obj = filtered_objects.objects.at(j);
        if (node_param_.enable_z_axis_obstacle_filtering) {
          if (!intersectsInZAxis(obj, z_axis_min, z_axis_max)) {
            continue;
          }
        }
        Polygon2d one_step_move_slow_down_range;
        bool found_slow_down_object = false;
        Polygon2d object_polygon{};
        if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
          object_polygon = convertCylindricalObjectToGeometryPolygon(
            obj.kinematics.initial_pose_with_covariance.pose, obj.shape);
          createOneStepPolygon(
            p_front, p_back, one_step_move_slow_down_range, vehicle_info,
            slow_down_param_.pedestrian_lateral_margin);
          found_slow_down_object = bg::intersects(one_step_move_slow_down_range, object_polygon);

        } else if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
          createOneStepPolygon(
            p_front, p_back, one_step_move_slow_down_range, vehicle_info,
            slow_down_param_.vehicle_lateral_margin);
          const double & length_m = obj.shape.dimensions.x / 2;
          const double & width_m = obj.shape.dimensions.y / 2;
          object_polygon = convertBoundingBoxObjectToGeometryPolygon(
            obj.kinematics.initial_pose_with_covariance.pose, length_m, length_m, width_m);

          found_slow_down_object = bg::intersects(one_step_move_slow_down_range, object_polygon);

        } else if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
          createOneStepPolygon(
            p_front, p_back, one_step_move_slow_down_range, vehicle_info,
            slow_down_param_.unknown_lateral_margin);

          object_polygon = convertPolygonObjectToGeometryPolygon(
            obj.kinematics.initial_pose_with_covariance.pose, obj.shape);

          found_slow_down_object = bg::intersects(one_step_move_slow_down_range, object_polygon);

        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000, "Object type is not supported. type: %d",
            obj.shape.type);
          continue;
        }
        if (found_slow_down_object) {
          geometry_msgs::msg::PoseArray slow_down_points_tmp;

          std::vector<Point2d> slow_down_point;
          bg::intersection(one_step_move_slow_down_range, object_polygon, slow_down_point);
          for (const auto & point : slow_down_point) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = point.x();
            pose.position.y = point.y();
            slow_down_points_tmp.poses.push_back(pose);
          }

          // Also check the corner points
          for (const auto & point : object_polygon.outer()) {
            if (bg::within(point, one_step_move_slow_down_range)) {
              geometry_msgs::msg::Pose pose;
              pose.position.x = point.x();
              pose.position.y = point.y();
              slow_down_points_tmp.poses.push_back(pose);
            }
          }
          geometry_msgs::msg::Point nearest_slow_down_point_tmp;
          const double norm = getNearestPointAndDistanceForPredictedObject(
            slow_down_points_tmp, p_front, &nearest_slow_down_point_tmp);
          if (norm < min_slow_down_norm || !is_init) {
            min_slow_down_norm = norm;
            nearest_slow_down_point = nearest_slow_down_point_tmp;
            is_init = true;
            nearest_slow_down_object_index = j;
            slow_down_points = slow_down_points_tmp;
          }
        }
      }

      planner_data.found_slow_down_points = is_init;

      const auto found_first_slow_down_points =
        planner_data.found_slow_down_points && !planner_data.slow_down_require;

      if (found_first_slow_down_points) {
        // found nearest slow down obstacle
        planner_data.decimate_trajectory_slow_down_index = i;
        planner_data.slow_down_require = true;
        planner_data.nearest_slow_down_point =
          pointToPcl(nearest_slow_down_point.x, nearest_slow_down_point.y, p_front.position.z);
        planner_data.nearest_collision_point_time = filtered_objects.header.stamp;
        planner_data.slow_down_object_shape =
          filtered_objects.objects.at(nearest_slow_down_object_index).shape;

        // TODO(brkay54): lateral_nearest_slow_down_point_pose and nearest_slow_down_point_pose are
        // not used
        getLateralNearestPointForPredictedObject(
          slow_down_points, p_front, &planner_data.lateral_nearest_slow_down_point,
          &planner_data.lateral_deviation);

        // Push slow down debugging points
        Polygon2d one_step_move_slow_down_vehicle_polygon;

        const auto & obj = filtered_objects.objects.at(nearest_slow_down_object_index);
        if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
          createOneStepPolygon(
            p_front, p_back, one_step_move_slow_down_vehicle_polygon, vehicle_info,
            slow_down_param_.pedestrian_lateral_margin);
        } else if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
          createOneStepPolygon(
            p_front, p_back, one_step_move_slow_down_vehicle_polygon, vehicle_info,
            slow_down_param_.vehicle_lateral_margin);
        } else if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
          createOneStepPolygon(
            p_front, p_back, one_step_move_slow_down_vehicle_polygon, vehicle_info,
            slow_down_param_.unknown_lateral_margin);
        }
        debug_ptr_->pushObstaclePoint(planner_data.nearest_slow_down_point, PointType::SlowDown);

        debug_ptr_->pushPolygon(
          one_step_move_slow_down_vehicle_polygon, p_front.position.z, PolygonType::SlowDown);
      } else {
        // only used for pedestrian and debugging
        Polygon2d one_step_move_slow_down_range_dbg;
        createOneStepPolygon(
          p_front, p_back, one_step_move_slow_down_range_dbg, vehicle_info,
          slow_down_param_.pedestrian_lateral_margin);

        debug_ptr_->pushPolygon(
          one_step_move_slow_down_range_dbg, p_front.position.z, PolygonType::SlowDownRange);
      }
    }

    {
      double min_collision_norm = 0.0;
      bool is_init = false;
      size_t nearest_collision_object_index = 0;
      geometry_msgs::msg::Point nearest_collision_point;

      for (size_t j = 0; j < filtered_objects.objects.size(); ++j) {
        const auto & obj = filtered_objects.objects.at(j);
        if (node_param_.enable_z_axis_obstacle_filtering) {
          if (!intersectsInZAxis(obj, z_axis_min, z_axis_max)) {
            continue;
          }
        }
        Polygon2d one_step_move_collision_polygon;
        bool found_collision_object = false;
        Polygon2d object_polygon{};
        if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
          object_polygon = convertCylindricalObjectToGeometryPolygon(
            obj.kinematics.initial_pose_with_covariance.pose, obj.shape);

          createOneStepPolygon(
            p_front, p_back, one_step_move_collision_polygon, vehicle_info,
            stop_param.pedestrian_lateral_margin);

          found_collision_object = bg::intersects(one_step_move_collision_polygon, object_polygon);
        } else if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
          const double & length_m = obj.shape.dimensions.x / 2;
          const double & width_m = obj.shape.dimensions.y / 2;
          object_polygon = convertBoundingBoxObjectToGeometryPolygon(
            obj.kinematics.initial_pose_with_covariance.pose, length_m, length_m, width_m);
          createOneStepPolygon(
            p_front, p_back, one_step_move_collision_polygon, vehicle_info,
            stop_param.vehicle_lateral_margin);

          found_collision_object = bg::intersects(one_step_move_collision_polygon, object_polygon);
        } else if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
          object_polygon = convertPolygonObjectToGeometryPolygon(
            obj.kinematics.initial_pose_with_covariance.pose, obj.shape);
          createOneStepPolygon(
            p_front, p_back, one_step_move_collision_polygon, vehicle_info,
            stop_param.unknown_lateral_margin);

          found_collision_object = bg::intersects(one_step_move_collision_polygon, object_polygon);
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000, "Object type is not supported. type: %d",
            obj.shape.type);
          continue;
        }
        if (found_collision_object) {
          geometry_msgs::msg::PoseArray collision_points_tmp;

          std::vector<Point2d> collision_point;
          bg::intersection(one_step_move_collision_polygon, object_polygon, collision_point);
          for (const auto & point : collision_point) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = point.x();
            pose.position.y = point.y();
            collision_points_tmp.poses.push_back(pose);
          }

          // Also check the corner points
          for (const auto & point : object_polygon.outer()) {
            if (bg::within(point, one_step_move_collision_polygon)) {
              geometry_msgs::msg::Pose pose;
              pose.position.x = point.x();
              pose.position.y = point.y();
              collision_points_tmp.poses.push_back(pose);
            }
          }
          geometry_msgs::msg::Point nearest_collision_point_tmp;
          const double norm = getNearestPointAndDistanceForPredictedObject(
            collision_points_tmp, p_front, &nearest_collision_point_tmp);
          if (norm < min_collision_norm || !is_init) {
            min_collision_norm = norm;
            nearest_collision_point = nearest_collision_point_tmp;
            is_init = true;
            nearest_collision_object_index = j;
          }
        }
      }
      if (is_init) {
        predicted_object_history_.emplace_back(
          now, nearest_collision_point,
          filtered_objects.objects.at(nearest_collision_object_index));
        break;
      }

      // only used for pedestrian
      Polygon2d one_step_move_collision_dbg;
      createOneStepPolygon(
        p_front, p_back, one_step_move_collision_dbg, vehicle_info,
        stop_param.pedestrian_lateral_margin);
      if (node_param_.enable_z_axis_obstacle_filtering) {
        debug_ptr_->pushPolyhedron(
          one_step_move_collision_dbg, z_axis_min, z_axis_max, PolygonType::Vehicle);
      } else {
        debug_ptr_->pushPolygon(
          one_step_move_collision_dbg, p_front.position.z, PolygonType::Vehicle);
      }
    }
  }

  for (size_t i = 0; i < decimate_trajectory.size() - 1; ++i) {
    if (predicted_object_history_.empty()) {
      break;
    }

    // create one step circle center for vehicle
    const auto & p_front = decimate_trajectory.at(i).pose;
    const auto & p_back = decimate_trajectory.at(i + 1).pose;
    const auto z_axis_min = p_front.position.z;
    const auto z_axis_max =
      p_front.position.z + vehicle_info.vehicle_height_m + node_param_.z_axis_filtering_buffer;

    double min_collision_norm = 0.0;
    bool is_init = false;
    size_t nearest_collision_object_index = 0;

    for (size_t j = 0; j < predicted_object_history_.size(); ++j) {
      // check new collision points
      const auto & obj = predicted_object_history_.at(j).object;
      if (node_param_.enable_z_axis_obstacle_filtering) {
        if (!intersectsInZAxis(obj, z_axis_min, z_axis_max)) {
          continue;
        }
      }
      Point2d collision_point;
      collision_point.x() = predicted_object_history_.at(j).point.x;
      collision_point.y() = predicted_object_history_.at(j).point.y;
      Polygon2d one_step_move_vehicle_polygon;
      // create one step polygon for vehicle
      if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
        createOneStepPolygon(
          p_front, p_back, one_step_move_vehicle_polygon, vehicle_info,
          stop_param.pedestrian_lateral_margin);

      } else if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
        createOneStepPolygon(
          p_front, p_back, one_step_move_vehicle_polygon, vehicle_info,
          stop_param.vehicle_lateral_margin);

      } else if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
        createOneStepPolygon(
          p_front, p_back, one_step_move_vehicle_polygon, vehicle_info,
          stop_param.unknown_lateral_margin);

      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000, "Object type is not supported. type: %d",
          obj.shape.type);
        continue;
      }
      if (bg::within(collision_point, one_step_move_vehicle_polygon)) {
        const double norm = calcDistance2d(predicted_object_history_.at(j).point, p_front.position);
        if (norm < min_collision_norm || !is_init) {
          min_collision_norm = norm;
          is_init = true;
          nearest_collision_object_index = j;
        }
      }
    }

    planner_data.found_collision_points = is_init;

    if (planner_data.found_collision_points) {
      planner_data.nearest_collision_point = pointToPcl(
        predicted_object_history_.at(nearest_collision_object_index).point.x,
        predicted_object_history_.at(nearest_collision_object_index).point.y, p_front.position.z);

      planner_data.decimate_trajectory_collision_index = i;

      planner_data.nearest_collision_point_time =
        predicted_object_history_.at(nearest_collision_object_index).detection_time;

      // create one step polygon for vehicle collision debug
      Polygon2d one_step_move_vehicle_polygon;
      Polygon2d object_polygon{};

      const auto & obj = predicted_object_history_.at(nearest_collision_object_index).object;
      if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
        createOneStepPolygon(
          p_front, p_back, one_step_move_vehicle_polygon, vehicle_info,
          stop_param.pedestrian_lateral_margin);
        object_polygon = convertCylindricalObjectToGeometryPolygon(
          obj.kinematics.initial_pose_with_covariance.pose, obj.shape);
      } else if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
        createOneStepPolygon(
          p_front, p_back, one_step_move_vehicle_polygon, vehicle_info,
          stop_param.vehicle_lateral_margin);
        const double & length_m = obj.shape.dimensions.x / 2;
        const double & width_m = obj.shape.dimensions.y / 2;
        object_polygon = convertBoundingBoxObjectToGeometryPolygon(
          obj.kinematics.initial_pose_with_covariance.pose, length_m, length_m, width_m);

      } else if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
        createOneStepPolygon(
          p_front, p_back, one_step_move_vehicle_polygon, vehicle_info,
          stop_param.unknown_lateral_margin);
        object_polygon = convertPolygonObjectToGeometryPolygon(
          obj.kinematics.initial_pose_with_covariance.pose, obj.shape);
      }
      debug_ptr_->pushObstaclePoint(planner_data.nearest_collision_point, PointType::Stop);

      if (node_param_.enable_z_axis_obstacle_filtering) {
        debug_ptr_->pushPolyhedron(
          one_step_move_vehicle_polygon, z_axis_min, z_axis_max, PolygonType::Collision);
      } else {
        debug_ptr_->pushPolygon(
          one_step_move_vehicle_polygon, p_front.position.z, PolygonType::Collision);
      }

      if (node_param_.publish_obstacle_polygon) {
        debug_ptr_->pushPolygon(object_polygon, p_front.position.z, PolygonType::Obstacle);
      }

      planner_data.stop_require = planner_data.found_collision_points;
      mutex_.lock();
      const auto current_odometry_ptr = current_odometry_ptr_;
      const auto latest_object_ptr = object_ptr_;
      mutex_.unlock();
      // find latest state of predicted object to get latest velocity and acceleration values
      auto obj_latest_state = getObstacleFromUuid(*latest_object_ptr, obj.object_id);
      if (!obj_latest_state) {
        // Can not find the object in the latest object list. Send previous state.
        obj_latest_state = obj;
      }

      acc_controller_->insertAdaptiveCruiseVelocity(
        decimate_trajectory, planner_data.decimate_trajectory_collision_index,
        planner_data.current_pose, planner_data.nearest_collision_point,
        planner_data.nearest_collision_point_time, current_odometry_ptr, &planner_data.stop_require,
        &output, trajectory_header, *obj_latest_state);

      if (!planner_data.stop_require) {
        predicted_object_history_.clear();
      }
      break;
    }
  }
}

void ObstacleStopPlannerNode::insertVelocity(
  TrajectoryPoints & output, PlannerData & planner_data,
  [[maybe_unused]] const Header & trajectory_header, const VehicleInfo & vehicle_info,
  const double current_acc, const double current_vel, const StopParam & stop_param)
{
  const auto & base_link2front = vehicle_info.max_longitudinal_offset_m;

  if (planner_data.stop_require) {
    // insert stop point
    const auto traj_end_idx = output.size() - 1;
    const auto idx = planner_data.decimate_trajectory_index_map.at(
                       planner_data.decimate_trajectory_collision_index) +
                     planner_data.trajectory_trim_index;
    std::optional<std::pair<size_t, double>> index_with_dist_remain;

    index_with_dist_remain = findNearestFrontIndex(
      std::min(idx, traj_end_idx), output,
      createPoint(
        planner_data.nearest_collision_point.x, planner_data.nearest_collision_point.y, 0));

    if (index_with_dist_remain) {
      const auto vehicle_idx = std::min(planner_data.trajectory_trim_index, traj_end_idx);
      const auto dist_baselink_to_obstacle =
        calcSignedArcLength(output, vehicle_idx, index_with_dist_remain.value().first);

      debug_ptr_->setDebugValues(
        DebugValues::TYPE::COLLISION_OBSTACLE_DISTANCE,
        dist_baselink_to_obstacle + index_with_dist_remain.value().second - base_link2front);

      const auto stop_point = searchInsertPoint(
        index_with_dist_remain.value().first, output, index_with_dist_remain.value().second,
        stop_param);

      const auto & ego_pose = planner_data.current_pose;
      const size_t ego_seg_idx = findFirstNearestIndexWithSoftConstraints(
        output, ego_pose, node_param_.ego_nearest_dist_threshold,
        node_param_.ego_nearest_yaw_threshold);

      const double stop_point_distance = [&]() {
        if (output.size() < 2) {
          return 0.0;
        }

        if (ego_seg_idx == output.size() - 1) {
          return 0.0;
        }

        size_t stop_seg_idx = 0;
        if (stop_point.index < output.size() - 1) {
          const double lon_offset =
            calcLongitudinalOffsetToSegment(output, stop_point.index, getPoint(stop_point.point));
          if (lon_offset < 0) {
            stop_seg_idx = std::max(static_cast<size_t>(0), stop_point.index - 1);
          } else {
            stop_seg_idx = std::min(output.size() - 2, stop_point.index);
          }
        } else {
          stop_seg_idx = output.size() - 2;
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
          current_stop_pos.point.pose = ego_pos_on_path.value();

          insertStopPoint(current_stop_pos, output, planner_data.stop_reason_diag);

          debug_ptr_->pushPose(getPose(stop_point.point), PoseType::TargetStop);
          debug_ptr_->pushPose(getPose(current_stop_pos.point), PoseType::Stop);
        }

      } else {
        insertStopPoint(stop_point, output, planner_data.stop_reason_diag);

        debug_ptr_->pushPose(getPose(stop_point.point), PoseType::TargetStop);
        debug_ptr_->pushPose(getPose(stop_point.point), PoseType::Stop);
      }
    }
  }

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
        calcSignedArcLength(output, vehicle_idx, index_with_dist_remain.value().first);

      debug_ptr_->setDebugValues(
        DebugValues::TYPE::SLOWDOWN_OBSTACLE_DISTANCE,
        dist_baselink_to_obstacle + index_with_dist_remain.value().second - base_link2front);
      const auto slow_down_section = createSlowDownSection(
        index_with_dist_remain.value().first, output, planner_data.lateral_deviation,
        index_with_dist_remain.value().second, dist_baselink_to_obstacle, vehicle_info, current_acc,
        current_vel);

      if (
        !latest_slow_down_section_ &&
        dist_baselink_to_obstacle + index_with_dist_remain.value().second <
          vehicle_info.max_longitudinal_offset_m) {
        latest_slow_down_section_ = slow_down_section;
      }

      insertSlowDownSection(slow_down_section, output);
    }
  }

  if (node_param_.enable_slow_down && latest_slow_down_section_) {
    // check whether ego is in slow down section or not
    const auto & p_start = latest_slow_down_section_.value().start_point.pose.position;
    const auto & p_end = latest_slow_down_section_.value().end_point.pose.position;
    const auto reach_slow_down_start_point =
      isInFrontOfTargetPoint(planner_data.current_pose, p_start);
    const auto reach_slow_down_end_point = isInFrontOfTargetPoint(planner_data.current_pose, p_end);
    const auto is_in_slow_down_section = reach_slow_down_start_point && !reach_slow_down_end_point;
    const auto index_with_dist_remain = findNearestFrontIndex(0, output, p_end);

    if (is_in_slow_down_section && index_with_dist_remain) {
      const auto end_insert_point_with_idx = getBackwardInsertPointFromBasePoint(
        index_with_dist_remain.value().first, output, -index_with_dist_remain.value().second);

      double slow_down_velocity;
      if (node_param_.use_predicted_objects) {
        if (
          planner_data.slow_down_object_shape.type ==
          autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
          slow_down_velocity =
            slow_down_param_.min_slow_down_velocity +
            (slow_down_param_.max_slow_down_velocity - slow_down_param_.min_slow_down_velocity) *
              std::max(planner_data.lateral_deviation - vehicle_info.vehicle_width_m / 2, 0.0) /
              slow_down_param_.pedestrian_lateral_margin;
        } else if (
          planner_data.slow_down_object_shape.type ==
          autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
          slow_down_velocity =
            slow_down_param_.min_slow_down_velocity +
            (slow_down_param_.max_slow_down_velocity - slow_down_param_.min_slow_down_velocity) *
              std::max(planner_data.lateral_deviation - vehicle_info.vehicle_width_m / 2, 0.0) /
              slow_down_param_.vehicle_lateral_margin;
        } else if (
          planner_data.slow_down_object_shape.type ==
          autoware_auto_perception_msgs::msg::Shape::POLYGON) {
          slow_down_velocity =
            slow_down_param_.min_slow_down_velocity +
            (slow_down_param_.max_slow_down_velocity - slow_down_param_.min_slow_down_velocity) *
              std::max(planner_data.lateral_deviation - vehicle_info.vehicle_width_m / 2, 0.0) /
              slow_down_param_.unknown_lateral_margin;
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000, "Object type is not supported. type: %d",
            planner_data.slow_down_object_shape.type);
        }
      } else {
        slow_down_velocity =
          slow_down_param_.min_slow_down_velocity +
          (slow_down_param_.max_slow_down_velocity - slow_down_param_.min_slow_down_velocity) *
            std::max(planner_data.lateral_deviation - vehicle_info.vehicle_width_m / 2, 0.0) /
            slow_down_param_.lateral_margin;
      }

      const auto target_velocity = slow_down_param_.consider_constraints
                                     ? slow_down_param_.slow_down_velocity
                                     : slow_down_velocity;

      SlowDownSection slow_down_section{};
      slow_down_section.slow_down_start_idx = 0;
      slow_down_section.start_point = output.front();
      slow_down_section.slow_down_end_idx = end_insert_point_with_idx.value().first;
      slow_down_section.end_point = end_insert_point_with_idx.value().second;
      slow_down_section.velocity =
        set_velocity_limit_ ? std::numeric_limits<double>::max() : target_velocity;

      insertSlowDownSection(slow_down_section, output);
    } else {
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
  const bool is_behind_goal = static_cast<size_t>(idx) == base_trajectory.size() - 1;
  const double max_longitudinal_margin = is_behind_goal
                                           ? stop_param.max_longitudinal_margin_behind_goal
                                           : stop_param.max_longitudinal_margin;

  const auto max_dist_stop_point =
    createTargetPoint(idx, max_longitudinal_margin, base_trajectory, dist_remain);
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
  stop_point.index = insert_point_with_idx.value().first;
  stop_point.point = insert_point_with_idx.value().second;

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

    const auto relax_target_vel = margin_with_vel.has_value();
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
                         : margin_with_vel.value().first - dist_remain;
    const auto update_backward_margin_from_vehicle =
      slow_down_param_.longitudinal_backward_margin + dist_remain;

    const auto velocity =
      use_velocity_limit ? std::numeric_limits<double>::max() : margin_with_vel.value().second;

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
  slow_down_section.slow_down_start_idx = start_insert_point_with_idx.value().first;
  slow_down_section.start_point = start_insert_point_with_idx.value().second;
  slow_down_section.slow_down_end_idx = end_insert_point_with_idx.value().first;
  slow_down_section.end_point = end_insert_point_with_idx.value().second;
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
  current_odometry_ptr_ = input_msg;
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
  for (const auto & trajectory_point : trajectory) {
    center_points.push_back(getVehicleCenterFromBase(trajectory_point.pose, vehicle_info).position);
  }
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

void ObstacleStopPlannerNode::filterObstacles(
  const PredictedObjects & input_objects, const Pose & ego_pose, const TrajectoryPoints & traj,
  const double dist_threshold, PredictedObjects & filtered_objects)
{
  filtered_objects.header = input_objects.header;

  for (auto & object : input_objects.objects) {
    // Check is it in front of ego vehicle
    if (!isFrontObstacle(ego_pose, object.kinematics.initial_pose_with_covariance.pose.position)) {
      continue;
    }

    // Check is it near to trajectory
    const double max_length = calcObstacleMaxLength(object.shape);
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
    filtered_objects.objects.push_back(filtered_object);
  }
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
