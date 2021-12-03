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

#include "surround_obstacle_checker/node.hpp"

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

SurroundObstacleCheckerNode::SurroundObstacleCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("surround_obstacle_checker_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  // Parameters
  use_pointcloud_ = this->declare_parameter("use_pointcloud", true);
  use_dynamic_object_ = this->declare_parameter("use_dynamic_object", true);
  surround_check_distance_ = this->declare_parameter("surround_check_distance", 2.0);
  surround_check_recover_distance_ =
    this->declare_parameter("surround_check_recover_distance", 2.5);
  state_clear_time_ = this->declare_parameter("state_clear_time", 2.0);
  stop_state_ego_speed_ = this->declare_parameter("stop_state_ego_speed", 0.1);
  debug_ptr_ = std::make_shared<SurroundObstacleCheckerDebugNode>(
    vehicle_info_.max_longitudinal_offset_m, this->get_clock(), *this);
  self_poly_ = createSelfPolygon();

  // Publishers
  path_pub_ =
    this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/output/trajectory", 1);
  stop_reason_diag_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/output/no_start_reason", 1);

  // Subscriber
  path_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/input/trajectory", 1,
    std::bind(&SurroundObstacleCheckerNode::pathCallback, this, std::placeholders::_1));
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&SurroundObstacleCheckerNode::pointCloudCallback, this, std::placeholders::_1));
  dynamic_object_sub_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "~/input/objects", 1,
      std::bind(&SurroundObstacleCheckerNode::dynamicObjectCallback, this, std::placeholders::_1));
  current_velocity_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1,
    std::bind(&SurroundObstacleCheckerNode::currentVelocityCallback, this, std::placeholders::_1));
}

void SurroundObstacleCheckerNode::pathCallback(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg)
{
  if (use_pointcloud_ && !pointcloud_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000 /* ms */, "waiting for pointcloud info...");
    return;
  }

  if (use_dynamic_object_ && !object_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000 /* ms */, "waiting for dynamic object info...");
    return;
  }

  if (!current_velocity_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000 /* ms */, "waiting for current velocity...");
    return;
  }

  // parameter description
  TrajectoryPoints output_trajectory_points =
    autoware_utils::convertToTrajectoryPointArray(*input_msg);

  diagnostic_msgs::msg::DiagnosticStatus no_start_reason_diag;

  // get current pose in traj frame
  geometry_msgs::msg::Pose current_pose;
  if (!getPose(input_msg->header.frame_id, "base_link", current_pose)) {
    return;
  }

  // get closest idx
  const size_t closest_idx = getClosestIdx(output_trajectory_points, current_pose);

  // get nearest object
  double min_dist_to_obj = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point nearest_obj_point;
  getNearestObstacle(&min_dist_to_obj, &nearest_obj_point);

  // check current obstacle status (exist or not)
  const auto is_obstacle_found = isObstacleFound(min_dist_to_obj);

  // check current stop status (stop or not)
  const auto is_stopped = checkStop(input_msg->points.at(closest_idx));

  const auto is_stop_required = isStopRequired(is_obstacle_found, is_stopped);

  // insert stop velocity
  if (is_stop_required) {
    state_ = State::STOP;

    // do not start when there is a obstacle near the ego vehicle.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 500 /* ms */,
      "do not start because there is obstacle near the ego vehicle.");
    insertStopVelocity(closest_idx, &output_trajectory_points);

    // visualization for debug
    if (is_obstacle_found) {
      debug_ptr_->pushObstaclePoint(nearest_obj_point, PointType::NoStart);
    }
    debug_ptr_->pushPose(input_msg->points.at(closest_idx).pose, PoseType::NoStart);
    no_start_reason_diag = makeStopReasonDiag("obstacle", input_msg->points.at(closest_idx).pose);
  } else {
    state_ = State::PASS;
  }

  // publish trajectory and debug info
  auto output_msg = autoware_utils::convertToTrajectory(output_trajectory_points);
  output_msg.header = input_msg->header;
  path_pub_->publish(output_msg);
  stop_reason_diag_pub_->publish(no_start_reason_diag);
  debug_ptr_->publish();
}

void SurroundObstacleCheckerNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  pointcloud_ptr_ = input_msg;
}

void SurroundObstacleCheckerNode::dynamicObjectCallback(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr input_msg)
{
  object_ptr_ = input_msg;
}

void SurroundObstacleCheckerNode::currentVelocityCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr input_msg)
{
  current_velocity_ptr_ = input_msg;
}

void SurroundObstacleCheckerNode::insertStopVelocity(
  const size_t closest_idx, TrajectoryPoints * traj)
{
  // set zero velocity from closest idx to last idx
  for (size_t i = closest_idx; i < traj->size(); i++) {
    traj->at(i).longitudinal_velocity_mps = 0.0;
  }
}

bool SurroundObstacleCheckerNode::getPose(
  const std::string & source, const std::string & target, geometry_msgs::msg::Pose & pose)
{
  try {
    // get transform from source to target
    geometry_msgs::msg::TransformStamped ros_src2tgt =
      tf_buffer_.lookupTransform(source, target, tf2::TimePointZero);
    // convert geometry_msgs::msg::Transform to geometry_msgs::msg::Pose
    tf2::Transform transform;
    tf2::fromMsg(ros_src2tgt.transform, transform);
    tf2::toMsg(transform, pose);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), 500 /* ms */,
      "cannot get tf from " << source << " to " << target);
    return false;
  }
  return true;
}

bool SurroundObstacleCheckerNode::convertPose(
  const geometry_msgs::msg::Pose & pose, const std::string & source, const std::string & target,
  const rclcpp::Time & time, geometry_msgs::msg::Pose & conv_pose)
{
  tf2::Transform src2tgt;
  try {
    // get transform from source to target
    geometry_msgs::msg::TransformStamped ros_src2tgt =
      tf_buffer_.lookupTransform(source, target, time, tf2::durationFromSec(0.1));
    tf2::fromMsg(ros_src2tgt.transform, src2tgt);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), 500 /* ms */,
      "cannot get tf from " << source << " to " << target);
    return false;
  }

  tf2::Transform src2obj;
  tf2::fromMsg(pose, src2obj);
  tf2::Transform tgt2obj = src2tgt.inverse() * src2obj;
  tf2::toMsg(tgt2obj, conv_pose);
  return true;
}

size_t SurroundObstacleCheckerNode::getClosestIdx(
  const TrajectoryPoints & traj, const geometry_msgs::msg::Pose current_pose)
{
  double min_dist = std::numeric_limits<double>::max();
  size_t min_dist_idx = 0;
  for (size_t i = 0; i < traj.size(); ++i) {
    const double x = traj.at(i).pose.position.x - current_pose.position.x;
    const double y = traj.at(i).pose.position.y - current_pose.position.y;
    const double dist = std::hypot(x, y);
    if (dist < min_dist) {
      min_dist_idx = i;
      min_dist = dist;
    }
  }
  return min_dist_idx;
}

void SurroundObstacleCheckerNode::getNearestObstacle(
  double * min_dist_to_obj, geometry_msgs::msg::Point * nearest_obj_point)
{
  if (use_pointcloud_) {
    getNearestObstacleByPointCloud(min_dist_to_obj, nearest_obj_point);
  }

  if (use_dynamic_object_) {
    getNearestObstacleByDynamicObject(min_dist_to_obj, nearest_obj_point);
  }
}

void SurroundObstacleCheckerNode::getNearestObstacleByPointCloud(
  double * min_dist_to_obj, geometry_msgs::msg::Point * nearest_obj_point)
{
  // wait to transform pointcloud
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      "base_link", pointcloud_ptr_->header.frame_id, pointcloud_ptr_->header.stamp,
      tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), 500 /* ms */,
      "failed to get base_link to " << pointcloud_ptr_->header.frame_id << " transform.");
    return;
  }

  Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ> pcl;
  pcl::fromROSMsg(*pointcloud_ptr_, pcl);
  pcl::transformPointCloud(pcl, pcl, isometry);
  for (const auto & p : pcl) {
    // create boost point
    Point2d boost_p(p.x, p.y);

    // calc distance
    const double dist_to_obj = boost::geometry::distance(self_poly_, boost_p);

    // get minimum distance to obj
    if (dist_to_obj < *min_dist_to_obj) {
      *min_dist_to_obj = dist_to_obj;
      nearest_obj_point->x = p.x;
      nearest_obj_point->y = p.y;
      nearest_obj_point->z = p.z;
    }
  }
}

void SurroundObstacleCheckerNode::getNearestObstacleByDynamicObject(
  double * min_dist_to_obj, geometry_msgs::msg::Point * nearest_obj_point)
{
  const auto obj_frame = object_ptr_->header.frame_id;
  const auto obj_time = object_ptr_->header.stamp;
  for (const auto & obj : object_ptr_->objects) {
    // change frame of obj_pose to base_link
    geometry_msgs::msg::Pose pose_baselink;
    if (!convertPose(
          obj.kinematics.initial_pose_with_covariance.pose, obj_frame, "base_link", obj_time,
          pose_baselink)) {
      return;
    }

    // create obj polygon
    Polygon2d obj_poly;
    if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
      obj_poly = createObjPolygon(pose_baselink, obj.shape.footprint);
    } else {
      obj_poly = createObjPolygon(pose_baselink, obj.shape.dimensions);
    }

    // calc distance
    const double dist_to_obj = boost::geometry::distance(self_poly_, obj_poly);

    // get minimum distance to obj
    if (dist_to_obj < *min_dist_to_obj) {
      *min_dist_to_obj = dist_to_obj;
      *nearest_obj_point = obj.kinematics.initial_pose_with_covariance.pose.position;
    }
  }
}

bool SurroundObstacleCheckerNode::isObstacleFound(const double min_dist_to_obj)
{
  const auto is_obstacle_inside_range = min_dist_to_obj < surround_check_distance_;
  const auto is_obstacle_outside_range = min_dist_to_obj > surround_check_recover_distance_;

  if (state_ == State::PASS) {
    return is_obstacle_inside_range;
  }

  if (state_ == State::STOP) {
    return !is_obstacle_outside_range;
  }

  throw std::runtime_error("invalid state");
}

bool SurroundObstacleCheckerNode::isStopRequired(
  const bool is_obstacle_found, const bool is_stopped)
{
  if (!is_stopped) {
    return false;
  }

  if (is_obstacle_found) {
    last_obstacle_found_time_ = std::make_shared<const rclcpp::Time>(this->now());
    return true;
  }

  if (state_ != State::STOP) {
    return false;
  }

  // Keep stop state
  if (last_obstacle_found_time_) {
    const auto elapsed_time = this->now() - *last_obstacle_found_time_;
    if (elapsed_time.seconds() <= state_clear_time_) {
      return true;
    }
  }

  last_obstacle_found_time_ = {};
  return false;
}

bool SurroundObstacleCheckerNode::checkStop(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & closest_point)
{
  if (std::fabs(current_velocity_ptr_->twist.twist.linear.x) > stop_state_ego_speed_) {
    // ego vehicle has high velocity now. not stop.
    return false;
  }

  const double closest_vel = closest_point.longitudinal_velocity_mps;
  const double closest_acc = closest_point.acceleration_mps2;

  if (closest_vel * closest_acc < 0) {
    // ego vehicle is about to stop (during deceleration). not stop.
    return false;
  }

  return true;
}

Polygon2d SurroundObstacleCheckerNode::createSelfPolygon()
{
  const double front = vehicle_info_.max_longitudinal_offset_m;
  const double rear = vehicle_info_.min_longitudinal_offset_m;
  const double left = vehicle_info_.max_lateral_offset_m;
  const double right = vehicle_info_.min_lateral_offset_m;

  Polygon2d poly;
  boost::geometry::exterior_ring(poly) = boost::assign::list_of<Point2d>(front, left)(front, right)(
    rear, right)(rear, left)(front, left);
  return poly;
}

Polygon2d SurroundObstacleCheckerNode::createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size)
{
  // rename
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double h = size.x;
  const double w = size.y;
  const double yaw = tf2::getYaw(pose.orientation);

  // create base polygon
  Polygon2d obj_poly;
  boost::geometry::exterior_ring(obj_poly) = boost::assign::list_of<Point2d>(h / 2.0, w / 2.0)(
    -h / 2.0, w / 2.0)(-h / 2.0, -w / 2.0)(h / 2.0, -w / 2.0)(h / 2.0, w / 2.0);

  // rotate polygon(yaw)
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>
    rotate(-yaw);  // anti-clockwise -> :clockwise rotation
  Polygon2d rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);

  // translate polygon(x, y)
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(x, y);
  Polygon2d translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

Polygon2d SurroundObstacleCheckerNode::createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Polygon & footprint)
{
  // rename
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double yaw = tf2::getYaw(pose.orientation);

  // create base polygon
  Polygon2d obj_poly;
  for (const auto point : footprint.points) {
    const Point2d point2d(point.x, point.y);
    obj_poly.outer().push_back(point2d);
  }

  // rotate polygon(yaw)
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>
    rotate(-yaw);  // anti-clockwise -> :clockwise rotation
  Polygon2d rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);

  // translate polygon(x, y)
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(x, y);
  Polygon2d translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

diagnostic_msgs::msg::DiagnosticStatus SurroundObstacleCheckerNode::makeStopReasonDiag(
  const std::string no_start_reason, const geometry_msgs::msg::Pose & stop_pose)
{
  diagnostic_msgs::msg::DiagnosticStatus no_start_reason_diag;
  diagnostic_msgs::msg::KeyValue no_start_reason_diag_kv;
  no_start_reason_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  no_start_reason_diag.name = "no_start_reason";
  no_start_reason_diag.message = no_start_reason;
  no_start_reason_diag_kv.key = "no_start_pose";
  no_start_reason_diag_kv.value = jsonDumpsPose(stop_pose);
  no_start_reason_diag.values.push_back(no_start_reason_diag_kv);
  return no_start_reason_diag;
}

std::string SurroundObstacleCheckerNode::jsonDumpsPose(const geometry_msgs::msg::Pose & pose)
{
  const std::string json_dumps_pose =
    (boost::format(
       R"({"position":{"x":%lf,"y":%lf,"z":%lf},"orientation":{"w":%lf,"x":%lf,"y":%lf,"z":%lf}})") %
     pose.position.x % pose.position.y % pose.position.z % pose.orientation.w % pose.orientation.x %
     pose.orientation.y % pose.orientation.z)
      .str();
  return json_dumps_pose;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SurroundObstacleCheckerNode)
