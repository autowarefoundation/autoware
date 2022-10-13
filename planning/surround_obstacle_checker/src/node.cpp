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

#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace surround_obstacle_checker
{
namespace bg = boost::geometry;
using Point2d = bg::model::d2::point_xy<double>;
using Polygon2d = bg::model::polygon<Point2d>;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::pose2transform;

namespace
{
std::string jsonDumpsPose(const geometry_msgs::msg::Pose & pose)
{
  const std::string json_dumps_pose =
    (boost::format(
       R"({"position":{"x":%lf,"y":%lf,"z":%lf},"orientation":{"w":%lf,"x":%lf,"y":%lf,"z":%lf}})") %
     pose.position.x % pose.position.y % pose.position.z % pose.orientation.w % pose.orientation.x %
     pose.orientation.y % pose.orientation.z)
      .str();
  return json_dumps_pose;
}

diagnostic_msgs::msg::DiagnosticStatus makeStopReasonDiag(
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

geometry_msgs::msg::Point32 createPoint32(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

Polygon2d createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Polygon & footprint)
{
  geometry_msgs::msg::Polygon transformed_polygon{};
  geometry_msgs::msg::TransformStamped geometry_tf{};
  geometry_tf.transform = pose2transform(pose);
  tf2::doTransform(footprint, transformed_polygon, geometry_tf);

  Polygon2d object_polygon;
  for (const auto & p : transformed_polygon.points) {
    object_polygon.outer().push_back(Point2d(p.x, p.y));
  }

  bg::correct(object_polygon);

  return object_polygon;
}

Polygon2d createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size)
{
  const double & length_m = size.x / 2.0;
  const double & width_m = size.y / 2.0;

  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(length_m, -width_m, 0.0));
  polygon.points.push_back(createPoint32(length_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-length_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-length_m, -width_m, 0.0));

  return createObjPolygon(pose, polygon);
}

Polygon2d createSelfPolygon(const VehicleInfo & vehicle_info)
{
  const double & front_m = vehicle_info.max_longitudinal_offset_m;
  const double & width_left_m = vehicle_info.max_lateral_offset_m;
  const double & width_right_m = vehicle_info.min_lateral_offset_m;
  const double & rear_m = vehicle_info.min_longitudinal_offset_m;

  Polygon2d ego_polygon;

  ego_polygon.outer().push_back(Point2d(front_m, width_left_m));
  ego_polygon.outer().push_back(Point2d(front_m, width_right_m));
  ego_polygon.outer().push_back(Point2d(rear_m, width_right_m));
  ego_polygon.outer().push_back(Point2d(rear_m, width_left_m));

  bg::correct(ego_polygon);

  return ego_polygon;
}
}  // namespace

SurroundObstacleCheckerNode::SurroundObstacleCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("surround_obstacle_checker_node", node_options)
{
  // Parameters
  {
    auto & p = node_param_;
    p.use_pointcloud = this->declare_parameter("use_pointcloud", true);
    p.use_dynamic_object = this->declare_parameter("use_dynamic_object", true);
    p.surround_check_distance = this->declare_parameter("surround_check_distance", 2.0);
    p.surround_check_recover_distance =
      this->declare_parameter("surround_check_recover_distance", 2.5);
    p.state_clear_time = this->declare_parameter("state_clear_time", 2.0);
    p.publish_debug_footprints = this->declare_parameter("publish_debug_footprints", true);
  }

  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  // Publishers
  pub_stop_reason_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/output/no_start_reason", 1);
  pub_clear_velocity_limit_ = this->create_publisher<VelocityLimitClearCommand>(
    "~/output/velocity_limit_clear_command", rclcpp::QoS{1}.transient_local());
  pub_velocity_limit_ = this->create_publisher<VelocityLimit>(
    "~/output/max_velocity", rclcpp::QoS{1}.transient_local());

  // Subscribers
  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&SurroundObstacleCheckerNode::onPointCloud, this, std::placeholders::_1));
  sub_dynamic_objects_ = this->create_subscription<PredictedObjects>(
    "~/input/objects", 1,
    std::bind(&SurroundObstacleCheckerNode::onDynamicObjects, this, std::placeholders::_1));
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1,
    std::bind(&SurroundObstacleCheckerNode::onOdometry, this, std::placeholders::_1));

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&SurroundObstacleCheckerNode::onTimer, this));

  // Stop Checker
  vehicle_stop_checker_ = std::make_unique<VehicleStopChecker>(this);

  // Debug
  auto const self_polygon = createSelfPolygon(vehicle_info_);
  odometry_ptr_ = std::make_shared<nav_msgs::msg::Odometry>();

  debug_ptr_ = std::make_shared<SurroundObstacleCheckerDebugNode>(
    self_polygon, vehicle_info_.max_longitudinal_offset_m, node_param_.surround_check_distance,
    node_param_.surround_check_recover_distance, odometry_ptr_->pose.pose, this->get_clock(),
    *this);
}

void SurroundObstacleCheckerNode::onTimer()
{
  if (!odometry_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current velocity...");
    return;
  }

  if (node_param_.publish_debug_footprints) {
    debug_ptr_->publishFootprints();
  }

  if (node_param_.use_pointcloud && !pointcloud_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for pointcloud info...");
    return;
  }

  if (node_param_.use_dynamic_object && !object_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for dynamic object info...");
    return;
  }

  const auto nearest_obstacle = getNearestObstacle();
  const auto is_vehicle_stopped = vehicle_stop_checker_->isVehicleStopped();

  switch (state_) {
    case State::PASS: {
      const auto is_obstacle_found =
        !nearest_obstacle ? false
                          : nearest_obstacle.get().first < node_param_.surround_check_distance;

      if (!isStopRequired(is_obstacle_found, is_vehicle_stopped)) {
        break;
      }

      state_ = State::STOP;

      auto velocity_limit = std::make_shared<VelocityLimit>();
      velocity_limit->stamp = this->now();
      velocity_limit->max_velocity = 0.0;
      velocity_limit->use_constraints = false;
      velocity_limit->sender = "surround_obstacle_checker";

      pub_velocity_limit_->publish(*velocity_limit);

      // do not start when there is a obstacle near the ego vehicle.
      RCLCPP_WARN(get_logger(), "do not start because there is obstacle near the ego vehicle.");

      break;
    }

    case State::STOP: {
      const auto is_obstacle_found =
        !nearest_obstacle
          ? false
          : nearest_obstacle.get().first < node_param_.surround_check_recover_distance;

      if (isStopRequired(is_obstacle_found, is_vehicle_stopped)) {
        break;
      }

      state_ = State::PASS;

      auto velocity_limit_clear_command = std::make_shared<VelocityLimitClearCommand>();
      velocity_limit_clear_command->stamp = this->now();
      velocity_limit_clear_command->command = true;
      velocity_limit_clear_command->sender = "surround_obstacle_checker";

      pub_clear_velocity_limit_->publish(*velocity_limit_clear_command);

      break;
    }

    default:
      break;
  }

  if (nearest_obstacle) {
    debug_ptr_->pushObstaclePoint(nearest_obstacle.get().second, PointType::NoStart);
  }

  diagnostic_msgs::msg::DiagnosticStatus no_start_reason_diag;
  if (state_ == State::STOP) {
    debug_ptr_->pushPose(odometry_ptr_->pose.pose, PoseType::NoStart);
    no_start_reason_diag = makeStopReasonDiag("obstacle", odometry_ptr_->pose.pose);
  }

  pub_stop_reason_->publish(no_start_reason_diag);
  debug_ptr_->publish();
}

void SurroundObstacleCheckerNode::onPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pointcloud_ptr_ = msg;
}

void SurroundObstacleCheckerNode::onDynamicObjects(const PredictedObjects::ConstSharedPtr msg)
{
  object_ptr_ = msg;
}

void SurroundObstacleCheckerNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odometry_ptr_ = msg;
}

boost::optional<Obstacle> SurroundObstacleCheckerNode::getNearestObstacle() const
{
  boost::optional<Obstacle> nearest_pointcloud{boost::none};
  boost::optional<Obstacle> nearest_object{boost::none};

  if (node_param_.use_pointcloud) {
    nearest_pointcloud = getNearestObstacleByPointCloud();
  }

  if (node_param_.use_dynamic_object) {
    nearest_object = getNearestObstacleByDynamicObject();
  }

  if (!nearest_pointcloud && !nearest_object) {
    return {};
  }

  if (!nearest_pointcloud) {
    return nearest_object;
  }

  if (!nearest_object) {
    return nearest_pointcloud;
  }

  return nearest_pointcloud.get().first < nearest_object.get().first ? nearest_pointcloud
                                                                     : nearest_object;
}

boost::optional<Obstacle> SurroundObstacleCheckerNode::getNearestObstacleByPointCloud() const
{
  const auto transform_stamped =
    getTransform("base_link", pointcloud_ptr_->header.frame_id, pointcloud_ptr_->header.stamp, 0.5);

  geometry_msgs::msg::Point nearest_point;
  auto minimum_distance = std::numeric_limits<double>::max();

  if (!transform_stamped) {
    return {};
  }

  Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.get().transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(*pointcloud_ptr_, transformed_pointcloud);
  pcl::transformPointCloud(transformed_pointcloud, transformed_pointcloud, isometry);

  const auto ego_polygon = createSelfPolygon(vehicle_info_);

  for (const auto & p : transformed_pointcloud) {
    Point2d boost_point(p.x, p.y);

    const auto distance_to_object = bg::distance(ego_polygon, boost_point);

    if (distance_to_object < minimum_distance) {
      nearest_point = createPoint(p.x, p.y, p.z);
      minimum_distance = distance_to_object;
    }
  }

  return std::make_pair(minimum_distance, nearest_point);
}

boost::optional<Obstacle> SurroundObstacleCheckerNode::getNearestObstacleByDynamicObject() const
{
  const auto transform_stamped =
    getTransform(object_ptr_->header.frame_id, "base_link", object_ptr_->header.stamp, 0.5);

  geometry_msgs::msg::Point nearest_point;
  auto minimum_distance = std::numeric_limits<double>::max();

  if (!transform_stamped) {
    return {};
  }

  tf2::Transform tf_src2target;
  tf2::fromMsg(transform_stamped.get().transform, tf_src2target);

  const auto ego_polygon = createSelfPolygon(vehicle_info_);

  for (const auto & object : object_ptr_->objects) {
    const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;

    tf2::Transform tf_src2object;
    tf2::fromMsg(object_pose, tf_src2object);

    geometry_msgs::msg::Pose transformed_object_pose;
    tf2::toMsg(tf_src2target.inverse() * tf_src2object, transformed_object_pose);

    const auto object_polygon =
      object.shape.type == Shape::POLYGON
        ? createObjPolygon(transformed_object_pose, object.shape.footprint)
        : createObjPolygon(transformed_object_pose, object.shape.dimensions);

    const auto distance_to_object = bg::distance(ego_polygon, object_polygon);

    if (distance_to_object < minimum_distance) {
      nearest_point = object_pose.position;
      minimum_distance = distance_to_object;
    }
  }

  return std::make_pair(minimum_distance, nearest_point);
}

boost::optional<geometry_msgs::msg::TransformStamped> SurroundObstacleCheckerNode::getTransform(
  const std::string & source, const std::string & target, const rclcpp::Time & stamp,
  double duration_sec) const
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped =
      tf_buffer_.lookupTransform(source, target, stamp, tf2::durationFromSec(duration_sec));
  } catch (tf2::TransformException & ex) {
    return {};
  }

  return transform_stamped;
}

bool SurroundObstacleCheckerNode::isStopRequired(
  const bool is_obstacle_found, const bool is_vehicle_stopped)
{
  if (!is_vehicle_stopped) {
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
    if (elapsed_time.seconds() <= node_param_.state_clear_time) {
      return true;
    }
  }

  last_obstacle_found_time_ = {};
  return false;
}

}  // namespace surround_obstacle_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(surround_obstacle_checker::SurroundObstacleCheckerNode)
