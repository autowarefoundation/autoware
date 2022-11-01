// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include "obstacle_collision_checker/obstacle_collision_checker_node.hpp"

#include "obstacle_collision_checker/util/create_vehicle_footprint.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}
}  // namespace

namespace obstacle_collision_checker
{
ObstacleCollisionCheckerNode::ObstacleCollisionCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_collision_checker_node", node_options), updater_(this)
{
  using std::placeholders::_1;

  // Node Parameter
  node_param_.update_rate = declare_parameter("update_rate", 10.0);

  // Core Parameter
  param_.delay_time = declare_parameter("delay_time", 0.3);
  param_.footprint_margin = declare_parameter("footprint_margin", 0.0);
  param_.max_deceleration = declare_parameter("max_deceleration", 2.0);
  param_.resample_interval = declare_parameter("resample_interval", 0.5);
  param_.search_radius = declare_parameter("search_radius", 5.0);

  // Dynamic Reconfigure
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleCollisionCheckerNode::paramCallback, this, _1));

  // Core
  obstacle_collision_checker_ = std::make_unique<ObstacleCollisionChecker>(*this);
  obstacle_collision_checker_->setParam(param_);

  // Subscriber
  self_pose_listener_ = std::make_shared<tier4_autoware_utils::SelfPoseListener>(this);
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  sub_obstacle_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/obstacle_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ObstacleCollisionCheckerNode::onObstaclePointcloud, this, _1));
  sub_reference_trajectory_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "input/reference_trajectory", 1,
    std::bind(&ObstacleCollisionCheckerNode::onReferenceTrajectory, this, _1));
  sub_predicted_trajectory_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "input/predicted_trajectory", 1,
    std::bind(&ObstacleCollisionCheckerNode::onPredictedTrajectory, this, _1));
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "input/odometry", 1, std::bind(&ObstacleCollisionCheckerNode::onOdom, this, _1));

  // Publisher
  debug_publisher_ = std::make_shared<tier4_autoware_utils::DebugPublisher>(this, "debug/marker");
  time_publisher_ = std::make_shared<tier4_autoware_utils::ProcessingTimePublisher>(this);

  // Diagnostic Updater
  updater_.setHardwareID("obstacle_collision_checker");

  updater_.add(
    "obstacle_collision_checker", this, &ObstacleCollisionCheckerNode::checkLaneDeparture);

  // Wait for first self pose
  self_pose_listener_->waitForFirstPose();

  // Timer
  initTimer(1.0 / node_param_.update_rate);
}

void ObstacleCollisionCheckerNode::onObstaclePointcloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  obstacle_pointcloud_ = msg;
}

void ObstacleCollisionCheckerNode::onReferenceTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  reference_trajectory_ = msg;
}

void ObstacleCollisionCheckerNode::onPredictedTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  predicted_trajectory_ = msg;
}

void ObstacleCollisionCheckerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_twist_ = std::make_shared<geometry_msgs::msg::Twist>(msg->twist.twist);
}

void ObstacleCollisionCheckerNode::initTimer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ObstacleCollisionCheckerNode::onTimer, this));
}

bool ObstacleCollisionCheckerNode::isDataReady()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current_pose...");
    return false;
  }

  if (!obstacle_pointcloud_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "waiting for obstacle_pointcloud msg...");
    return false;
  }

  if (!obstacle_transform_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for obstacle_transform...");
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

  return true;
}

bool ObstacleCollisionCheckerNode::isDataTimeout()
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

void ObstacleCollisionCheckerNode::onTimer()
{
  current_pose_ = self_pose_listener_->getCurrentPose();
  if (obstacle_pointcloud_) {
    const auto & header = obstacle_pointcloud_->header;
    try {
      obstacle_transform_ = transform_listener_->getTransform(
        "map", header.frame_id, header.stamp, rclcpp::Duration::from_seconds(0.01));
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform map to %s: %s", header.frame_id.c_str(),
        ex.what());
      return;
    }
  }

  if (!isDataReady()) {
    return;
  }

  if (isDataTimeout()) {
    return;
  }

  input_.current_pose = current_pose_;
  input_.obstacle_pointcloud = obstacle_pointcloud_;
  input_.obstacle_transform = obstacle_transform_;
  input_.reference_trajectory = reference_trajectory_;
  input_.predicted_trajectory = predicted_trajectory_;
  input_.current_twist = current_twist_;

  output_ = obstacle_collision_checker_->update(input_);

  updater_.force_update();

  debug_publisher_->publish("marker_array", createMarkerArray());

  time_publisher_->publish(output_.processing_time_map);
}

rcl_interfaces::msg::SetParametersResult ObstacleCollisionCheckerNode::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(parameters, "update_rate", p.update_rate);
    }

    auto & p = param_;

    update_param(parameters, "delay_time", p.delay_time);
    update_param(parameters, "footprint_margin", p.footprint_margin);
    update_param(parameters, "max_deceleration", p.max_deceleration);
    update_param(parameters, "resample_interval", p.resample_interval);
    update_param(parameters, "search_radius", p.search_radius);

    if (obstacle_collision_checker_) {
      obstacle_collision_checker_->setParam(param_);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }
  return result;
}

void ObstacleCollisionCheckerNode::checkLaneDeparture(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (output_.will_collide) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "vehicle will collide with obstacles";
  }

  stat.summary(level, msg);
}

visualization_msgs::msg::MarkerArray ObstacleCollisionCheckerNode::createMarkerArray() const
{
  using tier4_autoware_utils::createDefaultMarker;
  using tier4_autoware_utils::createMarkerColor;
  using tier4_autoware_utils::createMarkerScale;

  visualization_msgs::msg::MarkerArray marker_array;

  const auto base_link_z = current_pose_->pose.position.z;

  if (output_.resampled_trajectory.points.size() >= 2) {
    // Line of resampled_trajectory
    {
      auto marker = createDefaultMarker(
        "map", this->now(), "resampled_trajectory_line", 0,
        visualization_msgs::msg::Marker::LINE_STRIP, createMarkerScale(0.05, 0, 0),
        createMarkerColor(1.0, 1.0, 1.0, 0.999));

      for (const auto & p : output_.resampled_trajectory.points) {
        marker.points.push_back(p.pose.position);
        marker.colors.push_back(marker.color);
      }

      marker_array.markers.push_back(marker);
    }

    // Points of resampled_trajectory
    {
      auto marker = createDefaultMarker(
        "map", this->now(), "resampled_trajectory_points", 0,
        visualization_msgs::msg::Marker::SPHERE_LIST, createMarkerScale(0.1, 0.1, 0.1),
        createMarkerColor(0.0, 1.0, 0.0, 0.999));

      for (const auto & p : output_.resampled_trajectory.points) {
        marker.points.push_back(p.pose.position);
        marker.colors.push_back(marker.color);
      }

      marker_array.markers.push_back(marker);
    }
  }

  // Vehicle Footprints
  {
    const auto color_ok = createMarkerColor(0.0, 1.0, 0.0, 0.5);
    const auto color_will_collide = createMarkerColor(1.0, 0.0, 0.0, 0.5);

    auto color = color_ok;
    if (output_.will_collide) {
      color = color_will_collide;
    }

    auto marker = createDefaultMarker(
      "map", this->now(), "vehicle_footprints", 0, visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.05, 0, 0), color);

    for (const auto & vehicle_footprint : output_.vehicle_footprints) {
      for (size_t i = 0; i < vehicle_footprint.size() - 1; ++i) {
        const auto p1 = vehicle_footprint.at(i);
        const auto p2 = vehicle_footprint.at(i + 1);

        marker.points.push_back(toMsg(p1.to_3d(base_link_z)));
        marker.points.push_back(toMsg(p2.to_3d(base_link_z)));
      }
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}
}  // namespace obstacle_collision_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_collision_checker::ObstacleCollisionCheckerNode)
