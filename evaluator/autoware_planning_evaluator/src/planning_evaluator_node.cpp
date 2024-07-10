// Copyright 2021 Tier IV, Inc.
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

#include "autoware/planning_evaluator/planning_evaluator_node.hpp"

#include "autoware/evaluator_utils/evaluator_utils.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <diagnostic_msgs/msg/detail/diagnostic_status__struct.hpp>

#include "boost/lexical_cast.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace planning_diagnostics
{
PlanningEvaluatorNode::PlanningEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("planning_evaluator", node_options)
{
  using std::placeholders::_1;
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Timer callback to publish evaluator diagnostics
  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&PlanningEvaluatorNode::onTimer, this));
  // Parameters
  metrics_calculator_.parameters.trajectory.min_point_dist_m =
    declare_parameter<double>("trajectory.min_point_dist_m");
  metrics_calculator_.parameters.trajectory.lookahead.max_dist_m =
    declare_parameter<double>("trajectory.lookahead.max_dist_m");
  metrics_calculator_.parameters.trajectory.lookahead.max_time_s =
    declare_parameter<double>("trajectory.lookahead.max_time_s");
  metrics_calculator_.parameters.obstacle.dist_thr_m =
    declare_parameter<double>("obstacle.dist_thr_m");

  output_file_str_ = declare_parameter<std::string>("output_file");
  ego_frame_str_ = declare_parameter<std::string>("ego_frame");

  planning_diag_sub_ = create_subscription<DiagnosticArray>(
    "~/input/diagnostics", 1, std::bind(&PlanningEvaluatorNode::onDiagnostics, this, _1));

  // List of metrics to calculate and publish
  metrics_pub_ = create_publisher<DiagnosticArray>("~/metrics", 1);
  for (const std::string & selected_metric :
       declare_parameter<std::vector<std::string>>("selected_metrics")) {
    Metric metric = str_to_metric.at(selected_metric);
    metrics_.push_back(metric);
  }
}

PlanningEvaluatorNode::~PlanningEvaluatorNode()
{
  if (!output_file_str_.empty()) {
    // column width is the maximum size we might print + 1 for the space between columns
    // Write data using format
    std::ofstream f(output_file_str_);
    f << std::fixed << std::left;
    // header
    f << "#Stamp(ns)";
    for (Metric metric : metrics_) {
      f << " " << metric_descriptions.at(metric);
      f << " . .";  // extra "columns" to align columns headers
    }
    f << std::endl;
    f << "#.";
    for (Metric metric : metrics_) {
      (void)metric;
      f << " min max mean";
    }
    f << std::endl;
    // data
    for (size_t i = 0; i < stamps_.size(); ++i) {
      f << stamps_[i].nanoseconds();
      for (Metric metric : metrics_) {
        const auto & stat = metric_stats_[static_cast<size_t>(metric)][i];
        f << " " << stat;
      }
      f << std::endl;
    }
    f.close();
  }
}

void PlanningEvaluatorNode::onDiagnostics(const DiagnosticArray::ConstSharedPtr diag_msg)
{
  // add target diagnostics to the queue and remove old ones
  for (const auto & function : target_functions_) {
    autoware::evaluator_utils::updateDiagnosticQueue(*diag_msg, function, now(), diag_queue_);
  }
}

DiagnosticStatus PlanningEvaluatorNode::generateDiagnosticEvaluationStatus(
  const DiagnosticStatus & diag)
{
  DiagnosticStatus status;
  status.name = diag.name;

  const auto it = std::find_if(diag.values.begin(), diag.values.end(), [](const auto & key_value) {
    return key_value.key.find("decision") != std::string::npos;
  });
  const bool found = it != diag.values.end();
  status.level = (found) ? status.OK : status.ERROR;
  status.values.push_back((found) ? *it : diagnostic_msgs::msg::KeyValue{});
  return status;
}

void PlanningEvaluatorNode::getRouteData()
{
  // route
  {
    const auto msg = route_subscriber_.takeNewData();
    if (msg) {
      if (msg->segments.empty()) {
        RCLCPP_ERROR(get_logger(), "input route is empty. ignored");
      } else {
        route_handler_.setRoute(*msg);
      }
    }
  }

  // map
  {
    const auto msg = vector_map_subscriber_.takeNewData();
    if (msg) {
      route_handler_.setMap(*msg);
    }
  }
}

DiagnosticStatus PlanningEvaluatorNode::generateLaneletDiagnosticStatus(
  const Odometry::ConstSharedPtr ego_state_ptr)
{
  const auto & ego_pose = ego_state_ptr->pose.pose;
  const auto current_lanelets = [&]() {
    lanelet::ConstLanelet closest_route_lanelet;
    route_handler_.getClosestLaneletWithinRoute(ego_pose, &closest_route_lanelet);
    const auto shoulder_lanelets = route_handler_.getShoulderLaneletsAtPose(ego_pose);
    lanelet::ConstLanelets closest_lanelets{closest_route_lanelet};
    closest_lanelets.insert(
      closest_lanelets.end(), shoulder_lanelets.begin(), shoulder_lanelets.end());
    return closest_lanelets;
  }();
  const auto arc_coordinates = lanelet::utils::getArcCoordinates(current_lanelets, ego_pose);
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanelets, ego_pose, &current_lane);

  DiagnosticStatus status;
  status.name = "ego_lane_info";
  status.level = status.OK;
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "lane_id";
  key_value.value = std::to_string(current_lane.id());
  status.values.push_back(key_value);
  key_value.key = "s";
  key_value.value = std::to_string(arc_coordinates.length);
  status.values.push_back(key_value);
  key_value.key = "t";
  key_value.value = std::to_string(arc_coordinates.distance);
  status.values.push_back(key_value);
  return status;
}

DiagnosticStatus PlanningEvaluatorNode::generateKinematicStateDiagnosticStatus(
  const AccelWithCovarianceStamped & accel_stamped, const Odometry::ConstSharedPtr ego_state_ptr)
{
  DiagnosticStatus status;
  status.name = "kinematic_state";
  status.level = status.OK;
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "vel";
  key_value.value = std::to_string(ego_state_ptr->twist.twist.linear.x);
  status.values.push_back(key_value);
  key_value.key = "acc";
  const auto & acc = accel_stamped.accel.accel.linear.x;
  key_value.value = std::to_string(acc);
  status.values.push_back(key_value);
  key_value.key = "jerk";
  const auto jerk = [&]() {
    if (!prev_acc_stamped_.has_value()) {
      prev_acc_stamped_ = accel_stamped;
      return 0.0;
    }
    const auto t = static_cast<double>(accel_stamped.header.stamp.sec) +
                   static_cast<double>(accel_stamped.header.stamp.nanosec) * 1e-9;
    const auto prev_t = static_cast<double>(prev_acc_stamped_.value().header.stamp.sec) +
                        static_cast<double>(prev_acc_stamped_.value().header.stamp.nanosec) * 1e-9;
    const auto dt = t - prev_t;
    if (dt < std::numeric_limits<double>::epsilon()) return 0.0;

    const auto prev_acc = prev_acc_stamped_.value().accel.accel.linear.x;
    prev_acc_stamped_ = accel_stamped;
    return (acc - prev_acc) / dt;
  }();
  key_value.value = std::to_string(jerk);
  status.values.push_back(key_value);
  return status;
}

DiagnosticStatus PlanningEvaluatorNode::generateDiagnosticStatus(
  const Metric & metric, const Stat<double> & metric_stat) const
{
  DiagnosticStatus status;
  status.level = status.OK;
  status.name = metric_to_str.at(metric);
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "min";
  key_value.value = boost::lexical_cast<decltype(key_value.value)>(metric_stat.min());
  status.values.push_back(key_value);
  key_value.key = "max";
  key_value.value = boost::lexical_cast<decltype(key_value.value)>(metric_stat.max());
  status.values.push_back(key_value);
  key_value.key = "mean";
  key_value.value = boost::lexical_cast<decltype(key_value.value)>(metric_stat.mean());
  status.values.push_back(key_value);
  return status;
}

void PlanningEvaluatorNode::onTimer()
{
  metrics_msg_.header.stamp = now();

  const auto ego_state_ptr = odometry_sub_.takeData();
  onOdometry(ego_state_ptr);
  {
    const auto objects_msg = objects_sub_.takeData();
    onObjects(objects_msg);
  }

  {
    const auto ref_traj_msg = ref_sub_.takeData();
    onReferenceTrajectory(ref_traj_msg);
  }

  {
    const auto traj_msg = traj_sub_.takeData();
    onTrajectory(traj_msg, ego_state_ptr);
  }
  {
    const auto modified_goal_msg = modified_goal_sub_.takeData();
    onModifiedGoal(modified_goal_msg, ego_state_ptr);
  }

  {
    // generate decision diagnostics from input diagnostics
    for (const auto & function : target_functions_) {
      const auto it = std::find_if(
        diag_queue_.begin(), diag_queue_.end(),
        [&function](const std::pair<diagnostic_msgs::msg::DiagnosticStatus, rclcpp::Time> & p) {
          return p.first.name.find(function) != std::string::npos;
        });
      if (it == diag_queue_.end()) {
        continue;
      }
      // generate each decision diagnostics
      metrics_msg_.status.push_back(generateDiagnosticEvaluationStatus(it->first));
    }
  }

  if (!metrics_msg_.status.empty()) {
    metrics_pub_->publish(metrics_msg_);
  }
  metrics_msg_ = DiagnosticArray{};
}

void PlanningEvaluatorNode::onTrajectory(
  const Trajectory::ConstSharedPtr traj_msg, const Odometry::ConstSharedPtr ego_state_ptr)
{
  if (!ego_state_ptr || !traj_msg) {
    return;
  }

  auto start = now();
  if (!output_file_str_.empty()) {
    stamps_.push_back(traj_msg->header.stamp);
  }

  for (Metric metric : metrics_) {
    const auto metric_stat = metrics_calculator_.calculate(Metric(metric), *traj_msg);
    if (!metric_stat) {
      continue;
    }

    if (!output_file_str_.empty()) {
      metric_stats_[static_cast<size_t>(metric)].push_back(*metric_stat);
    }

    if (metric_stat->count() > 0) {
      metrics_msg_.status.push_back(generateDiagnosticStatus(metric, *metric_stat));
    }
  }

  metrics_calculator_.setPreviousTrajectory(*traj_msg);
  auto runtime = (now() - start).seconds();
  RCLCPP_DEBUG(get_logger(), "Planning evaluation calculation time: %2.2f ms", runtime * 1e3);
}

void PlanningEvaluatorNode::onModifiedGoal(
  const PoseWithUuidStamped::ConstSharedPtr modified_goal_msg,
  const Odometry::ConstSharedPtr ego_state_ptr)
{
  if (!modified_goal_msg || !ego_state_ptr) {
    return;
  }
  auto start = now();

  for (Metric metric : metrics_) {
    const auto metric_stat = metrics_calculator_.calculate(
      Metric(metric), modified_goal_msg->pose, ego_state_ptr->pose.pose);
    if (!metric_stat) {
      continue;
    }
    metric_stats_[static_cast<size_t>(metric)].push_back(*metric_stat);
    if (metric_stat->count() > 0) {
      metrics_msg_.status.push_back(generateDiagnosticStatus(metric, *metric_stat));
    }
  }
  auto runtime = (now() - start).seconds();
  RCLCPP_DEBUG(
    get_logger(), "Planning evaluation modified goal deviation calculation time: %2.2f ms",
    runtime * 1e3);
}

void PlanningEvaluatorNode::onOdometry(const Odometry::ConstSharedPtr odometry_msg)
{
  if (!odometry_msg) return;
  metrics_calculator_.setEgoPose(odometry_msg->pose.pose);
  {
    getRouteData();
    if (route_handler_.isHandlerReady() && odometry_msg) {
      metrics_msg_.status.push_back(generateLaneletDiagnosticStatus(odometry_msg));
    }

    const auto acc_msg = accel_sub_.takeData();
    if (acc_msg && odometry_msg) {
      metrics_msg_.status.push_back(generateKinematicStateDiagnosticStatus(*acc_msg, odometry_msg));
    }
  }
}

void PlanningEvaluatorNode::onReferenceTrajectory(const Trajectory::ConstSharedPtr traj_msg)
{
  if (!traj_msg) {
    return;
  }
  metrics_calculator_.setReferenceTrajectory(*traj_msg);
}

void PlanningEvaluatorNode::onObjects(const PredictedObjects::ConstSharedPtr objects_msg)
{
  if (!objects_msg) {
    return;
  }
  metrics_calculator_.setPredictedObjects(*objects_msg);
}

bool PlanningEvaluatorNode::isFinite(const TrajectoryPoint & point)
{
  const auto & o = point.pose.orientation;
  const auto & p = point.pose.position;
  const auto & v = point.longitudinal_velocity_mps;
  const auto & w = point.lateral_velocity_mps;
  const auto & a = point.acceleration_mps2;
  const auto & z = point.heading_rate_rps;
  const auto & f = point.front_wheel_angle_rad;
  const auto & r = point.rear_wheel_angle_rad;

  return std::isfinite(o.x) && std::isfinite(o.y) && std::isfinite(o.z) && std::isfinite(o.w) &&
         std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(v) &&
         std::isfinite(w) && std::isfinite(a) && std::isfinite(z) && std::isfinite(f) &&
         std::isfinite(r);
}
}  // namespace planning_diagnostics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planning_diagnostics::PlanningEvaluatorNode)
