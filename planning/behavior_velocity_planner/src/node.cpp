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

#include "behavior_velocity_planner/node.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <utilization/path_utilization.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_routing/Route.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.h>

#include <functional>
#include <memory>

// Scene modules
#include <scene_module/blind_spot/manager.hpp>
#include <scene_module/crosswalk/manager.hpp>
#include <scene_module/detection_area/manager.hpp>
#include <scene_module/intersection/manager.hpp>
#include <scene_module/no_stopping_area/manager.hpp>
#include <scene_module/occlusion_spot/manager.hpp>
#include <scene_module/stop_line/manager.hpp>
#include <scene_module/traffic_light/manager.hpp>
#include <scene_module/virtual_traffic_light/manager.hpp>

namespace behavior_velocity_planner
{
namespace
{
geometry_msgs::msg::PoseStamped transform2pose(
  const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

autoware_auto_planning_msgs::msg::Path to_path(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path_with_id)
{
  autoware_auto_planning_msgs::msg::Path path;
  for (const auto & path_point : path_with_id.points) {
    path.points.push_back(path_point.point);
  }
  return path;
}
}  // namespace

BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("behavior_velocity_planner_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  planner_data_(*this)
{
  using std::placeholders::_1;
  // Trigger Subscriber
  trigger_sub_path_with_lane_id_ =
    this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>(
      "~/input/path_with_lane_id", 1, std::bind(&BehaviorVelocityPlannerNode::onTrigger, this, _1));

  // Subscribers
  sub_predicted_objects_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "~/input/dynamic_objects", 1,
      std::bind(&BehaviorVelocityPlannerNode::onPredictedObjects, this, _1));
  sub_no_ground_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/no_ground_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&BehaviorVelocityPlannerNode::onNoGroundPointCloud, this, _1));
  sub_vehicle_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/vehicle_odometry", 1,
    std::bind(&BehaviorVelocityPlannerNode::onVehicleVelocity, this, _1));
  sub_lanelet_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/vector_map", rclcpp::QoS(10).transient_local(),
    std::bind(&BehaviorVelocityPlannerNode::onLaneletMap, this, _1));
  sub_traffic_signals_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::TrafficSignalArray>(
      "~/input/traffic_signals", 10,
      std::bind(&BehaviorVelocityPlannerNode::onTrafficSignals, this, _1));
  sub_external_crosswalk_states_ =
    this->create_subscription<autoware_api_msgs::msg::CrosswalkStatus>(
      "~/input/external_crosswalk_states", 10,
      std::bind(&BehaviorVelocityPlannerNode::onExternalCrosswalkStates, this, _1));
  sub_external_intersection_states_ =
    this->create_subscription<autoware_api_msgs::msg::IntersectionStatus>(
      "~/input/external_intersection_states", 10,
      std::bind(&BehaviorVelocityPlannerNode::onExternalIntersectionStates, this, _1));
  sub_external_traffic_signals_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::TrafficSignalArray>(
      "~/input/external_traffic_signals", 10,
      std::bind(&BehaviorVelocityPlannerNode::onExternalTrafficSignals, this, _1));
  sub_virtual_traffic_light_states_ =
    this->create_subscription<autoware_v2x_msgs::msg::VirtualTrafficLightStateArray>(
      "~/input/virtual_traffic_light_states", 10,
      std::bind(&BehaviorVelocityPlannerNode::onVirtualTrafficLightStates, this, _1));
  sub_occupancy_grid_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "~/input/occupancy_grid", 1,
    std::bind(&BehaviorVelocityPlannerNode::onOccupancyGrid, this, _1));

  // Publishers
  path_pub_ = this->create_publisher<autoware_auto_planning_msgs::msg::Path>("~/output/path", 1);
  stop_reason_diag_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/output/stop_reason", 1);
  debug_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/path", 1);

  // Parameters
  forward_path_length_ = this->declare_parameter("forward_path_length", 1000.0);
  backward_path_length_ = this->declare_parameter("backward_path_length", 5.0);
  // TODO(yukkysaito): This will become unnecessary when acc output from localization is available.
  planner_data_.accel_lowpass_gain_ = this->declare_parameter("lowpass_gain", 0.5);
  planner_data_.stop_line_extend_length = this->declare_parameter("stop_line_extend_length", 5.0);

  // Initialize PlannerManager
  if (this->declare_parameter("launch_crosswalk", true)) {
    planner_manager_.launchSceneModule(std::make_shared<CrosswalkModuleManager>(*this));
  }
  if (this->declare_parameter("launch_traffic_light", true)) {
    planner_manager_.launchSceneModule(std::make_shared<TrafficLightModuleManager>(*this));
  }
  if (this->declare_parameter("launch_intersection", true)) {
    planner_manager_.launchSceneModule(std::make_shared<IntersectionModuleManager>(*this));
  }
  if (this->declare_parameter("launch_blind_spot", true)) {
    planner_manager_.launchSceneModule(std::make_shared<BlindSpotModuleManager>(*this));
  }
  if (this->declare_parameter("launch_detection_area", true)) {
    planner_manager_.launchSceneModule(std::make_shared<DetectionAreaModuleManager>(*this));
  }
  if (this->declare_parameter("launch_virtual_traffic_light", true)) {
    planner_manager_.launchSceneModule(std::make_shared<VirtualTrafficLightModuleManager>(*this));
  }
  if (this->declare_parameter("launch_occlusion_spot", true)) {
    planner_manager_.launchSceneModule(std::make_shared<OcclusionSpotModuleManager>(*this));
  }
  // this module requires all the stop line.Therefore this modules should be placed at the bottom.
  if (this->declare_parameter("launch_no_stopping_area", true)) {
    planner_manager_.launchSceneModule(std::make_shared<NoStoppingAreaModuleManager>(*this));
  }
  // permanent stop line module should be after no stopping area
  if (this->declare_parameter("launch_stop_line", true)) {
    planner_manager_.launchSceneModule(std::make_shared<StopLineModuleManager>(*this));
  }
}

bool BehaviorVelocityPlannerNode::isDataReady()
{
  const auto & d = planner_data_;

  // from tf
  if (d.current_pose.header.frame_id == "") {
    return false;
  }

  // from callbacks
  if (!d.current_velocity) {
    return false;
  }
  if (!d.predicted_objects) {
    return false;
  }
  if (!d.no_ground_pointcloud) {
    return false;
  }
  if (!d.lanelet_map) {
    return false;
  }

  return true;
}

void BehaviorVelocityPlannerNode::onOccupancyGrid(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  planner_data_.occupancy_grid = msg;
}

void BehaviorVelocityPlannerNode::onPredictedObjects(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  planner_data_.predicted_objects = msg;
}

void BehaviorVelocityPlannerNode::onNoGroundPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(*msg, pc);

  Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(pc, *pc_transformed, affine);

  planner_data_.no_ground_pointcloud = pc_transformed;
}

void BehaviorVelocityPlannerNode::onVehicleVelocity(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  auto current_velocity = std::make_shared<geometry_msgs::msg::TwistStamped>();
  current_velocity->header = msg->header;
  current_velocity->twist = msg->twist.twist;
  planner_data_.current_velocity = current_velocity;

  planner_data_.updateCurrentAcc();

  // Add velocity to buffer
  planner_data_.velocity_buffer.push_front(*current_velocity);
  const auto now = this->now();
  while (true) {
    // Check oldest data time
    const auto time_diff = now - planner_data_.velocity_buffer.back().header.stamp;

    // Finish when oldest data is newer than threshold
    if (time_diff.seconds() <= PlannerData::velocity_buffer_time_sec) {
      break;
    }

    // Remove old data
    planner_data_.velocity_buffer.pop_back();
  }
}

void BehaviorVelocityPlannerNode::onLaneletMap(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  // Load map
  planner_data_.lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, planner_data_.lanelet_map, &planner_data_.traffic_rules, &planner_data_.routing_graph);

  // Build graph
  {
    using lanelet::Locations;
    using lanelet::Participants;
    using lanelet::routing::RoutingGraph;
    using lanelet::routing::RoutingGraphConstPtr;
    using lanelet::routing::RoutingGraphContainer;
    using lanelet::traffic_rules::TrafficRulesFactory;

    const auto traffic_rules =
      TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
    const auto pedestrian_rules =
      TrafficRulesFactory::create(Locations::Germany, Participants::Pedestrian);

    RoutingGraphConstPtr vehicle_graph =
      RoutingGraph::build(*planner_data_.lanelet_map, *traffic_rules);
    RoutingGraphConstPtr pedestrian_graph =
      RoutingGraph::build(*planner_data_.lanelet_map, *pedestrian_rules);
    RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});

    planner_data_.overall_graphs = std::make_shared<const RoutingGraphContainer>(overall_graphs);
  }
}

void BehaviorVelocityPlannerNode::onTrafficSignals(
  const autoware_auto_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr msg)
{
  for (const auto & signal : msg->signals) {
    autoware_auto_perception_msgs::msg::TrafficSignalStamped traffic_signal;
    traffic_signal.header = msg->header;
    traffic_signal.signal = signal;
    planner_data_.traffic_light_id_map[signal.map_primitive_id] = traffic_signal;
  }
}

void BehaviorVelocityPlannerNode::onExternalCrosswalkStates(
  const autoware_api_msgs::msg::CrosswalkStatus::ConstSharedPtr msg)
{
  planner_data_.external_crosswalk_status_input = *msg;
}

void BehaviorVelocityPlannerNode::onExternalIntersectionStates(
  const autoware_api_msgs::msg::IntersectionStatus::ConstSharedPtr msg)
{
  planner_data_.external_intersection_status_input = *msg;
}

void BehaviorVelocityPlannerNode::onExternalTrafficSignals(
  const autoware_auto_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr msg)
{
  for (const auto & signal : msg->signals) {
    autoware_auto_perception_msgs::msg::TrafficSignalStamped traffic_signal;
    traffic_signal.header = msg->header;
    traffic_signal.signal = signal;
    planner_data_.external_traffic_light_id_map[signal.map_primitive_id] = traffic_signal;
  }
}

void BehaviorVelocityPlannerNode::onVirtualTrafficLightStates(
  const autoware_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr msg)
{
  planner_data_.virtual_traffic_light_states = msg;
}

void BehaviorVelocityPlannerNode::onTrigger(
  const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg)
{
  // Check ready
  try {
    planner_data_.current_pose =
      transform2pose(tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero));
  } catch (tf2::TransformException & e) {
    RCLCPP_INFO(get_logger(), "waiting for transform from `map` to `base_link`");
    return;
  }

  if (!isDataReady()) {
    return;
  }

  // Plan path velocity
  const auto velocity_planned_path = planner_manager_.planPathVelocity(
    std::make_shared<const PlannerData>(planner_data_), *input_path_msg);

  // screening
  const auto filtered_path = filterLitterPathPoint(to_path(velocity_planned_path));

  // interpolation
  const auto interpolated_path_msg = interpolatePath(filtered_path, forward_path_length_);

  // check stop point
  auto output_path_msg = filterStopPathPoint(interpolated_path_msg);
  output_path_msg.header.frame_id = "map";
  output_path_msg.header.stamp = this->now();

  // TODO(someone): This must be updated in each scene module, but copy from input message for now.
  output_path_msg.drivable_area = input_path_msg->drivable_area;

  path_pub_->publish(output_path_msg);
  stop_reason_diag_pub_->publish(planner_manager_.getStopReasonDiag());

  if (debug_viz_pub_->get_subscription_count() > 0) {
    publishDebugMarker(output_path_msg);
  }
}

void BehaviorVelocityPlannerNode::publishDebugMarker(
  const autoware_auto_planning_msgs::msg::Path & path)
{
  visualization_msgs::msg::MarkerArray output_msg;
  for (size_t i = 0; i < path.points.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.pose = path.points.at(i).pose;
    marker.scale.y = marker.scale.z = 0.05;
    marker.scale.x = 0.25;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    output_msg.markers.push_back(marker);
  }
  debug_viz_pub_->publish(output_msg);
}
}  // namespace behavior_velocity_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(behavior_velocity_planner::BehaviorVelocityPlannerNode)
