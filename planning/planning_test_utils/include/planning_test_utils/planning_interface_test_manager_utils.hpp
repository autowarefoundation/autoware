// Copyright 2023 Tier IV, Inc.
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

#ifndef PLANNING_TEST_UTILS__PLANNING_INTERFACE_TEST_MANAGER_UTILS_HPP_
#define PLANNING_TEST_UTILS__PLANNING_INTERFACE_TEST_MANAGER_UTILS_HPP_

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include <component_interface_specs/planning.hpp>
#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <route_handler/route_handler.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <cxxabi.h>
#include <lanelet2_io/Io.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace test_utils
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using route_handler::RouteHandler;
using sensor_msgs::msg::PointCloud2;
using tf2_msgs::msg::TFMessage;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_planning_msgs::msg::Scenario;
using unique_identifier_msgs::msg::UUID;

geometry_msgs::msg::Pose createPose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position = createPoint(x, y, z);
  p.orientation = createQuaternionFromRPY(roll, pitch, yaw);
  return p;
}

geometry_msgs::msg::Pose createPose(const std::array<double, 4> & pose3d)
{
  return createPose(pose3d[0], pose3d[1], pose3d[2], 0.0, 0.0, pose3d[3]);
}

template <class T>
T generateTrajectory(
  const size_t num_points, const double point_interval, const double velocity = 0.0,
  const double init_theta = 0.0, const double delta_theta = 0.0,
  const size_t overlapping_point_index = std::numeric_limits<size_t>::max())
{
  using Point = typename T::_points_type::value_type;

  T traj;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    Point p;
    p.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.longitudinal_velocity_mps = velocity;
    traj.points.push_back(p);

    if (i == overlapping_point_index) {
      Point value_to_insert = traj.points[overlapping_point_index];
      traj.points.insert(traj.points.begin() + overlapping_point_index + 1, value_to_insert);
    }
  }

  return traj;
}

LaneletSegment createLaneletSegment(int id)
{
  LaneletPrimitive primitive;
  primitive.id = id;
  primitive.primitive_type = "lane";
  LaneletSegment segment;
  segment.preferred_primitive.id = id;
  segment.primitives.push_back(primitive);
  return segment;
}

lanelet::LaneletMapPtr loadMap(const std::string & lanelet2_filename)
{
  lanelet::ErrorMessages errors{};
  lanelet::projection::MGRSProjector projector{};
  const lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);
  if (errors.empty()) {
    return map;
  }

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
  }
  return nullptr;
}

HADMapBin convertToMapBinMsg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename, const rclcpp::Time & now)
{
  std::string format_version{}, map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  HADMapBin map_bin_msg;
  map_bin_msg.header.stamp = now;
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.format_version = format_version;
  map_bin_msg.map_version = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  return map_bin_msg;
}

LaneletRoute makeNormalRoute()
{
  const std::array<double, 4> start_pose{5.5, 4., 0., M_PI_2};
  const std::array<double, 4> goal_pose{8.0, 26.3, 0, 0};
  LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose = createPose(start_pose);
  route.goal_pose = createPose(goal_pose);
  return route;
}

OccupancyGrid makeCostMapMsg(size_t width = 150, size_t height = 150, double resolution = 0.2)
{
  nav_msgs::msg::OccupancyGrid costmap_msg;

  // create info
  costmap_msg.header.frame_id = "map";
  costmap_msg.info.width = width;
  costmap_msg.info.height = height;
  costmap_msg.info.resolution = resolution;

  // create data
  const size_t n_elem = width * height;
  for (size_t i = 0; i < n_elem; ++i) {
    costmap_msg.data.push_back(0.0);
  }
  return costmap_msg;
}

HADMapBin makeMapBinMsg()
{
  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("planning_test_utils");
  const auto lanelet2_path = planning_test_utils_dir + "/test_map/lanelet2_map.osm";
  double center_line_resolution = 5.0;
  // load map from file
  const auto map = loadMap(lanelet2_path);
  if (!map) {
    return autoware_auto_mapping_msgs::msg::HADMapBin_<std::allocator<void>>{};
  }

  // overwrite centerline
  lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);

  // create map bin msg
  const auto map_bin_msg =
    convertToMapBinMsg(map, lanelet2_path, rclcpp::Clock(RCL_ROS_TIME).now());
  return map_bin_msg;
}

Odometry makeOdometry(const double shift = 0.0)
{
  Odometry odometry;
  const std::array<double, 4> start_pose{0.0, shift, 0.0, 0.0};
  odometry.pose.pose = createPose(start_pose);
  odometry.header.frame_id = "map";
  return odometry;
}

Odometry makeInitialPose(const double shift = 0.0)
{
  Odometry current_odometry;
  const auto yaw = 0.9724497591854532;
  const auto shift_x = shift * std::sin(yaw);
  const auto shift_y = shift * std::cos(yaw);
  const std::array<double, 4> start_pose{
    3722.16015625 + shift_x, 73723.515625 + shift_y, 0.233112560494183, yaw};
  current_odometry.pose.pose = test_utils::createPose(start_pose);
  current_odometry.header.frame_id = "map";
  return current_odometry;
}

TFMessage makeTFMsg(
  rclcpp::Node::SharedPtr target_node, std::string frame_id = "", std::string child_frame_id = "")
{
  TFMessage tf_msg;
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.;
  quaternion.y = 0.;
  quaternion.z = 0.23311256049418302;
  quaternion.w = 0.9724497591854532;

  TransformStamped tf;
  tf.header.stamp = target_node->get_clock()->now();
  tf.header.frame_id = frame_id;
  tf.child_frame_id = child_frame_id;
  tf.transform.translation.x = 3722.16015625;
  tf.transform.translation.y = 73723.515625;
  tf.transform.translation.z = 0;
  tf.transform.rotation = quaternion;

  tf_msg.transforms.emplace_back(std::move(tf));
  return tf_msg;
}

Scenario makeScenarioMsg(const std::string scenario)
{
  Scenario scenario_msg;
  scenario_msg.current_scenario = scenario;
  scenario_msg.activating_scenarios = {scenario};
  return scenario_msg;
}

Pose createPoseFromLaneID(const lanelet::Id & lane_id)
{
  auto map_bin_msg = makeMapBinMsg();
  // create route_handler
  auto route_handler = std::make_shared<RouteHandler>();
  route_handler->setMap(map_bin_msg);

  // get middle idx of the lanelet
  const auto lanelet = route_handler->getLaneletsFromId(lane_id);
  const auto center_line = lanelet.centerline();
  const size_t middle_point_idx = std::floor(center_line.size() / 2.0);

  // get middle position of the lanelet
  geometry_msgs::msg::Point middle_pos;
  middle_pos.x = center_line[middle_point_idx].x();
  middle_pos.y = center_line[middle_point_idx].y();

  // get next middle position of the lanelet
  geometry_msgs::msg::Point next_middle_pos;
  next_middle_pos.x = center_line[middle_point_idx + 1].x();
  next_middle_pos.y = center_line[middle_point_idx + 1].y();

  // calculate middle pose
  geometry_msgs::msg::Pose middle_pose;
  middle_pose.position = middle_pos;
  const double yaw = tier4_autoware_utils::calcAzimuthAngle(middle_pos, next_middle_pos);
  middle_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);

  return middle_pose;
}

Odometry makeInitialPoseFromLaneId(const lanelet::Id & lane_id)
{
  Odometry current_odometry;
  current_odometry.pose.pose = createPoseFromLaneID(lane_id);
  current_odometry.header.frame_id = "map";

  return current_odometry;
}

RouteSections combineConsecutiveRouteSections(
  const RouteSections & route_sections1, const RouteSections & route_sections2)
{
  RouteSections route_sections;
  route_sections.reserve(route_sections1.size() + route_sections2.size());
  if (!route_sections1.empty()) {
    // remove end route section because it is overlapping with first one in next route_section
    route_sections.insert(route_sections.end(), route_sections1.begin(), route_sections1.end() - 1);
  }
  if (!route_sections2.empty()) {
    route_sections.insert(route_sections.end(), route_sections2.begin(), route_sections2.end());
  }
  return route_sections;
}

// Function to create a route from given start and goal lanelet ids
// start pose and goal pose are set to the middle of the lanelet
LaneletRoute makeBehaviorRouteFromLaneId(const int & start_lane_id, const int & goal_lane_id)
{
  LaneletRoute route;
  route.header.frame_id = "map";
  auto start_pose = createPoseFromLaneID(start_lane_id);
  auto goal_pose = createPoseFromLaneID(goal_lane_id);
  route.start_pose = start_pose;
  route.goal_pose = goal_pose;

  auto map_bin_msg = makeMapBinMsg();
  // create route_handler
  auto route_handler = std::make_shared<RouteHandler>();
  route_handler->setMap(map_bin_msg);

  LaneletRoute route_msg;
  RouteSections route_sections;
  lanelet::ConstLanelets all_route_lanelets;

  // Plan the path between checkpoints (start and goal poses)
  lanelet::ConstLanelets path_lanelets;
  if (!route_handler->planPathLaneletsBetweenCheckpoints(start_pose, goal_pose, &path_lanelets)) {
    return route_msg;
  }

  // Add all path_lanelets to all_route_lanelets
  for (const auto & lane : path_lanelets) {
    all_route_lanelets.push_back(lane);
  }
  // create local route sections
  route_handler->setRouteLanelets(path_lanelets);
  const auto local_route_sections = route_handler->createMapSegments(path_lanelets);
  route_sections = combineConsecutiveRouteSections(route_sections, local_route_sections);
  for (const auto & route_section : route_sections) {
    for (const auto & primitive : route_section.primitives) {
      std::cerr << "primitive: " << primitive.id << std::endl;
    }
    std::cerr << "preferred_primitive id : " << route_section.preferred_primitive.id << std::endl;
  }
  route_handler->setRouteLanelets(all_route_lanelets);
  route.segments = route_sections;

  route.allow_modification = false;
  return route;
}

// this is for the test lanelet2_map.osm
// file hash: a9f84cff03b55a64917bc066451276d2293b0a54f5c088febca0c7fdf2f245d5
LaneletRoute makeBehaviorNormalRoute()
{
  LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose =
    createPose({3722.16015625, 73723.515625, 0.233112560494183, 0.9724497591854532});
  route.goal_pose =
    createPose({3778.362060546875, 73721.2734375, -0.5107480274693206, 0.8597304533609347});

  std::vector<int> primitive_ids = {9102, 9178, 54, 112};
  for (int id : primitive_ids) {
    route.segments.push_back(createLaneletSegment(id));
  }

  std::array<uint8_t, 16> uuid_bytes{210, 87,  16,  126, 98,  151, 58, 28,
                                     252, 221, 230, 92,  122, 170, 46, 6};
  route.uuid.uuid = uuid_bytes;

  route.allow_modification = false;
  return route;
}

template <typename T>
void createPublisherWithQoS(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Publisher<T>> & publisher)
{
  if constexpr (
    std::is_same_v<T, LaneletRoute> || std::is_same_v<T, HADMapBin> ||
    std::is_same_v<T, OperationModeState>) {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();
    publisher = rclcpp::create_publisher<T>(test_node, topic_name, qos);
  } else {
    publisher = rclcpp::create_publisher<T>(test_node, topic_name, 1);
  }
}

template <typename T>
void setPublisher(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Publisher<T>> & publisher)
{
  createPublisherWithQoS(test_node, topic_name, publisher);
}

template <typename T>
void createSubscription(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::function<void(const typename T::ConstSharedPtr)> callback,
  std::shared_ptr<rclcpp::Subscription<T>> & subscriber)
{
  if constexpr (std::is_same_v<T, Trajectory>) {
    subscriber = test_node->create_subscription<T>(topic_name, rclcpp::QoS{1}, callback);
  } else {
    subscriber = test_node->create_subscription<T>(topic_name, 10, callback);
  }
}

template <typename T>
void setSubscriber(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Subscription<T>> & subscriber, size_t & count)
{
  createSubscription(
    test_node, topic_name, [&count](const typename T::ConstSharedPtr) { count++; }, subscriber);
}

void spinSomeNodes(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node,
  const int repeat_count = 1)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_node);
  executor.add_node(target_node);
  for (int i = 0; i < repeat_count; i++) {
    executor.spin_some(std::chrono::milliseconds(100));
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

template <typename T>
void publishToTargetNode(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<T>::SharedPtr publisher, T data, const int repeat_count = 3)
{
  if (topic_name.empty()) {
    int status;
    char * demangled_name = abi::__cxa_demangle(typeid(data).name(), nullptr, nullptr, &status);
    if (status == 0) {
      throw std::runtime_error(std::string("Topic name for ") + demangled_name + " is empty");
    } else {
      throw std::runtime_error(std::string("Topic name for ") + typeid(data).name() + " is empty");
    }
  }

  test_utils::setPublisher<T>(test_node, topic_name, publisher);
  publisher->publish(data);

  if (target_node->count_subscribers(topic_name) == 0) {
    throw std::runtime_error("No subscriber for " + topic_name);
  }
  test_utils::spinSomeNodes(test_node, target_node, repeat_count);
}

void updateNodeOptions(
  rclcpp::NodeOptions & node_options, const std::vector<std::string> & params_files)
{
  std::vector<const char *> arguments;
  arguments.push_back("--ros-args");
  arguments.push_back("--params-file");
  for (const auto & param_file : params_files) {
    arguments.push_back(param_file.c_str());
    arguments.push_back("--params-file");
  }
  arguments.pop_back();

  node_options.arguments(std::vector<std::string>{arguments.begin(), arguments.end()});
}

Path toPath(const PathWithLaneId & input)
{
  Path output{};
  output.header = input.header;
  output.left_bound = input.left_bound;
  output.right_bound = input.right_bound;
  output.points.resize(input.points.size());
  for (size_t i = 0; i < input.points.size(); ++i) {
    output.points.at(i) = input.points.at(i).point;
  }
  return output;
}

PathWithLaneId loadPathWithLaneIdInYaml()
{
  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("planning_test_utils");
  const auto yaml_path = planning_test_utils_dir + "/config/path_with_lane_id_data.yaml";
  YAML::Node yaml_node = YAML::LoadFile(yaml_path);
  PathWithLaneId path_msg;

  // Convert YAML data to PathWithLaneId message
  // Fill the header
  path_msg.header.stamp.sec = yaml_node["header"]["stamp"]["sec"].as<int>();
  path_msg.header.stamp.nanosec = yaml_node["header"]["stamp"]["nanosec"].as<uint32_t>();
  path_msg.header.frame_id = yaml_node["header"]["frame_id"].as<std::string>();

  // Fill the points
  for (const auto & point_node : yaml_node["points"]) {
    PathPointWithLaneId point;
    // Fill the PathPoint data
    point.point.pose.position.x = point_node["point"]["pose"]["position"]["x"].as<double>();
    point.point.pose.position.y = point_node["point"]["pose"]["position"]["y"].as<double>();
    point.point.pose.position.z = point_node["point"]["pose"]["position"]["z"].as<double>();
    point.point.pose.orientation.x = point_node["point"]["pose"]["orientation"]["x"].as<double>();
    point.point.pose.orientation.y = point_node["point"]["pose"]["orientation"]["y"].as<double>();
    point.point.pose.orientation.z = point_node["point"]["pose"]["orientation"]["z"].as<double>();
    point.point.pose.orientation.w = point_node["point"]["pose"]["orientation"]["w"].as<double>();
    point.point.longitudinal_velocity_mps =
      point_node["point"]["longitudinal_velocity_mps"].as<float>();
    point.point.lateral_velocity_mps = point_node["point"]["lateral_velocity_mps"].as<float>();
    point.point.heading_rate_rps = point_node["point"]["heading_rate_rps"].as<float>();
    point.point.is_final = point_node["point"]["is_final"].as<bool>();
    // Fill the lane_ids
    for (const auto & lane_id_node : point_node["lane_ids"]) {
      point.lane_ids.push_back(lane_id_node.as<int64_t>());
    }

    path_msg.points.push_back(point);
  }

  // Fill the left_bound
  for (const auto & point_node : yaml_node["left_bound"]) {
    Point point;
    // Fill the Point data (left_bound)
    point.x = point_node["x"].as<double>();
    point.y = point_node["y"].as<double>();
    point.z = point_node["z"].as<double>();

    path_msg.left_bound.push_back(point);
  }

  // Fill the right_bound
  for (const auto & point_node : yaml_node["right_bound"]) {
    Point point;
    // Fill the Point data
    point.x = point_node["x"].as<double>();
    point.y = point_node["y"].as<double>();
    point.z = point_node["z"].as<double>();

    path_msg.right_bound.push_back(point);
  }
  return path_msg;
}

}  // namespace test_utils

#endif  // PLANNING_TEST_UTILS__PLANNING_INTERFACE_TEST_MANAGER_UTILS_HPP_
