// Copyright 2024 TIER IV, Inc.
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <lanelet2_core/geometry/LineString.h>

#include <utility>

namespace autoware::test_utils
{

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
  lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);
  if (!errors.empty()) {
    for (const auto & error : errors) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
    }
    return nullptr;
  }

  for (lanelet::Point3d point : map->pointLayer) {
    if (point.hasAttribute("local_x")) {
      point.x() = point.attribute("local_x").asDouble().value();
    }
    if (point.hasAttribute("local_y")) {
      point.y() = point.attribute("local_y").asDouble().value();
    }
  }

  // realign lanelet borders using updated points
  for (lanelet::Lanelet lanelet : map->laneletLayer) {
    auto left = lanelet.leftBound();
    auto right = lanelet.rightBound();
    std::tie(left, right) = lanelet::geometry::align(left, right);
    lanelet.setLeftBound(left);
    lanelet.setRightBound(right);
  }

  return map;
}

LaneletMapBin convertToMapBinMsg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename, const rclcpp::Time & now)
{
  std::string format_version{};
  std::string map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  LaneletMapBin map_bin_msg;
  map_bin_msg.header.stamp = now;
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.version_map_format = format_version;
  map_bin_msg.version_map = map_version;
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

OccupancyGrid makeCostMapMsg(size_t width, size_t height, double resolution)
{
  nav_msgs::msg::OccupancyGrid costmap_msg;

  // create info
  costmap_msg.header.frame_id = "map";
  costmap_msg.info.width = width;
  costmap_msg.info.height = height;
  costmap_msg.info.resolution = static_cast<float>(resolution);

  // create data
  const size_t n_elem = width * height;
  for (size_t i = 0; i < n_elem; ++i) {
    costmap_msg.data.push_back(0.0);
  }
  return costmap_msg;
}

std::string get_absolute_path_to_lanelet_map(
  const std::string & package_name, const std::string & map_filename)
{
  const auto dir = ament_index_cpp::get_package_share_directory(package_name);
  return dir + "/test_map/" + map_filename;
}

std::string get_absolute_path_to_route(
  const std::string & package_name, const std::string & route_filename)
{
  const auto dir = ament_index_cpp::get_package_share_directory(package_name);
  return dir + "/test_route/" + route_filename;
}

std::string get_absolute_path_to_config(
  const std::string & package_name, const std::string & config_filename)
{
  const auto dir = ament_index_cpp::get_package_share_directory(package_name);
  return dir + "/config/" + config_filename;
}

LaneletMapBin make_map_bin_msg(
  const std::string & absolute_path, const double center_line_resolution)
{
  const auto map = loadMap(absolute_path);
  if (!map) {
    return autoware_map_msgs::msg::LaneletMapBin_<std::allocator<void>>{};
  }

  // overwrite centerline
  lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);

  // create map bin msg
  const auto map_bin_msg =
    convertToMapBinMsg(map, absolute_path, rclcpp::Clock(RCL_ROS_TIME).now());
  return map_bin_msg;
}

LaneletMapBin makeMapBinMsg()
{
  return make_map_bin_msg(
    get_absolute_path_to_lanelet_map("autoware_test_utils", "lanelet2_map.osm"));
}

Odometry makeOdometry(const double shift)
{
  Odometry odometry;
  const std::array<double, 4> start_pose{0.0, shift, 0.0, 0.0};
  odometry.pose.pose = createPose(start_pose);
  odometry.header.frame_id = "map";
  return odometry;
}

Odometry makeInitialPose(const double shift)
{
  Odometry current_odometry;
  const auto yaw = 0.9724497591854532;
  const auto shift_x = shift * std::sin(yaw);
  const auto shift_y = shift * std::cos(yaw);
  const std::array<double, 4> start_pose{
    3722.16015625 + shift_x, 73723.515625 + shift_y, 0.233112560494183, yaw};
  current_odometry.pose.pose = autoware::test_utils::createPose(start_pose);
  current_odometry.header.frame_id = "map";
  return current_odometry;
}

TFMessage makeTFMsg(
  rclcpp::Node::SharedPtr target_node, std::string && frame_id, std::string && child_frame_id)
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

Scenario makeScenarioMsg(const std::string & scenario)
{
  Scenario scenario_msg;
  scenario_msg.current_scenario = scenario;
  scenario_msg.activating_scenarios = {scenario};
  return scenario_msg;
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

void spinSomeNodes(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, const int repeat_count)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_node);
  executor.add_node(target_node);
  for (int i = 0; i < repeat_count; i++) {
    executor.spin_some(std::chrono::milliseconds(100));
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
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

PathWithLaneId loadPathWithLaneIdInYaml()
{
  const auto yaml_path =
    get_absolute_path_to_config("autoware_test_utils", "path_with_lane_id_data.yaml");
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

}  // namespace autoware::test_utils
