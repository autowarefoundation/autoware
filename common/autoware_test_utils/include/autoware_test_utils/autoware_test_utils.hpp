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

#ifndef AUTOWARE_TEST_UTILS__AUTOWARE_TEST_UTILS_HPP_
#define AUTOWARE_TEST_UTILS__AUTOWARE_TEST_UTILS_HPP_

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <component_interface_specs/planning.hpp>
#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
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
#include <unordered_map>
#include <vector>

namespace autoware::test_utils
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::Trajectory;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
using autoware::universe_utils::createPoint;
using autoware::universe_utils::createQuaternionFromRPY;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using tf2_msgs::msg::TFMessage;
using tier4_planning_msgs::msg::Scenario;
using unique_identifier_msgs::msg::UUID;

/**
 * @brief Creates a Pose message with the specified position and orientation.
 *
 * This function initializes a geometry_msgs::msg::Pose message with the
 * given position (x, y, z) and orientation (roll, pitch, yaw).
 *
 * @param x The x-coordinate of the position.
 * @param y The y-coordinate of the position.
 * @param z The z-coordinate of the position.
 * @param roll The roll component of the orientation (in radians).
 * @param pitch The pitch component of the orientation (in radians).
 * @param yaw The yaw component of the orientation (in radians).
 * @return A geometry_msgs::msg::Pose message with the specified position
 * and orientation.
 */
geometry_msgs::msg::Pose createPose(
  double x, double y, double z, double roll, double pitch, double yaw);

/**
 * @brief Creates a Pose message from a 4-element array representing a 3D pose.
 *
 * This function initializes a geometry_msgs::msg::Pose message using a
 * std::array of four doubles, where the first three elements represent the
 * position (x, y, z) and the fourth element represents the yaw orientation.
 * The roll and pitch orientations are assumed to be zero.
 *
 * @param pose3d A std::array of four doubles representing the 3D pose. The
 * first element is the x-coordinate, the second is the y-coordinate, the
 * third is the z-coordinate, and the fourth is the yaw orientation.
 * @return A geometry_msgs::msg::Pose message with the specified position
 * and yaw orientation, with roll and pitch set to zero.
 */
geometry_msgs::msg::Pose createPose(const std::array<double, 4> & pose3d);

/**
 * @brief Creates a LaneletSegment with the specified ID.
 *
 * Initializes a LaneletSegment containing a single LaneletPrimitive with the
 * given ID and a primitive type of "lane".
 *
 * @param id The ID for the LaneletPrimitive and preferred primitive.
 * @return A LaneletSegment with the specified ID.
 */
LaneletSegment createLaneletSegment(int id);

/**
 * @brief Loads a Lanelet map from a specified file.
 *
 * This function loads a Lanelet2 map using the given filename. It uses the MGRS
 * projector and checks for any errors during the loading process. If errors
 * are found, they are logged, and the function returns nullptr.
 *
 * @param lanelet2_filename The filename of the Lanelet2 map to load.
 * @return A LaneletMapPtr to the loaded map, or nullptr if there were errors.
 */
lanelet::LaneletMapPtr loadMap(const std::string & lanelet2_filename);

/**
 * @brief Converts a Lanelet map to a LaneletMapBin message.
 *
 * This function converts a given Lanelet map to a LaneletMapBin message. It also
 * parses the format and map versions from the specified filename and includes
 * them in the message. The timestamp for the message is set to the provided time.
 *
 * @param map The Lanelet map to convert.
 * @param lanelet2_filename The filename of the Lanelet2 map, used to parse format and map versions.
 * @param now The current time to set in the message header.
 * @return A LaneletMapBin message containing the converted map data.
 */
LaneletMapBin convertToMapBinMsg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename,
  const rclcpp::Time & now);

/**
 * @brief Creates a normal Lanelet route with predefined start and goal poses.
 *
 * This function initializes a LaneletRoute with a predefined start and goal pose.
 *
 * @return A LaneletRoute with the specified start and goal poses.
 */
LaneletRoute makeNormalRoute();

/**
 * @brief Creates an OccupancyGrid message with the specified dimensions and resolution.
 *
 * This function initializes an OccupancyGrid message with given width, height, and resolution.
 * All cells in the grid are initialized to zero.
 *
 * @param width The width of the occupancy grid.
 * @param height The height of the occupancy grid.
 * @param resolution The resolution of the occupancy grid.
 * @return An OccupancyGrid message with the specified dimensions and resolution.
 */
OccupancyGrid makeCostMapMsg(size_t width = 150, size_t height = 150, double resolution = 0.2);

/**
 * @brief Get the absolute path to the lanelet map file.
 *
 * This function retrieves the absolute path to a lanelet map file located in the
 * package's share directory under the "test_map" folder.
 *
 * @param package_name The name of the package containing the map file.
 * @param map_filename The name of the map file.
 * @return A string representing the absolute path to the lanelet map file.
 */
std::string get_absolute_path_to_lanelet_map(
  const std::string & package_name, const std::string & map_filename);

/**
 * @brief Get the absolute path to the route file.
 *
 * This function retrieves the absolute path to a route file located in the
 * package's share directory under the "test_route" folder.
 *
 * @param package_name The name of the package containing the route file.
 * @param route_filename The name of the route file.
 * @return A string representing the absolute path to the route file.
 */
std::string get_absolute_path_to_route(
  const std::string & package_name, const std::string & route_filename);

/**
 * @brief Get the absolute path to the config file.
 *
 * This function retrieves the absolute path to a route file located in the
 * package's share directory under the "config" folder.
 *
 * @param package_name The name of the package containing the route file.
 * @param route_filename The name of the config file.
 * @return A string representing the absolute path to the config file.
 */
std::string get_absolute_path_to_config(
  const std::string & package_name, const std::string & config_filename);

/**
 * @brief Creates a LaneletMapBin message from a Lanelet map file.
 *
 * This function loads a Lanelet map from the given file, overwrites the
 * centerline with the specified resolution, and converts the map to a LaneletMapBin message.
 *
 * @param absolute_path The absolute path to the Lanelet2 map file.
 * @param center_line_resolution The resolution for the centerline.
 * @return A LaneletMapBin message containing the map data.
 */
LaneletMapBin make_map_bin_msg(
  const std::string & absolute_path, const double center_line_resolution = 5.0);

/**
 * @brief Creates a LaneletMapBin message using a predefined Lanelet2 map file.
 *
 * This function loads a lanelet2_map.osm from the test_map folder in the
 * autoware_test_utils package, overwrites the centerline with a resolution of 5.0,
 * and converts the map to a LaneletMapBin message.
 *
 * @return A LaneletMapBin message containing the map data.
 */
LaneletMapBin makeMapBinMsg();

/**
 * @brief Creates an Odometry message with a specified shift.
 *
 * This function initializes an Odometry message with a pose shifted by the given amount at y
 * direction. x pose, z pose, and yaw angle remains zero.
 *
 * @param shift The amount by which to shift the pose.
 * @return An Odometry message with the specified shift.
 */
Odometry makeOdometry(const double shift = 0.0);

/**
 * @brief Creates an initial Odometry message with a specified shift.
 *
 * This function initializes an Odometry message with a pose shifted by the given amount,
 * accounting for a specific yaw angle.
 *
 * @param shift The amount by which to shift the pose.
 * @return An Odometry message with the specified shift.
 */
Odometry makeInitialPose(const double shift = 0.0);

/**
 * @brief Creates a TFMessage with the specified frame IDs.
 *
 * This function initializes a TFMessage with a transform containing the given frame IDs.
 * The transform includes a specific translation and rotation.
 *
 * @param target_node The node to use for obtaining the current time.
 * @param frame_id The ID of the parent frame.
 * @param child_frame_id The ID of the child frame.
 * @return A TFMessage containing the transform.
 */
TFMessage makeTFMsg(
  rclcpp::Node::SharedPtr target_node, std::string && frame_id = "",
  std::string && child_frame_id = "");

/**
 * @brief Creates a Scenario message with the specified scenario.
 *
 * This function initializes a Scenario message with the current scenario and a list of activating
 * scenarios.
 *
 * @param scenario The name of the current scenario.
 * @return A Scenario message with the specified scenario.
 */
Scenario makeScenarioMsg(const std::string & scenario);

/**
 * @brief Combines two sets of consecutive RouteSections.
 *
 * This function combines two sets of RouteSections, removing the overlapping end section from the
 * first set.
 *
 * @param route_sections1 The first set of RouteSections.
 * @param route_sections2 The second set of RouteSections.
 * @return A combined set of RouteSections.
 */
RouteSections combineConsecutiveRouteSections(
  const RouteSections & route_sections1, const RouteSections & route_sections2);

/**
 * @brief Creates a predefined behavior Lanelet route.
 *
 * This function initializes a LaneletRoute with predefined start and goal poses,
 * a list of lanelet segment IDs, and a fixed UUID.
 * this is for the test lanelet2_map.osm
 * file hash: a9f84cff03b55a64917bc066451276d2293b0a54f5c088febca0c7fdf2f245d5
 *
 * @return A LaneletRoute with the specified attributes.
 */
LaneletRoute makeBehaviorNormalRoute();

/**
 * @brief Spins multiple ROS nodes a specified number of times.
 *
 * This function spins the given test and target nodes for the specified number of iterations.
 *
 * @param test_node The test node to spin.
 * @param target_node The target node to spin.
 * @param repeat_count The number of times to spin the nodes.
 */
void spinSomeNodes(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node,
  const int repeat_count = 1);

/**
 * @brief Updates node options with parameter files.
 *
 * This function updates the given node options to include the specified parameter files.
 *
 * @param node_options The node options to update.
 * @param params_files A vector of parameter file paths to add to the node options.
 */
void updateNodeOptions(
  rclcpp::NodeOptions & node_options, const std::vector<std::string> & params_files);

/**
 * @brief Loads a PathWithLaneId message from a YAML file.
 *
 * This function loads a PathWithLaneId message from a YAML file located in the
 * autoware_test_utils package.
 *
 * @return A PathWithLaneId message containing the loaded data.
 */
PathWithLaneId loadPathWithLaneIdInYaml();

/**
 * @brief Generates a trajectory with specified parameters.
 *
 * This function generates a trajectory of type T with a given number of points,
 * point interval, velocity, initial theta, delta theta, and optionally an
 * overlapping point index.
 *
 * @tparam T The type of the trajectory.
 * @param num_points The number of points in the trajectory.
 * @param point_interval The distance between consecutive points.
 * @param velocity The longitudinal velocity for each point.
 * @param init_theta The initial theta angle.
 * @param delta_theta The change in theta per point.
 * @param overlapping_point_index The index of the point to overlap (default is max size_t value).
 * @return A trajectory of type T with the specified parameters.
 */
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

/**
 * @brief Creates a publisher with appropriate QoS settings.
 *
 * This function creates a publisher for a given topic name and message type with appropriate
 * QoS settings, depending on the message type.
 *
 * @tparam T The type of the message to publish.
 * @param test_node The node to create the publisher on.
 * @param topic_name The name of the topic to publish to.
 * @param publisher A reference to the publisher to be created.
 */
template <typename T>
void createPublisherWithQoS(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Publisher<T>> & publisher)
{
  if constexpr (
    std::is_same_v<T, LaneletRoute> || std::is_same_v<T, LaneletMapBin> ||
    std::is_same_v<T, OperationModeState>) {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();
    publisher = rclcpp::create_publisher<T>(test_node, topic_name, qos);
  } else {
    publisher = rclcpp::create_publisher<T>(test_node, topic_name, 1);
  }
}

/**
 * @brief Sets up a publisher for a given topic.
 *
 * This function sets up a publisher for a given topic using createPublisherWithQoS.
 *
 * @tparam T The type of the message to publish.
 * @param test_node The node to create the publisher on.
 * @param topic_name The name of the topic to publish to.
 * @param publisher A reference to the publisher to be set.
 */
template <typename T>
void setPublisher(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Publisher<T>> & publisher)
{
  createPublisherWithQoS(test_node, topic_name, publisher);
}

/**
 * @brief Creates a subscription with appropriate QoS settings.
 *
 * This function creates a subscription for a given topic name and message type with appropriate
 * QoS settings, depending on the message type.
 *
 * @tparam T The type of the message to subscribe to.
 * @param test_node The node to create the subscription on.
 * @param topic_name The name of the topic to subscribe to.
 * @param callback The callback function to call when a message is received.
 * @param subscriber A reference to the subscription to be created.
 */
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

/**
 * @brief Sets up a subscriber for a given topic.
 *
 * This function sets up a subscriber for a given topic using createSubscription.
 *
 * @tparam T The type of the message to subscribe to.
 * @param test_node The node to create the subscription on.
 * @param topic_name The name of the topic to subscribe to.
 * @param subscriber A reference to the subscription to be set.
 * @param count A reference to a counter that increments on message receipt.
 */
template <typename T>
void setSubscriber(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Subscription<T>> & subscriber, size_t & count)
{
  createSubscription(
    test_node, topic_name, [&count](const typename T::ConstSharedPtr) { count++; }, subscriber);
}

/**
 * @brief Publishes data to a target node.
 *
 * This function publishes data to a target node on a given topic, ensuring that the topic has
 * subscribers and retrying a specified number of times.
 *
 * @tparam T The type of the message to publish.
 * @param test_node The node to create the publisher on.
 * @param target_node The target node to publish the message to.
 * @param topic_name The name of the topic to publish to.
 * @param publisher A shared pointer to the publisher.
 * @param data The data to publish.
 * @param repeat_count The number of times to retry publishing.
 */
template <typename T>
void publishToTargetNode(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<T>::SharedPtr publisher, T data, const int repeat_count = 3)
{
  if (topic_name.empty()) {
    int status = 1;
    char * demangled_name = abi::__cxa_demangle(typeid(data).name(), nullptr, nullptr, &status);
    if (status == 0) {
      throw std::runtime_error(std::string("Topic name for ") + demangled_name + " is empty");
    }
    throw std::runtime_error(std::string("Topic name for ") + typeid(data).name() + " is empty");
  }

  autoware::test_utils::setPublisher<T>(test_node, topic_name, publisher);
  publisher->publish(data);

  if (target_node->count_subscribers(topic_name) == 0) {
    throw std::runtime_error("No subscriber for " + topic_name);
  }
  autoware::test_utils::spinSomeNodes(test_node, target_node, repeat_count);
}

/**
 * @brief Manages publishing and subscribing to ROS topics for testing Autoware.
 *
 * The AutowareTestManager class provides utility functions to facilitate
 * the publishing of messages to specified topics and the setting up of
 * subscribers to listen for messages on specified topics. This class
 * simplifies the setup of test environments in Autoware.
 */
class AutowareTestManager
{
public:
  AutowareTestManager()
  {
    test_node_ = std::make_shared<rclcpp::Node>("autoware_test_manager_node");
    pub_clock_ = test_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
  }

  template <typename MessageType>
  void test_pub_msg(
    rclcpp::Node::SharedPtr target_node, const std::string & topic_name, MessageType & msg)
  {
    if (publishers_.find(topic_name) == publishers_.end()) {
      auto publisher = test_node_->create_publisher<MessageType>(topic_name, 10);
      publishers_[topic_name] = std::static_pointer_cast<rclcpp::PublisherBase>(publisher);
    }

    auto publisher =
      std::dynamic_pointer_cast<rclcpp::Publisher<MessageType>>(publishers_[topic_name]);

    autoware::test_utils::publishToTargetNode(test_node_, target_node, topic_name, publisher, msg);
    RCLCPP_INFO(test_node_->get_logger(), "Published message on topic '%s'", topic_name.c_str());
  }

  template <typename MessageType>
  void set_subscriber(
    const std::string & topic_name,
    std::function<void(const typename MessageType::ConstSharedPtr)> callback)
  {
    if (subscribers_.find(topic_name) == subscribers_.end()) {
      std::shared_ptr<rclcpp::Subscription<MessageType>> subscriber;
      autoware::test_utils::createSubscription<MessageType>(
        test_node_, topic_name, callback, subscriber);
      subscribers_[topic_name] = std::static_pointer_cast<rclcpp::SubscriptionBase>(subscriber);
    } else {
      RCLCPP_WARN(test_node_->get_logger(), "Subscriber %s already set.", topic_name.c_str());
    }
  }

  /**
   * @brief Publishes a ROS Clock message with the specified time.
   *
   * This function publishes a ROS Clock message with the specified time.
   * Be careful when using this function, as it can affect the behavior of
   * the system under test. Consider using ament_add_ros_isolated_gtest to
   * isolate the system under test from the ROS clock.
   *
   * @param time The time to publish.
   */
  void jump_clock(const rclcpp::Time & time)
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = time;
    pub_clock_->publish(clock);
  }

protected:
  // Publisher
  std::unordered_map<std::string, std::shared_ptr<rclcpp::PublisherBase>> publishers_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> subscribers_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_clock_;

  // Node
  rclcpp::Node::SharedPtr test_node_;
};  // class AutowareTestManager

}  // namespace autoware::test_utils

#endif  // AUTOWARE_TEST_UTILS__AUTOWARE_TEST_UTILS_HPP_
