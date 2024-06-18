// Copyright 2024 The Autoware Contributors
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_storage/bag_metadata.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_internal_msgs/msg/published_time.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// The reaction_analyzer namespace contains utility functions for the Reaction Analyzer tool.
namespace reaction_analyzer
{
using autoware_control_msgs::msg::Control;
using autoware_internal_msgs::msg::PublishedTime;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using sensor_msgs::msg::PointCloud2;

/**
 * @brief A pair containing the ReactionPair.
 * The first element is name of the node that published the PublishedTime msg.
 * The second element is the PublishedTime msg itself
 */
using ReactionPair = std::pair<std::string, PublishedTime>;

/**
 * @brief A map containing the pipeline and the reaction pairs.
 * The key is the time at which the pipeline was executed.
 * The value is a vector of ReactionPair.
 */
using PipelineMap = std::map<rclcpp::Time, std::vector<ReactionPair>>;

/**
 * @brief A vector containing the pipeline and the reaction pairs.
 * The first element is the time at which the pipeline was executed.
 * The second element is a vector of ReactionPair.
 */
using TimestampedReactionPairsVector =
  std::vector<std::tuple<rclcpp::Time, std::vector<ReactionPair>>>;

/**
 * @brief Enum for the different types of messages that can be published.
 */
enum class PublisherMessageType {
  UNKNOWN = 0,
  CAMERA_INFO = 1,
  IMAGE = 2,
  POINTCLOUD2 = 3,
  POSE_WITH_COVARIANCE_STAMPED = 4,
  POSE_STAMPED = 5,
  ODOMETRY = 6,
  IMU = 7,
  CONTROL_MODE_REPORT = 8,
  GEAR_REPORT = 9,
  HAZARD_LIGHTS_REPORT = 10,
  STEERING_REPORT = 11,
  TURN_INDICATORS_REPORT = 12,
  VELOCITY_REPORT = 13,
};

/**
 * @brief Enum for the different types of messages that can be subscribed to.
 */
enum class SubscriberMessageType {
  UNKNOWN = 0,
  CONTROL = 1,
  TRAJECTORY = 2,
  POINTCLOUD2 = 3,
  DETECTED_OBJECTS = 4,
  PREDICTED_OBJECTS = 5,
  TRACKED_OBJECTS = 6,
};

/**
 * @brief Enum for the different types of reactions that can be analyzed.
 */
enum class ReactionType {
  UNKNOWN = 0,
  FIRST_BRAKE = 1,
  SEARCH_ZERO_VEL = 2,
  SEARCH_ENTITY = 3,
};

/**
 * @brief Enum for the different running modes of the Reaction Analyzer.
 */
enum class RunningMode {
  PerceptionPlanning = 0,
  PlanningControl = 1,
};

/**
 * @brief Structs containing the parameters of a pose.
 */
struct PoseParams
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

struct EntityParams
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  double x_l;
  double y_l;
  double z_l;
};

/**
 * @brief Struct containing statistics of the latencies.
 */
struct LatencyStats
{
  double min;
  double max;
  double mean;
  double median;
  double std_dev;
};

/**
 * @brief Convert string to SubscriberMessageType.
 */
SubscriberMessageType get_subscriber_message_type(const std::string & message_type);

/**
 * @brief Convert string to PublisherMessageType.
 */
PublisherMessageType get_publisher_message_type(const std::string & message_type);

/**
 * @brief Get the PublisherMessageType for a given topic.
 */
PublisherMessageType get_publisher_message_type_for_topic(
  const std::vector<rosbag2_storage::TopicInformation> & topics, const std::string & topic_name);

/**
 * @brief Convert string to ReactionType.
 */
ReactionType get_reaction_type(const std::string & reaction_type);

/**
 * @brief Calculates the statistics of a vector of latencies.
 * @param latency_vec The vector of latencies.
 * @return The statistics of the latencies.
 */
LatencyStats calculate_statistics(const std::vector<double> & latency_vec);

/**
 * @brief Generates a UUID message from a string.
 * @param input The string to generate the UUID from.
 * @return The generated UUID message.
 */
unique_identifier_msgs::msg::UUID generate_uuid_msg(const std::string & input);

/**
 * @brief Generate pose from PoseParams.
 * @param pose_params
 * @return Pose
 */
geometry_msgs::msg::Pose pose_params_to_pose(const PoseParams & pose_params);

/**
 * @brief Generate entity pose from entity params.
 * @param entity_params
 * @return Pose
 */
geometry_msgs::msg::Pose create_entity_pose(const EntityParams & entity_params);

/**
 * @brief Generate entity pose from entity params.
 * @param entity_params
 * @return Pose
 */
double calculate_entity_search_radius(const EntityParams & entity_params);

/**
 * @brief Create a pointcloud from entity params.
 * @param entity_params
 * @param pointcloud_sampling_distance
 * @return PointCloud2::SharedPtr
 */
PointCloud2::SharedPtr create_entity_pointcloud_ptr(
  const EntityParams & entity_params, const double pointcloud_sampling_distance);

/**
 * @brief Create a predicted objects message from entity params.
 * @param entity_params
 * @return PredictedObjects::SharedPtr
 */
PredictedObjects::SharedPtr create_entity_predicted_objects_ptr(const EntityParams & entity_params);

/**
 * @brief Creates a subscription option with a new callback group.
 *
 * @param node A pointer to the node for which the subscription options are being created.
 * @return The created SubscriptionOptions.
 */
rclcpp::SubscriptionOptions create_subscription_options(rclcpp::Node * node);

/**
 * @brief Creates a visualization marker for a polyhedron based on the provided entity parameters.
 *
 * @param params The parameters of the entity for which the marker is to be created. It includes the
 * position, orientation, and dimensions of the entity.
 * @return The created visualization marker for the polyhedron.
 */
visualization_msgs::msg::Marker create_polyhedron_marker(const EntityParams & params);

/**
 * @brief Splits a string by a given delimiter.
 *
 * @param str The string to be split.
 * @param delimiter The delimiter to split the string by.
 * @return A vector of strings, each of which is a segment of the original string split by the
 * delimiter.
 */
std::vector<std::string> split(const std::string & str, char delimiter);

/**
 * @brief Checks if a folder exists.
 *
 * @param path The path to the folder.
 * @return True if the folder exists, false otherwise.
 */
bool does_folder_exist(const std::string & path);

/**
 * @brief Get the index of the trajectory point that is a certain distance away from the current
 * point.
 *
 * @param traj The trajectory to search.
 * @param curr_id The index of the current point in the trajectory.
 * @param distance The distance to search for a point.
 * @return The index of the point that is at least the specified distance away from the current
 * point.
 */
size_t get_index_after_distance(
  const Trajectory & traj, const size_t curr_id, const double distance);

/**
 * @brief Search for a pointcloud near the pose.
 * @param pcl_pointcloud, pose, search_radius
 * @return bool
 */
bool search_pointcloud_near_pose(
  const pcl::PointCloud<pcl::PointXYZ> & pcl_pointcloud, const geometry_msgs::msg::Pose & pose,
  const double search_radius);
/**
 *
 * @brief Search for a predicted object near the pose.
 * @param predicted_objects, pose, search_radius
 * @return bool
 */
bool search_predicted_objects_near_pose(
  const PredictedObjects & predicted_objects, const geometry_msgs::msg::Pose & pose,
  const double search_radius);
/**
 * @brief Search for a detected object near the pose.
 * @param detected_objects, pose, search_radius
 * @return bool
 */
bool search_detected_objects_near_pose(
  const DetectedObjects & detected_objects, const geometry_msgs::msg::Pose & pose,
  const double search_radius);
/**
 * @brief Search for a tracked object near the pose.
 * @param tracked_objects, pose, search_radius
 * @return bool
 */
bool search_tracked_objects_near_pose(
  const TrackedObjects & tracked_objects, const geometry_msgs::msg::Pose & pose,
  const double search_radius);

/**
 * Calculates the time difference in milliseconds between two rclcpp::Time instances.
 *
 * @param start The start time.
 * @param end The end time.
 * @return The time difference in milliseconds as a double.
 */
double calculate_time_diff_ms(const rclcpp::Time & start, const rclcpp::Time & end);

/**
 * @brief Converts a PipelineMap to a PipelineMapVector.
 *
 * @param pipelineMap The PipelineMap to be converted.
 * @return The PipelineMapVector that is equivalent to the PipelineMap.
 */
TimestampedReactionPairsVector convert_pipeline_map_to_sorted_vector(
  const PipelineMap & pipelineMap);

/**
 * @brief Writes the results to a file.
 *
 * @param node A pointer to the node for which the results are being written.
 * @param output_file_path The path to the file where the results will be written.
 * @param node_running_mode The running mode of the node.
 * @param pipeline_map_vector The vector of PipelineMap containing the results to be written.
 */
void write_results(
  rclcpp::Node * node, const std::string & output_file_path, const RunningMode & node_running_mode,
  const std::vector<PipelineMap> & pipeline_map_vector);
}  // namespace reaction_analyzer

#endif  // UTILS_HPP_
