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

#include "utils.hpp"

namespace reaction_analyzer
{
SubscriberMessageType get_subscriber_message_type(const std::string & message_type)
{
  if (message_type == "autoware_control_msgs/msg/Control") {
    return SubscriberMessageType::CONTROL;
  }
  if (message_type == "autoware_planning_msgs/msg/Trajectory") {
    return SubscriberMessageType::TRAJECTORY;
  }
  if (message_type == "sensor_msgs/msg/PointCloud2") {
    return SubscriberMessageType::POINTCLOUD2;
  }
  if (message_type == "autoware_perception_msgs/msg/PredictedObjects") {
    return SubscriberMessageType::PREDICTED_OBJECTS;
  }
  if (message_type == "autoware_perception_msgs/msg/DetectedObjects") {
    return SubscriberMessageType::DETECTED_OBJECTS;
  }
  if (message_type == "autoware_perception_msgs/msg/TrackedObjects") {
    return SubscriberMessageType::TRACKED_OBJECTS;
  }
  return SubscriberMessageType::UNKNOWN;
}

PublisherMessageType get_publisher_message_type(const std::string & message_type)
{
  if (message_type == "sensor_msgs/msg/PointCloud2") {
    return PublisherMessageType::POINTCLOUD2;
  }
  if (message_type == "sensor_msgs/msg/CameraInfo") {
    return PublisherMessageType::CAMERA_INFO;
  }
  if (message_type == "sensor_msgs/msg/Image") {
    return PublisherMessageType::IMAGE;
  }
  if (message_type == "geometry_msgs/msg/PoseWithCovarianceStamped") {
    return PublisherMessageType::POSE_WITH_COVARIANCE_STAMPED;
  }
  if (message_type == "geometry_msgs/msg/PoseStamped") {
    return PublisherMessageType::POSE_STAMPED;
  }
  if (message_type == "nav_msgs/msg/Odometry") {
    return PublisherMessageType::ODOMETRY;
  }
  if (message_type == "sensor_msgs/msg/Imu") {
    return PublisherMessageType::IMU;
  }
  if (message_type == "autoware_vehicle_msgs/msg/ControlModeReport") {
    return PublisherMessageType::CONTROL_MODE_REPORT;
  }
  if (message_type == "autoware_vehicle_msgs/msg/GearReport") {
    return PublisherMessageType::GEAR_REPORT;
  }
  if (message_type == "autoware_vehicle_msgs/msg/HazardLightsReport") {
    return PublisherMessageType::HAZARD_LIGHTS_REPORT;
  }
  if (message_type == "autoware_vehicle_msgs/msg/SteeringReport") {
    return PublisherMessageType::STEERING_REPORT;
  }
  if (message_type == "autoware_vehicle_msgs/msg/TurnIndicatorsReport") {
    return PublisherMessageType::TURN_INDICATORS_REPORT;
  }
  if (message_type == "autoware_vehicle_msgs/msg/VelocityReport") {
    return PublisherMessageType::VELOCITY_REPORT;
  }
  return PublisherMessageType::UNKNOWN;
}

PublisherMessageType get_publisher_message_type_for_topic(
  const std::vector<rosbag2_storage::TopicInformation> & topics, const std::string & topic_name)
{
  auto it = std::find_if(topics.begin(), topics.end(), [&topic_name](const auto & topic) {
    return topic.topic_metadata.name == topic_name;
  });
  if (it != topics.end()) {
    return get_publisher_message_type(it->topic_metadata.type);  // Return the message type if found
  }
  return PublisherMessageType::UNKNOWN;
}

ReactionType get_reaction_type(const std::string & reaction_type)
{
  if (reaction_type == "first_brake_params") {
    return ReactionType::FIRST_BRAKE;
  }
  if (reaction_type == "search_zero_vel_params") {
    return ReactionType::SEARCH_ZERO_VEL;
  }
  if (reaction_type == "search_entity_params") {
    return ReactionType::SEARCH_ENTITY;
  }
  return ReactionType::UNKNOWN;
}

rclcpp::SubscriptionOptions create_subscription_options(rclcpp::Node * node)
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}

visualization_msgs::msg::Marker create_polyhedron_marker(const EntityParams & params)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Time(0);
  marker.ns = "entity";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = params.x;
  marker.pose.position.y = params.y;
  marker.pose.position.z = params.z;

  tf2::Quaternion quaternion;
  quaternion.setRPY(
    autoware::universe_utils::deg2rad(params.roll), autoware::universe_utils::deg2rad(params.pitch),
    autoware::universe_utils::deg2rad(params.yaw));
  marker.pose.orientation = tf2::toMsg(quaternion);

  marker.scale.x = 0.1;  // Line width

  marker.color.a = 1.0;  // Alpha
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  // Define the 8 corners of the polyhedron
  geometry_msgs::msg::Point p1, p2, p3, p4, p5, p6, p7, p8;

  p1.x = params.x_l / 2.0;
  p1.y = params.y_l / 2.0;
  p1.z = params.z_l / 2.0;
  p2.x = -params.x_l / 2.0;
  p2.y = params.y_l / 2.0;
  p2.z = params.z_l / 2.0;
  p3.x = -params.x_l / 2.0;
  p3.y = -params.y_l / 2.0;
  p3.z = params.z_l / 2.0;
  p4.x = params.x_l / 2.0;
  p4.y = -params.y_l / 2.0;
  p4.z = params.z_l / 2.0;
  p5.x = params.x_l / 2.0;
  p5.y = params.y_l / 2.0;
  p5.z = -params.z_l / 2.0;
  p6.x = -params.x_l / 2.0;
  p6.y = params.y_l / 2.0;
  p6.z = -params.z_l / 2.0;
  p7.x = -params.x_l / 2.0;
  p7.y = -params.y_l / 2.0;
  p7.z = -params.z_l / 2.0;
  p8.x = params.x_l / 2.0;
  p8.y = -params.y_l / 2.0;
  p8.z = -params.z_l / 2.0;

  // Add points to the marker
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.points.push_back(p3);
  marker.points.push_back(p4);
  marker.points.push_back(p1);

  marker.points.push_back(p5);
  marker.points.push_back(p6);
  marker.points.push_back(p7);
  marker.points.push_back(p8);
  marker.points.push_back(p5);

  marker.points.push_back(p1);
  marker.points.push_back(p5);
  marker.points.push_back(p6);
  marker.points.push_back(p2);
  marker.points.push_back(p3);
  marker.points.push_back(p7);
  marker.points.push_back(p4);
  marker.points.push_back(p8);

  return marker;
}

std::vector<std::string> split(const std::string & str, char delimiter)
{
  std::vector<std::string> elements;
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, delimiter)) {
    elements.push_back(item);
  }
  return elements;
}

bool does_folder_exist(const std::string & path)
{
  return std::filesystem::exists(path) && std::filesystem::is_directory(path);
}

size_t get_index_after_distance(
  const Trajectory & traj, const size_t curr_id, const double distance)
{
  const TrajectoryPoint & curr_p = traj.points.at(curr_id);

  size_t target_id = curr_id;
  for (size_t traj_id = curr_id + 1; traj_id < traj.points.size(); ++traj_id) {
    const double current_distance =
      autoware::universe_utils::calcDistance3d(traj.points.at(traj_id), curr_p);
    if (current_distance >= distance) {
      break;
    }
    target_id = traj_id;
  }
  return target_id;
}

double calculate_time_diff_ms(const rclcpp::Time & start, const rclcpp::Time & end)
{
  const auto duration = end - start;

  const auto duration_ns = duration.to_chrono<std::chrono::nanoseconds>();
  return static_cast<double>(duration_ns.count()) / 1e6;
}

TimestampedReactionPairsVector convert_pipeline_map_to_sorted_vector(
  const PipelineMap & pipelineMap)
{
  std::vector<std::tuple<rclcpp::Time, std::vector<ReactionPair>>> sorted_vector;

  for (const auto & entry : pipelineMap) {
    auto sorted_reactions = entry.second;
    // Sort the vector of ReactionPair based on the published stamp
    std::sort(
      sorted_reactions.begin(), sorted_reactions.end(),
      [](const ReactionPair & a, const ReactionPair & b) {
        return rclcpp::Time(a.second.published_stamp) < rclcpp::Time(b.second.published_stamp);
      });

    // Add to the vector as a tuple
    sorted_vector.emplace_back(std::make_tuple(entry.first, sorted_reactions));
  }

  // Sort the vector of tuples by rclcpp::Time
  std::sort(sorted_vector.begin(), sorted_vector.end(), [](const auto & a, const auto & b) {
    return std::get<0>(a) < std::get<0>(b);
  });

  return sorted_vector;
}

unique_identifier_msgs::msg::UUID generate_uuid_msg(const std::string & input)
{
  static auto generate_uuid = boost::uuids::name_generator(boost::uuids::random_generator()());
  const auto uuid = generate_uuid(input);

  unique_identifier_msgs::msg::UUID uuid_msg;
  std::copy(uuid.begin(), uuid.end(), uuid_msg.uuid.begin());
  return uuid_msg;
}

geometry_msgs::msg::Pose create_entity_pose(const EntityParams & entity_params)
{
  geometry_msgs::msg::Pose entity_pose;
  entity_pose.position.x = entity_params.x;
  entity_pose.position.y = entity_params.y;
  entity_pose.position.z = entity_params.z;

  tf2::Quaternion entity_q_orientation;
  entity_q_orientation.setRPY(
    autoware::universe_utils::deg2rad(entity_params.roll),
    autoware::universe_utils::deg2rad(entity_params.pitch),
    autoware::universe_utils::deg2rad(entity_params.yaw));
  entity_pose.orientation = tf2::toMsg(entity_q_orientation);
  return entity_pose;
}

geometry_msgs::msg::Pose pose_params_to_pose(const PoseParams & pose_params)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = pose_params.x;
  pose.position.y = pose_params.y;
  pose.position.z = pose_params.z;

  tf2::Quaternion pose_q_orientation;
  pose_q_orientation.setRPY(
    autoware::universe_utils::deg2rad(pose_params.roll),
    autoware::universe_utils::deg2rad(pose_params.pitch),
    autoware::universe_utils::deg2rad(pose_params.yaw));
  pose.orientation = tf2::toMsg(pose_q_orientation);
  return pose;
}

PointCloud2::SharedPtr create_entity_pointcloud_ptr(
  const EntityParams & entity_params, const double pointcloud_sampling_distance)
{
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  tf2::Quaternion entity_q_orientation;

  entity_q_orientation.setRPY(
    autoware::universe_utils::deg2rad(entity_params.roll),
    autoware::universe_utils::deg2rad(entity_params.pitch),
    autoware::universe_utils::deg2rad(entity_params.yaw));
  tf2::Transform tf(entity_q_orientation);
  const auto origin = tf2::Vector3(entity_params.x, entity_params.y, entity_params.z);
  tf.setOrigin(origin);

  const double it_x = entity_params.x_l / pointcloud_sampling_distance;
  const double it_y = entity_params.y_l / pointcloud_sampling_distance;
  const double it_z = entity_params.z_l / pointcloud_sampling_distance;

  // Sample the box and rotate
  for (int i = 0; i <= it_z; ++i) {
    for (int j = 0; j <= it_y; ++j) {
      for (int k = 0; k <= it_x; ++k) {
        const double p_x = -entity_params.x_l / 2 + k * pointcloud_sampling_distance;
        const double p_y = -entity_params.y_l / 2 + j * pointcloud_sampling_distance;
        const double p_z = -entity_params.z_l / 2 + i * pointcloud_sampling_distance;
        const auto tmp = tf2::Vector3(p_x, p_y, p_z);
        tf2::Vector3 data_out = tf * tmp;
        point_cloud.emplace_back(pcl::PointXYZ(
          static_cast<float>(data_out.x()), static_cast<float>(data_out.y()),
          static_cast<float>(data_out.z())));
      }
    }
  }
  PointCloud2::SharedPtr entity_pointcloud_ptr;
  entity_pointcloud_ptr = std::make_shared<PointCloud2>();
  pcl::toROSMsg(point_cloud, *entity_pointcloud_ptr);
  return entity_pointcloud_ptr;
}

PredictedObjects::SharedPtr create_entity_predicted_objects_ptr(const EntityParams & entity_params)
{
  unique_identifier_msgs::msg::UUID uuid_msg;

  PredictedObject obj;
  const auto entity_pose = create_entity_pose(entity_params);
  geometry_msgs::msg::Vector3 dimension;
  dimension.set__x(entity_params.x_l);
  dimension.set__y(entity_params.y_l);
  dimension.set__z(entity_params.z_l);
  obj.shape.set__dimensions(dimension);

  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.existence_probability = 1.0;
  obj.kinematics.initial_pose_with_covariance.pose = entity_pose;

  autoware_perception_msgs::msg::PredictedPath path;
  path.confidence = 1.0;
  path.path.emplace_back(entity_pose);
  obj.kinematics.predicted_paths.emplace_back(path);

  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 1.0;
  obj.classification.emplace_back(classification);
  obj.set__object_id(generate_uuid_msg("test_obstacle"));

  PredictedObjects pred_objects;
  pred_objects.objects.emplace_back(obj);
  return std::make_shared<PredictedObjects>(pred_objects);
}

double calculate_entity_search_radius(const EntityParams & entity_params)
{
  return std::sqrt(
           std::pow(entity_params.x_l, 2) + std::pow(entity_params.y_l, 2) +
           std::pow(entity_params.z_l, 2)) /
         2.0;
}

bool search_pointcloud_near_pose(
  const pcl::PointCloud<pcl::PointXYZ> & pcl_pointcloud, const geometry_msgs::msg::Pose & pose,
  const double search_radius)
{
  return std::any_of(
    pcl_pointcloud.points.begin(), pcl_pointcloud.points.end(),
    [pose, search_radius](const auto & point) {
      return autoware::universe_utils::calcDistance3d(pose.position, point) <= search_radius;
    });
}

bool search_predicted_objects_near_pose(
  const PredictedObjects & predicted_objects, const geometry_msgs::msg::Pose & pose,
  const double search_radius)
{
  return std::any_of(
    predicted_objects.objects.begin(), predicted_objects.objects.end(),
    [pose, search_radius](const PredictedObject & object) {
      return autoware::universe_utils::calcDistance3d(
               pose.position, object.kinematics.initial_pose_with_covariance.pose.position) <=
             search_radius;
    });
  ;
}

bool search_detected_objects_near_pose(
  const DetectedObjects & detected_objects, const geometry_msgs::msg::Pose & pose,
  const double search_radius)
{
  return std::any_of(
    detected_objects.objects.begin(), detected_objects.objects.end(),
    [pose, search_radius](const DetectedObject & object) {
      return autoware::universe_utils::calcDistance3d(
               pose.position, object.kinematics.pose_with_covariance.pose.position) <=
             search_radius;
    });
}

bool search_tracked_objects_near_pose(
  const TrackedObjects & tracked_objects, const geometry_msgs::msg::Pose & pose,
  const double search_radius)
{
  return std::any_of(
    tracked_objects.objects.begin(), tracked_objects.objects.end(),
    [pose, search_radius](const TrackedObject & object) {
      return autoware::universe_utils::calcDistance3d(
               pose.position, object.kinematics.pose_with_covariance.pose.position) <=
             search_radius;
    });
}

LatencyStats calculate_statistics(const std::vector<double> & latency_vec)
{
  LatencyStats stats{0.0, 0.0, 0.0, 0.0, 0.0};
  stats.max = *max_element(latency_vec.begin(), latency_vec.end());
  stats.min = *min_element(latency_vec.begin(), latency_vec.end());

  const double sum = std::accumulate(latency_vec.begin(), latency_vec.end(), 0.0);
  stats.mean = sum / static_cast<double>(latency_vec.size());

  std::vector<double> sorted_latencies = latency_vec;
  std::sort(sorted_latencies.begin(), sorted_latencies.end());
  stats.median = sorted_latencies.size() % 2 == 0
                   ? (sorted_latencies[sorted_latencies.size() / 2 - 1] +
                      sorted_latencies[sorted_latencies.size() / 2]) /
                       2
                   : sorted_latencies[sorted_latencies.size() / 2];

  const double sq_sum =
    std::inner_product(latency_vec.begin(), latency_vec.end(), latency_vec.begin(), 0.0);
  stats.std_dev =
    std::sqrt(sq_sum / static_cast<double>(latency_vec.size()) - stats.mean * stats.mean);
  return stats;
}

void write_results(
  rclcpp::Node * node, const std::string & output_file_path, const RunningMode & node_running_mode,
  const std::vector<PipelineMap> & pipeline_map_vector)
{
  // create csv file
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << output_file_path;
  if (!output_file_path.empty() && output_file_path.back() != '/') {
    ss << "/";  // Ensure the path ends with a slash
  }
  if (node_running_mode == RunningMode::PlanningControl) {
    ss << "planning_control-";
  } else {
    ss << "perception_planning-";
  }

  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
  ss << "-reaction-results.csv";

  // open file
  std::ofstream file(ss.str());

  // Check if the file was opened successfully
  if (!file.is_open()) {
    RCLCPP_ERROR_ONCE(node->get_logger(), "Failed to open file: %s", ss.str().c_str());
    return;
  }

  // tmp map to store latency results for statistics
  std::map<std::string, std::vector<std::tuple<double, double, double>>> tmp_latency_map;

  size_t test_count = 0;
  for (const auto & pipeline_map : pipeline_map_vector) {
    test_count++;
    // convert pipeline_map to vector of tuples
    file << "Test " << test_count << "\n";
    const auto sorted_results_vector = convert_pipeline_map_to_sorted_vector(pipeline_map);
    const auto spawn_cmd_time = std::get<0>(*sorted_results_vector.begin());

    for (size_t i = 0; i < sorted_results_vector.size(); ++i) {
      const auto & [pipeline_header_time, pipeline_reactions] = sorted_results_vector[i];

      // total time pipeline lasts
      file << "Pipeline - " << i << ",";

      // pipeline nodes
      for (const auto & [node_name, reaction] : pipeline_reactions) {
        file << node_name << ",";
      }

      file << "\nNode - Pipeline - Total Latency [ms],";

      for (size_t j = 0; j < pipeline_reactions.size(); ++j) {
        const auto & reaction = pipeline_reactions[j].second;
        const auto & node_name = pipeline_reactions[j].first;
        if (j == 0) {
          const auto node_latency =
            calculate_time_diff_ms(reaction.header.stamp, reaction.published_stamp);
          const auto pipeline_latency =
            calculate_time_diff_ms(pipeline_header_time, reaction.published_stamp);
          const auto total_latency =
            calculate_time_diff_ms(spawn_cmd_time, reaction.published_stamp);
          file << node_latency << " - " << pipeline_latency << " - " << total_latency << ",";
          tmp_latency_map[node_name].emplace_back(
            std::make_tuple(node_latency, pipeline_latency, total_latency));
        } else {
          const auto & prev_reaction = pipeline_reactions[j - 1].second;
          const auto node_latency =
            calculate_time_diff_ms(prev_reaction.published_stamp, reaction.published_stamp);
          const auto pipeline_latency =
            calculate_time_diff_ms(pipeline_header_time, reaction.published_stamp);
          const auto total_latency =
            calculate_time_diff_ms(spawn_cmd_time, reaction.published_stamp);
          file << node_latency << " - " << pipeline_latency << " - " << total_latency << ",";
          tmp_latency_map[node_name].emplace_back(
            std::make_tuple(node_latency, pipeline_latency, total_latency));
        }
      }
      file << "\n";
    }
  }

  // write statistics

  file << "\nStatistics\n";
  file << "Node "
          "Name,Min-NL,Max-NL,Mean-NL,Median-NL,Std-Dev-NL,Min-PL,Max-PL,Mean-PL,Median-PL,Std-Dev-"
          "PL,Min-TL,Max-TL,Mean-TL,Median-TL,Std-Dev-TL\n";
  for (const auto & [node_name, latency_vec] : tmp_latency_map) {
    file << node_name << ",";

    std::vector<double> node_latencies;
    std::vector<double> pipeline_latencies;
    std::vector<double> total_latencies;

    // Extract latencies
    for (const auto & latencies : latency_vec) {
      double node_latency, pipeline_latency, total_latency;
      std::tie(node_latency, pipeline_latency, total_latency) = latencies;
      node_latencies.push_back(node_latency);
      pipeline_latencies.push_back(pipeline_latency);
      total_latencies.push_back(total_latency);
    }

    const auto stats_node_latency = calculate_statistics(node_latencies);
    const auto stats_pipeline_latency = calculate_statistics(pipeline_latencies);
    const auto stats_total_latency = calculate_statistics(total_latencies);

    file << stats_node_latency.min << "," << stats_node_latency.max << ","
         << stats_node_latency.mean << "," << stats_node_latency.median << ","
         << stats_node_latency.std_dev << "," << stats_pipeline_latency.min << ","
         << stats_pipeline_latency.max << "," << stats_pipeline_latency.mean << ","
         << stats_pipeline_latency.median << "," << stats_pipeline_latency.std_dev << ","
         << stats_total_latency.min << "," << stats_total_latency.max << ","
         << stats_total_latency.mean << "," << stats_total_latency.median << ","
         << stats_total_latency.std_dev << "\n";
  }
  file.close();
  RCLCPP_INFO(node->get_logger(), "Results written to: %s", ss.str().c_str());
}
}  // namespace reaction_analyzer
