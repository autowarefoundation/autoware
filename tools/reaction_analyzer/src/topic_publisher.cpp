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

#include "topic_publisher.hpp"

#include <algorithm>
#include <memory>

namespace reaction_analyzer::topic_publisher
{

TopicPublisher::TopicPublisher(
  rclcpp::Node * node, std::atomic<bool> & spawn_object_cmd, std::atomic<bool> & ego_initialized,
  std::optional<rclcpp::Time> & spawn_cmd_time, const RunningMode & node_running_mode,
  const EntityParams & entity_params)
: node_(node),
  node_running_mode_(node_running_mode),
  spawn_object_cmd_(spawn_object_cmd),
  ego_initialized_(ego_initialized),
  entity_params_(entity_params),
  spawn_cmd_time_(spawn_cmd_time)
{
  if (node_running_mode_ == RunningMode::PerceptionPlanning) {
    // get perception_planning mode parameters
    topic_publisher_params_.path_bag_with_object =
      node_->get_parameter("topic_publisher.path_bag_with_object").as_string();
    topic_publisher_params_.path_bag_without_object =
      node_->get_parameter("topic_publisher.path_bag_without_object").as_string();
    topic_publisher_params_.pointcloud_publisher_type =
      node_->get_parameter("topic_publisher.pointcloud_publisher.pointcloud_publisher_type")
        .as_string();
    topic_publisher_params_.pointcloud_publisher_period =
      node_->get_parameter("topic_publisher.pointcloud_publisher.pointcloud_publisher_period")
        .as_double();
    topic_publisher_params_.publish_only_pointcloud_with_object =
      node_
        ->get_parameter("topic_publisher.pointcloud_publisher.publish_only_pointcloud_with_object")
        .as_bool();

    // set pointcloud publisher type
    if (topic_publisher_params_.pointcloud_publisher_type == "sync_header_sync_publish") {
      pointcloud_publisher_type_ = PointcloudPublisherType::SYNC_HEADER_SYNC_PUBLISHER;
    } else if (topic_publisher_params_.pointcloud_publisher_type == "async_header_sync_publish") {
      pointcloud_publisher_type_ = PointcloudPublisherType::ASYNC_HEADER_SYNC_PUBLISHER;
    } else if (topic_publisher_params_.pointcloud_publisher_type == "async_publish") {
      pointcloud_publisher_type_ = PointcloudPublisherType::ASYNC_PUBLISHER;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Invalid pointcloud_publisher_type");
      rclcpp::shutdown();
      return;
    }

    // Init the publishers which will read the messages from the rosbag
    init_rosbag_publishers();
  } else if (node_running_mode_ == RunningMode::PlanningControl) {
    // init tf
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // get parameters
    topic_publisher_params_.dummy_perception_publisher_period =
      node_->get_parameter("topic_publisher.dummy_perception_publisher_period").as_double();
    topic_publisher_params_.spawned_pointcloud_sampling_distance =
      node_->get_parameter("topic_publisher.spawned_pointcloud_sampling_distance").as_double();

    // init the messages that will be published to spawn the object
    entity_pointcloud_ptr_ = create_entity_pointcloud_ptr(
      entity_params_, topic_publisher_params_.spawned_pointcloud_sampling_distance);
    predicted_objects_ptr_ = create_entity_predicted_objects_ptr(entity_params_);

    // init the publishers
    pub_pointcloud_ =
      node_->create_publisher<PointCloud2>("output/pointcloud", rclcpp::SensorDataQoS());
    pub_predicted_objects_ =
      node_->create_publisher<PredictedObjects>("output/objects", rclcpp::QoS(1));

    // init dummy perception publisher
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(topic_publisher_params_.dummy_perception_publisher_period));

    dummy_perception_timer_ = rclcpp::create_timer(
      node_, node_->get_clock(), period_ns,
      std::bind(&TopicPublisher::dummy_perception_publisher, this));
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Invalid running mode");
    rclcpp::shutdown();
    return;
  }
}

void TopicPublisher::pointcloud_messages_sync_publisher(const PointcloudPublisherType type)
{
  const auto current_time = node_->now();
  const bool is_object_spawned = spawn_object_cmd_;

  switch (type) {
    case PointcloudPublisherType::SYNC_HEADER_SYNC_PUBLISHER: {
      PublisherVarAccessor accessor;
      for (const auto & publisher_var_pair : lidar_pub_variable_pair_map_) {
        accessor.publish_with_current_time(
          *publisher_var_pair.second.first, current_time,
          topic_publisher_params_.publish_only_pointcloud_with_object || is_object_spawned);
        accessor.publish_with_current_time(
          *publisher_var_pair.second.second, current_time,
          topic_publisher_params_.publish_only_pointcloud_with_object || is_object_spawned);
      }
      if (is_object_spawned && !is_object_spawned_message_published_) {
        is_object_spawned_message_published_ = true;
        mutex_.lock();
        spawn_cmd_time_ = current_time;
        mutex_.unlock();
      }
      break;
    }
    case PointcloudPublisherType::ASYNC_HEADER_SYNC_PUBLISHER: {
      PublisherVarAccessor accessor;
      const auto period_pointcloud_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(topic_publisher_params_.pointcloud_publisher_period));
      const auto phase_dif = period_pointcloud_ns / lidar_pub_variable_pair_map_.size();

      size_t counter = 0;
      for (const auto & publisher_var_pair : lidar_pub_variable_pair_map_) {
        const auto header_time =
          current_time - std::chrono::nanoseconds(counter * phase_dif.count());
        accessor.publish_with_current_time(
          *publisher_var_pair.second.first, header_time,
          topic_publisher_params_.publish_only_pointcloud_with_object || is_object_spawned);
        accessor.publish_with_current_time(
          *publisher_var_pair.second.second, header_time,
          topic_publisher_params_.publish_only_pointcloud_with_object || is_object_spawned);
        counter++;
      }
      if (is_object_spawned && !is_object_spawned_message_published_) {
        is_object_spawned_message_published_ = true;
        mutex_.lock();
        spawn_cmd_time_ = current_time;
        mutex_.unlock();
      }
      break;
    }
    default:
      break;
  }
}

void TopicPublisher::pointcloud_messages_async_publisher(
  const std::pair<
    std::shared_ptr<PublisherVariables<PointCloud2>>,
    std::shared_ptr<PublisherVariables<PointCloud2>>> & lidar_output_pair_)
{
  PublisherVarAccessor accessor;
  const auto current_time = node_->now();
  const bool is_object_spawned = spawn_object_cmd_;
  accessor.publish_with_current_time(
    *lidar_output_pair_.first, current_time,
    topic_publisher_params_.publish_only_pointcloud_with_object || is_object_spawned);
  accessor.publish_with_current_time(
    *lidar_output_pair_.second, current_time,
    topic_publisher_params_.publish_only_pointcloud_with_object || is_object_spawned);

  if (is_object_spawned && !is_object_spawned_message_published_) {
    is_object_spawned_message_published_ = true;
    mutex_.lock();
    spawn_cmd_time_ = current_time;
    mutex_.unlock();
  }
}

void TopicPublisher::generic_message_publisher(const std::string & topic_name)
{
  PublisherVarAccessor accessor;
  const bool is_object_spawned = spawn_object_cmd_;
  const auto current_time = node_->now();
  const auto & publisher_variant = topic_publisher_map_[topic_name];

  std::visit(
    [&](const auto & var) {
      accessor.publish_with_current_time(var, current_time, is_object_spawned);
    },
    publisher_variant);
}

void TopicPublisher::dummy_perception_publisher()
{
  if (!ego_initialized_) {
    return;  // do not publish anything if ego is not initialized
  }
  if (!spawn_object_cmd_) {
    // do not spawn it, send empty pointcloud
    pcl::PointCloud<pcl::PointXYZ> pcl_empty;
    PointCloud2 empty_pointcloud;
    PredictedObjects empty_predicted_objects;
    pcl::toROSMsg(pcl_empty, empty_pointcloud);

    const auto current_time = node_->now();
    empty_pointcloud.header.frame_id = "base_link";
    empty_pointcloud.header.stamp = current_time;

    empty_predicted_objects.header.frame_id = "map";
    empty_predicted_objects.header.stamp = current_time;

    pub_pointcloud_->publish(empty_pointcloud);
    pub_predicted_objects_->publish(empty_predicted_objects);
  } else {
    // transform pointcloud
    geometry_msgs::msg::TransformStamped transform_stamped{};
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        "base_link", "map", node_->now(), rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to look up transform from map to base_link");
      return;
    }

    // transform by using eigen matrix
    PointCloud2 transformed_points{};
    const Eigen::Matrix4f affine_matrix =
      tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(affine_matrix, *entity_pointcloud_ptr_, transformed_points);
    const auto current_time = node_->now();

    transformed_points.header.frame_id = "base_link";
    transformed_points.header.stamp = current_time;

    predicted_objects_ptr_->header.frame_id = "map";
    predicted_objects_ptr_->header.stamp = current_time;

    pub_pointcloud_->publish(transformed_points);
    pub_predicted_objects_->publish(*predicted_objects_ptr_);
    if (!is_object_spawned_message_published_) {
      mutex_.lock();
      spawn_cmd_time_ = current_time;
      mutex_.unlock();
      is_object_spawned_message_published_ = true;
    }
  }
}

void TopicPublisher::reset()
{
  is_object_spawned_message_published_ = false;
}

void TopicPublisher::init_rosbag_publishers()
{
  // read messages without object
  init_rosbag_publisher_buffer(topic_publisher_params_.path_bag_without_object, true);

  // read messages with object
  init_rosbag_publisher_buffer(topic_publisher_params_.path_bag_with_object, false);

  // before create publishers and timers, check all the messages are correctly initialized with
  // their conjugate messages.
  if (check_publishers_initialized_correctly()) {
    RCLCPP_INFO(node_->get_logger(), "Messages are initialized correctly");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Messages are not initialized correctly");
    rclcpp::shutdown();
  }
  set_publishers_and_timers_to_variable();
}

template <typename MessageType>
void TopicPublisher::set_message(
  const std::string & topic_name, const rosbag2_storage::SerializedBagMessage & bag_message,
  const bool is_empty_area_message)
{
  rclcpp::Serialization<MessageType> serialization;
  rclcpp::SerializedMessage extracted_serialized_msg(*bag_message.serialized_data);

  // Deserialize the message
  auto deserialized_message = std::make_shared<MessageType>();
  serialization.deserialize_message(&extracted_serialized_msg, &*deserialized_message);
  auto & publisher_variant = topic_publisher_map_[topic_name];

  if (!std::holds_alternative<PublisherVariables<MessageType>>(publisher_variant)) {
    publisher_variant = PublisherVariables<MessageType>{};
  }

  auto & publisher_variable = std::get<PublisherVariables<MessageType>>(publisher_variant);

  if (is_empty_area_message) {
    if (!publisher_variable.empty_area_message) {
      publisher_variable.empty_area_message = deserialized_message;
    }
  } else {
    if (!publisher_variable.object_spawned_message) {
      publisher_variable.object_spawned_message = deserialized_message;
    }
  }
}

void TopicPublisher::set_period(const std::map<std::string, std::vector<rclcpp::Time>> & time_map)
{
  for (auto & topic_pair : time_map) {
    auto timestamps_tmp = topic_pair.second;

    // Sort the timestamps
    std::sort(timestamps_tmp.begin(), timestamps_tmp.end());

    // Then proceed with the frequency calculation
    std::string topic_name = topic_pair.first;
    if (timestamps_tmp.size() > 1) {
      int64_t total_time_diff_ns = 0;

      // Accumulate the differences in nanoseconds
      for (size_t i = 1; i < timestamps_tmp.size(); ++i) {
        total_time_diff_ns += (timestamps_tmp[i] - timestamps_tmp[i - 1]).nanoseconds();
      }

      // Conversion to std::chrono::milliseconds
      auto total_duration_ns = std::chrono::nanoseconds(total_time_diff_ns);
      auto period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(total_duration_ns) /
                       (timestamps_tmp.size() - 1);

      PublisherVariablesVariant & publisher_variant = topic_publisher_map_[topic_name];
      PublisherVarAccessor accessor;

      std::visit([&](auto & var) { accessor.set_period(var, period_ms); }, publisher_variant);
    }
  }
}

void TopicPublisher::set_publishers_and_timers_to_variable()
{
  std::map<std::string, PublisherVariables<PointCloud2>>
    pointcloud_variables_map;  // temp map for pointcloud publishers

  // initialize timers and message publishers except pointcloud messages
  for (auto & [topic_name, variant] : topic_publisher_map_) {
    PublisherVarAccessor accessor;
    const auto & topic_ref = topic_name;
    const auto period_ns = std::chrono::duration<double, std::nano>(
      std::visit([&](const auto & var) { return accessor.get_period(var); }, variant));

    // Dynamically create the correct publisher type based on the topic
    std::visit(
      [&](auto & var) {
        using MessageType = typename decltype(var.empty_area_message)::element_type;

        if constexpr (
          std::is_same_v<MessageType, sensor_msgs::msg::PointCloud2> ||
          std::is_same_v<MessageType, sensor_msgs::msg::Image> ||
          std::is_same_v<MessageType, sensor_msgs::msg::CameraInfo>) {
          var.publisher = node_->create_publisher<MessageType>(topic_ref, rclcpp::SensorDataQoS());
        } else {
          // For other message types, use the QoS setting depth of 1
          var.publisher = node_->create_publisher<MessageType>(topic_ref, rclcpp::QoS(1));
        }
      },
      variant);

    // Conditionally create the timer based on the message type, if message type is not
    // PointCloud2
    std::visit(
      [&](auto & var) {
        using MessageType = typename decltype(var.empty_area_message)::element_type;

        if constexpr (!std::is_same_v<MessageType, sensor_msgs::msg::PointCloud2>) {
          var.timer = node_->create_wall_timer(
            period_ns, [this, topic_ref]() { this->generic_message_publisher(topic_ref); });
        } else {
          // For PointCloud2, Store the variables in a temporary map
          pointcloud_variables_map[topic_ref] = var;
        }
      },
      variant);
  }

  // To be able to publish pointcloud messages with async, I need to create a timer for each lidar
  // output. So different operations are needed for pointcloud messages.
  set_timers_for_pointcloud_msgs(pointcloud_variables_map);
}

void TopicPublisher::set_timers_for_pointcloud_msgs(
  const std::map<std::string, PublisherVariables<PointCloud2>> & pointcloud_variables_map)
{
  // Set the point cloud publisher timers
  if (pointcloud_variables_map.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No pointcloud publishers found!");
    rclcpp::shutdown();
  }

  // Arrange the PointCloud2 variables w.r.t. the lidars' name
  for (auto & [topic_name, pointcloud_variant] : pointcloud_variables_map) {
    const auto lidar_name = split(topic_name, '/').at(3);

    if (lidar_pub_variable_pair_map_.find(lidar_name) == lidar_pub_variable_pair_map_.end()) {
      lidar_pub_variable_pair_map_[lidar_name] = std::make_pair(
        std::make_shared<PublisherVariables<PointCloud2>>(pointcloud_variant), nullptr);
    } else {
      if (lidar_pub_variable_pair_map_[lidar_name].second) {
        RCLCPP_ERROR_STREAM(
          node_->get_logger(),
          "Lidar name: " << lidar_name << " is already used by another pointcloud publisher");
        rclcpp::shutdown();
      }
      lidar_pub_variable_pair_map_[lidar_name].second =
        std::make_shared<PublisherVariables<PointCloud2>>(pointcloud_variant);
    }
  }

  // Create the timer(s) to publish PointCloud2 Messages
  const auto period_pointcloud_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(topic_publisher_params_.pointcloud_publisher_period));

  if (pointcloud_publisher_type_ != PointcloudPublisherType::ASYNC_PUBLISHER) {
    // Create 1 timer to publish all PointCloud2 messages
    pointcloud_sync_publish_timer_ = node_->create_wall_timer(period_pointcloud_ns, [this]() {
      this->pointcloud_messages_sync_publisher(this->pointcloud_publisher_type_);
    });
  } else {
    // Create multiple timers which will run with a phase difference
    const auto phase_dif = period_pointcloud_ns / lidar_pub_variable_pair_map_.size();

    // Create a timer to create phase difference bw timers which will be created for each lidar
    // topics
    auto one_shot_timer = node_->create_wall_timer(phase_dif, [this, period_pointcloud_ns]() {
      for (const auto & publisher_var_pair : lidar_pub_variable_pair_map_) {
        const auto & lidar_name = publisher_var_pair.first;
        const auto & publisher_var = publisher_var_pair.second;
        if (
          pointcloud_publish_timers_map_.find(lidar_name) == pointcloud_publish_timers_map_.end()) {
          auto periodic_timer = node_->create_wall_timer(
            period_pointcloud_ns,
            [this, publisher_var]() { this->pointcloud_messages_async_publisher(publisher_var); });
          pointcloud_publish_timers_map_[lidar_name] = periodic_timer;
          return;
        }
      }
      one_shot_timer_shared_ptr_->cancel();
    });
    one_shot_timer_shared_ptr_ = one_shot_timer;
  }
}

bool TopicPublisher::check_publishers_initialized_correctly()
{
  // check messages are correctly initialized or not from rosbags
  for (const auto & [topic_name, variant] : topic_publisher_map_) {
    PublisherVarAccessor accessor;
    auto empty_area_message =
      std::visit([&](const auto & var) { return accessor.get_empty_area_message(var); }, variant);
    auto object_spawned_message = std::visit(
      [&](const auto & var) { return accessor.get_object_spawned_message(var); }, variant);

    if (!empty_area_message) {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Empty area message couldn't found in the topic named: " << topic_name);
      return false;
    }
    if (!object_spawned_message) {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Object spawned message couldn't found in the topic named: " << topic_name);
      return false;
    }
  }
  return true;
}

void TopicPublisher::init_rosbag_publisher_buffer(
  const std::string & bag_path, const bool is_empty_area_message)
{
  rosbag2_cpp::Reader reader;

  try {
    reader.open(bag_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Error opening bag file: " << e.what());
    rclcpp::shutdown();
    return;
  }

  const auto & topics = reader.get_metadata().topics_with_message_count;

  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    const auto current_topic = bag_message->topic_name;

    const auto message_type = get_publisher_message_type_for_topic(topics, current_topic);

    if (message_type == PublisherMessageType::UNKNOWN) {
      continue;
    }
    if (message_type == PublisherMessageType::CAMERA_INFO) {
      set_message<sensor_msgs::msg::CameraInfo>(current_topic, *bag_message, is_empty_area_message);
    } else if (message_type == PublisherMessageType::IMAGE) {
      set_message<sensor_msgs::msg::Image>(current_topic, *bag_message, is_empty_area_message);
    } else if (message_type == PublisherMessageType::POINTCLOUD2) {
      set_message<PointCloud2>(current_topic, *bag_message, is_empty_area_message);
    } else if (message_type == PublisherMessageType::POSE_WITH_COVARIANCE_STAMPED) {
      set_message<geometry_msgs::msg::PoseWithCovarianceStamped>(
        current_topic, *bag_message, is_empty_area_message);
    } else if (message_type == PublisherMessageType::POSE_STAMPED) {
      set_message<geometry_msgs::msg::PoseStamped>(
        current_topic, *bag_message, is_empty_area_message);
    } else if (message_type == PublisherMessageType::ODOMETRY) {
      set_message<nav_msgs::msg::Odometry>(current_topic, *bag_message, is_empty_area_message);
    } else if (message_type == PublisherMessageType::IMU) {
      set_message<sensor_msgs::msg::Imu>(current_topic, *bag_message, is_empty_area_message);
    } else if (message_type == PublisherMessageType::CONTROL_MODE_REPORT) {
      set_message<autoware_vehicle_msgs::msg::ControlModeReport>(
        current_topic, *bag_message, is_empty_area_message);
    } else if (message_type == PublisherMessageType::GEAR_REPORT) {
      set_message<autoware_vehicle_msgs::msg::GearReport>(
        current_topic, *bag_message, is_empty_area_message);
    } else if (message_type == PublisherMessageType::HAZARD_LIGHTS_REPORT) {
      set_message<autoware_vehicle_msgs::msg::HazardLightsReport>(
        current_topic, *bag_message, is_empty_area_message);
    } else if (message_type == PublisherMessageType::STEERING_REPORT) {
      set_message<autoware_vehicle_msgs::msg::SteeringReport>(
        current_topic, *bag_message, is_empty_area_message);
    } else if (message_type == PublisherMessageType::TURN_INDICATORS_REPORT) {
      set_message<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
        current_topic, *bag_message, is_empty_area_message);
    } else if (message_type == PublisherMessageType::VELOCITY_REPORT) {
      set_message<autoware_vehicle_msgs::msg::VelocityReport>(
        current_topic, *bag_message, is_empty_area_message);
    }
  }
  reader.close();
}
}  // namespace reaction_analyzer::topic_publisher
