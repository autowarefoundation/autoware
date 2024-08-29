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

#ifndef TOPIC_PUBLISHER_HPP_
#define TOPIC_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <utils.hpp>

#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace reaction_analyzer::topic_publisher
{
using autoware_control_msgs::msg::Control;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

struct TopicPublisherParams
{
  double dummy_perception_publisher_period;  // Only for planning_control mode
  double spawned_pointcloud_sampling_distance;
  std::string path_bag_without_object;       // Path to the bag file without object
  std::string path_bag_with_object;          // Path to the bag file with object
  std::string pointcloud_publisher_type;     // Type of the pointcloud publisher
  double pointcloud_publisher_period;        // Period of the pointcloud publisher
  bool publish_only_pointcloud_with_object;  // Publish only pointcloud with object for only
                                             // perception pipeline debug purpose make it true.
};

enum class PointcloudPublisherType {
  ASYNC_PUBLISHER = 0,              // Asynchronous publisher
  SYNC_HEADER_SYNC_PUBLISHER = 1,   // Synchronous publisher with header synchronization
  ASYNC_HEADER_SYNC_PUBLISHER = 2,  // Asynchronous publisher with header synchronization
};

/**
 * @brief Message type template struct for the variables of the Publisher.
 */
template <typename MessageType>
struct PublisherVariables
{
  std::chrono::milliseconds period_ms{0};
  typename MessageType::SharedPtr empty_area_message;
  typename MessageType::SharedPtr object_spawned_message;
  typename rclcpp::Publisher<MessageType>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;
};

/**
 * @brief Struct for accessing the variables of the Publisher.
 */
struct PublisherVarAccessor
{
  // Template struct to check if a type has a header member.
  template <typename T, typename = std::void_t<>>
  struct HasHeader : std::false_type
  {
  };

  template <typename T>
  struct HasHeader<T, std::void_t<decltype(T::header)>> : std::true_type
  {
  };

  // Template struct to check if a type has a stamp member.
  template <typename T, typename = std::void_t<>>
  struct HasStamp : std::false_type
  {
  };

  template <typename T>
  struct HasStamp<T, std::void_t<decltype(T::stamp)>> : std::true_type
  {
  };

  template <typename T>
  void publish_with_current_time(
    const PublisherVariables<T> & publisher_var, const rclcpp::Time & current_time,
    const bool is_object_spawned) const
  {
    std::unique_ptr<T> msg_to_be_published = std::make_unique<T>();

    if (is_object_spawned) {
      *msg_to_be_published = *publisher_var.object_spawned_message;
    } else {
      *msg_to_be_published = *publisher_var.empty_area_message;
    }
    if constexpr (HasHeader<T>::value) {
      msg_to_be_published->header.stamp = current_time;
    } else if constexpr (HasStamp<T>::value) {
      msg_to_be_published->stamp = current_time;
    }
    publisher_var.publisher->publish(std::move(msg_to_be_published));
  }

  template <typename T>
  void set_period(T & publisher_var, std::chrono::milliseconds new_period)
  {
    publisher_var.period_ms = new_period;
  }

  template <typename T>
  std::chrono::milliseconds get_period(const T & publisher_var) const
  {
    return publisher_var.period_ms;
  }

  template <typename T>
  std::shared_ptr<void> get_empty_area_message(const T & publisher_var) const
  {
    return std::static_pointer_cast<void>(publisher_var.empty_area_message);
  }

  template <typename T>
  std::shared_ptr<void> get_object_spawned_message(const T & publisher_var) const
  {
    return std::static_pointer_cast<void>(publisher_var.object_spawned_message);
  }
};

using PublisherVariablesVariant = std::variant<
  PublisherVariables<PointCloud2>, PublisherVariables<sensor_msgs::msg::CameraInfo>,
  PublisherVariables<sensor_msgs::msg::Image>,
  PublisherVariables<geometry_msgs::msg::PoseWithCovarianceStamped>,
  PublisherVariables<geometry_msgs::msg::PoseStamped>, PublisherVariables<nav_msgs::msg::Odometry>,
  PublisherVariables<sensor_msgs::msg::Imu>,
  PublisherVariables<autoware_vehicle_msgs::msg::ControlModeReport>,
  PublisherVariables<autoware_vehicle_msgs::msg::GearReport>,
  PublisherVariables<autoware_vehicle_msgs::msg::HazardLightsReport>,
  PublisherVariables<autoware_vehicle_msgs::msg::SteeringReport>,
  PublisherVariables<autoware_vehicle_msgs::msg::TurnIndicatorsReport>,
  PublisherVariables<autoware_vehicle_msgs::msg::VelocityReport>>;

using LidarOutputPair = std::pair<
  std::shared_ptr<PublisherVariables<PointCloud2>>,
  std::shared_ptr<PublisherVariables<PointCloud2>>>;

class TopicPublisher
{
public:
  explicit TopicPublisher(
    rclcpp::Node * node, std::atomic<bool> & spawn_object_cmd, std::atomic<bool> & ego_initialized,
    std::optional<rclcpp::Time> & spawn_cmd_time, const RunningMode & node_running_mode,
    const EntityParams & entity_params);

  void reset();

private:
  std::mutex mutex_;

  // Initialized variables
  rclcpp::Node * node_;
  RunningMode node_running_mode_;
  std::atomic<bool> & spawn_object_cmd_;
  std::atomic<bool> & ego_initialized_;  // used for planning_control mode because if pointcloud is
                                         // published before ego is initialized, it causes crash
  EntityParams entity_params_;
  std::optional<rclcpp::Time> & spawn_cmd_time_;  // Set by a publisher function when the
                                                  // spawn_object_cmd_ is true

  // tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  TopicPublisherParams topic_publisher_params_;

  // Variables planning_control mode
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_predicted_objects_;
  PointCloud2::SharedPtr entity_pointcloud_ptr_;
  PredictedObjects::SharedPtr predicted_objects_ptr_;

  // Variables perception_planning mode
  PointcloudPublisherType pointcloud_publisher_type_;
  std::map<std::string, PublisherVariablesVariant> topic_publisher_map_;
  std::map<std::string, LidarOutputPair>
    lidar_pub_variable_pair_map_;  // used to publish pointcloud_raw and pointcloud_raw_ex
  bool is_object_spawned_message_published_{false};
  std::shared_ptr<rclcpp::TimerBase> one_shot_timer_shared_ptr_;

  // Functions
  template <typename MessageType>
  void set_message(
    const std::string & topic_name, const rosbag2_storage::SerializedBagMessage & bag_message,
    const bool is_empty_area_message);
  void set_period(const std::map<std::string, std::vector<rclcpp::Time>> & time_map);
  void set_publishers_and_timers_to_variable();
  void set_timers_for_pointcloud_msgs(
    const std::map<std::string, PublisherVariables<PointCloud2>> & pointcloud_variables_map);
  bool check_publishers_initialized_correctly();
  void init_rosbag_publishers();
  void init_rosbag_publisher_buffer(const std::string & bag_path, const bool is_empty_area_message);
  void pointcloud_messages_sync_publisher(const PointcloudPublisherType type);
  void pointcloud_messages_async_publisher(
    const std::pair<
      std::shared_ptr<PublisherVariables<PointCloud2>>,
      std::shared_ptr<PublisherVariables<PointCloud2>>> & lidar_output_pair_);
  void generic_message_publisher(const std::string & topic_name);
  void dummy_perception_publisher();  // Only for planning_control mode

  // Timers
  std::map<std::string, rclcpp::TimerBase::SharedPtr> pointcloud_publish_timers_map_;
  rclcpp::TimerBase::SharedPtr pointcloud_sync_publish_timer_;
  rclcpp::TimerBase::SharedPtr dummy_perception_timer_;
};
}  // namespace reaction_analyzer::topic_publisher

#endif  // TOPIC_PUBLISHER_HPP_
