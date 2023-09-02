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

#include "dummy_perception_publisher/node.hpp"

#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;

ObjectInfo::ObjectInfo(
  const dummy_perception_publisher::msg::Object & object, const rclcpp::Time & current_time)
: length(object.shape.dimensions.x),
  width(object.shape.dimensions.y),
  height(object.shape.dimensions.z),
  std_dev_x(std::sqrt(object.initial_state.pose_covariance.covariance[0])),
  std_dev_y(std::sqrt(object.initial_state.pose_covariance.covariance[7])),
  std_dev_z(std::sqrt(object.initial_state.pose_covariance.covariance[14])),
  std_dev_yaw(std::sqrt(object.initial_state.pose_covariance.covariance[35])),
  twist_covariance_(object.initial_state.twist_covariance),
  pose_covariance_(object.initial_state.pose_covariance)
{
  // calculate current pose
  const auto & initial_pose = object.initial_state.pose_covariance.pose;
  const double initial_vel = std::clamp(
    object.initial_state.twist_covariance.twist.linear.x, static_cast<double>(object.min_velocity),
    static_cast<double>(object.max_velocity));
  const double initial_acc = object.initial_state.accel_covariance.accel.linear.x;

  const double elapsed_time = current_time.seconds() - rclcpp::Time(object.header.stamp).seconds();

  double move_distance;
  double current_vel = initial_vel + initial_acc * elapsed_time;
  if (initial_acc == 0.0) {
    move_distance = initial_vel * elapsed_time;
  } else {
    if (initial_acc < 0 && 0 < initial_vel) {
      current_vel = std::max(current_vel, 0.0);
    }
    if (0 < initial_acc && initial_vel < 0) {
      current_vel = std::min(current_vel, 0.0);
    }

    // add distance on acceleration or deceleration
    current_vel = std::clamp(
      current_vel, static_cast<double>(object.min_velocity),
      static_cast<double>(object.max_velocity));
    move_distance = (std::pow(current_vel, 2) - std::pow(initial_vel, 2)) * 0.5 / initial_acc;

    // add distance after reaching max_velocity
    if (0 < initial_acc) {
      const double time_to_reach_max_vel =
        std::max(static_cast<double>(object.max_velocity) - initial_vel, 0.0) / initial_acc;
      move_distance += static_cast<double>(object.max_velocity) *
                       std::max(elapsed_time - time_to_reach_max_vel, 0.0);
    }

    // add distance after reaching min_velocity
    if (initial_acc < 0) {
      const double time_to_reach_min_vel =
        std::min(static_cast<double>(object.min_velocity) - initial_vel, 0.0) / initial_acc;
      move_distance += static_cast<double>(object.min_velocity) *
                       std::max(elapsed_time - time_to_reach_min_vel, 0.0);
    }
  }

  const auto current_pose =
    tier4_autoware_utils::calcOffsetPose(initial_pose, move_distance, 0.0, 0.0);

  // calculate tf from map to moved_object
  geometry_msgs::msg::Transform ros_map2moved_object;
  ros_map2moved_object.translation.x = current_pose.position.x;
  ros_map2moved_object.translation.y = current_pose.position.y;
  ros_map2moved_object.translation.z = current_pose.position.z;
  ros_map2moved_object.rotation = current_pose.orientation;
  tf2::fromMsg(ros_map2moved_object, tf_map2moved_object);
  // set twist and pose information
  twist_covariance_.twist.linear.x = current_vel;
  pose_covariance_.pose = current_pose;
}

TrackedObject ObjectInfo::toTrackedObject(
  const dummy_perception_publisher::msg::Object & object) const
{
  TrackedObject tracked_object;
  tracked_object.kinematics.pose_with_covariance = pose_covariance_;
  tracked_object.kinematics.twist_with_covariance = twist_covariance_;
  tracked_object.classification.push_back(object.classification);
  tracked_object.shape.type = object.shape.type;
  tracked_object.shape.dimensions.x = length;
  tracked_object.shape.dimensions.y = width;
  tracked_object.shape.dimensions.z = height;
  tracked_object.object_id = object.id;
  return tracked_object;
}

DummyPerceptionPublisherNode::DummyPerceptionPublisherNode()
: Node("dummy_perception_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  visible_range_ = this->declare_parameter("visible_range", 100.0);
  detection_successful_rate_ = this->declare_parameter("detection_successful_rate", 0.8);
  enable_ray_tracing_ = this->declare_parameter("enable_ray_tracing", true);
  use_object_recognition_ = this->declare_parameter("use_object_recognition", true);
  use_base_link_z_ = this->declare_parameter("use_base_link_z", true);
  const bool object_centric_pointcloud =
    this->declare_parameter("object_centric_pointcloud", false);
  publish_ground_truth_objects_ = this->declare_parameter("publish_ground_truth", false);
  const unsigned int random_seed =
    static_cast<unsigned int>(this->declare_parameter("random_seed", 0));
  const bool use_fixed_random_seed = this->declare_parameter("use_fixed_random_seed", false);

  if (object_centric_pointcloud) {
    pointcloud_creator_ =
      std::unique_ptr<PointCloudCreator>(new ObjectCentricPointCloudCreator(enable_ray_tracing_));
  } else {
    pointcloud_creator_ =
      std::unique_ptr<PointCloudCreator>(new EgoCentricPointCloudCreator(visible_range_));
  }

  // parameters for vehicle centric point cloud generation
  angle_increment_ = this->declare_parameter("angle_increment", 0.25 * M_PI / 180.0);

  if (use_fixed_random_seed) {
    random_generator_.seed(random_seed);
  } else {
    std::random_device seed_gen;
    random_generator_.seed(seed_gen());
  }

  // create subscriber and publisher
  rclcpp::QoS qos{1};
  qos.transient_local();
  detected_object_with_feature_pub_ =
    this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
      "output/dynamic_object", qos);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/points_raw", qos);
  object_sub_ = this->create_subscription<dummy_perception_publisher::msg::Object>(
    "input/object", 100,
    std::bind(&DummyPerceptionPublisherNode::objectCallback, this, std::placeholders::_1));

  // optional ground truth publisher
  if (publish_ground_truth_objects_) {
    ground_truth_objects_pub_ =
      this->create_publisher<autoware_auto_perception_msgs::msg::TrackedObjects>(
        "~/output/debug/ground_truth_objects", qos);
  }

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&DummyPerceptionPublisherNode::timerCallback, this));
}

void DummyPerceptionPublisherNode::timerCallback()
{
  // output msgs
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output_dynamic_object_msg;
  autoware_auto_perception_msgs::msg::TrackedObjects output_ground_truth_objects_msg;
  geometry_msgs::msg::PoseStamped output_moved_object_pose;
  sensor_msgs::msg::PointCloud2 output_pointcloud_msg;
  std_msgs::msg::Header header;
  rclcpp::Time current_time = this->now();

  // avoid terminal contamination.
  static rclcpp::Time failed_tf_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  if ((this->now() - failed_tf_time).seconds() < 5.0) {
    return;
  }

  std::string error;
  if (!tf_buffer_.canTransform("base_link", /*src*/ "map", tf2::TimePointZero, &error)) {
    failed_tf_time = this->now();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "map->base_link is not available yet");
    return;
  }

  tf2::Transform tf_base_link2map;
  try {
    geometry_msgs::msg::TransformStamped ros_base_link2map;
    ros_base_link2map = tf_buffer_.lookupTransform(
      /*target*/ "base_link", /*src*/ "map", current_time, rclcpp::Duration::from_seconds(0.5));
    tf2::fromMsg(ros_base_link2map.transform, tf_base_link2map);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    return;
  }

  std::vector<size_t> selected_indices{};
  std::vector<ObjectInfo> obj_infos;
  static std::uniform_real_distribution<> detection_successful_random(0.0, 1.0);
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (detection_successful_rate_ >= detection_successful_random(random_generator_)) {
      selected_indices.push_back(i);
    }
    const auto obj_info = ObjectInfo(objects_.at(i), current_time);
    obj_infos.push_back(obj_info);
  }

  // publish ground truth
  // add Tracked Object
  if (publish_ground_truth_objects_) {
    for (auto object : objects_) {
      const auto object_info = ObjectInfo(object, current_time);
      TrackedObject gt_tracked_object = object_info.toTrackedObject(object);
      gt_tracked_object.existence_probability = 1.0;
      output_ground_truth_objects_msg.objects.push_back(gt_tracked_object);
    }
  }

  // publish noised detected objects
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr detected_merged_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  if (objects_.empty()) {
    pcl::toROSMsg(*merged_pointcloud_ptr, output_pointcloud_msg);
  } else {
    pointcloud_creator_->create_pointclouds(
      obj_infos, tf_base_link2map, random_generator_, merged_pointcloud_ptr);
    pcl::toROSMsg(*merged_pointcloud_ptr, output_pointcloud_msg);
  }
  if (!selected_indices.empty()) {
    std::vector<ObjectInfo> detected_obj_infos;
    for (const auto selected_idx : selected_indices) {
      const auto detected_obj_info = ObjectInfo(objects_.at(selected_idx), current_time);
      tf2::toMsg(detected_obj_info.tf_map2moved_object, output_moved_object_pose.pose);
      detected_obj_infos.push_back(detected_obj_info);
    }

    const auto pointclouds = pointcloud_creator_->create_pointclouds(
      detected_obj_infos, tf_base_link2map, random_generator_, detected_merged_pointcloud_ptr);

    std::vector<size_t> delete_idxs;
    for (size_t i = 0; i < selected_indices.size(); ++i) {
      const auto pointcloud = pointclouds[i];
      const size_t selected_idx = selected_indices[i];
      const auto & object = objects_.at(selected_idx);
      const auto object_info = ObjectInfo(object, current_time);
      // dynamic object
      std::normal_distribution<> x_random(0.0, object_info.std_dev_x);
      std::normal_distribution<> y_random(0.0, object_info.std_dev_y);
      std::normal_distribution<> yaw_random(0.0, object_info.std_dev_yaw);
      tf2::Quaternion noised_quat;
      noised_quat.setRPY(0, 0, yaw_random(random_generator_));
      tf2::Transform tf_moved_object2noised_moved_object(
        noised_quat, tf2::Vector3(x_random(random_generator_), y_random(random_generator_), 0.0));
      tf2::Transform tf_base_link2noised_moved_object;
      tf_base_link2noised_moved_object =
        tf_base_link2map * object_info.tf_map2moved_object * tf_moved_object2noised_moved_object;

      // add DetectedObjectWithFeature
      tier4_perception_msgs::msg::DetectedObjectWithFeature feature_object;
      feature_object.object.classification.push_back(object.classification);
      feature_object.object.kinematics.pose_with_covariance = object.initial_state.pose_covariance;
      feature_object.object.kinematics.twist_with_covariance =
        object.initial_state.twist_covariance;
      feature_object.object.kinematics.orientation_availability =
        autoware_auto_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;
      feature_object.object.kinematics.has_twist = false;
      tf2::toMsg(
        tf_base_link2noised_moved_object,
        feature_object.object.kinematics.pose_with_covariance.pose);
      feature_object.object.shape = object.shape;
      pcl::toROSMsg(*pointcloud, feature_object.feature.cluster);
      output_dynamic_object_msg.feature_objects.push_back(feature_object);

      // check delete idx
      tf2::Transform tf_base_link2moved_object;
      tf_base_link2moved_object = tf_base_link2map * object_info.tf_map2moved_object;
      double dist = std::sqrt(
        tf_base_link2moved_object.getOrigin().x() * tf_base_link2moved_object.getOrigin().x() +
        tf_base_link2moved_object.getOrigin().y() * tf_base_link2moved_object.getOrigin().y());
      if (visible_range_ < dist) {
        delete_idxs.push_back(selected_idx);
      }
    }

    // delete
    for (int delete_idx = delete_idxs.size() - 1; 0 <= delete_idx; --delete_idx) {
      objects_.erase(objects_.begin() + delete_idxs.at(delete_idx));
    }
  }

  // create output header
  output_moved_object_pose.header.frame_id = "map";
  output_moved_object_pose.header.stamp = current_time;
  output_dynamic_object_msg.header.frame_id = "base_link";
  output_dynamic_object_msg.header.stamp = current_time;
  output_pointcloud_msg.header.frame_id = "base_link";
  output_pointcloud_msg.header.stamp = current_time;
  output_ground_truth_objects_msg.header.frame_id = "map";
  output_ground_truth_objects_msg.header.stamp = current_time;

  // publish
  pointcloud_pub_->publish(output_pointcloud_msg);
  if (use_object_recognition_) {
    detected_object_with_feature_pub_->publish(output_dynamic_object_msg);
  }
  if (publish_ground_truth_objects_) {
    ground_truth_objects_pub_->publish(output_ground_truth_objects_msg);
  }
}

void DummyPerceptionPublisherNode::objectCallback(
  const dummy_perception_publisher::msg::Object::ConstSharedPtr msg)
{
  switch (msg->action) {
    case dummy_perception_publisher::msg::Object::ADD: {
      tf2::Transform tf_input2map;
      tf2::Transform tf_input2object_origin;
      tf2::Transform tf_map2object_origin;
      try {
        geometry_msgs::msg::TransformStamped ros_input2map;
        ros_input2map = tf_buffer_.lookupTransform(
          /*target*/ msg->header.frame_id, /*src*/ "map", msg->header.stamp,
          rclcpp::Duration::from_seconds(0.5));
        tf2::fromMsg(ros_input2map.transform, tf_input2map);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
        return;
      }
      tf2::fromMsg(msg->initial_state.pose_covariance.pose, tf_input2object_origin);
      tf_map2object_origin = tf_input2map.inverse() * tf_input2object_origin;
      dummy_perception_publisher::msg::Object object;
      object = *msg;
      tf2::toMsg(tf_map2object_origin, object.initial_state.pose_covariance.pose);

      // Use base_link Z
      if (use_base_link_z_) {
        geometry_msgs::msg::TransformStamped ros_map2base_link;
        try {
          ros_map2base_link = tf_buffer_.lookupTransform(
            "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
          object.initial_state.pose_covariance.pose.position.z =
            ros_map2base_link.transform.translation.z + 0.5 * object.shape.dimensions.z;
        } catch (tf2::TransformException & ex) {
          RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
          return;
        }
      }

      objects_.push_back(object);
      break;
    }
    case dummy_perception_publisher::msg::Object::DELETE: {
      for (size_t i = 0; i < objects_.size(); ++i) {
        if (objects_.at(i).id.uuid == msg->id.uuid) {
          objects_.erase(objects_.begin() + i);
          break;
        }
      }
      break;
    }
    case dummy_perception_publisher::msg::Object::MODIFY: {
      for (size_t i = 0; i < objects_.size(); ++i) {
        if (objects_.at(i).id.uuid == msg->id.uuid) {
          tf2::Transform tf_input2map;
          tf2::Transform tf_input2object_origin;
          tf2::Transform tf_map2object_origin;
          try {
            geometry_msgs::msg::TransformStamped ros_input2map;
            ros_input2map = tf_buffer_.lookupTransform(
              /*target*/ msg->header.frame_id, /*src*/ "map", msg->header.stamp,
              rclcpp::Duration::from_seconds(0.5));
            tf2::fromMsg(ros_input2map.transform, tf_input2map);
          } catch (tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
            return;
          }
          tf2::fromMsg(msg->initial_state.pose_covariance.pose, tf_input2object_origin);
          tf_map2object_origin = tf_input2map.inverse() * tf_input2object_origin;
          dummy_perception_publisher::msg::Object object;
          objects_.at(i) = *msg;
          tf2::toMsg(tf_map2object_origin, objects_.at(i).initial_state.pose_covariance.pose);
          if (use_base_link_z_) {
            // Use base_link Z
            geometry_msgs::msg::TransformStamped ros_map2base_link;
            try {
              ros_map2base_link = tf_buffer_.lookupTransform(
                "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
              objects_.at(i).initial_state.pose_covariance.pose.position.z =
                ros_map2base_link.transform.translation.z + 0.5 * objects_.at(i).shape.dimensions.z;
            } catch (tf2::TransformException & ex) {
              RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
              return;
            }
          }

          break;
        }
      }
      break;
    }
    case dummy_perception_publisher::msg::Object::DELETEALL: {
      objects_.clear();
      break;
    }
  }
}
