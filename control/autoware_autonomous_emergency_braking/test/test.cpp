// Copyright 2024 TIER IV
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

#include "test.hpp"

#include "autoware/autonomous_emergency_braking/node.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/detail/shape__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>

#include <gtest/gtest.h>
#include <pcl/memory.h>
#include <tf2/LinearMath/Transform.h>

#include <memory>
#include <thread>

namespace autoware::motion::control::autonomous_emergency_braking::test
{
using autoware::universe_utils::Polygon2d;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::Header;

Header get_header(const char * const frame_id, rclcpp::Time t)
{
  std_msgs::msg::Header header;
  header.stamp = t;
  header.frame_id = frame_id;
  return header;
};

Imu make_imu_message(
  const Header & header, const double ax, const double ay, const double yaw,
  const double angular_velocity_z)
{
  Imu imu_msg;
  imu_msg.header = header;
  imu_msg.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);
  imu_msg.angular_velocity.z = angular_velocity_z;
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  return imu_msg;
};

VelocityReport make_velocity_report_msg(
  const Header & header, const double lat_velocity, const double long_velocity,
  const double heading_rate)
{
  VelocityReport velocity_msg;
  velocity_msg.header = header;
  velocity_msg.lateral_velocity = lat_velocity;
  velocity_msg.longitudinal_velocity = long_velocity;
  velocity_msg.heading_rate = heading_rate;
  return velocity_msg;
}

std::shared_ptr<AEB> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};

  const auto aeb_dir =
    ament_index_cpp::get_package_share_directory("autoware_autonomous_emergency_braking");
  const auto vehicle_info_util_dir =
    ament_index_cpp::get_package_share_directory("autoware_vehicle_info_utils");

  node_options.arguments(
    {"--ros-args", "--params-file", aeb_dir + "/config/autonomous_emergency_braking.param.yaml",
     "--ros-args", "--params-file", vehicle_info_util_dir + "/config/vehicle_info.param.yaml"});
  return std::make_shared<AEB>(node_options);
};

std::shared_ptr<PubSubNode> generatePubSubNode()
{
  auto node_options = rclcpp::NodeOptions{};
  node_options.arguments({"--ros-args"});
  return std::make_shared<PubSubNode>(node_options);
};

PubSubNode::PubSubNode(const rclcpp::NodeOptions & node_options)
: Node("test_aeb_pubsub", node_options)
{
  rclcpp::QoS qos{1};
  qos.transient_local();

  pub_imu_ = create_publisher<Imu>("~/input/imu", qos);
  pub_point_cloud_ = create_publisher<PointCloud2>("~/input/pointcloud", qos);
  pub_velocity_ = create_publisher<VelocityReport>("~/input/velocity", qos);
  pub_predicted_traj_ = create_publisher<Trajectory>("~/input/predicted_trajectory", qos);
  pub_predicted_objects_ = create_publisher<PredictedObjects>("~/input/objects", qos);
  pub_autoware_state_ = create_publisher<AutowareState>("autoware/state", qos);
}

TEST_F(TestAEB, checkCollision)
{
  constexpr double longitudinal_velocity = 3.0;
  ObjectData object_collision;
  object_collision.distance_to_object = 0.5;
  object_collision.velocity = 0.1;
  ASSERT_TRUE(aeb_node_->hasCollision(longitudinal_velocity, object_collision));

  ObjectData object_no_collision;
  object_no_collision.distance_to_object = 10.0;
  object_no_collision.velocity = 0.1;
  ASSERT_FALSE(aeb_node_->hasCollision(longitudinal_velocity, object_no_collision));
}

TEST_F(TestAEB, checkImuPathGeneration)
{
  constexpr double longitudinal_velocity = 3.0;
  constexpr double yaw_rate = 0.05;
  const auto imu_path = aeb_node_->generateEgoPath(longitudinal_velocity, yaw_rate);
  ASSERT_FALSE(imu_path.empty());

  const double dt = aeb_node_->imu_prediction_time_interval_;
  const double horizon = aeb_node_->imu_prediction_time_horizon_;
  ASSERT_TRUE(imu_path.size() >= static_cast<size_t>(horizon / dt));

  const auto footprint = aeb_node_->generatePathFootprint(imu_path, 0.0);
  ASSERT_FALSE(footprint.empty());
  ASSERT_TRUE(footprint.size() == imu_path.size() - 1);

  const auto stamp = rclcpp::Time();
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_points_ptr =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  {
    const double x_start{0.0};
    const double y_start{0.0};

    for (size_t i = 0; i < 15; ++i) {
      pcl::PointXYZ p1(
        x_start + static_cast<double>(i / 100.0), y_start - static_cast<double>(i / 100.0), 0.5);
      pcl::PointXYZ p2(
        x_start + static_cast<double>((i + 10) / 100.0), y_start - static_cast<double>(i / 100.0),
        0.5);
      obstacle_points_ptr->push_back(p1);
      obstacle_points_ptr->push_back(p2);
    }
  }
  PointCloud::Ptr points_belonging_to_cluster_hulls = pcl::make_shared<PointCloud>();
  MarkerArray debug_markers;
  aeb_node_->getPointsBelongingToClusterHulls(
    obstacle_points_ptr, points_belonging_to_cluster_hulls, debug_markers);
  std::vector<ObjectData> objects;
  aeb_node_->getClosestObjectsOnPath(
    imu_path, footprint, stamp, points_belonging_to_cluster_hulls, objects);
  ASSERT_FALSE(objects.empty());
}

TEST_F(TestAEB, checkIncompleteImuPathGeneration)
{
  const double dt = aeb_node_->imu_prediction_time_interval_;
  const double horizon = aeb_node_->imu_prediction_time_horizon_;
  const double min_generated_path_length = aeb_node_->min_generated_path_length_;
  const double slow_velocity = min_generated_path_length / (2.0 * horizon);
  constexpr double yaw_rate = 0.05;
  const auto imu_path = aeb_node_->generateEgoPath(slow_velocity, yaw_rate);

  ASSERT_FALSE(imu_path.empty());
  ASSERT_TRUE(imu_path.size() >= static_cast<size_t>(horizon / dt));
  ASSERT_TRUE(autoware::motion_utils::calcArcLength(imu_path) >= min_generated_path_length);

  const auto footprint = aeb_node_->generatePathFootprint(imu_path, 0.0);
  ASSERT_FALSE(footprint.empty());
  ASSERT_TRUE(footprint.size() == imu_path.size() - 1);
}

TEST_F(TestAEB, checkEmptyPathAtZeroSpeed)
{
  const double velocity = 0.0;
  constexpr double yaw_rate = 0.0;
  const auto imu_path = aeb_node_->generateEgoPath(velocity, yaw_rate);
  ASSERT_EQ(imu_path.size(), 1);
}

TEST_F(TestAEB, checkParamUpdate)
{
  std::vector<rclcpp::Parameter> parameters{rclcpp::Parameter("param")};
  const auto result = aeb_node_->onParameter(parameters);
  ASSERT_TRUE(result.successful);
}

TEST_F(TestAEB, checkEmptyFetchData)
{
  ASSERT_FALSE(aeb_node_->fetchLatestData());
}

TEST_F(TestAEB, checkConvertObjectToPolygon)
{
  using autoware_perception_msgs::msg::Shape;
  PredictedObject obj_cylinder;
  obj_cylinder.shape.type = Shape::CYLINDER;
  obj_cylinder.shape.dimensions.x = 1.0;
  Pose obj_cylinder_pose;
  obj_cylinder_pose.position.x = 1.0;
  obj_cylinder_pose.position.y = 1.0;
  obj_cylinder.kinematics.initial_pose_with_covariance.pose = obj_cylinder_pose;
  const auto cylinder_polygon = utils::convertObjToPolygon(obj_cylinder);
  ASSERT_FALSE(cylinder_polygon.outer().empty());

  PredictedObject obj_box;
  obj_box.shape.type = Shape::BOUNDING_BOX;
  obj_box.shape.dimensions.x = 1.0;
  obj_box.shape.dimensions.y = 2.0;
  Pose obj_box_pose;
  obj_box_pose.position.x = 1.0;
  obj_box_pose.position.y = 1.0;
  obj_box.kinematics.initial_pose_with_covariance.pose = obj_box_pose;
  const auto box_polygon = utils::convertObjToPolygon(obj_box);
  ASSERT_FALSE(box_polygon.outer().empty());

  geometry_msgs::msg::TransformStamped tf_stamped;
  geometry_msgs::msg::Transform transform;

  constexpr double yaw{0.0};
  transform.rotation = autoware::universe_utils::createQuaternionFromYaw(yaw);
  geometry_msgs::msg::Vector3 translation;
  translation.x = 1.0;
  translation.y = 0.0;
  translation.z = 0.0;
  transform.translation = translation;
  tf_stamped.set__transform(transform);
  const auto t_obj_box = utils::transformObjectFrame(obj_box, tf_stamped);
  const auto t_pose = t_obj_box.kinematics.initial_pose_with_covariance.pose;
  Pose expected_pose;
  expected_pose.position.x = obj_box_pose.position.x + translation.x;
  expected_pose.position.y = obj_box_pose.position.y + translation.y;
  expected_pose.position.z = obj_box_pose.position.z + translation.z;

  ASSERT_DOUBLE_EQ(expected_pose.position.x, t_pose.position.x);
  ASSERT_DOUBLE_EQ(expected_pose.position.y, t_pose.position.y);
  ASSERT_DOUBLE_EQ(expected_pose.position.z, t_pose.position.z);
}

TEST_F(TestAEB, CollisionDataKeeper)
{
  using namespace std::literals::chrono_literals;
  constexpr double collision_keeping_sec{1.0}, previous_obstacle_keep_time{1.0};
  CollisionDataKeeper collision_data_keeper_(aeb_node_->get_clock());
  collision_data_keeper_.setTimeout(collision_keeping_sec, previous_obstacle_keep_time);
  ASSERT_TRUE(collision_data_keeper_.checkCollisionExpired());
  ASSERT_TRUE(collision_data_keeper_.checkPreviousObjectDataExpired());

  ObjectData obj;
  obj.stamp = aeb_node_->now();
  obj.velocity = 0.0;
  obj.position.x = 0.0;
  rclcpp::sleep_for(100ms);

  ObjectData obj2;
  obj2.stamp = aeb_node_->now();
  obj2.velocity = 0.0;
  obj2.position.x = 0.1;
  rclcpp::sleep_for(100ms);

  constexpr double ego_longitudinal_velocity = 3.0;
  constexpr double yaw_rate = 0.0;
  const auto imu_path = aeb_node_->generateEgoPath(ego_longitudinal_velocity, yaw_rate);

  const auto speed_null =
    collision_data_keeper_.calcObjectSpeedFromHistory(obj, imu_path, ego_longitudinal_velocity);
  ASSERT_FALSE(speed_null.has_value());

  const auto median_velocity =
    collision_data_keeper_.calcObjectSpeedFromHistory(obj2, imu_path, ego_longitudinal_velocity);
  ASSERT_TRUE(median_velocity.has_value());

  // object speed is 1.0 m/s greater than ego's = 0.1 [m] / 0.1 [s] + longitudinal_velocity
  ASSERT_TRUE(std::abs(median_velocity.value() - 4.0) < 1e-2);
  rclcpp::sleep_for(1100ms);
  ASSERT_TRUE(collision_data_keeper_.checkCollisionExpired());
}

TEST_F(TestAEB, TestCropPointCloud)
{
  constexpr double longitudinal_velocity = 3.0;
  constexpr double yaw_rate = 0.05;
  const auto imu_path = aeb_node_->generateEgoPath(longitudinal_velocity, yaw_rate);
  ASSERT_FALSE(imu_path.empty());

  constexpr size_t n_points{15};
  // Create n_points inside the path and 1 point outside.
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_points_ptr =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  {
    constexpr double x_start{0.0};
    constexpr double y_start{0.0};

    for (size_t i = 0; i < n_points; ++i) {
      const double offset_1 = static_cast<double>(i / 100.0);
      const double offset_2 = static_cast<double>((i + 10) / 100.0);
      pcl::PointXYZ p1(x_start + offset_1, y_start - offset_1, 0.5);
      pcl::PointXYZ p2(x_start + offset_2, y_start - offset_1, 0.5);
      obstacle_points_ptr->push_back(p1);
      obstacle_points_ptr->push_back(p2);
    }
    pcl::PointXYZ p_out(x_start + 100.0, y_start + 100, 0.5);
    obstacle_points_ptr->push_back(p_out);
  }
  aeb_node_->obstacle_ros_pointcloud_ptr_ = std::make_shared<PointCloud2>();
  pcl::toROSMsg(*obstacle_points_ptr, *aeb_node_->obstacle_ros_pointcloud_ptr_);
  const auto footprint = aeb_node_->generatePathFootprint(imu_path, 0.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_objects =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  aeb_node_->cropPointCloudWithEgoFootprintPath(footprint, filtered_objects);
  // Check if the point outside the path was excluded
  ASSERT_TRUE(filtered_objects->points.size() == 2 * n_points);
}

}  // namespace autoware::motion::control::autonomous_emergency_braking::test
