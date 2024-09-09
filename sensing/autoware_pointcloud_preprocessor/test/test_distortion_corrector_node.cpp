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

// Note: To regenerate the ground truth (GT) for the expected undistorted point cloud values,
// set the "debug_" value to true to display the point cloud values. Then,
// replace the expected values with the newly displayed undistorted point cloud values.
//
// Also, make sure the point stamp, twist stamp, and imu stamp are not identical.
// In the current hardcoded design, timestamp of pointcloud, twist, and imu msg are listed below
// pointcloud (1 msgs, 10points):
// 10.10, 10.11, 10.12, 10.13, 10.14, 10.15, 10.16, 10.17, 10.18, 10.19
// twist (6msgs):
// 10.095, 10.119, 10.143, 10.167, 10.191, 10.215
// imu (6msgs):
// 10.09, 10.117, 10.144, 10.171, 10.198, 10.225

#include "autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"
#include "autoware/universe_utils/math/trigonometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cassert>

class DistortionCorrectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_node");
    distortion_corrector_2d_ =
      std::make_shared<autoware::pointcloud_preprocessor::DistortionCorrector2D>(node_.get(), true);
    distortion_corrector_3d_ =
      std::make_shared<autoware::pointcloud_preprocessor::DistortionCorrector3D>(node_.get(), true);

    // Setup TF
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    tf_broadcaster_->sendTransform(generateStaticTransformMsg());

    // Spin the node for a while to ensure transforms are published
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(100);
    while (std::chrono::steady_clock::now() - start < timeout) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void TearDown() override {}

  void checkInput(int ms) { ASSERT_LT(ms, 1000) << "ms should be less than a second."; }

  rclcpp::Time addMilliseconds(rclcpp::Time stamp, int ms)
  {
    checkInput(ms);
    auto ms_in_ns = rclcpp::Duration(0, ms * 1000000);
    return stamp + ms_in_ns;
  }

  rclcpp::Time subtractMilliseconds(rclcpp::Time stamp, int ms)
  {
    checkInput(ms);
    auto ms_in_ns = rclcpp::Duration(0, ms * 1000000);
    return stamp - ms_in_ns;
  }

  geometry_msgs::msg::TransformStamped generateTransformMsg(
    const std::string & parent_frame, const std::string & child_frame, double x, double y, double z,
    double qx, double qy, double qz, double qw)
  {
    rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = timestamp;
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;
    return tf_msg;
  }

  std::vector<geometry_msgs::msg::TransformStamped> generateStaticTransformMsg()
  {
    // generate defined transformations
    return {
      generateTransformMsg("base_link", "lidar_top", 5.0, 5.0, 5.0, 0.683, 0.5, 0.183, 0.499),
      generateTransformMsg("base_link", "imu_link", 1.0, 1.0, 3.0, 0.278, 0.717, 0.441, 0.453)};
  }

  std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> generateTwistMsg(
    double linear_x, double angular_z, rclcpp::Time stamp)
  {
    auto twist_msg = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
    twist_msg->header.stamp = stamp;
    twist_msg->header.frame_id = "base_link";
    twist_msg->twist.twist.linear.x = linear_x;
    twist_msg->twist.twist.angular.z = angular_z;
    return twist_msg;
  }

  std::shared_ptr<sensor_msgs::msg::Imu> generateImuMsg(
    double angular_vel_x, double angular_vel_y, double angular_vel_z, rclcpp::Time stamp)
  {
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = stamp;
    imu_msg->header.frame_id = "imu_link";
    imu_msg->angular_velocity.x = angular_vel_x;
    imu_msg->angular_velocity.y = angular_vel_y;
    imu_msg->angular_velocity.z = angular_vel_z;
    return imu_msg;
  }

  std::vector<std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped>> generateTwistMsgs(
    rclcpp::Time pointcloud_timestamp)
  {
    std::vector<std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped>> twist_msgs;
    rclcpp::Time twist_stamp = subtractMilliseconds(pointcloud_timestamp, 5);

    for (int i = 0; i < number_of_twist_msgs_; ++i) {
      auto twist_msg = generateTwistMsg(
        twist_linear_x_ + i * twist_linear_x_increment_,
        twist_angular_z_ + i * twist_angular_z_increment_, twist_stamp);
      twist_msgs.push_back(twist_msg);

      twist_stamp = addMilliseconds(twist_stamp, twist_msgs_interval_ms_);
    }

    return twist_msgs;
  }

  std::vector<std::shared_ptr<sensor_msgs::msg::Imu>> generateImuMsgs(
    rclcpp::Time pointcloud_timestamp)
  {
    std::vector<std::shared_ptr<sensor_msgs::msg::Imu>> imu_msgs;
    rclcpp::Time imu_stamp = subtractMilliseconds(pointcloud_timestamp, 10);

    for (int i = 0; i < number_of_imu_msgs_; ++i) {
      auto imu_msg = generateImuMsg(
        imu_angular_x_ + i * imu_angular_x_increment_,
        imu_angular_y_ + i * imu_angular_y_increment_,
        imu_angular_z_ + i * imu_angular_z_increment_, imu_stamp);
      imu_msgs.push_back(imu_msg);
      imu_stamp = addMilliseconds(imu_stamp, imu_msgs_interval_ms_);
    }

    return imu_msgs;
  }

  sensor_msgs::msg::PointCloud2 generatePointCloudMsg(
    bool generate_points, bool is_lidar_frame, rclcpp::Time stamp)
  {
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pointcloud_msg.header.stamp = stamp;
    pointcloud_msg.header.frame_id = is_lidar_frame ? "lidar_top" : "base_link";
    pointcloud_msg.height = 1;
    pointcloud_msg.is_dense = true;
    pointcloud_msg.is_bigendian = false;

    if (generate_points) {
      std::array<Eigen::Vector3f, number_of_points_> points = {{
        Eigen::Vector3f(10.0f, 0.0f, 0.0f),   // point 1
        Eigen::Vector3f(0.0f, 10.0f, 0.0f),   // point 2
        Eigen::Vector3f(0.0f, 0.0f, 10.0f),   // point 3
        Eigen::Vector3f(20.0f, 0.0f, 0.0f),   // point 4
        Eigen::Vector3f(0.0f, 20.0f, 0.0f),   // point 5
        Eigen::Vector3f(0.0f, 0.0f, 20.0f),   // point 6
        Eigen::Vector3f(30.0f, 0.0f, 0.0f),   // point 7
        Eigen::Vector3f(0.0f, 30.0f, 0.0f),   // point 8
        Eigen::Vector3f(0.0f, 0.0f, 30.0f),   // point 9
        Eigen::Vector3f(10.0f, 10.0f, 10.0f)  // point 10
      }};

      // Generate timestamps for the points
      std::vector<std::uint32_t> timestamps = generatePointTimestamps(stamp, number_of_points_);

      sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
      modifier.setPointCloud2Fields(
        10, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::UINT8, "return_type", 1,
        sensor_msgs::msg::PointField::UINT8, "channel", 1, sensor_msgs::msg::PointField::UINT16,
        "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32, "elevation", 1,
        sensor_msgs::msg::PointField::FLOAT32, "distance", 1, sensor_msgs::msg::PointField::FLOAT32,
        "time_stamp", 1, sensor_msgs::msg::PointField::UINT32);

      modifier.resize(number_of_points_);

      sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
      sensor_msgs::PointCloud2Iterator<std::uint32_t> iter_t(pointcloud_msg, "time_stamp");

      for (size_t i = 0; i < number_of_points_; ++i) {
        *iter_x = points[i].x();
        *iter_y = points[i].y();
        *iter_z = points[i].z();
        *iter_t = timestamps[i];
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_t;
      }
    } else {
      pointcloud_msg.width = 0;
      pointcloud_msg.row_step = 0;
    }

    return pointcloud_msg;
  }

  std::vector<std::uint32_t> generatePointTimestamps(
    rclcpp::Time pointcloud_timestamp, size_t number_of_points)
  {
    std::vector<std::uint32_t> timestamps;
    rclcpp::Time global_point_stamp = pointcloud_timestamp;
    for (size_t i = 0; i < number_of_points; ++i) {
      std::uint32_t relative_timestamp = (global_point_stamp - pointcloud_timestamp).nanoseconds();
      timestamps.push_back(relative_timestamp);
      global_point_stamp = addMilliseconds(global_point_stamp, points_interval_ms_);
    }

    return timestamps;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware::pointcloud_preprocessor::DistortionCorrector2D>
    distortion_corrector_2d_;
  std::shared_ptr<autoware::pointcloud_preprocessor::DistortionCorrector3D>
    distortion_corrector_3d_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  static constexpr float standard_tolerance_{1e-4};
  static constexpr float coarse_tolerance_{5e-3};
  static constexpr int number_of_twist_msgs_{6};
  static constexpr int number_of_imu_msgs_{6};
  static constexpr size_t number_of_points_{10};
  static constexpr int32_t timestamp_seconds_{10};
  static constexpr uint32_t timestamp_nanoseconds_{100000000};

  static constexpr double twist_linear_x_{10.0};
  static constexpr double twist_angular_z_{0.02};
  static constexpr double twist_linear_x_increment_{2.0};
  static constexpr double twist_angular_z_increment_{0.01};

  static constexpr double imu_angular_x_{0.01};
  static constexpr double imu_angular_y_{-0.02};
  static constexpr double imu_angular_z_{0.05};
  static constexpr double imu_angular_x_increment_{0.005};
  static constexpr double imu_angular_y_increment_{0.005};
  static constexpr double imu_angular_z_increment_{0.005};

  static constexpr int points_interval_ms_{10};
  static constexpr int twist_msgs_interval_ms_{24};
  static constexpr int imu_msgs_interval_ms_{27};

  // for debugging or regenerating the ground truth point cloud
  bool debug_{false};
};

TEST_F(DistortionCorrectorTest, TestProcessTwistMessage)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  auto twist_msg = generateTwistMsg(twist_linear_x_, twist_angular_z_, timestamp);
  distortion_corrector_2d_->processTwistMessage(twist_msg);

  ASSERT_FALSE(distortion_corrector_2d_->get_twist_queue().empty());
  EXPECT_EQ(distortion_corrector_2d_->get_twist_queue().front().twist.linear.x, twist_linear_x_);
  EXPECT_EQ(distortion_corrector_2d_->get_twist_queue().front().twist.angular.z, twist_angular_z_);
}

TEST_F(DistortionCorrectorTest, TestProcessIMUMessage)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  auto imu_msg = generateImuMsg(imu_angular_x_, imu_angular_y_, imu_angular_z_, timestamp);
  distortion_corrector_2d_->processIMUMessage("base_link", imu_msg);

  ASSERT_FALSE(distortion_corrector_2d_->get_angular_velocity_queue().empty());
  EXPECT_NEAR(
    distortion_corrector_2d_->get_angular_velocity_queue().front().vector.z, -0.03159,
    standard_tolerance_);
}

TEST_F(DistortionCorrectorTest, TestIsInputValid)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);

  // input normal pointcloud without twist
  sensor_msgs::msg::PointCloud2 pointcloud = generatePointCloudMsg(true, false, timestamp);
  bool result = distortion_corrector_2d_->isInputValid(pointcloud);
  EXPECT_FALSE(result);

  // input normal pointcloud with valid twist
  auto twist_msg = generateTwistMsg(twist_linear_x_, twist_angular_z_, timestamp);
  distortion_corrector_2d_->processTwistMessage(twist_msg);

  pointcloud = generatePointCloudMsg(true, false, timestamp);
  result = distortion_corrector_2d_->isInputValid(pointcloud);
  EXPECT_TRUE(result);

  // input empty pointcloud
  pointcloud = generatePointCloudMsg(false, false, timestamp);
  result = distortion_corrector_2d_->isInputValid(pointcloud);
  EXPECT_FALSE(result);
}

TEST_F(DistortionCorrectorTest, TestSetPointCloudTransformWithBaseLink)
{
  distortion_corrector_2d_->setPointCloudTransform("base_link", "base_link");
  EXPECT_TRUE(distortion_corrector_2d_->pointcloud_transform_exists());
  EXPECT_FALSE(distortion_corrector_2d_->pointcloud_transform_needed());
}

TEST_F(DistortionCorrectorTest, TestSetPointCloudTransformWithLidarFrame)
{
  distortion_corrector_2d_->setPointCloudTransform("base_link", "lidar_top");
  EXPECT_TRUE(distortion_corrector_2d_->pointcloud_transform_exists());
  EXPECT_TRUE(distortion_corrector_2d_->pointcloud_transform_needed());
}

TEST_F(DistortionCorrectorTest, TestSetPointCloudTransformWithMissingFrame)
{
  distortion_corrector_2d_->setPointCloudTransform("base_link", "missing_lidar_frame");
  EXPECT_FALSE(distortion_corrector_2d_->pointcloud_transform_exists());
  EXPECT_FALSE(distortion_corrector_2d_->pointcloud_transform_needed());
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudWithEmptyTwist)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  // Generate the point cloud message
  sensor_msgs::msg::PointCloud2 pointcloud = generatePointCloudMsg(true, false, timestamp);

  // Process empty twist queue
  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->undistortPointCloud(false, pointcloud);

  // Verify the point cloud is not changed
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  std::array<Eigen::Vector3f, number_of_points_> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 10.0f, 0.0f),
     Eigen::Vector3f(0.0f, 0.0f, 10.0f), Eigen::Vector3f(20.0f, 0.0f, 0.0f),
     Eigen::Vector3f(0.0f, 20.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 20.0f),
     Eigen::Vector3f(30.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 30.0f, 0.0f),
     Eigen::Vector3f(0.0f, 0.0f, 30.0f), Eigen::Vector3f(10.0f, 10.0f, 10.0f)}};

  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudWithEmptyPointCloud)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }
  // Generate an empty point cloud message
  sensor_msgs::msg::PointCloud2 empty_pointcloud = generatePointCloudMsg(false, false, timestamp);

  // Process empty point cloud
  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->undistortPointCloud(true, empty_pointcloud);

  // Verify the point cloud is still empty
  EXPECT_EQ(empty_pointcloud.width, 0);
  EXPECT_EQ(empty_pointcloud.row_step, 0);
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud2dWithoutImuInBaseLink)
{
  // Generate the point cloud message
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud = generatePointCloudMsg(true, false, timestamp);

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }

  // Test using only twist
  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_2d_->undistortPointCloud(false, pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  // Expected undistorted point cloud values
  std::array<Eigen::Vector3f, 10> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, 0.0f, 0.0f), Eigen::Vector3f(0.117124f, 10.0f, 0.0f),
     Eigen::Vector3f(0.26f, 0.000135182f, 10.0f), Eigen::Vector3f(20.4f, 0.0213818f, 0.0f),
     Eigen::Vector3f(0.50932f, 20.0005f, 0.0f), Eigen::Vector3f(0.699999f, 0.000819721f, 20.0f),
     Eigen::Vector3f(30.8599f, 0.076f, 0.0f), Eigen::Vector3f(0.947959f, 30.0016f, 0.0f),
     Eigen::Vector3f(1.22f, 0.00244382f, 30.0f), Eigen::Vector3f(11.3568f, 10.0463f, 10.0f)}};

  // Verify each point in the undistorted point cloud
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud2dWithImuInBaseLink)
{
  // Generate the point cloud message
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud = generatePointCloudMsg(true, false, timestamp);

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }

  // Generate and process multiple IMU messages
  auto imu_msgs = generateImuMsgs(timestamp);
  for (const auto & imu_msg : imu_msgs) {
    distortion_corrector_2d_->processIMUMessage("base_link", imu_msg);
  }

  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_2d_->undistortPointCloud(true, pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  // Expected undistorted point cloud values
  std::array<Eigen::Vector3f, 10> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, 0.0f, 0.0f), Eigen::Vector3f(0.122876f, 9.99996f, 0.0f),
     Eigen::Vector3f(0.26f, -0.000115049f, 10.0f), Eigen::Vector3f(20.4f, -0.0174931f, 0.0f),
     Eigen::Vector3f(0.56301f, 19.9996f, 0.0f), Eigen::Vector3f(0.7f, -0.000627014f, 20.0f),
     Eigen::Vector3f(30.86f, -0.052675f, 0.0f), Eigen::Vector3f(1.1004f, 29.9987f, 0.0f),
     Eigen::Vector3f(1.22f, -0.00166245f, 30.0f), Eigen::Vector3f(11.4249f, 9.97293f, 10.0f)}};

  // Verify each point in the undistorted point cloud
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud2dWithImuInLidarFrame)
{
  // Generate the point cloud message
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud = generatePointCloudMsg(true, true, timestamp);

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }

  // Generate and process multiple IMU messages
  auto imu_msgs = generateImuMsgs(timestamp);
  for (const auto & imu_msg : imu_msgs) {
    distortion_corrector_2d_->processIMUMessage("base_link", imu_msg);
  }

  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "lidar_top");
  distortion_corrector_2d_->undistortPointCloud(true, pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  // Expected undistorted point cloud values
  std::array<Eigen::Vector3f, 10> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, -1.77636e-15f, -4.44089e-16f),
     Eigen::Vector3f(0.049989f, 10.0608f, 0.0924992f),
     Eigen::Vector3f(0.106107f, 0.130237f, 10.1986f),
     Eigen::Vector3f(20.1709f, 0.210011f, 0.32034f),
     Eigen::Vector3f(0.220674f, 20.2734f, 0.417974f),
     Eigen::Vector3f(0.274146f, 0.347043f, 20.5341f),
     Eigen::Vector3f(30.3673f, 0.457564f, 0.700818f),
     Eigen::Vector3f(0.418014f, 30.5259f, 0.807963f),
     Eigen::Vector3f(0.464088f, 0.600081f, 30.9292f),
     Eigen::Vector3f(10.5657f, 10.7121f, 11.094f)}};

  // Verify each point in the undistorted point cloud
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud3dWithoutImuInBaseLink)
{
  // Generate the point cloud message
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud = generatePointCloudMsg(true, false, timestamp);

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_3d_->processTwistMessage(twist_msg);
  }

  // Test using only twist
  distortion_corrector_3d_->initialize();
  distortion_corrector_3d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_3d_->undistortPointCloud(false, pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  // Expected undistorted point cloud values
  std::array<Eigen::Vector3f, 10> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, 0.0f, 0.0f), Eigen::Vector3f(0.117f, 10.0f, 0.0f),
     Eigen::Vector3f(0.26f, 9.27035e-05f, 10.0f), Eigen::Vector3f(20.4f, 0.0222176f, 0.0f),
     Eigen::Vector3f(0.51f, 20.0004f, 0.0f), Eigen::Vector3f(0.7f, 0.000706573f, 20.0f),
     Eigen::Vector3f(30.8599f, 0.0760946f, 0.0f), Eigen::Vector3f(0.946998f, 30.0015f, 0.0f),
     Eigen::Vector3f(1.22f, 0.00234201f, 30.0f), Eigen::Vector3f(11.3569f, 10.046f, 10.0f)}};

  // Verify each point in the undistorted point cloud
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud3dWithImuInBaseLink)
{
  // Generate the point cloud message
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud = generatePointCloudMsg(true, false, timestamp);

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_3d_->processTwistMessage(twist_msg);
  }

  // Generate and process multiple IMU messages
  auto imu_msgs = generateImuMsgs(timestamp);
  for (const auto & imu_msg : imu_msgs) {
    distortion_corrector_3d_->processIMUMessage("base_link", imu_msg);
  }

  distortion_corrector_3d_->initialize();
  distortion_corrector_3d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_3d_->undistortPointCloud(true, pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  // Expected undistorted point cloud values
  std::array<Eigen::Vector3f, 10> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, 0.0f, 0.0f), Eigen::Vector3f(0.123015f, 9.99998f, 0.00430552f),
     Eigen::Vector3f(0.266103f, -0.00895269f, 9.99992f),
     Eigen::Vector3f(20.4f, -0.0176539f, -0.0193392f),
     Eigen::Vector3f(0.563265f, 19.9997f, 0.035628f),
     Eigen::Vector3f(0.734597f, -0.046068f, 19.9993f),
     Eigen::Vector3f(30.8599f, -0.0517931f, -0.0658165f),
     Eigen::Vector3f(1.0995f, 29.9989f, 0.0956997f),
     Eigen::Vector3f(1.31283f, -0.113544f, 29.9977f),
     Eigen::Vector3f(11.461f, 9.93096f, 10.0035f)}};

  // Verify each point in the undistorted point cloud
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud3dWithImuInLidarFrame)
{
  // Generate the point cloud message
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud = generatePointCloudMsg(true, true, timestamp);

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_3d_->processTwistMessage(twist_msg);
  }

  // Generate and process multiple IMU messages
  auto imu_msgs = generateImuMsgs(timestamp);
  for (const auto & imu_msg : imu_msgs) {
    distortion_corrector_3d_->processIMUMessage("base_link", imu_msg);
  }

  distortion_corrector_3d_->initialize();
  distortion_corrector_3d_->setPointCloudTransform("base_link", "lidar_top");
  distortion_corrector_3d_->undistortPointCloud(true, pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  // Expected undistorted point cloud values
  std::array<Eigen::Vector3f, 10> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, 0.0f, 0.0f), Eigen::Vector3f(0.046484f, 10.0622f, 0.098484f),
     Eigen::Vector3f(0.107595f, 0.123767f, 10.2026f),
     Eigen::Vector3f(20.1667f, 0.22465f, 0.313351f),
     Eigen::Vector3f(0.201149f, 20.2784f, 0.464665f),
     Eigen::Vector3f(0.290531f, 0.303489f, 20.5452f),
     Eigen::Vector3f(30.3598f, 0.494116f, 0.672914f),
     Eigen::Vector3f(0.375848f, 30.5336f, 0.933633f),
     Eigen::Vector3f(0.510001f, 0.479651f, 30.9493f),
     Eigen::Vector3f(10.5629f, 10.6855f, 11.1461f)}};

  // Verify each point in the undistorted point cloud
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudWithPureLinearMotion)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 test2d_pointcloud = generatePointCloudMsg(true, false, timestamp);
  sensor_msgs::msg::PointCloud2 test3d_pointcloud = generatePointCloudMsg(true, false, timestamp);

  // Generate and process a single twist message with constant linear velocity
  auto twist_msg = generateTwistMsg(1.0, 0.0, timestamp);

  distortion_corrector_2d_->processTwistMessage(twist_msg);
  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_2d_->undistortPointCloud(false, test2d_pointcloud);

  distortion_corrector_3d_->processTwistMessage(twist_msg);
  distortion_corrector_3d_->initialize();
  distortion_corrector_3d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_3d_->undistortPointCloud(false, test3d_pointcloud);

  // Generate expected point cloud for testing
  sensor_msgs::msg::PointCloud2 expected_pointcloud_msg =
    generatePointCloudMsg(true, false, timestamp);

  // Calculate expected point cloud values based on constant linear motion
  double velocity = 1.0;  // 1 m/s linear velocity
  sensor_msgs::PointCloud2Iterator<float> iter_x(expected_pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(expected_pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(expected_pointcloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<std::uint32_t> iter_t(expected_pointcloud_msg, "time_stamp");

  std::vector<Eigen::Vector3f> expected_points;
  for (; iter_t != iter_t.end(); ++iter_t, ++iter_x, ++iter_y, ++iter_z) {
    double time_offset = static_cast<double>(*iter_t) / 1e9;
    expected_points.emplace_back(
      *iter_x + static_cast<float>(velocity * time_offset), *iter_y, *iter_z);
  }

  // Verify each point in the undistorted point cloud
  sensor_msgs::PointCloud2Iterator<float> test2d_iter_x(test2d_pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> test2d_iter_y(test2d_pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> test2d_iter_z(test2d_pointcloud, "z");
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; test2d_iter_x != test2d_iter_x.end();
       ++test2d_iter_x, ++test2d_iter_y, ++test2d_iter_z, ++i) {
    oss << "Point " << i << ": (" << *test2d_iter_x << ", " << *test2d_iter_y << ", "
        << *test2d_iter_z << ")\n";
    EXPECT_FLOAT_EQ(*test2d_iter_x, expected_points[i].x());
    EXPECT_FLOAT_EQ(*test2d_iter_y, expected_points[i].y());
    EXPECT_FLOAT_EQ(*test2d_iter_z, expected_points[i].z());
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }

  // Verify each point in the undistorted 2d point cloud with undistorted 3d point cloud
  test2d_iter_x = sensor_msgs::PointCloud2Iterator<float>(test2d_pointcloud, "x");
  test2d_iter_y = sensor_msgs::PointCloud2Iterator<float>(test2d_pointcloud, "y");
  test2d_iter_z = sensor_msgs::PointCloud2Iterator<float>(test2d_pointcloud, "z");
  sensor_msgs::PointCloud2Iterator<float> test3d_iter_x(test3d_pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> test3d_iter_y(test3d_pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> test3d_iter_z(test3d_pointcloud, "z");

  i = 0;
  oss.str("");
  oss.clear();

  oss << "Expected pointcloud:\n";
  for (; test2d_iter_x != test2d_iter_x.end(); ++test2d_iter_x, ++test2d_iter_y, ++test2d_iter_z,
                                               ++test3d_iter_x, ++test3d_iter_y, ++test3d_iter_z,
                                               ++i) {
    oss << "Point " << i << " - 2D: (" << *test2d_iter_x << ", " << *test2d_iter_y << ", "
        << *test2d_iter_z << ")"
        << " vs 3D: (" << *test3d_iter_x << ", " << *test3d_iter_y << ", " << *test3d_iter_z
        << ")\n";
    EXPECT_FLOAT_EQ(*test2d_iter_x, *test3d_iter_x);
    EXPECT_FLOAT_EQ(*test2d_iter_y, *test3d_iter_y);
    EXPECT_FLOAT_EQ(*test2d_iter_z, *test3d_iter_z);
  }
  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudWithPureRotationalMotion)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 test2d_pointcloud = generatePointCloudMsg(true, false, timestamp);
  sensor_msgs::msg::PointCloud2 test3d_pointcloud = generatePointCloudMsg(true, false, timestamp);

  // Generate and process a single twist message with constant angular velocity
  auto twist_msg = generateTwistMsg(0.0, 0.1, timestamp);

  distortion_corrector_2d_->processTwistMessage(twist_msg);
  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_2d_->undistortPointCloud(false, test2d_pointcloud);

  distortion_corrector_3d_->processTwistMessage(twist_msg);
  distortion_corrector_3d_->initialize();
  distortion_corrector_3d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_3d_->undistortPointCloud(false, test3d_pointcloud);

  // Generate expected point cloud for testing
  sensor_msgs::msg::PointCloud2 expected_pointcloud_msg =
    generatePointCloudMsg(true, false, timestamp);

  // Calculate expected point cloud values based on constant rotational motion
  double angular_velocity = 0.1;  // 0.1 rad/s rotational velocity
  sensor_msgs::PointCloud2Iterator<float> iter_x(expected_pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(expected_pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(expected_pointcloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<std::uint32_t> iter_t(expected_pointcloud_msg, "time_stamp");

  std::vector<Eigen::Vector3f> expected_pointcloud;
  for (; iter_t != iter_t.end(); ++iter_t, ++iter_x, ++iter_y, ++iter_z) {
    double time_offset = static_cast<double>(*iter_t) / 1e9;
    float angle = angular_velocity * time_offset;

    // Set the quaternion for the current angle
    tf2::Quaternion quaternion;
    quaternion.setValue(
      0, 0, autoware::universe_utils::sin(angle * 0.5f),
      autoware::universe_utils::cos(angle * 0.5f));

    tf2::Vector3 point(*iter_x, *iter_y, *iter_z);
    tf2::Vector3 rotated_point = tf2::quatRotate(quaternion, point);
    expected_pointcloud.emplace_back(
      static_cast<float>(rotated_point.x()), static_cast<float>(rotated_point.y()),
      static_cast<float>(rotated_point.z()));
  }

  // Verify each point in the undistorted 2D point cloud
  sensor_msgs::PointCloud2Iterator<float> test2d_iter_x(test2d_pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> test2d_iter_y(test2d_pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> test2d_iter_z(test2d_pointcloud, "z");

  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; test2d_iter_x != test2d_iter_x.end();
       ++test2d_iter_x, ++test2d_iter_y, ++test2d_iter_z, ++i) {
    oss << "Point " << i << ": (" << *test2d_iter_x << ", " << *test2d_iter_y << ", "
        << *test2d_iter_z << ")\n";
    EXPECT_FLOAT_EQ(*test2d_iter_x, expected_pointcloud[i].x());
    EXPECT_FLOAT_EQ(*test2d_iter_y, expected_pointcloud[i].y());
    EXPECT_FLOAT_EQ(*test2d_iter_z, expected_pointcloud[i].z());
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }

  // Verify each point in the undistorted 2D point cloud with undistorted 3D point cloud
  test2d_iter_x = sensor_msgs::PointCloud2Iterator<float>(test2d_pointcloud, "x");
  test2d_iter_y = sensor_msgs::PointCloud2Iterator<float>(test2d_pointcloud, "y");
  test2d_iter_z = sensor_msgs::PointCloud2Iterator<float>(test2d_pointcloud, "z");
  sensor_msgs::PointCloud2Iterator<float> test3d_iter_x(test3d_pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> test3d_iter_y(test3d_pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> test3d_iter_z(test3d_pointcloud, "z");

  i = 0;
  oss.str("");
  oss.clear();

  oss << "Expected pointcloud:\n";
  for (; test2d_iter_x != test2d_iter_x.end(); ++test2d_iter_x, ++test2d_iter_y, ++test2d_iter_z,
                                               ++test3d_iter_x, ++test3d_iter_y, ++test3d_iter_z,
                                               ++i) {
    oss << "Point " << i << " - 2D: (" << *test2d_iter_x << ", " << *test2d_iter_y << ", "
        << *test2d_iter_z << ")"
        << " vs 3D: (" << *test3d_iter_x << ", " << *test3d_iter_y << ", " << *test3d_iter_z
        << ")\n";
    EXPECT_NEAR(*test2d_iter_x, *test3d_iter_x, coarse_tolerance_);
    EXPECT_NEAR(*test2d_iter_y, *test3d_iter_y, coarse_tolerance_);
    EXPECT_NEAR(*test2d_iter_z, *test3d_iter_z, coarse_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
