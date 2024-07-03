// Copyright 2024 Tier IV, Inc.
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

#include "pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

class DistortionCorrectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_node");
    distortion_corrector_2d_ =
      std::make_shared<pointcloud_preprocessor::DistortionCorrector2D>(node_.get());
    distortion_corrector_3d_ =
      std::make_shared<pointcloud_preprocessor::DistortionCorrector3D>(node_.get());

    // Setup TF
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    tf_broadcaster_->sendTransform(generateStaticTransformMsg());

    // Spin the node for a while to ensure transforms are published
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() - start < timeout) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void TearDown() override {}

  rclcpp::Time addMilliseconds(rclcpp::Time stamp, int ms)
  {
    auto ms_in_ns = rclcpp::Duration(0, ms * 1000000);
    return stamp + ms_in_ns;
  }

  rclcpp::Time subtractMilliseconds(rclcpp::Time stamp, int ms)
  {
    auto ms_in_ns = rclcpp::Duration(0, ms * 1000000);
    return stamp - ms_in_ns;
  }

  geometry_msgs::msg::TransformStamped generateTransformMsg(
    const std::string & parent_frame, const std::string & child_frame, double x, double y, double z,
    double qx, double qy, double qz, double qw)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = node_->get_clock()->now();
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

    for (int i = 0; i < 6; ++i) {
      auto twist_msg = generateTwistMsg(10.0 + i * 2, 0.02 + i * 0.01, twist_stamp);
      twist_msgs.push_back(twist_msg);
      // Make sure the twist stamp is not identical to any point stamp, and imu stamp
      twist_stamp = addMilliseconds(twist_stamp, 24);
    }

    return twist_msgs;
  }

  std::vector<std::shared_ptr<sensor_msgs::msg::Imu>> generateImuMsgs(
    rclcpp::Time pointcloud_timestamp)
  {
    std::vector<std::shared_ptr<sensor_msgs::msg::Imu>> imu_msgs;
    rclcpp::Time imu_stamp = subtractMilliseconds(pointcloud_timestamp, 10);

    for (int i = 0; i < 6; ++i) {
      auto imu_msg =
        generateImuMsg(0.01 + i * 0.005, -0.02 + i * 0.005, 0.05 + i * 0.005, imu_stamp);
      imu_msgs.push_back(imu_msg);
      // Make sure the imu stamp is not identical to any point stamp, and twist stamp
      imu_stamp = addMilliseconds(imu_stamp, 27);
    }

    return imu_msgs;
  }

  sensor_msgs::msg::PointCloud2 generatePointCloudMsg(
    bool is_generate_points, bool is_lidar_frame, rclcpp::Time stamp)
  {
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pointcloud_msg.header.stamp = stamp;
    pointcloud_msg.header.frame_id = is_lidar_frame ? "lidar_top" : "base_link";
    pointcloud_msg.height = 1;
    pointcloud_msg.is_dense = true;
    pointcloud_msg.is_bigendian = false;
    pointcloud_msg.point_step =
      20;  // 3 float32 fields * 4 bytes/field + 1 float64 field * 8 bytes/field

    if (is_generate_points) {
      std::vector<float> points = {
        10.0f, 0.0f,  0.0f,   // point 1
        0.0f,  10.0f, 0.0f,   // point 2
        0.0f,  0.0f,  10.0f,  // point 3
        20.0f, 0.0f,  0.0f,   // point 4
        0.0f,  20.0f, 0.0f,   // point 5
        0.0f,  0.0f,  20.0f,  // point 6
        30.0f, 0.0f,  0.0f,   // point 7
        0.0f,  30.0f, 0.0f,   // point 8
        0.0f,  0.0f,  30.0f,  // point 9
        10.0f, 10.0f, 10.0f   // point 10
      };

      size_t number_of_points = points.size() / 3;
      std::vector<double> timestamps =
        generatePointTimestamps(is_generate_points, stamp, number_of_points);

      std::vector<uint8_t> data(number_of_points * pointcloud_msg.point_step);

      for (size_t i = 0; i < number_of_points; ++i) {
        std::memcpy(data.data() + i * pointcloud_msg.point_step, &points[i * 3], 3 * sizeof(float));
        std::memcpy(
          data.data() + i * pointcloud_msg.point_step + 12, &timestamps[i], sizeof(double));
      }

      pointcloud_msg.width = number_of_points;
      pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width;
      pointcloud_msg.data = std::move(data);
    } else {
      pointcloud_msg.width = 0;
      pointcloud_msg.row_step = 0;
    }

    sensor_msgs::msg::PointField x_field;
    x_field.name = "x";
    x_field.offset = 0;
    x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    x_field.count = 1;

    sensor_msgs::msg::PointField y_field;
    y_field.name = "y";
    y_field.offset = 4;
    y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    y_field.count = 1;

    sensor_msgs::msg::PointField z_field;
    z_field.name = "z";
    z_field.offset = 8;
    z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    z_field.count = 1;

    sensor_msgs::msg::PointField timestamp_field;
    timestamp_field.name = "time_stamp";
    timestamp_field.offset = 12;
    timestamp_field.datatype = sensor_msgs::msg::PointField::FLOAT64;
    timestamp_field.count = 1;

    pointcloud_msg.fields = {x_field, y_field, z_field, timestamp_field};

    return pointcloud_msg;
  }

  std::vector<double> generatePointTimestamps(
    bool is_generate_points, rclcpp::Time pointcloud_timestamp, size_t number_of_points)
  {
    std::vector<double> timestamps;
    if (is_generate_points) {
      rclcpp::Time point_stamp = pointcloud_timestamp;
      for (size_t i = 0; i < number_of_points; ++i) {
        double timestamp = point_stamp.seconds();
        timestamps.push_back(timestamp);
        if (i > 0) {
          point_stamp = point_stamp + rclcpp::Duration::from_seconds(
                                        0.001);  // Assuming 1ms offset for demonstration
        }
      }
    }
    return timestamps;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<pointcloud_preprocessor::DistortionCorrector2D> distortion_corrector_2d_;
  std::shared_ptr<pointcloud_preprocessor::DistortionCorrector3D> distortion_corrector_3d_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

TEST_F(DistortionCorrectorTest, TestProcessTwistMessage)
{
  auto twist_msg = generateTwistMsg(1.0, 0.5, node_->get_clock()->now());
  distortion_corrector_2d_->processTwistMessage(twist_msg);

  ASSERT_FALSE(distortion_corrector_2d_->twist_queue_.empty());
  EXPECT_EQ(distortion_corrector_2d_->twist_queue_.front().twist.linear.x, 1.0);
  EXPECT_EQ(distortion_corrector_2d_->twist_queue_.front().twist.angular.z, 0.5);
}

TEST_F(DistortionCorrectorTest, TestProcessIMUMessage)
{
  auto imu_msg = generateImuMsg(0.5, 0.3, 0.1, node_->get_clock()->now());
  distortion_corrector_2d_->processIMUMessage("base_link", imu_msg);

  ASSERT_FALSE(distortion_corrector_2d_->angular_velocity_queue_.empty());
  EXPECT_NEAR(distortion_corrector_2d_->angular_velocity_queue_.front().vector.z, 0.0443032, 1e-5);
}

TEST_F(DistortionCorrectorTest, TestIsInputValid)
{
  // input normal pointcloud without twist
  sensor_msgs::msg::PointCloud2 pointcloud =
    generatePointCloudMsg(true, false, node_->get_clock()->now());
  bool result = distortion_corrector_2d_->isInputValid(pointcloud);
  EXPECT_FALSE(result);

  // input normal pointcloud with valid twist
  auto twist_msg = generateTwistMsg(1.0, 0.5, node_->get_clock()->now());
  distortion_corrector_2d_->processTwistMessage(twist_msg);

  pointcloud = generatePointCloudMsg(true, false, node_->get_clock()->now());
  result = distortion_corrector_2d_->isInputValid(pointcloud);
  EXPECT_TRUE(result);

  // input empty pointcloud
  pointcloud = generatePointCloudMsg(false, false, node_->get_clock()->now());
  result = distortion_corrector_2d_->isInputValid(pointcloud);
  EXPECT_FALSE(result);
}

TEST_F(DistortionCorrectorTest, TestSetPointCloudTransformWithBaseLink)
{
  distortion_corrector_2d_->setPointCloudTransform("base_link", "base_link");
  EXPECT_TRUE(distortion_corrector_2d_->pointcloud_transform_exists_);
  EXPECT_FALSE(distortion_corrector_2d_->pointcloud_transform_needed_);
}

TEST_F(DistortionCorrectorTest, TestSetPointCloudTransformWithLidarFrame)
{
  distortion_corrector_2d_->setPointCloudTransform("base_link", "lidar_top");
  EXPECT_TRUE(distortion_corrector_2d_->pointcloud_transform_exists_);
  EXPECT_TRUE(distortion_corrector_2d_->pointcloud_transform_needed_);
}

TEST_F(DistortionCorrectorTest, TestSetPointCloudTransformWithMissingFrame)
{
  distortion_corrector_2d_->setPointCloudTransform("base_link", "missing_lidar_frame");
  EXPECT_FALSE(distortion_corrector_2d_->pointcloud_transform_exists_);
  EXPECT_FALSE(distortion_corrector_2d_->pointcloud_transform_needed_);
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudWithEmptyTwist)
{
  rclcpp::Time timestamp = node_->get_clock()->now();
  // Generate the point cloud message
  sensor_msgs::msg::PointCloud2 pointcloud = generatePointCloudMsg(true, false, timestamp);

  // Process empty twist queue
  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->undistortPointCloud(false, pointcloud);

  // Verify the point cloud is not changed
  const float * data_ptr = reinterpret_cast<const float *>(pointcloud.data.data());
  std::vector<std::array<float, 3>> expected_pointcloud = {
    {10.0f, 0.0f, 0.0f}, {0.0f, 10.0f, 0.0f},   {0.0f, 0.0f, 10.0f}, {20.0f, 0.0f, 0.0f},
    {0.0f, 20.0f, 0.0f}, {0.0f, 0.0f, 20.0f},   {30.0f, 0.0f, 0.0f}, {0.0f, 30.0f, 0.0f},
    {0.0f, 0.0f, 30.0f}, {10.0f, 10.0f, 10.0f},
  };

  for (size_t i = 0; i < expected_pointcloud.size(); ++i) {
    EXPECT_NEAR(data_ptr[i * 5], expected_pointcloud[i][0], 1e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 1], expected_pointcloud[i][1], 1e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 2], expected_pointcloud[i][2], 1e-5);
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudWithEmptyPointCloud)
{
  rclcpp::Time timestamp = node_->get_clock()->now();
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
  rclcpp::Time timestamp = node_->get_clock()->now();
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

  const float * data_ptr = reinterpret_cast<const float *>(pointcloud.data.data());

  // Expected undistorted point cloud values
  std::vector<std::array<float, 3>> expected_pointcloud = {
    {10.0f, 0.0f, 0.0f},          {0.0f, 10.0f, 0.0f},       {0.012002f, 5.75338e-07f, 10.0f},
    {20.024f, 0.00191863f, 0.0f}, {0.0340828f, 20.0f, 0.0f}, {0.0479994f, 4.02654e-06f, 20.0f},
    {30.06f, 0.00575818f, 0.0f},  {0.0662481f, 30.0f, 0.0f}, {0.0839996f, 1.03542e-05f, 30.0f},
    {10.0931f, 10.0029f, 10.0f},
  };

  // Verify each point in the undistorted point cloud
  for (size_t i = 0; i < expected_pointcloud.size(); ++i) {
    EXPECT_NEAR(data_ptr[i * 5], expected_pointcloud[i][0], 5e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 1], expected_pointcloud[i][1], 5e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 2], expected_pointcloud[i][2], 5e-5);
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud2dWithImuInBaseLink)
{
  // Generate the point cloud message
  rclcpp::Time timestamp = node_->get_clock()->now();
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

  const float * data_ptr = reinterpret_cast<const float *>(pointcloud.data.data());

  // Expected undistorted point cloud values
  std::vector<std::array<float, 3>> expected_pointcloud = {
    {10.0f, 0.0f, 0.0f},         {0.0f, 10.0f, 0.0f},       {0.0119991f, 5.75201e-07f, 10.0f},
    {20.024f, 0.0019192f, 0.0f}, {0.0321653f, 20.0f, 0.0f}, {0.0479994f, 6.32762e-06f, 20.0f},
    {30.06f, 0.00863842f, 0.0f}, {0.063369f, 30.0f, 0.0f},  {0.0839996f, 1.84079e-05f, 30.0f},
    {10.0912f, 10.0048f, 10.0f},
  };

  // Verify each point in the undistorted point cloud
  for (size_t i = 0; i < expected_pointcloud.size(); ++i) {
    EXPECT_NEAR(data_ptr[i * 5], expected_pointcloud[i][0], 5e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 1], expected_pointcloud[i][1], 5e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 2], expected_pointcloud[i][2], 5e-5);
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud2dWithImuInLidarFrame)
{
  // Generate the point cloud message
  rclcpp::Time timestamp = node_->get_clock()->now();
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

  const float * data_ptr = reinterpret_cast<const float *>(pointcloud.data.data());
  // Expected undistorted point cloud values
  std::vector<std::array<float, 3>> expected_pointcloud = {
    {10.0f, -1.77636e-15f, -4.44089e-16f}, {-2.66454e-15f, 10.0f, -8.88178e-16f},
    {0.00622853f, 0.00600991f, 10.0084f},  {20.0106f, 0.010948f, 0.0157355f},
    {0.0176543f, 20.0176f, 0.0248379f},    {0.024499f, 0.0245179f, 20.0348f},
    {30.0266f, 0.0255826f, 0.0357166f},    {0.0355204f, 30.0353f, 0.0500275f},
    {0.047132f, 0.0439795f, 30.0606f},     {10.0488f, 10.046f, 10.0636f},
  };

  // Verify each point in the undistorted point cloud
  for (size_t i = 0; i < expected_pointcloud.size(); ++i) {
    EXPECT_NEAR(data_ptr[i * 5], expected_pointcloud[i][0], 5e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 1], expected_pointcloud[i][1], 5e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 2], expected_pointcloud[i][2], 5e-5);
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud3dWithoutImuInBaseLink)
{
  // Generate the point cloud message
  rclcpp::Time timestamp = node_->get_clock()->now();
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

  const float * data_ptr = reinterpret_cast<const float *>(pointcloud.data.data());

  // Expected undistorted point cloud values
  std::vector<std::array<float, 3>> expected_pointcloud = {
    {10.0f, 0.0f, 0.0f},          {0.0f, 10.0f, 0.0f},       {0.0119991f, 0.0f, 10.0f},
    {20.024f, 0.00120042f, 0.0f}, {0.0342002f, 20.0f, 0.0f}, {0.0479994f, 2.15994e-06f, 20.0f},
    {30.06f, 0.00450349f, 0.0f},  {0.0666005f, 30.0f, 0.0f}, {0.0839996f, 7.55993e-06f, 30.0f},
    {10.0936f, 10.0024f, 10.0f},
  };

  // Verify each point in the undistorted point cloud
  for (size_t i = 0; i < expected_pointcloud.size(); ++i) {
    EXPECT_NEAR(data_ptr[i * 5], expected_pointcloud[i][0], 5e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 1], expected_pointcloud[i][1], 5e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 2], expected_pointcloud[i][2], 5e-5);
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud3dWithImuInBaseLink)
{
  // Generate the point cloud message
  rclcpp::Time timestamp = node_->get_clock()->now();
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

  const float * data_ptr = reinterpret_cast<const float *>(pointcloud.data.data());

  // Expected undistorted point cloud values
  std::vector<std::array<float, 3>> expected_pointcloud = {
    {10.0f, 0.0f, 0.0f},
    {0.0f, 10.0f, 0.0f},
    {0.0118491f, -0.000149993f, 10.0f},
    {20.024f, 0.00220075f, 0.000600241f},
    {0.0327002f, 20.0f, 0.000900472f},
    {0.0468023f, -0.00119623f, 20.0f},
    {30.06f, 0.0082567f, 0.00225216f},
    {0.0621003f, 30.0f, 0.00270227f},
    {0.0808503f, -0.00313673f, 30.0f},
    {10.0904f, 10.0032f, 10.0024f},
  };

  // Verify each point in the undistorted point cloud
  for (size_t i = 0; i < expected_pointcloud.size(); ++i) {
    EXPECT_NEAR(data_ptr[i * 5], expected_pointcloud[i][0], 5e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 1], expected_pointcloud[i][1], 5e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 2], expected_pointcloud[i][2], 5e-5);
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud3dWithImuInLidarFrame)
{
  // Generate the point cloud message
  rclcpp::Time timestamp = node_->get_clock()->now();
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

  const float * data_ptr = reinterpret_cast<const float *>(pointcloud.data.data());
  // Expected undistorted point cloud values
  std::vector<std::array<float, 3>> expected_pointcloud = {
    {10.0f, 0.0f, 0.0f},
    {-4.76837e-07f, 10.0f, 4.76837e-07f},
    {0.00572586f, 0.00616837f, 10.0086f},
    {20.0103f, 0.0117249f, 0.0149349f},
    {0.0158343f, 20.0179f, 0.024497f},
    {0.0251098f, 0.0254798f, 20.0343f},
    {30.0259f, 0.0290527f, 0.034577f},
    {0.0319824f, 30.0358f, 0.0477753f},
    {0.0478067f, 0.0460052f, 30.06f},
    {10.0462f, 10.0489f, 10.0625f},
  };

  // Verify each point in the undistorted point cloud
  for (size_t i = 0; i < expected_pointcloud.size(); ++i) {
    EXPECT_NEAR(data_ptr[i * 5], expected_pointcloud[i][0], 5e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 1], expected_pointcloud[i][1], 5e-5);
    EXPECT_NEAR(data_ptr[i * 5 + 2], expected_pointcloud[i][2], 5e-5);
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
