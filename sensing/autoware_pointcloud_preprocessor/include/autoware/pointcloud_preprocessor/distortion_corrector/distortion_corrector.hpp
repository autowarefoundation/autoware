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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_HPP_

#include <Eigen/Core>
#include <autoware/universe_utils/ros/static_transform_buffer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sophus/se3.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <string>

namespace autoware::pointcloud_preprocessor
{

class DistortionCorrectorBase
{
public:
  virtual bool pointcloud_transform_exists() = 0;
  virtual bool pointcloud_transform_needed() = 0;
  virtual std::deque<geometry_msgs::msg::TwistStamped> get_twist_queue() = 0;
  virtual std::deque<geometry_msgs::msg::Vector3Stamped> get_angular_velocity_queue() = 0;

  virtual void processTwistMessage(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg) = 0;
  virtual void processIMUMessage(
    const std::string & base_frame, const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) = 0;
  virtual void setPointCloudTransform(
    const std::string & base_frame, const std::string & lidar_frame) = 0;
  virtual void initialize() = 0;
  virtual void undistortPointCloud(bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud) = 0;
};

template <class T>
class DistortionCorrector : public DistortionCorrectorBase
{
protected:
  geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr_;
  bool pointcloud_transform_needed_{false};
  bool pointcloud_transform_exists_{false};
  bool imu_transform_exists_{false};
  rclcpp::Node * node_;
  std::unique_ptr<autoware::universe_utils::StaticTransformBuffer> static_tf_buffer_{nullptr};

  std::deque<geometry_msgs::msg::TwistStamped> twist_queue_;
  std::deque<geometry_msgs::msg::Vector3Stamped> angular_velocity_queue_;

  void getIMUTransformation(const std::string & base_frame, const std::string & imu_frame);
  void enqueueIMU(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
  void getTwistAndIMUIterator(
    bool use_imu, double first_point_time_stamp_sec,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu);
  void warnIfTimestampIsTooLate(bool is_twist_time_stamp_too_late, bool is_imu_time_stamp_too_late);
  void undistortPoint(
    sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
    sensor_msgs::PointCloud2Iterator<float> & it_z,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, float const & time_offset,
    const bool & is_twist_valid, const bool & is_imu_valid)
  {
    static_cast<T *>(this)->undistortPointImplementation(
      it_x, it_y, it_z, it_twist, it_imu, time_offset, is_twist_valid, is_imu_valid);
  };
  void convertMatrixToTransform(const Eigen::Matrix4f & matrix, tf2::Transform & transform);

public:
  explicit DistortionCorrector(rclcpp::Node * node) : node_(node)
  {
    static_tf_buffer_ = std::make_unique<autoware::universe_utils::StaticTransformBuffer>();
  }
  bool pointcloud_transform_exists();
  bool pointcloud_transform_needed();
  std::deque<geometry_msgs::msg::TwistStamped> get_twist_queue();
  std::deque<geometry_msgs::msg::Vector3Stamped> get_angular_velocity_queue();
  void processTwistMessage(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg) override;

  void processIMUMessage(
    const std::string & base_frame, const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) override;
  void undistortPointCloud(bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud) override;
  bool isInputValid(sensor_msgs::msg::PointCloud2 & pointcloud);
};

class DistortionCorrector2D : public DistortionCorrector<DistortionCorrector2D>
{
private:
  // defined outside of for loop for performance reasons.
  tf2::Quaternion baselink_quat_;
  tf2::Transform baselink_tf_odom_;
  tf2::Vector3 point_tf_;
  tf2::Vector3 undistorted_point_tf_;
  float theta_;
  float x_;
  float y_;

  // TF
  tf2::Transform tf2_lidar_to_base_link_;
  tf2::Transform tf2_base_link_to_lidar_;

public:
  explicit DistortionCorrector2D(rclcpp::Node * node) : DistortionCorrector(node) {}
  void initialize() override;
  void undistortPointImplementation(
    sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
    sensor_msgs::PointCloud2Iterator<float> & it_z,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, const float & time_offset,
    const bool & is_twist_valid, const bool & is_imu_valid);

  void setPointCloudTransform(
    const std::string & base_frame, const std::string & lidar_frame) override;
};

class DistortionCorrector3D : public DistortionCorrector<DistortionCorrector3D>
{
private:
  // defined outside of for loop for performance reasons.
  Eigen::Vector4f point_eigen_;
  Eigen::Vector4f undistorted_point_eigen_;
  Eigen::Matrix4f transformation_matrix_;
  Eigen::Matrix4f prev_transformation_matrix_;

  // TF
  Eigen::Matrix4f eigen_lidar_to_base_link_;
  Eigen::Matrix4f eigen_base_link_to_lidar_;

public:
  explicit DistortionCorrector3D(rclcpp::Node * node) : DistortionCorrector(node) {}
  void initialize() override;
  void undistortPointImplementation(
    sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
    sensor_msgs::PointCloud2Iterator<float> & it_z,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, const float & time_offset,
    const bool & is_twist_valid, const bool & is_imu_valid);
  void setPointCloudTransform(
    const std::string & base_frame, const std::string & lidar_frame) override;
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_HPP_
