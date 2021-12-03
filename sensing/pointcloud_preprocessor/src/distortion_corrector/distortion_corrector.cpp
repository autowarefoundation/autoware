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

#include "pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"

#include <deque>
#include <string>
#include <utility>

namespace pointcloud_preprocessor
{
/** @brief Constructor. */
DistortionCorrectorComponent::DistortionCorrectorComponent(const rclcpp::NodeOptions & options)
: Node("distortion_corrector_node", options)
{
  // Parameter
  time_stamp_field_name_ = declare_parameter("time_stamp_field_name", "time_stamp");

  // Publisher
  undistorted_points_pub_ =
    this->create_publisher<PointCloud2>("~/output/pointcloud", rclcpp::SensorDataQoS());

  // Subscriber
  velocity_report_sub_ = this->create_subscription<VelocityReport>(
    "~/input/velocity_report", 10,
    std::bind(&DistortionCorrectorComponent::onVelocityReport, this, std::placeholders::_1));
  input_points_sub_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&DistortionCorrectorComponent::onPointCloud, this, std::placeholders::_1));
}

void DistortionCorrectorComponent::onVelocityReport(
  const VelocityReport::ConstSharedPtr velocity_report_msg)
{
  velocity_report_queue_.push_back(*velocity_report_msg);

  while (!velocity_report_queue_.empty()) {
    // for replay rosbag
    if (
      rclcpp::Time(velocity_report_queue_.front().header.stamp) >
      rclcpp::Time(velocity_report_msg->header.stamp)) {
      velocity_report_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(velocity_report_queue_.front().header.stamp) <
      rclcpp::Time(velocity_report_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      velocity_report_queue_.pop_front();
    }
    break;
  }
}

void DistortionCorrectorComponent::onPointCloud(PointCloud2::UniquePtr points_msg)
{
  const auto points_sub_count = undistorted_points_pub_->get_subscription_count() +
                                undistorted_points_pub_->get_intra_process_subscription_count();

  if (points_sub_count < 1) {
    return;
  }

  tf2::Transform tf2_base_link_to_sensor{};
  getTransform(points_msg->header.frame_id, base_link_frame_, &tf2_base_link_to_sensor);

  undistortPointCloud(velocity_report_queue_, tf2_base_link_to_sensor, *points_msg);

  if (points_sub_count > 0) {
    undistorted_points_pub_->publish(std::move(points_msg));
  }
}

bool DistortionCorrectorComponent::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  tf2::Transform * tf2_transform_ptr)
{
  if (target_frame == source_frame) {
    tf2_transform_ptr->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return true;
  }

  try {
    const auto transform_msg =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    tf2::convert(transform_msg.transform, *tf2_transform_ptr);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    tf2_transform_ptr->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return false;
  }
  return true;
}

bool DistortionCorrectorComponent::undistortPointCloud(
  const std::deque<VelocityReport> & velocity_report_queue_,
  const tf2::Transform & tf2_base_link_to_sensor, PointCloud2 & points)
{
  if (points.data.empty() || velocity_report_queue_.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000 /* ms */,
      "input_pointcloud->points or velocity_report_queue_ is empty.");
    return false;
  }

  auto time_stamp_field_it = std::find_if(
    std::cbegin(points.fields), std::cend(points.fields),
    [this](const sensor_msgs::msg::PointField & field) {
      return field.name == time_stamp_field_name_;
    });
  if (time_stamp_field_it == points.fields.cend()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000 /* ms */,
      "Required field time stamp doesn't exist in the point cloud.");
    return false;
  }

  sensor_msgs::PointCloud2Iterator<float> it_x(points, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(points, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(points, "z");
  sensor_msgs::PointCloud2ConstIterator<double> it_time_stamp(points, time_stamp_field_name_);

  float theta{0.0f};
  float x{0.0f};
  float y{0.0f};
  double prev_time_stamp_sec{*it_time_stamp};
  const double first_point_time_stamp_sec{*it_time_stamp};

  auto velocity_report_it = std::lower_bound(
    std::begin(velocity_report_queue_), std::end(velocity_report_queue_),
    first_point_time_stamp_sec, [](const VelocityReport & x, const double t) {
      return rclcpp::Time(x.header.stamp).seconds() < t;
    });
  velocity_report_it = velocity_report_it == std::end(velocity_report_queue_)
                         ? std::end(velocity_report_queue_) - 1
                         : velocity_report_it;

  const tf2::Transform tf2_base_link_to_sensor_inv{tf2_base_link_to_sensor.inverse()};
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_time_stamp) {
    for (;
         (velocity_report_it != std::end(velocity_report_queue_) - 1 &&
          *it_time_stamp > rclcpp::Time(velocity_report_it->header.stamp).seconds());
         ++velocity_report_it) {
    }

    float v{static_cast<float>(velocity_report_it->longitudinal_velocity)};
    float w{static_cast<float>(velocity_report_it->heading_rate)};

    if (std::abs(*it_time_stamp - rclcpp::Time(velocity_report_it->header.stamp).seconds()) > 0.1) {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 10000 /* ms */,
        "velocity_report time_stamp is too late. Cloud not interpolate.");
      v = 0.0f;
      w = 0.0f;
    }

    const float time_offset = static_cast<float>(*it_time_stamp - prev_time_stamp_sec);

    const tf2::Vector3 sensorTF_point{*it_x, *it_y, *it_z};

    const tf2::Vector3 base_linkTF_point{tf2_base_link_to_sensor_inv * sensorTF_point};

    theta += w * time_offset;
    tf2::Quaternion baselink_quat{};
    baselink_quat.setRPY(0.0, 0.0, theta);
    const float dis = v * time_offset;
    x += dis * std::cos(theta);
    y += dis * std::sin(theta);

    tf2::Transform baselinkTF_odom{};
    baselinkTF_odom.setOrigin(tf2::Vector3(x, y, 0.0));
    baselinkTF_odom.setRotation(baselink_quat);

    const tf2::Vector3 base_linkTF_trans_point{baselinkTF_odom * base_linkTF_point};

    const tf2::Vector3 sensorTF_trans_point{tf2_base_link_to_sensor * base_linkTF_trans_point};

    *it_x = sensorTF_trans_point.getX();
    *it_y = sensorTF_trans_point.getY();
    *it_z = sensorTF_trans_point.getZ();

    prev_time_stamp_sec = *it_time_stamp;
  }
  return true;
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::DistortionCorrectorComponent)
