// Copyright 2023 Autoware Foundation
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

// This source code is derived from the https://github.com/pal-robotics/aruco_ros.
// Here is the license statement.
/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/

#include "ar_tag_based_localizer/ar_tag_based_localizer_core.hpp"

#include "ar_tag_based_localizer/utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cv_bridge/cv_bridge.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

ArTagBasedLocalizer::ArTagBasedLocalizer()
: Node("ar_tag_based_localizer"), cam_info_received_(false)
{
}

bool ArTagBasedLocalizer::setup()
{
  /*
    Declare node parameters
  */
  marker_size_ = static_cast<float>(this->declare_parameter<double>("marker_size"));
  target_tag_ids_ = this->declare_parameter<std::vector<std::string>>("target_tag_ids");
  base_covariance_ = this->declare_parameter<std::vector<double>>("base_covariance");
  distance_threshold_squared_ = std::pow(this->declare_parameter<double>("distance_threshold"), 2);
  std::string detection_mode = this->declare_parameter<std::string>("detection_mode");
  float min_marker_size = static_cast<float>(this->declare_parameter<double>("min_marker_size"));
  if (detection_mode == "DM_NORMAL") {
    detector_.setDetectionMode(aruco::DM_NORMAL, min_marker_size);
  } else if (detection_mode == "DM_FAST") {
    detector_.setDetectionMode(aruco::DM_FAST, min_marker_size);
  } else if (detection_mode == "DM_VIDEO_FAST") {
    detector_.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
  } else {
    // Error
    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid detection_mode: " << detection_mode);
    return false;
  }

  /*
    Log parameter info
  */
  RCLCPP_INFO_STREAM(this->get_logger(), "min_marker_size: " << min_marker_size);
  RCLCPP_INFO_STREAM(this->get_logger(), "detection_mode: " << detection_mode);
  RCLCPP_INFO_STREAM(this->get_logger(), "thresMethod: " << detector_.getParameters().thresMethod);
  RCLCPP_INFO_STREAM(this->get_logger(), "marker_size_: " << marker_size_);

  /*
    tf
  */
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  /*
    Initialize image transport
  */
  it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());

  /*
    Subscribers
  */
  rclcpp::QoS qos_sub(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_sub.best_effort();
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "~/input/image", qos_sub,
    std::bind(&ArTagBasedLocalizer::image_callback, this, std::placeholders::_1));
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/camera_info", qos_sub,
    std::bind(&ArTagBasedLocalizer::cam_info_callback, this, std::placeholders::_1));

  /*
    Publishers
  */
  rclcpp::QoS qos_pub(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  image_pub_ = it_->advertise("~/debug/result", 1);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/output/pose_with_covariance", qos_pub);

  RCLCPP_INFO(this->get_logger(), "Setup of ar_tag_based_localizer node is successful!");
  return true;
}

void ArTagBasedLocalizer::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if ((image_pub_.getNumSubscribers() == 0) && (pose_pub_->get_subscription_count() == 0)) {
    RCLCPP_DEBUG(this->get_logger(), "No subscribers, not looking for ArUco markers");
    return;
  }

  if (!cam_info_received_) {
    RCLCPP_DEBUG(this->get_logger(), "No cam_info has been received.");
    return;
  }

  builtin_interfaces::msg::Time curr_stamp = msg->header.stamp;
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat in_image = cv_ptr->image;

  // detection results will go into "markers"
  std::vector<aruco::Marker> markers;

  // ok, let's detect
  detector_.detect(in_image, markers, cam_param_, marker_size_, false);

  // for each marker, draw info and its boundaries in the image
  for (const aruco::Marker & marker : markers) {
    tf2::Transform tf_cam_to_marker = aruco_marker_to_tf2(marker);

    geometry_msgs::msg::TransformStamped tf_cam_to_marker_stamped;
    tf2::toMsg(tf_cam_to_marker, tf_cam_to_marker_stamped.transform);
    tf_cam_to_marker_stamped.header.stamp = curr_stamp;
    tf_cam_to_marker_stamped.header.frame_id = msg->header.frame_id;
    tf_cam_to_marker_stamped.child_frame_id = "detected_marker_" + std::to_string(marker.id);
    tf_broadcaster_->sendTransform(tf_cam_to_marker_stamped);

    geometry_msgs::msg::PoseStamped pose_cam_to_marker;
    tf2::toMsg(tf_cam_to_marker, pose_cam_to_marker.pose);
    pose_cam_to_marker.header.stamp = curr_stamp;
    pose_cam_to_marker.header.frame_id = std::to_string(marker.id);
    publish_pose_as_base_link(pose_cam_to_marker, msg->header.frame_id);

    // drawing the detected markers
    marker.draw(in_image, cv::Scalar(0, 0, 255), 2);
  }

  // draw a 3d cube in each marker if there is 3d info
  if (cam_param_.isValid()) {
    for (aruco::Marker & marker : markers) {
      aruco::CvDrawingUtils::draw3dAxis(in_image, marker, cam_param_);
    }
  }

  if (image_pub_.getNumSubscribers() > 0) {
    // show input with augmented information
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = curr_stamp;
    out_msg.encoding = sensor_msgs::image_encodings::RGB8;
    out_msg.image = in_image;
    image_pub_.publish(out_msg.toImageMsg());
  }
}

// wait for one camera info, then shut down that subscriber
void ArTagBasedLocalizer::cam_info_callback(const sensor_msgs::msg::CameraInfo & msg)
{
  if (cam_info_received_) {
    return;
  }

  cam_param_ = ros_camera_info_to_aruco_cam_params(msg, true);
  cam_info_received_ = true;
}

void ArTagBasedLocalizer::publish_pose_as_base_link(
  const geometry_msgs::msg::PoseStamped & msg, const std::string & camera_frame_id)
{
  // Check if frame_id is in target_tag_ids
  if (
    std::find(target_tag_ids_.begin(), target_tag_ids_.end(), msg.header.frame_id) ==
    target_tag_ids_.end()) {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "frame_id(" << msg.header.frame_id << ") is not in target_tag_ids");
    return;
  }

  // Range filter
  const double distance_squared = msg.pose.position.x * msg.pose.position.x +
                                  msg.pose.position.y * msg.pose.position.y +
                                  msg.pose.position.z * msg.pose.position.z;
  if (distance_threshold_squared_ < distance_squared) {
    return;
  }

  // Transform map to tag
  geometry_msgs::msg::TransformStamped map_to_tag_tf;
  try {
    map_to_tag_tf =
      tf_buffer_->lookupTransform("map", "tag_" + msg.header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform map to tag_%s: %s", msg.header.frame_id.c_str(),
      ex.what());
    return;
  }

  // Transform camera to base_link
  geometry_msgs::msg::TransformStamped camera_to_base_link_tf;
  try {
    camera_to_base_link_tf =
      tf_buffer_->lookupTransform(camera_frame_id, "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform base_link to camera: %s", ex.what());
    return;
  }

  // Convert ROS Transforms to Eigen matrices for easy matrix multiplication and inversion
  Eigen::Affine3d map_to_tag = tf2::transformToEigen(map_to_tag_tf.transform);
  Eigen::Affine3d camera_to_base_link = tf2::transformToEigen(camera_to_base_link_tf.transform);
  Eigen::Affine3d camera_to_tag = Eigen::Affine3d(
    Eigen::Translation3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) *
    Eigen::Quaterniond(
      msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
      msg.pose.orientation.z));
  Eigen::Affine3d tag_to_camera = camera_to_tag.inverse();

  // Final pose calculation
  Eigen::Affine3d map_to_base_link = map_to_tag * tag_to_camera * camera_to_base_link;

  // Construct output message
  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_stamped;
  pose_with_covariance_stamped.header.stamp = msg.header.stamp;
  pose_with_covariance_stamped.header.frame_id = "map";
  pose_with_covariance_stamped.pose.pose = tf2::toMsg(map_to_base_link);

  // ~5[m]: base_covariance
  // 5~[m]: scaling base_covariance by std::pow(distance/5, 3)
  const double distance = std::sqrt(distance_squared);
  const double scale = distance / 5;
  const double coeff = std::max(1.0, std::pow(scale, 3));
  for (int i = 0; i < 36; i++) {
    pose_with_covariance_stamped.pose.covariance[i] = coeff * base_covariance_[i];
  }

  pose_pub_->publish(pose_with_covariance_stamped);
}
