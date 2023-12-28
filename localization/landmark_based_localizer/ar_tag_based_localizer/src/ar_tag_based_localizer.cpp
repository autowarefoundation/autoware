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

#include "ar_tag_based_localizer.hpp"

#include "localization_util/util_func.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/quaternion.hpp>

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Transform.h>

#include <algorithm>
#include <limits>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <tier4_autoware_utils/geometry/geometry.hpp>

ArTagBasedLocalizer::ArTagBasedLocalizer(const rclcpp::NodeOptions & options)
: Node("ar_tag_based_localizer", options), cam_info_received_(false)
{
  /*
    Declare node parameters
  */
  marker_size_ = static_cast<float>(this->declare_parameter<double>("marker_size"));
  target_tag_ids_ = this->declare_parameter<std::vector<std::string>>("target_tag_ids");
  base_covariance_ = this->declare_parameter<std::vector<double>>("base_covariance");
  distance_threshold_ = this->declare_parameter<double>("distance_threshold");
  consider_orientation_ = this->declare_parameter<bool>("consider_orientation");
  ekf_time_tolerance_ = this->declare_parameter<double>("ekf_time_tolerance");
  ekf_position_tolerance_ = this->declare_parameter<double>("ekf_position_tolerance");
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
    return;
  }
  ekf_pose_buffer_ = std::make_unique<SmartPoseBuffer>(
    this->get_logger(), ekf_time_tolerance_, ekf_position_tolerance_);

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

  /*
    Subscribers
  */
  using std::placeholders::_1;
  map_bin_sub_ = this->create_subscription<HADMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&ArTagBasedLocalizer::map_bin_callback, this, _1));

  rclcpp::QoS qos_sub(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_sub.best_effort();
  image_sub_ = this->create_subscription<Image>(
    "~/input/image", qos_sub, std::bind(&ArTagBasedLocalizer::image_callback, this, _1));
  cam_info_sub_ = this->create_subscription<CameraInfo>(
    "~/input/camera_info", qos_sub, std::bind(&ArTagBasedLocalizer::cam_info_callback, this, _1));
  ekf_pose_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
    "~/input/ekf_pose", qos_sub, std::bind(&ArTagBasedLocalizer::ekf_pose_callback, this, _1));

  /*
    Publishers
  */
  const rclcpp::QoS qos_pub_once = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
  const rclcpp::QoS qos_pub_periodic(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/output/pose_with_covariance", qos_pub_periodic);
  image_pub_ = this->create_publisher<Image>("~/debug/image", qos_pub_periodic);
  mapped_tag_pose_pub_ = this->create_publisher<MarkerArray>("~/debug/mapped_tag", qos_pub_once);
  detected_tag_pose_pub_ =
    this->create_publisher<PoseArray>("~/debug/detected_tag", qos_pub_periodic);
  diag_pub_ = this->create_publisher<DiagnosticArray>("/diagnostics", qos_pub_periodic);

  RCLCPP_INFO(this->get_logger(), "Setup of ar_tag_based_localizer node is successful!");
}

void ArTagBasedLocalizer::map_bin_callback(const HADMapBin::ConstSharedPtr & msg)
{
  landmark_manager_.parse_landmarks(msg, "apriltag_16h5");
  const MarkerArray marker_msg = landmark_manager_.get_landmarks_as_marker_array_msg();
  mapped_tag_pose_pub_->publish(marker_msg);
}

void ArTagBasedLocalizer::image_callback(const Image::ConstSharedPtr & msg)
{
  // check subscribers
  if ((image_pub_->get_subscription_count() == 0) && (pose_pub_->get_subscription_count() == 0)) {
    RCLCPP_DEBUG(this->get_logger(), "No subscribers, not looking for ArUco markers");
    return;
  }

  // check cam_info
  if (!cam_info_received_) {
    RCLCPP_DEBUG(this->get_logger(), "No cam_info has been received.");
    return;
  }

  const builtin_interfaces::msg::Time sensor_stamp = msg->header.stamp;

  // get self pose
  const std::optional<SmartPoseBuffer::InterpolateResult> interpolate_result =
    ekf_pose_buffer_->interpolate(sensor_stamp);
  if (!interpolate_result) {
    return;
  }
  ekf_pose_buffer_->pop_old(sensor_stamp);
  const Pose self_pose = interpolate_result.value().interpolated_pose.pose.pose;

  // detect
  const std::vector<Landmark> landmarks = detect_landmarks(msg);
  if (landmarks.empty()) {
    return;
  }

  // for debug
  if (detected_tag_pose_pub_->get_subscription_count() > 0) {
    PoseArray pose_array_msg;
    pose_array_msg.header.stamp = sensor_stamp;
    pose_array_msg.header.frame_id = "map";
    for (const Landmark & landmark : landmarks) {
      const Pose detected_marker_on_map =
        tier4_autoware_utils::transformPose(landmark.pose, self_pose);
      pose_array_msg.poses.push_back(detected_marker_on_map);
    }
    detected_tag_pose_pub_->publish(pose_array_msg);
  }

  // calc new_self_pose
  const Pose new_self_pose =
    landmark_manager_.calculate_new_self_pose(landmarks, self_pose, consider_orientation_);
  const Pose diff_pose = tier4_autoware_utils::inverseTransformPose(new_self_pose, self_pose);
  const double distance =
    std::hypot(diff_pose.position.x, diff_pose.position.y, diff_pose.position.z);

  // check distance
  if (distance > ekf_position_tolerance_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "distance: " << distance);
    return;
  }

  // publish
  PoseWithCovarianceStamped pose_with_covariance_stamped;
  pose_with_covariance_stamped.header.stamp = sensor_stamp;
  pose_with_covariance_stamped.header.frame_id = "map";
  pose_with_covariance_stamped.pose.pose = new_self_pose;

  // ~5[m]: base_covariance
  // 5~[m]: scaling base_covariance by std::pow(distance / 5, 3)
  const double coeff = std::max(1.0, std::pow(distance / 5, 3));
  for (int i = 0; i < 36; i++) {
    pose_with_covariance_stamped.pose.covariance[i] = coeff * base_covariance_[i];
  }

  pose_pub_->publish(pose_with_covariance_stamped);

  // publish diagnostics
  const int detected_tags = static_cast<int>(landmarks.size());

  diagnostic_msgs::msg::DiagnosticStatus diag_status;

  if (detected_tags > 0) {
    diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_status.message = "AR tags detected. The number of tags: " + std::to_string(detected_tags);
  } else {
    diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_status.message = "No AR tags detected.";
  }

  diag_status.name = "localization: " + std::string(this->get_name());
  diag_status.hardware_id = this->get_name();

  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "Number of Detected AR Tags";
  key_value.value = std::to_string(detected_tags);
  diag_status.values.push_back(key_value);

  DiagnosticArray diag_msg;
  diag_msg.header.stamp = this->now();
  diag_msg.status.push_back(diag_status);

  diag_pub_->publish(diag_msg);
}

// wait for one camera info, then shut down that subscriber
void ArTagBasedLocalizer::cam_info_callback(const CameraInfo::ConstSharedPtr & msg)
{
  if (cam_info_received_) {
    return;
  }

  // copy camera matrix
  cv::Mat camera_matrix(3, 4, CV_64FC1);
  std::copy(msg->p.begin(), msg->p.end(), camera_matrix.begin<double>());

  // all 0
  cv::Mat distortion_coeff(4, 1, CV_64FC1, 0.0);

  const cv::Size size(static_cast<int>(msg->width), static_cast<int>(msg->height));

  cam_param_ = aruco::CameraParameters(camera_matrix, distortion_coeff, size);

  cam_info_received_ = true;
}

void ArTagBasedLocalizer::ekf_pose_callback(const PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  if (msg->header.frame_id == "map") {
    ekf_pose_buffer_->push_back(msg);
  } else {
    RCLCPP_ERROR_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "Received initial pose message with frame_id "
        << msg->header.frame_id << ", but expected map. "
        << "Please check the frame_id in the input topic and ensure it is correct.");
  }
}

std::vector<landmark_manager::Landmark> ArTagBasedLocalizer::detect_landmarks(
  const Image::ConstSharedPtr & msg)
{
  const builtin_interfaces::msg::Time sensor_stamp = msg->header.stamp;

  // get image
  cv::Mat in_image;
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
    cv_ptr->image.copyTo(in_image);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return std::vector<Landmark>{};
  }

  // get transform from base_link to camera
  TransformStamped transform_sensor_to_base_link;
  try {
    transform_sensor_to_base_link =
      tf_buffer_->lookupTransform("base_link", msg->header.frame_id, sensor_stamp);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform base_link to camera: %s", ex.what());
    return std::vector<Landmark>{};
  }

  // detect
  std::vector<aruco::Marker> markers;
  detector_.detect(in_image, markers, cam_param_, marker_size_, false);

  // parse
  std::vector<Landmark> landmarks;

  for (aruco::Marker & marker : markers) {
    // convert marker pose to tf
    const cv::Quat<float> q = cv::Quat<float>::createFromRvec(marker.Rvec);
    Pose pose;
    pose.position.x = marker.Tvec.at<float>(0, 0);
    pose.position.y = marker.Tvec.at<float>(1, 0);
    pose.position.z = marker.Tvec.at<float>(2, 0);
    pose.orientation.x = q.x;
    pose.orientation.y = q.y;
    pose.orientation.z = q.z;
    pose.orientation.w = q.w;
    const double distance = std::hypot(pose.position.x, pose.position.y, pose.position.z);
    if (distance <= distance_threshold_) {
      tf2::doTransform(pose, pose, transform_sensor_to_base_link);
      landmarks.push_back(Landmark{std::to_string(marker.id), pose});
    }

    // for debug, drawing the detected markers
    marker.draw(in_image, cv::Scalar(0, 0, 255), 2);
    aruco::CvDrawingUtils::draw3dAxis(in_image, marker, cam_param_);
  }

  // for debug
  if (image_pub_->get_subscription_count() > 0) {
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = sensor_stamp;
    out_msg.encoding = sensor_msgs::image_encodings::RGB8;
    out_msg.image = in_image;
    image_pub_->publish(*out_msg.toImageMsg());
  }

  return landmarks;
}
