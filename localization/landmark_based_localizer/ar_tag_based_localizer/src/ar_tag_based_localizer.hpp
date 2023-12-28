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

#ifndef AR_TAG_BASED_LOCALIZER_HPP_
#define AR_TAG_BASED_LOCALIZER_HPP_

#include "landmark_manager/landmark_manager.hpp"
#include "localization_util/smart_pose_buffer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <aruco/aruco.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

class ArTagBasedLocalizer : public rclcpp::Node
{
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using PoseArray = geometry_msgs::msg::PoseArray;
  using TransformStamped = geometry_msgs::msg::TransformStamped;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
  using Landmark = landmark_manager::Landmark;

public:
  explicit ArTagBasedLocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void map_bin_callback(const HADMapBin::ConstSharedPtr & msg);
  void image_callback(const Image::ConstSharedPtr & msg);
  void cam_info_callback(const CameraInfo::ConstSharedPtr & msg);
  void ekf_pose_callback(const PoseWithCovarianceStamped::ConstSharedPtr & msg);
  std::vector<Landmark> detect_landmarks(const Image::ConstSharedPtr & msg);

  // Parameters
  float marker_size_{};
  std::vector<std::string> target_tag_ids_;
  std::vector<double> base_covariance_;
  double distance_threshold_{};
  bool consider_orientation_{};
  double ekf_time_tolerance_{};
  double ekf_position_tolerance_{};

  // tf
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscribers
  rclcpp::Subscription<HADMapBin>::SharedPtr map_bin_sub_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr ekf_pose_sub_;

  // Publishers
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<Image>::SharedPtr image_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr detected_tag_pose_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr mapped_tag_pose_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;

  // Others
  aruco::MarkerDetector detector_;
  aruco::CameraParameters cam_param_;
  bool cam_info_received_;
  std::unique_ptr<SmartPoseBuffer> ekf_pose_buffer_;
  landmark_manager::LandmarkManager landmark_manager_;
};

#endif  // AR_TAG_BASED_LOCALIZER_HPP_
