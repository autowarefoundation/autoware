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

#ifndef AR_TAG_BASED_LOCALIZER__AR_TAG_BASED_LOCALIZER_CORE_HPP_
#define AR_TAG_BASED_LOCALIZER__AR_TAG_BASED_LOCALIZER_CORE_HPP_

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <aruco/aruco.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

class ArTagBasedLocalizer : public rclcpp::Node
{
public:
  ArTagBasedLocalizer();
  bool setup();

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void cam_info_callback(const sensor_msgs::msg::CameraInfo & msg);
  void ekf_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void publish_pose_as_base_link(
    const geometry_msgs::msg::PoseStamped & msg, const std::string & camera_frame_id);

  // Parameters
  float marker_size_{};
  std::vector<std::string> target_tag_ids_;
  std::vector<double> base_covariance_;
  double distance_threshold_squared_{};
  double ekf_time_tolerance_{};
  double ekf_position_tolerance_{};

  // tf
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // image transport
  std::unique_ptr<image_transport::ImageTransport> it_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pose_sub_;

  // Publishers
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  // Others
  aruco::MarkerDetector detector_;
  aruco::CameraParameters cam_param_;
  bool cam_info_received_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_ekf_pose_{};
};

#endif  // AR_TAG_BASED_LOCALIZER__AR_TAG_BASED_LOCALIZER_CORE_HPP_
