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

#include "ar_tag_based_localizer/utils.hpp"

#include <opencv4/opencv2/calib3d.hpp>
#include <rclcpp/logging.hpp>

aruco::CameraParameters ros_camera_info_to_aruco_cam_params(
  const sensor_msgs::msg::CameraInfo & cam_info, bool use_rectified_parameters)
{
  cv::Mat camera_matrix(3, 4, CV_64FC1, 0.0);
  cv::Mat distortion_coeff(4, 1, CV_64FC1);
  cv::Size size(static_cast<int>(cam_info.width), static_cast<int>(cam_info.height));

  if (use_rectified_parameters) {
    camera_matrix.setTo(0);
    camera_matrix.at<double>(0, 0) = cam_info.p[0];
    camera_matrix.at<double>(0, 1) = cam_info.p[1];
    camera_matrix.at<double>(0, 2) = cam_info.p[2];
    camera_matrix.at<double>(0, 3) = cam_info.p[3];
    camera_matrix.at<double>(1, 0) = cam_info.p[4];
    camera_matrix.at<double>(1, 1) = cam_info.p[5];
    camera_matrix.at<double>(1, 2) = cam_info.p[6];
    camera_matrix.at<double>(1, 3) = cam_info.p[7];
    camera_matrix.at<double>(2, 0) = cam_info.p[8];
    camera_matrix.at<double>(2, 1) = cam_info.p[9];
    camera_matrix.at<double>(2, 2) = cam_info.p[10];
    camera_matrix.at<double>(2, 3) = cam_info.p[11];

    for (int i = 0; i < 4; ++i) {
      distortion_coeff.at<double>(i, 0) = 0;
    }
  } else {
    cv::Mat camera_matrix_from_k(3, 3, CV_64FC1, 0.0);
    for (int i = 0; i < 9; ++i) {
      camera_matrix_from_k.at<double>(i % 3, i - (i % 3) * 3) = cam_info.k[i];
    }
    camera_matrix_from_k.copyTo(camera_matrix(cv::Rect(0, 0, 3, 3)));

    if (cam_info.d.size() == 4) {
      for (int i = 0; i < 4; ++i) {
        distortion_coeff.at<double>(i, 0) = cam_info.d[i];
      }
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("ar_tag_based_localizer"),
        "length of camera_info D vector is not 4, assuming zero distortion...");
      for (int i = 0; i < 4; ++i) {
        distortion_coeff.at<double>(i, 0) = 0;
      }
    }
  }

  return {camera_matrix, distortion_coeff, size};
}

tf2::Transform aruco_marker_to_tf2(const aruco::Marker & marker)
{
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Mat r_vec64;
  marker.Rvec.convertTo(r_vec64, CV_64FC1);
  cv::Rodrigues(r_vec64, rot);
  cv::Mat tran64;
  marker.Tvec.convertTo(tran64, CV_64FC1);

  tf2::Matrix3x3 tf_rot(
    rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2), rot.at<double>(1, 0),
    rot.at<double>(1, 1), rot.at<double>(1, 2), rot.at<double>(2, 0), rot.at<double>(2, 1),
    rot.at<double>(2, 2));

  tf2::Vector3 tf_orig(tran64.at<double>(0, 0), tran64.at<double>(1, 0), tran64.at<double>(2, 0));

  return tf2::Transform(tf_rot, tf_orig);
}
