/*
 *  Copyright (c) 2018, Tokyo University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************
 *  v1.0: Yuki Tsuji (yukitsuji020832@gmail.com)
 *
 *  Created on: Aug 8th, 2018
 */

#ifndef VISION_BEYOND_TRACK_H
#define VISION_BEYOND_TRACK_H

#include <iostream>
#include <chrono>

#include <tf/tf.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "detection.h"
#include "hungarian.h"

#define __APP_NAME__ "vision_beyond_track"

namespace beyondtrack
{
  class BeyondTracker
  {
  private:
    int global_id_ = 1;
    bool initialized_ = false;
    std::vector<Detection> cur_detections_;
    std::vector<Detection> prev_detections_;
    cv::Mat cur_pose_;
    cv::Mat prev_pose_;
    double all_wts_[4] = {0.6, 0.4, 0.2, 0.0};

    cv::Mat camera_k_;
    cv::Mat camera_inv_k_;
    cv::Mat motion_;
    cv::Mat canonical_cuboid_ = create_cuboid();

    void initialize(cv::Mat n, double h);

    cv::Mat create_cuboid();

    void propagate_detections(cv::Mat n, double h);

    std::vector<vector<double>> generate_score_matrices();

  public:

    BeyondTracker()
    {
    };

    BeyondTracker(cv::Mat k_);

    ~BeyondTracker()
    {
    };

    void process(std::vector<Detection> in_detections, cv::Mat in_pose, cv::Mat in_angle, double in_height);

    std::vector<Detection> get_results();

    void set_intrinsic(cv::Mat k_);

    double get_3d2d_score(Detection cd, Detection pd);

    double get_3d3d_score(Detection cd, Detection pd);

  };
}

class BeyondTrackerNode
{
  ros::Subscriber rect_image_subscriber_;
  ros::Subscriber intrinsics_subscriber_;
  ros::Subscriber detections_vision_subscriber_;
  ros::Subscriber ego_motion_subscriber_;

  ros::Publisher objects_publisher_;
  ros::NodeHandle node_handle_;

  beyondtrack::BeyondTracker tracker_;

  double image_ratio_;//resize ratio used to fit input image to network input size
  uint32_t image_top_bottom_border_;//black strips added to the input image to maintain aspect ratio while resizing it to fit the network input size
  uint32_t image_left_right_border_;
  std::vector<cv::Scalar> colors_;

  cv::Size image_size_;
  cv::Mat camera_instrinsics_;
  bool camera_info_ok_;

  bool use_motion_ = false;

  //std::vector<beyondtrack::Detection> detections_;
  cv::Mat pose_;
  cv::Mat ground_angle_;
  double camera_height_;

  std::vector<beyondtrack::Detection>
  parse_detected_object(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections);

  void vision_detection_callback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections);

  void intrinsics_callback(const sensor_msgs::CameraInfo &in_message);

  void detection_to_objects(const std::vector<beyondtrack::Detection> &in_objects,
                            autoware_msgs::DetectedObjectArray &out_message);

public:
  void Run();
};

#endif
