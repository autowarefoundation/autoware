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

#include "read_data.h"
#include "vision_beyond_track.h"
#include "detection.h"
#include "visualizer.h"

#include <iostream>
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>

static int seqNo = 2;
static int num_frame = 25;
static string image_dir = "../../Data/images/image_02/test/%04d/";
static string calib_path = "../../Data/calib/calib_all_test.txt";
static string det_dir = "../../Data/RRC_Detections_txt/test/%02d/";
static string pose_path = "../../Data/ORBSLAM_pose/test/%04d/KITTITrajectoryComplete_new";


std::vector<beyondtrack::Detection> convert_detection(std::vector<std::vector<double>> raw_dets) {
  std::vector<beyondtrack::Detection> detections;
  for (auto&& e: raw_dets) {
    beyondtrack::Detection det(e);
    detections.push_back(det);
  }
  return detections;
}

int main(int argc, const char** argv) {
  for (int i=0; i<argc; i++) {
    std::cout << argv[i] << '\t';
  }
  std::cout << '\n';

  if (argc == 1) {
    std::cout << "Default parameter are used for inference\n";
  } else if (argc == 2) {
    seqNo = atoi(argv[1]);
  }

  image_dir = (boost::format(image_dir) % seqNo).str();
  det_dir = (boost::format(det_dir) % seqNo).str();
  pose_path = (boost::format(pose_path) % seqNo).str();

  std::cout << "Image dir: " << image_dir << '\n';
  std::cout << "Detection dir: " << det_dir << '\n';
  std::cout << "Pose dir: " << pose_path << '\n';
  std::cout << "Calib file: " << calib_path << '\n';

  cv::Mat k_ = beyondtrack::read_calib(calib_path, seqNo);
  std::vector<std::vector<std::vector<double>>> detection_list = beyondtrack::read_detection(det_dir);
  std::vector<cv::Mat> pose_list = beyondtrack::read_pose(pose_path);
  std::vector<string> img_list = beyondtrack::read_img(image_dir);

  cv::Mat n = cv::Mat::zeros(1, 3, CV_64FC1);
  n.at<double>(0, 1) = 1;
  double h = 1.72;

  std::cout << "K_:\n" << k_ << '\n';
  std::cout << "n:\n" << n << '\n';
  std::cout << "h: " << h << '\n';
  std::cout << '\n';

  beyondtrack::BeyondTracker tracker = beyondtrack::BeyondTracker(k_);

  for (int sn=0; sn<num_frame; ++sn) {
    std::cout << "----------------------------\n";
    std::cout << boost::format("Seq<%02d> | frame: %04d\n") % seqNo % sn;
    std::vector<beyondtrack::Detection> detections = convert_detection(detection_list[sn]);
    auto start = std::chrono::high_resolution_clock::now(); //std::chrono::system_clock::now();

    tracker.process(detections, pose_list[sn], n, h);

    auto end = std::chrono::high_resolution_clock::now();//std::chrono::system_clock::now();
    auto dur = end - start;
    auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    std::cout << "Tracking: " << msec << " milli sec \n";

    cv::Mat src_img = cv::imread(img_list[sn], 1);
    beyondtrack::visualize_results(src_img, tracker.get_results());
  }
  return 0;
}
