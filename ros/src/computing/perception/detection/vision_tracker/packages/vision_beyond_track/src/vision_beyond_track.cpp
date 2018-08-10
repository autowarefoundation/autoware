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

#include "vision_beyond_track.h"
#include "visualizer.h"
#include <chrono>

using namespace cv;

const static double MAX_VALUE = 10000;
const static double THRES_SCORE = 1000;

static double avg_car_sz[3] = {4.3, 2, 2};
static double sz_ub[3] = {34, 31.5, 31.5};
static double sz_lb[3] = {-34, -31.5, -31.5};
beyondtrack::ObjectCuboid beyondtrack::Detection::params_carCuboid = beyondtrack::ObjectCuboid(avg_car_sz, sz_ub, sz_lb);

namespace beyondtrack {
  void BeyondTracker::initialize(cv::Mat n, double h) {
    for (auto&& cd: cur_det) {
      cd.dno = global_id;
      global_id++;
      cd.propagate_cur_det(canonicalCuboid, h, k, inv_k, n);
    }
    prev_det = cur_det;
    prev_pose = cur_pose;
    initialized = true;
  }

  cv::Mat BeyondTracker::create_cuboid() {
    double l = Detection::params_carCuboid.avg_l / 2;
    double h = Detection::params_carCuboid.avg_h / 2;
    double w = Detection::params_carCuboid.avg_w / 2;
    cv::Mat cuboid = (cv::Mat_<double>(8, 3) <<
                      -l, h, -w, l, h, -w, -l, h, w, l, h, w,
                      -l, -h, -w, l, -h, -w, -l, -h, w, l, -h, w);
    return cuboid;
  }

  void BeyondTracker::propagate_detections(cv::Mat n, double h) {
    for (auto&& cd: cur_det) {
      cd.propagate_cur_det(canonicalCuboid, h, k, inv_k, n);
    }
    for (auto&& pd: prev_det) {
      pd.propagate_prev_det(k, motion);
    }
  }

  vector<vector<double>> BeyondTracker::generateScoreMatrices() {
    vector<vector<double>> array;
    for (auto&& pd: prev_det) {
      vector<double> tmp_array;
      for (auto&& cd: cur_det) {
        double score_2d = get_3d2d_score(cd, pd);
        double score_3d = get_3d3d_score(cd, pd);
        double score = all_wts[0] * score_3d + all_wts[1] * score_2d;
        if (score > MAX_VALUE) {
          score = MAX_VALUE;
        }
        tmp_array.push_back(score);
        // std::cout << "Score 2d: " << score_2d << '\n';
        // std::cout << "Score 3d: " << score_3d << '\n';
        // std::cout << "Score: " << score << '\n';
        // std::cout << score << '\t';
      }
      // std::cout << '\n';
      array.push_back(tmp_array);
    }
    return array;
  }

  BeyondTracker::BeyondTracker(cv::Mat k_) {
    k = k_;
    inv_k = k.inv();
  }

  void BeyondTracker::process(vector<Detection> detection, cv::Mat pose, cv::Mat n, double h) {
    cur_det = detection;
    cur_pose = pose;

    if (detection.size() == 0) {
      initialized = false;
      return;
    }

    if (!initialized) {
      initialize(n, h);
      return;
    }

    motion = -(cur_pose - prev_pose);
    motion.at<double>(0, 3) = deg2rad(0); // TODO: subscribe from lidar localizer or camera localizer
    // motion *= (1.72 / 44);

    // auto start = std::chrono::system_clock::now();
    propagate_detections(n, h);
    // auto end = std::chrono::system_clock::now();
    // auto dur = end - start;
    // auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    // std::cout << "propagate_detections: " << msec << " milli sec \n";

    // Cost estimation
    // start = std::chrono::system_clock::now();
    vector<vector<double>> cost_matrix = generateScoreMatrices();
    // end = std::chrono::system_clock::now();
    // dur = end - start;
    // msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    // std::cout << "generateScoreMatrices: " << msec << " milli sec \n";

    // start = std::chrono::system_clock::now();
    HungarianAlgorithm HungAlgo;
    vector<int> assignment;

    //double cost =
    HungAlgo.Solve(cost_matrix, assignment);

    // end = std::chrono::system_clock::now();
    // dur = end - start;
    // msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    // std::cout << "HungarianAlgorithm: " << msec << " milli sec \n";

    for (unsigned int x = 0; x < cost_matrix.size(); x++) {
      if ((assignment[x] != -1) && (cost_matrix[x][assignment[x]] < THRES_SCORE)) {
        // std::cout << x << "," << assignment[x] << "\t";
        cur_det[assignment[x]].dno = prev_det[x].dno;
      }
    }

    for (auto&& cd: cur_det) if (cd.dno == -1) cd.dno = global_id++;

    prev_det = cur_det;
    prev_pose = cur_pose;
  }

  vector<Detection> BeyondTracker::get_results() {
    return cur_det;
  }

  void BeyondTracker::set_intrinsic(cv::Mat k_) {
    k = k_;
    inv_k = k.inv();
  }

  double BeyondTracker::get_3d2d_score(Detection cd, Detection pd) {
    Paths solution;
    Clipper c;
    c.AddPaths(pd.pd_2d_convhull_clip_, ptSubject, true);
    c.AddPaths(cd.cd_2d_convhull_clip_, ptClip, true);
    c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
    if (solution.size() > 0) {
      c.Execute(ctUnion, solution, pftNonZero, pftNonZero);
      auto area_in = Area(solution[0]);
      double area_target = cd.cd_area_;
      return area_in / area_target;
    } else {
      return MAX_VALUE;
    }
  }

  double BeyondTracker::get_3d3d_score(Detection cd, Detection pd) {
    Paths solution;
    Clipper c;
    c.AddPaths(cd.cd_3d_convhull_clip_, ptSubject, true);
    c.AddPaths(pd.pd_3d_convhull_clip_, ptClip, true);
    c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
    if (solution.size() > 0) {
      c.Execute(ctUnion, solution, pftNonZero, pftNonZero);
      auto area_in = Area(solution[0]);
      double area_target = cv::contourArea(cd.cd_3d_convhull_);
      return area_in / area_target;
    } else {
      return MAX_VALUE;
    }
  }
}

void BeyondTrackerNode::parse_detected_object(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections)
{
  // TODO
  detections_.clear();
  for (const auto& e: in_vision_detections->objects) {
    if (e.label == "person" || e.label == "car" || e.label == "truck") {
      beyondtrack::Detection detection(e.x, e.y, e.width, e.height);
      detections_.push_back(detection);
      // std::cout << "##########################\n";
      // std::cout << "Class: " << e.id << '\t' << e.label << '\t' << e.score << '\n';
      // std::cout << "Input bbox: " << detection.bbox[0] << '\t' << detection.bbox[1] << '\t' << detection.bbox[2] << '\t' << detection.bbox[3] << '\n';
    }
  }
}

void BeyondTrackerNode::vision_detection_callback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections)
{
  ROS_INFO("[%s] DetectedObjectArray obtained. Size: %d", __APP_NAME__, in_vision_detections->objects.size());

  if (camera_info_ok_) {
    if (!use_motion_) {
      pose_ = cv::Mat::zeros(1, 4, CV_64FC1);
    } else {
      // TODO
    }

    parse_detected_object(in_vision_detections);

    tracker_.process(detections_, pose_, ground_angle_, camera_height_);
  }

  if (image_ok_) {
    beyondtrack::visualize_results(mat_image_, tracker_.get_results());
    // cv::imshow("Image", mat_image_);
    // cv::waitKey(200);
  }
}


void BeyondTrackerNode::intrinsics_callback(const sensor_msgs::CameraInfo &in_message)
{
	image_size_.height = in_message.height;
	image_size_.width = in_message.width;

	camera_instrinsics_ = cv::Mat(3, 3, CV_64FC1);
	for (int row = 0; row < 3; ++row)
	{
		for (int col = 0; col < 3; ++col)
		{
			camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
		}
	}

  tracker_.set_intrinsic(camera_instrinsics_);

	intrinsics_subscriber_.shutdown();
	camera_info_ok_ = true;
	ROS_INFO("[%s] CameraIntrinsics obtained.", __APP_NAME__);
}


void BeyondTrackerNode::image_callback(const sensor_msgs::ImageConstPtr& in_image_message)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_message, "bgr8");
  mat_image_ = cv_image->image;

  // int image_height = msg->height;
  // int image_width = msg->width;
  image_ok_ = true;
}

void BeyondTrackerNode::config_cb(const autoware_msgs::ConfigSsd::ConstPtr& param)
{
    // score_threshold_ = param->score_threshold;
    // TODO
}


void BeyondTrackerNode::Run()
{
    //ROS STUFF
    ros::NodeHandle private_node_handle("~");//to receive args

    //RECEIVE IMAGE TOPIC NAME
    std::string image_rect_str;
    if (private_node_handle.getParam("image_rectified", image_rect_str))
    {
        ROS_INFO("Setting image node to %s", image_rect_str.c_str());
    }
    else
    {
        // ROS_INFO("No image node received, defaulting to /image_raw, you can use _image_rectified:=YOUR_TOPIC");
        image_rect_str = "/image_rectified";
    }

    std::string camera_info_str;
    if (private_node_handle.getParam("camera_info_str", camera_info_str))
    {
        ROS_INFO("Intrinsics topic: %s", camera_info_str.c_str());
    }
    else
    {
      // ROS_INFO("No image node received, defaulting to /camera_info, you can use _image_raw_node:=YOUR_TOPIC");
      camera_info_str = "/camera_info";
    }

    std::string detected_objects_vision_str;
    if (private_node_handle.getParam("detected_objects_vision_str", detected_objects_vision_str))
    {
        ROS_INFO("Intrinsics topic: %s", detected_objects_vision_str.c_str());
    }
    else
    {
      // ROS_INFO("No image node received, defaulting to /image_raw, you can use _image_raw_node:=YOUR_TOPIC");
      detected_objects_vision_str = "/detection/vision_objects";
    }

    private_node_handle.param<std::string>("image_str", image_rect_str, "/image_rectified");
    private_node_handle.param<std::string>("camera_info_str", camera_info_str, "/camera_info");
    private_node_handle.param<std::string>("detected_objects_vision", detected_objects_vision_str, "/detection/vision_objects");

    // publisher_objects_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>("/detection/vision_objects", 1);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, image_rect_str.c_str());
    rect_image_subscriber_  = private_node_handle.subscribe(image_rect_str, 1,
                                  &BeyondTrackerNode::image_callback, this);

    //generate subscribers and sychronizers
    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, camera_info_str.c_str());
    intrinsics_subscriber_ = private_node_handle.subscribe(
                                 camera_info_str, 1, &BeyondTrackerNode::intrinsics_callback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, detected_objects_vision_str.c_str());
    detections_vision_subscriber_ = private_node_handle.subscribe(detected_objects_vision_str,
                                        1, &BeyondTrackerNode::vision_detection_callback, this);

    // TODO
    private_node_handle.param<double>("camera_height", camera_height_, 1.2);
    ROS_INFO("[%s] camera height: %f",__APP_NAME__, camera_height_);


    ROS_INFO_STREAM( __APP_NAME__ << "" );

    ground_angle_ = cv::Mat::zeros(1, 3, CV_64FC1);
    ground_angle_.at<double>(0, 1) = 1;

    tracker_ = beyondtrack::BeyondTracker();

    ros::spin();

    ROS_INFO("END beyond_tracker");
}
