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

#if (CV_MAJOR_VERSION <= 2)
#include <opencv2/contrib/contrib.hpp>
#else

#include "gencolors.cpp"

#endif

const static double MAX_VALUE = 10000;
const static double THRES_SCORE = 1000;

static double avg_car_sz[3] = {4.3, 2, 2};
static double sz_ub[3] = {34, 31.5, 31.5};
static double sz_lb[3] = {-34, -31.5, -31.5};

beyondtrack::ObjectCuboid beyondtrack::Detection::params_car_cuboid_ = beyondtrack::ObjectCuboid(avg_car_sz, sz_ub,
                                                                                                 sz_lb);

namespace beyondtrack
{
  void BeyondTracker::initialize(cv::Mat in_angle, double in_height)
  {
    for (auto &&cd: cur_detections_)
    {
      cd.object_id_ = global_id_;
      global_id_++;
      cd.propagate_cur_det(canonical_cuboid_, in_height, camera_k_, camera_inv_k_, in_angle);
    }
    prev_detections_ = cur_detections_;
    prev_pose_ = cur_pose_;
    initialized_ = true;
  }

  cv::Mat BeyondTracker::create_cuboid()
  {
    double l = Detection::params_car_cuboid_.avg_l / 2;
    double h = Detection::params_car_cuboid_.avg_h / 2;
    double w = Detection::params_car_cuboid_.avg_w / 2;
    cv::Mat cuboid = (cv::Mat_<double>(8, 3) <<
                                             -l, h, -w, l, h, -w, -l, h, w, l, h, w,
      -l, -h, -w, l, -h, -w, -l, -h, w, l, -h, w);
    return cuboid;
  }

  void BeyondTracker::propagate_detections(cv::Mat n, double h)
  {
    for (auto &&cd: cur_detections_)
    {
      cd.propagate_cur_det(canonical_cuboid_, h, camera_k_, camera_inv_k_, n);
    }
    for (auto &&pd: prev_detections_)
    {
      pd.propagate_prev_det(camera_k_, motion_);
    }
  }

  std::vector<std::vector<double> > BeyondTracker::generate_score_matrices()
  {
    std::vector<std::vector<double> > array;
    for (auto &&pd: prev_detections_)
    {
      std::vector<double> tmp_array;
      for (auto &&cd: cur_detections_)
      {
        double score_2d = get_3d2d_score(cd, pd);
        double score_3d = get_3d3d_score(cd, pd);
        double score = all_wts_[0] * score_3d + all_wts_[1] * score_2d;
        if (score > MAX_VALUE)
        {
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

  BeyondTracker::BeyondTracker(cv::Mat k_)
  {
    camera_k_ = k_;
    camera_inv_k_ = camera_k_.inv();
  }

  void BeyondTracker::process(std::vector<Detection> in_detections, cv::Mat in_pose, cv::Mat in_angle, double in_height)
  {
    cur_detections_ = in_detections;
    cur_pose_ = in_pose;

    if (in_detections.empty())
    {
      initialized_ = false;
      return;
    }

    if (!initialized_)
    {
      initialize(in_angle, in_height);
      return;
    }

    motion_ = -(cur_pose_ - prev_pose_);
    motion_.at<double>(0, 3) = deg2rad(0); // TODO: subscribe from lidar localizer or camera localizer
    // motion *= (1.72 / 44);

    // auto start = std::chrono::system_clock::now();
    propagate_detections(in_angle, in_height);
    // auto end = std::chrono::system_clock::now();
    // auto dur = end - start;
    // auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    // std::cout << "propagate_detections: " << msec << " milli sec \in_angle";

    // Cost estimation
    // start = std::chrono::system_clock::now();
    std::vector<std::vector<double> > cost_matrix = generate_score_matrices();
    // end = std::chrono::system_clock::now();
    // dur = end - start;
    // msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    // std::cout << "generate_score_matrices: " << msec << " milli sec \in_angle";

    // start = std::chrono::system_clock::now();
    HungarianAlgorithm HungAlgo;
    std::vector<int> assignment;

    //double cost =
    HungAlgo.Solve(cost_matrix, assignment);

    // end = std::chrono::system_clock::now();
    // dur = end - start;
    // msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    // std::cout << "HungarianAlgorithm: " << msec << " milli sec \in_angle";

    for (unsigned int x = 0; x < cost_matrix.size(); x++)
    {
      if ((assignment[x] != -1) && (cost_matrix[x][assignment[x]] < THRES_SCORE))
      {
        // std::cout << x << "," << assignment[x] << "\t";
        cur_detections_[assignment[x]].object_id_ = prev_detections_[x].object_id_;
      }
    }

    for (auto &&cd: cur_detections_)
    {
      if (cd.object_id_ == -1)
      {
        cd.object_id_ = global_id_++;
      }
    }

    prev_detections_ = cur_detections_;
    prev_pose_ = cur_pose_;
  }

  std::vector<Detection> BeyondTracker::get_results()
  {
    return cur_detections_;
  }

  void BeyondTracker::set_intrinsic(cv::Mat k_)
  {
    camera_k_ = k_;
    camera_inv_k_ = camera_k_.inv();
  }

  double BeyondTracker::get_3d2d_score(Detection cd, Detection pd)
  {
    Paths solution;
    Clipper c;
    c.AddPaths(pd.pd_2d_convhull_clip_, ptSubject, true);
    c.AddPaths(cd.cd_2d_convhull_clip_, ptClip, true);
    c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
    if (solution.size() > 0)
    {
      c.Execute(ctUnion, solution, pftNonZero, pftNonZero);
      auto area_in = Area(solution[0]);
      double area_target = cd.cd_area_;
      return area_in / area_target;
    } else
    {
      return MAX_VALUE;
    }
  }

  double BeyondTracker::get_3d3d_score(Detection cd, Detection pd)
  {
    Paths solution;
    Clipper c;
    c.AddPaths(cd.cd_3d_convhull_clip_, ptSubject, true);
    c.AddPaths(pd.pd_3d_convhull_clip_, ptClip, true);
    c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
    if (solution.size() > 0)
    {
      c.Execute(ctUnion, solution, pftNonZero, pftNonZero);
      auto area_in = Area(solution[0]);
      double area_target = cv::contourArea(cd.cd_3d_convhull_);
      return area_in / area_target;
    } else
    {
      return MAX_VALUE;
    }
  }
}

std::vector<beyondtrack::Detection>
BeyondTrackerNode::parse_detected_object(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections)
{
  std::vector<beyondtrack::Detection> detections;
  for (const auto &e: in_vision_detections->objects)
  {
    //if (e.label == "person" || e.label == "car" || e.label == "truck")
    {
      beyondtrack::Detection detection(e.x, e.y, e.width, e.height, e.label);
      detections.push_back(detection);
      // std::cout << "##########################\n";
      // std::cout << "Class: " << e.id << '\t' << e.label << '\t' << e.score << '\n';
      // std::cout << "Input bbox_: " << detection.bbox_[0] << '\t' << detection.bbox_[1] << '\t' << detection.bbox_[2] << '\t' << detection.bbox_[3] << '\n';
    }
  }
  return detections;
}

void BeyondTrackerNode::detection_to_objects(const std::vector<beyondtrack::Detection> &in_objects,
                                             autoware_msgs::DetectedObjectArray &out_message)
{
  for (unsigned int i = 0; i < in_objects.size(); ++i)
  {
    autoware_msgs::DetectedObject obj;

    obj.x = (in_objects[i].bbox_[0]);
    obj.y = (in_objects[i].bbox_[1]);
    obj.width = (in_objects[i].bbox_[2] - in_objects[i].bbox_[0]);
    obj.height = (in_objects[i].bbox_[3] - in_objects[i].bbox_[1]);
    if (obj.x < 0)
      obj.x = 0;
    if (obj.y < 0)
      obj.y = 0;
    if (obj.width < 0)
      obj.width = 0;
    if (obj.height < 0)
      obj.height = 0;

    size_t color_index = in_objects[i].object_id_ % colors_.size();
    obj.color.r = colors_[color_index].val[0];
    obj.color.g = colors_[color_index].val[1];
    obj.color.b = colors_[color_index].val[2];
    obj.color.a = 1.0f;

    obj.score = 0.0;
    obj.label = in_objects[i].class_type_;
    obj.valid = true;

    obj.id = in_objects[i].object_id_;
    obj.image_frame = out_message.header.frame_id;

    // set bounding box direction
    tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, in_objects[i].yaw_);
    tf::quaternionTFToMsg(quat, obj.pose.orientation);

    out_message.objects.push_back(obj);
  }
}

void
BeyondTrackerNode::vision_detection_callback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections)
{
  if (camera_info_ok_)
  {
    if (!use_motion_)
    {
      pose_ = cv::Mat::zeros(1, 4, CV_64FC1);
    } else
    {
      // TODO
    }

    std::vector<beyondtrack::Detection> detections = parse_detected_object(in_vision_detections);

    tracker_.process(detections, pose_, ground_angle_, camera_height_);

    autoware_msgs::DetectedObjectArray final_objects;

    final_objects.header = in_vision_detections->header;
    detection_to_objects(tracker_.get_results(), final_objects);
    objects_publisher_.publish(final_objects);

    cv::Mat bg_image(image_size_, CV_8UC3, cv::Scalar(0, 0, 0));
    //visualize_results(bg_image, tracker_.get_results());

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


void BeyondTrackerNode::Run()
{
  ros::NodeHandle private_node_handle("~");//to receive args

  std::string image_topic_src, camera_info_src, objects_topic_src;

  private_node_handle.param<std::string>("camera_info_src", camera_info_src, "/camera_info");
  private_node_handle.param<std::string>("objects_topic_src", objects_topic_src,
                                         "/detection/image_detector/objects");

  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, camera_info_src.c_str());
  intrinsics_subscriber_ = private_node_handle.subscribe(
    camera_info_src, 1, &BeyondTrackerNode::intrinsics_callback, this);

  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, objects_topic_src.c_str());
  detections_vision_subscriber_ = private_node_handle.subscribe(objects_topic_src,
                                                                1, &BeyondTrackerNode::vision_detection_callback,
                                                                this);

  private_node_handle.param<double>("camera_height", camera_height_, 1.2);
  ROS_INFO("[%s] camera height: %f", __APP_NAME__, camera_height_);

  objects_publisher_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>("/detection/image_tracker/objects",
                                                                                  1);

#if (CV_MAJOR_VERSION <= 2)
  cv::generateColors(colors_, 20);
#else
  generateColors(colors_, 20);
#endif

  ground_angle_ = cv::Mat::zeros(1, 3, CV_64FC1);
  ground_angle_.at<double>(0, 1) = 1;

  tracker_ = beyondtrack::BeyondTracker();

  ros::spin();

  ROS_INFO("END beyond_tracker");
}
