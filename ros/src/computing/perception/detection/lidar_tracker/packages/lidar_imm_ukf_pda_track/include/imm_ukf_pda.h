/*
 *  Copyright (c) 2018, Nagoya University
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
 */

#ifndef OBJECT_TRACKING_IMM_UKF_JPDAF_H
#define OBJECT_TRACKING_IMM_UKF_JPDAF_H

#include <ros/ros.h>

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <tf/transform_listener.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <visualization_msgs/MarkerArray.h>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include "ukf.h"

class ImmUkfPda
{
private:
  int target_id_;
  bool init_;
  double timestamp_;

  std::vector<UKF> targets_;

  // probabilistic data association params
  double gating_thres_;
  double gate_probability_;
  double detection_probability_;

  // bbox association param
  double distance_thres_;
  int life_time_thres_;

  // bbox update params
  double bb_yaw_change_thres_;
  double static_velocity_thres_;
  double init_yaw_;

  // Tracking state paramas
  int stable_num_;
  int lost_num_;

  // switch sukf and ImmUkfPda
  bool use_sukf_;

  // whether if publish debug ros markers
  bool is_debug_;

  // prevent explode param for ukf
  double prevent_explosion_thres_;

  std::string input_topic_;
  std::string output_topic_;

  std::string pointcloud_frame_;
  std::string tracking_frame_;

  tf::TransformListener tf_listener_;
  tf::StampedTransform local2global_;

  ros::NodeHandle node_handle_;
  ros::Subscriber sub_detected_array_;
  ros::Publisher pub_object_array_;
  ros::Publisher pub_jskbbox_array_;
  ros::Publisher pub_adas_direction_array_;
  ros::Publisher pub_adas_prediction_array_;
  ros::Publisher pub_points_array_;
  ros::Publisher pub_texts_array_;

  void callback(const autoware_msgs::DetectedObjectArray& input);
  void setPredictionObject();
  void relayJskbbox(const autoware_msgs::DetectedObjectArray& input,
                    jsk_recognition_msgs::BoundingBoxArray& jskbboxes_output);
  void transformPoseToGlobal(const autoware_msgs::DetectedObjectArray& input,
                             autoware_msgs::DetectedObjectArray& transformed_input);
  void transformPoseToLocal(jsk_recognition_msgs::BoundingBoxArray& jskbboxes_output,
                            autoware_msgs::DetectedObjectArray& detected_objects_output);
  void measurementValidation(const autoware_msgs::DetectedObjectArray& input, UKF& target, const bool second_init,
                             const Eigen::VectorXd& max_det_z, const Eigen::MatrixXd& max_det_s,
                             std::vector<autoware_msgs::DetectedObject>& object_vec, std::vector<bool>& matching_vec);
  void getNearestEuclidCluster(const UKF& target, const std::vector<autoware_msgs::DetectedObject>& object_vec,
                               autoware_msgs::DetectedObject& object, double& min_dist);
  void getRightAngleBBox(const std::vector<double> nearest_bbox, std::vector<double>& rightAngle_bbox);
  void associateBB(const std::vector<autoware_msgs::DetectedObject>& object_vec, UKF& target);
  double getBBoxYaw(const UKF target);
  double getJskBBoxArea(const jsk_recognition_msgs::BoundingBox& jsk_bb);
  double getJskBBoxYaw(const jsk_recognition_msgs::BoundingBox& jsk_bb);
  void updateBB(UKF& target);
  void mergeOverSegmentation(const std::vector<UKF> targets);

  void updateJskLabel(const UKF& target, jsk_recognition_msgs::BoundingBox& bb);

  void initTracker(const autoware_msgs::DetectedObjectArray& input, double timestamp);
  void secondInit(UKF& target, const std::vector<autoware_msgs::DetectedObject>& object_vec, double dt);

  void updateTrackingNum(const std::vector<autoware_msgs::DetectedObject>& object_vec, UKF& target);

  void probabilisticDataAssociation(const autoware_msgs::DetectedObjectArray& input, const double dt,
                                    std::vector<bool>& matching_vec,
                                    std::vector<autoware_msgs::DetectedObject>& lambda_vec, UKF& target,
                                    bool& is_skip_target);
  void makeNewTargets(const double timestamp, const autoware_msgs::DetectedObjectArray& input,
                      const std::vector<bool>& matching_vec);

  void staticClassification();

  void makeOutput(const autoware_msgs::DetectedObjectArray& input,
                  jsk_recognition_msgs::BoundingBoxArray& jskbboxes_output,
                  autoware_msgs::DetectedObjectArray& detected_objects_output);

  void removeUnnecessaryTarget();

  void pubDebugRosMarker(const autoware_msgs::DetectedObjectArray& input);

  void tracker(const autoware_msgs::DetectedObjectArray& transformed_input,
               jsk_recognition_msgs::BoundingBoxArray& jskbboxes_output,
               autoware_msgs::DetectedObjectArray& detected_objects_output);

public:
  ImmUkfPda();
  void run();
};

#endif /* OBJECT_TRACKING_IMM_UKF_JPDAF_H */
