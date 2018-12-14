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

#ifndef NAIVE_MOTION_PREDICT_H
#define NAIVE_MOTION_PREDICT_H

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <visualization_msgs/MarkerArray.h>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

enum MotionModel : int
{
  CV = 0,    // constant velocity
  CTRV = 1,  // constant turn rate and velocity
  RM = 2,    // random motion
};

class NaiveMotionPredict
{
private:
  // nodehandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // ros publisher
  ros::Publisher predicted_objects_pub_;
  ros::Publisher predicted_paths_pub_;

  // ros Subscriber
  ros::Subscriber detected_objects_sub_;

  // prediction param
  double interval_sec_;
  int num_prediction_;
  double sensor_height_;

  void objectsCallback(const autoware_msgs::DetectedObjectArray& input);

  void initializeROSmarker(const std_msgs::Header& header, const geometry_msgs::Point& position, const int object_id,
                           visualization_msgs::Marker& predicted_line);

  void makePrediction(const autoware_msgs::DetectedObject& object,
                      std::vector<autoware_msgs::DetectedObject>& predicted_objects,
                      visualization_msgs::Marker& predicted_line);

  autoware_msgs::DetectedObject generatePredictedObject(const autoware_msgs::DetectedObject& object);

  autoware_msgs::DetectedObject moveConstantVelocity(const autoware_msgs::DetectedObject& object);

  autoware_msgs::DetectedObject moveConstantTurnRateVelocity(const autoware_msgs::DetectedObject& object);

  double generateYawFromQuaternion(const geometry_msgs::Quaternion& quaternion);

  bool isObjectValid(const autoware_msgs::DetectedObject &in_object);

public:
  NaiveMotionPredict();
  ~NaiveMotionPredict();
};

#endif  // NAIVE_MOTION_PREDICT_H
