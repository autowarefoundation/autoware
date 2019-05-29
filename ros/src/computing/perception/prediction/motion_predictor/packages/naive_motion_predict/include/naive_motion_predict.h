/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

  // max prediction score
  const double MAX_PREDICTION_SCORE_;

  // ros publisher
  ros::Publisher predicted_objects_pub_;
  ros::Publisher predicted_paths_pub_;

  // ros Subscriber
  ros::Subscriber detected_objects_sub_;

  // prediction param
  double interval_sec_;
  int num_prediction_;
  double sensor_height_;
  double filter_out_close_object_threshold_;

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
  
  geometry_msgs::PolygonStamped getPredictedConvexHull(const geometry_msgs::PolygonStamped& in_polygon, 
                                                       const double delta_x,
                                                       const double delta_y);

public:
  NaiveMotionPredict();
  ~NaiveMotionPredict();
};

#endif  // NAIVE_MOTION_PREDICT_H
