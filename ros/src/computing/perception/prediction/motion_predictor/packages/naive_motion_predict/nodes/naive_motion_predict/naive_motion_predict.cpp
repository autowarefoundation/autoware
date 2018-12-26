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

#include "naive_motion_predict.h"

NaiveMotionPredict::NaiveMotionPredict() : nh_(), private_nh_("~")
{
  private_nh_.param<double>("interval_sec", interval_sec_, 0.1);
  private_nh_.param<int>("num_prediction", num_prediction_, 10);
  private_nh_.param<double>("sensor_height_", sensor_height_, 2.0);

  predicted_objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/prediction/motion_predictor/objects", 1);
  predicted_paths_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/prediction/motion_predictor/path_markers", 1);
  detected_objects_sub_ = nh_.subscribe("/detection/objects", 1, &NaiveMotionPredict::objectsCallback, this);
}

NaiveMotionPredict::~NaiveMotionPredict()
{
}

void NaiveMotionPredict::initializeROSmarker(const std_msgs::Header& header, const geometry_msgs::Point& position,
                                             const int object_id, visualization_msgs::Marker& predicted_line)
{
  predicted_line.lifetime = ros::Duration(0.2);
  predicted_line.header.frame_id = header.frame_id;
  predicted_line.header.stamp = header.stamp;
  predicted_line.ns = "predicted_line";
  predicted_line.action = visualization_msgs::Marker::ADD;
  predicted_line.pose.orientation.w = 1.0;
  predicted_line.id = object_id;
  predicted_line.type = visualization_msgs::Marker::LINE_STRIP;
  predicted_line.scale.x = 2;

  // Points are green
  predicted_line.color.r = 1.0f;
  predicted_line.color.a = 0.5;

  geometry_msgs::Point p;
  p.x = position.x;
  p.y = position.y;
  p.z = -sensor_height_;
  predicted_line.points.push_back(p);
}

void NaiveMotionPredict::makePrediction(const autoware_msgs::DetectedObject& object,
                                        std::vector<autoware_msgs::DetectedObject>& predicted_objects,
                                        visualization_msgs::Marker& predicted_line)
{
  autoware_msgs::DetectedObject target_object = object;
  initializeROSmarker(object.header, object.pose.position, object.id, predicted_line);
  for (int i = 0; i < num_prediction_; i++)
  {
    autoware_msgs::DetectedObject predicted_object = generatePredictedObject(target_object);
    target_object = predicted_object;

    geometry_msgs::Point p;
    p.x = predicted_object.pose.position.x;
    p.y = predicted_object.pose.position.y;
    p.z = -sensor_height_;
    predicted_line.points.push_back(p);
  }
}

/*
This package is a template package for more sopisticated prediction packages.
Feel free to change/modify generatePredictedObject function
and send pull request to Autoware
*/

autoware_msgs::DetectedObject NaiveMotionPredict::generatePredictedObject(const autoware_msgs::DetectedObject& object)
{
  autoware_msgs::DetectedObject predicted_object;
  if (object.behavior_state == MotionModel::CV)
  {
    predicted_object = moveConstantVelocity(object);
  }
  else if (object.behavior_state == MotionModel::CTRV)
  {
    predicted_object = moveConstantTurnRateVelocity(object);
  }
  else
  {
    // This is because random motion's velocity is 0
    predicted_object = object;
  }

  return predicted_object;
}

autoware_msgs::DetectedObject NaiveMotionPredict::moveConstantVelocity(const autoware_msgs::DetectedObject& object)
{
  autoware_msgs::DetectedObject predicted_object;
  predicted_object = object;
  double px = object.pose.position.x;
  double py = object.pose.position.y;
  double velocity = object.velocity.linear.x;
  double yaw = generateYawFromQuaternion(object.pose.orientation);

  // predicted state values
  double prediction_px = px + velocity * cos(yaw) * interval_sec_;
  double prediction_py = py + velocity * sin(yaw) * interval_sec_;

  predicted_object.pose.position.x = prediction_px;
  predicted_object.pose.position.y = prediction_py;

  return predicted_object;
}

autoware_msgs::DetectedObject
NaiveMotionPredict::moveConstantTurnRateVelocity(const autoware_msgs::DetectedObject& object)
{
  autoware_msgs::DetectedObject predicted_object;
  predicted_object = object;
  double px = object.pose.position.x;
  double py = object.pose.position.y;
  double velocity = object.velocity.linear.x;
  double yaw = generateYawFromQuaternion(object.pose.orientation);
  double yawd = object.acceleration.linear.y;

  // predicted state values
  double prediction_px, prediction_py;

  // avoid division by zero
  if (fabs(yawd) > 0.001)
  {
    prediction_px = px + velocity / yawd * (sin(yaw + yawd * interval_sec_) - sin(yaw));
    prediction_py = py + velocity / yawd * (cos(yaw) - cos(yaw + yawd * interval_sec_));
  }
  else
  {
    prediction_px = px + velocity * interval_sec_ * cos(yaw);
    prediction_py = py + velocity * interval_sec_ * sin(yaw);
  }
  double prediction_yaw = yaw + yawd * interval_sec_;

  while (prediction_yaw > M_PI)
    prediction_yaw -= 2. * M_PI;
  while (prediction_yaw < -M_PI)
    prediction_yaw += 2. * M_PI;

  predicted_object.pose.position.x = prediction_px;
  predicted_object.pose.position.y = prediction_py;
  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, prediction_yaw);
  predicted_object.pose.orientation.x = q[0];
  predicted_object.pose.orientation.y = q[1];
  predicted_object.pose.orientation.z = q[2];
  predicted_object.pose.orientation.w = q[3];
  return predicted_object;
}

double NaiveMotionPredict::generateYawFromQuaternion(const geometry_msgs::Quaternion& quaternion)
{
  tf::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

void NaiveMotionPredict::objectsCallback(const autoware_msgs::DetectedObjectArray& input)
{
  autoware_msgs::DetectedObjectArray output;
  visualization_msgs::MarkerArray predicted_lines;
  output.header = input.header;

  for (const auto &object : input.objects)
  {
    std::vector<autoware_msgs::DetectedObject> predicted_objects_vec;
    visualization_msgs::Marker predicted_line;
    if (isObjectValid(object))
    {
      makePrediction(object, predicted_objects_vec, predicted_line);

      // concate to output object array
      output.objects.insert(output.objects.end(), predicted_objects_vec.begin(), predicted_objects_vec.end());

      // visualize only stably tracked objects
      if (!object.pose_reliable)
      {
        continue;
      }
      predicted_lines.markers.push_back(predicted_line);
    }
  }
  predicted_objects_pub_.publish(output);
  predicted_paths_pub_.publish(predicted_lines);
}

bool NaiveMotionPredict::isObjectValid(const autoware_msgs::DetectedObject &in_object)
{
  if (!in_object.valid ||
      std::isnan(in_object.pose.orientation.x) ||
      std::isnan(in_object.pose.orientation.y) ||
      std::isnan(in_object.pose.orientation.z) ||
      std::isnan(in_object.pose.orientation.w) ||
      std::isnan(in_object.pose.position.x) ||
      std::isnan(in_object.pose.position.y) ||
      std::isnan(in_object.pose.position.z) ||
      (in_object.pose.position.x <= 0) ||
      (in_object.pose.position.y <= 0) ||
      (in_object.dimensions.x <= 0) ||
      (in_object.dimensions.y <= 0) ||
      (in_object.dimensions.z <= 0)
    )
  {
    return false;
  }
  return true;
}//end IsObjectValid