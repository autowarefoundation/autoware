/*
 *  Copyright (c) 2015, Nagoya University
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

/*
 NDT matching validation

 By: Alexander Carballo
 */

#include <iostream>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <autoware_msgs/ndt_stat.h>

#define NDT_POSE_ARRAY_LEN 10
#define NDT_STAT_SCORE_THRESHOLD_WARN 1.0
#define NDT_STAT_SCORE_THRESHOLD_STOP 2.0

static ros::Time current_scan_time;
static ros::Time previous_scan_time;

static ros::Publisher initialpose_pub;
static ros::Publisher max_speed_pub;
static geometry_msgs::PoseWithCovarianceStamped initialpose;
static std::deque< geometry_msgs::PoseWithCovarianceStamped > ndt_pose_array;

static std::deque<double> ndt_stat_score_array;
static double last_score = NDT_STAT_SCORE_THRESHOLD_STOP;
static double _threshold_warning = NDT_STAT_SCORE_THRESHOLD_WARN;
static double _threshold_stop = NDT_STAT_SCORE_THRESHOLD_STOP;
static int _history = NDT_POSE_ARRAY_LEN;
static double _max_speed = 1.5;
static double _speed_down_factor = 0.5;

static bool pose_error = false;
static double score_avg = 0.0;
static double score_var = 0.0;
static double score_stddev = 0.0;

static std_msgs::Float32 max_speed_msg;
static double current_speed = _max_speed;

static void ndt_stat_callback(const autoware_msgs::ndt_stat::ConstPtr& input)
{
  ROS_INFO("ndt_stat");
  static double prev_avg = 0.0;
  static double prev_var = 0.0;
  static int score_count = 0;
  double score = input->score;
  double score_sum = 0.0;

  if (score_count < _history) {
    score_count++;
  }
  prev_avg = score_avg;
  prev_var = score_var;
  score_avg = prev_avg + (score - prev_avg) / score_count;
  score_var = prev_var + (score - prev_avg) * (score - score_avg);

  score_stddev = sqrt(score_var);
  last_score = score;
  // //if the score so far is good
  // double score_stddev = sqrt(score_var);
  // double delta = abs(last_score - score_avg);
  // if (score_avg < _threshold_warning && delta <= score_stddev) {

  // }
}

static void ndt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  ROS_INFO_STREAM("ndt_pose, score_avg -> " << score_avg << ", stddev -> " << score_stddev);
  //if the score so far is good
  double delta = abs(last_score - score_avg);
  geometry_msgs::PoseWithCovarianceStamped pose;

  pose.header = input->header;
  pose.pose.pose = input->pose;
  if (score_avg < _threshold_warning && delta <= score_stddev) {
    //keep a history of valid poses (score is small)
    if (ndt_pose_array.size() < _history) {
      //just push
      ndt_pose_array.push_back(pose);
    } else {
      //pop front and push
      ndt_pose_array.pop_front();
      ndt_pose_array.push_back(pose);
    }
    //update the last good pose
    initialpose = pose;
  }

  current_speed = _max_speed;
  if (score_avg > _threshold_warning && score_avg < _threshold_stop) {
    //show some warning
    //TODO: draw something on screen or on rviz
    //reduce speed
    current_speed = _max_speed * _speed_down_factor;
  } else if (score_avg > _threshold_stop) {
    //show some error
    //TODO: draw something on screen or on rviz
    //stop the robot
    current_speed = 0;
    //force the last known good pose as initialpose
    geometry_msgs::PoseWithCovarianceStamped initialpose_msg;
    initialpose_msg.header.stamp = input->header.stamp;
    initialpose_msg.pose = initialpose.pose;
    initialpose_pub.publish(initialpose_msg);
  }

  //publish the current max speed
  max_speed_msg.data = current_speed;
  max_speed_pub.publish(max_speed_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_matching_monitor");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Geting parameters
  private_nh.getParam("/monitor/threshold_warn", _threshold_warning);
  private_nh.getParam("/monitor/threshold_stop", _threshold_stop);
  private_nh.getParam("/monitor/history_size", _history);
  private_nh.getParam("/monitor/max_speed", _max_speed);
  private_nh.getParam("/monitor/speed_down_factor", _speed_down_factor);

  //swap values in case of error
  if (_threshold_warning > _threshold_stop) {
    auto aux = _threshold_stop;
    _threshold_stop = _threshold_warning;
    _threshold_warning = aux;
  }
  last_score = _threshold_stop + 1;
  current_speed = _max_speed;

  // Subscribers
  ros::Subscriber ndt_stat_sub = nh.subscribe("/ndt_stat", 10, ndt_stat_callback);
  ros::Subscriber ndt_pose_sub = nh.subscribe("ndt_pose", 10, ndt_pose_callback);

  // Publishers
  initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
  max_speed_pub = nh.advertise<std_msgs::Float32>("/ndt_monitor/max_speed", 1000);

  ros::spin();

  return 0;
}
