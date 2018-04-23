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

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <gnss/geo_pos_conv.hpp>

static ros::Publisher pose_publisher;

static ros::Publisher stat_publisher;
static std_msgs::Bool gnss_stat_msg;

static geometry_msgs::PoseStamped _prev_pose;
static geometry_msgs::Quaternion _quat;
static double yaw;

static int _plane;

static void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
  geo_pos_conv geo;

  geo.set_plane(_plane);
  geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);

  static tf::TransformBroadcaster pose_broadcaster;
  tf::Transform pose_transform;
  tf::Quaternion pose_q;

  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  // pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = geo.y();
  pose.pose.position.y = geo.x();
  pose.pose.position.z = geo.z();

  // set gnss_stat
  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
  {
    gnss_stat_msg.data = false;
  }
  else
  {
    gnss_stat_msg.data = true;
  }

  double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
                         pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));
  std::cout << "distance : " << distance << std::endl;

  if (distance > 0.2)
  {
    yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
    _quat = tf::createQuaternionMsgFromYaw(yaw);
    _prev_pose = pose;
  }

  pose.pose.orientation = _quat;
  pose_publisher.publish(pose);
  stat_publisher.publish(gnss_stat_msg);

  //座標変換
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  q.setRPY(0, 0, yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "gps"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fix2tfpose");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("plane", _plane);
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
  stat_publisher = nh.advertise<std_msgs::Bool>("/gnss_stat", 1000);
  ros::Subscriber gnss_pose_subscriber = nh.subscribe("fix", 100, GNSSCallback);

  ros::spin();
  return 0;
}
