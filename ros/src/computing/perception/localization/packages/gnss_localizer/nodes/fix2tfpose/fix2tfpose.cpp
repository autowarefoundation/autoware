/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
