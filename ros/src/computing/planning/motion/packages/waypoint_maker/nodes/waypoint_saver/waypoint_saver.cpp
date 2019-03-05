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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>

#include <fstream>

#include "waypoint_follower/libwaypoint_follower.h"

static const int SYNC_FRAMES = 50;

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, geometry_msgs::PoseStamped>
    TwistPoseSync;

class WaypointSaver
{
public:
  WaypointSaver();
  ~WaypointSaver();

private:
  // functions

  void TwistPoseCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg,
                         const geometry_msgs::PoseStampedConstPtr &pose_msg) const;
  void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg) const;
  void displayMarker(geometry_msgs::Pose pose, double velocity) const;
  void outputProcessing(geometry_msgs::Pose current_pose, double velocity) const;

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher waypoint_saver_pub_;

  // subscriber
  message_filters::Subscriber<geometry_msgs::TwistStamped> *twist_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> *pose_sub_;
  message_filters::Synchronizer<TwistPoseSync> *sync_tp_;

  // variables
  bool save_velocity_;
  double interval_;
  std::string filename_, pose_topic_, velocity_topic_;
};

WaypointSaver::WaypointSaver() : private_nh_("~")
{
  // parameter settings
  private_nh_.param<std::string>("save_filename", filename_, std::string("data.txt"));
  private_nh_.param<std::string>("pose_topic", pose_topic_, std::string("current_pose"));
  private_nh_.param<std::string>("velocity_topic", velocity_topic_, std::string("current_velocity"));
  private_nh_.param<double>("interval", interval_, 1.0);
  private_nh_.param<bool>("save_velocity", save_velocity_, false);

  // subscriber
  pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, pose_topic_, 50);

  if (save_velocity_)
  {
    twist_sub_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, velocity_topic_, 50);
    sync_tp_ = new message_filters::Synchronizer<TwistPoseSync>(TwistPoseSync(SYNC_FRAMES), *twist_sub_, *pose_sub_);
    sync_tp_->registerCallback(boost::bind(&WaypointSaver::TwistPoseCallback, this, _1, _2));
  }
  else
  {
    pose_sub_->registerCallback(boost::bind(&WaypointSaver::poseCallback, this, _1));
  }

  // publisher
  waypoint_saver_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("waypoint_saver_marker", 10, true);
}

WaypointSaver::~WaypointSaver()
{
  delete twist_sub_;
  delete pose_sub_;
  delete sync_tp_;
}

void WaypointSaver::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg) const
{
  outputProcessing(pose_msg->pose, 0);
}

void WaypointSaver::TwistPoseCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg,
                                      const geometry_msgs::PoseStampedConstPtr &pose_msg) const
{
  outputProcessing(pose_msg->pose, mps2kmph(twist_msg->twist.linear.x));
}

void WaypointSaver::outputProcessing(geometry_msgs::Pose current_pose, double velocity) const
{
  std::ofstream ofs(filename_.c_str(), std::ios::app);
  static geometry_msgs::Pose previous_pose;
  static bool receive_once = false;
  // first subscribe
  if (!receive_once)
  {
    ofs << "x,y,z,yaw,velocity,change_flag" << std::endl;
    ofs << std::fixed << std::setprecision(4) << current_pose.position.x << "," << current_pose.position.y << ","
        << current_pose.position.z << "," << tf::getYaw(current_pose.orientation) << ",0,0" << std::endl;
    receive_once = true;
    displayMarker(current_pose, 0);
    previous_pose = current_pose;
  }
  else
  {
    double distance = sqrt(pow((current_pose.position.x - previous_pose.position.x), 2) +
                           pow((current_pose.position.y - previous_pose.position.y), 2));

    // if car moves [interval] meter
    if (distance > interval_)
    {
      ofs << std::fixed << std::setprecision(4) << current_pose.position.x << "," << current_pose.position.y << ","
          << current_pose.position.z << "," << tf::getYaw(current_pose.orientation) << "," << velocity << ",0" << std::endl;

      displayMarker(current_pose, velocity);
      previous_pose = current_pose;
    }
  }
}

void WaypointSaver::displayMarker(geometry_msgs::Pose pose, double velocity) const
{
  static visualization_msgs::MarkerArray marray;
  static int id = 0;

  // initialize marker
  visualization_msgs::Marker marker;
  marker.id = id;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.frame_locked = true;

  // create saved waypoint marker
  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.ns = "saved_waypoint_arrow";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marray.markers.push_back(marker);

  // create saved waypoint velocity text
  marker.scale.z = 0.4;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.ns = "saved_waypoint_velocity";
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << velocity << " km/h";
  marker.text = oss.str();
  marray.markers.push_back(marker);

  waypoint_saver_pub_.publish(marray);
  id++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_saver");
  WaypointSaver ws;
  ros::spin();
  return 0;
}
