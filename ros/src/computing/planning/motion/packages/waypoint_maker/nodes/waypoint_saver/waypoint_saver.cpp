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

#include "vehicle_socket/CanInfo.h"
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
    ofs << std::fixed << std::setprecision(4) << current_pose.position.x << "," << current_pose.position.y << ","
        << current_pose.position.z << "," << tf::getYaw(current_pose.orientation) << std::endl;
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
          << current_pose.position.z << "," << tf::getYaw(current_pose.orientation) << "," << velocity << std::endl;

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
