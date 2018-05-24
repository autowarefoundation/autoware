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
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <vector>
#include <string>

#include <boost/circular_buffer.hpp>

#include "waypoint_follower/libwaypoint_follower.h"
#include "autoware_msgs/LaneArray.h"
#include "autoware_msgs/ConfigLaneStop.h"
#include "autoware_msgs/traffic_light.h"

class WaypointVelocityVisualizer
{
public:
  WaypointVelocityVisualizer();
  ~WaypointVelocityVisualizer();

private:
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped,
                                                          geometry_msgs::TwistStamped>
      ControlSyncPolicy;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  ros::Subscriber lane_waypoints_array_sub_;
  ros::Subscriber final_waypoints_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped>* current_pose_sub_;
  message_filters::Subscriber<geometry_msgs::TwistStamped>* current_twist_sub_;
  message_filters::Subscriber<geometry_msgs::TwistStamped>* command_twist_sub_;
  message_filters::Synchronizer<ControlSyncPolicy>* control_sync_;

  ros::Publisher velocity_marker_pub_;

  visualization_msgs::MarkerArray velocity_marker_array_;
  visualization_msgs::MarkerArray lane_waypoints_array_marker_array_;
  visualization_msgs::MarkerArray final_waypoints_marker_array_;
  visualization_msgs::MarkerArray current_twist_marker_array_;
  visualization_msgs::MarkerArray command_twist_marker_array_;

  bool use_bar_plot_ = false;
  bool use_line_plot_ = true;
  bool use_text_plot_ = true;
  int control_buffer_size_ = 100;
  double plot_height_ratio_ = 1.0;
  double plot_height_shift_ = 0.2;
  double plot_metric_interval_ = 1.0;
  std::vector<double> lane_waypoints_array_rgba_ = { 1.0, 1.0, 1.0, 0.5 };
  std::vector<double> final_waypoints_rgba_ = { 0.0, 1.0, 0.0, 0.5 };
  std::vector<double> current_twist_rgba_ = { 0.0, 0.0, 1.0, 0.5 };
  std::vector<double> command_twist_rgba_ = { 1.0, 0.0, 0.0, 0.5 };

  std_msgs::ColorRGBA lane_waypoints_array_color_;
  std_msgs::ColorRGBA final_waypoints_color_;
  std_msgs::ColorRGBA current_twist_color_;
  std_msgs::ColorRGBA command_twist_color_;

  ros::Time previous_time_;
  boost::circular_buffer<geometry_msgs::PoseStamped> current_pose_buf_;
  boost::circular_buffer<geometry_msgs::TwistStamped> current_twist_buf_;
  boost::circular_buffer<geometry_msgs::TwistStamped> command_twist_buf_;

  std_msgs::ColorRGBA vector2color(const std::vector<double>& v);
  void deleteMarkers();
  void resetBuffers();

  void laneWaypointsArrayCallback(const autoware_msgs::LaneArray::ConstPtr& msg);
  void finalWaypointsCallback(const autoware_msgs::lane::ConstPtr& msg);
  void controlCallback(const geometry_msgs::PoseStamped::ConstPtr& current_pose_msg,
                       const geometry_msgs::TwistStamped::ConstPtr& current_twist_msg,
                       const geometry_msgs::TwistStamped::ConstPtr& command_twist_msg);

  void publishVelocityMarker();

  void createVelocityMarker(const std::vector<nav_msgs::Odometry> waypoints, const std::string& ns,
                            const std_msgs::ColorRGBA& color, visualization_msgs::MarkerArray& markers);
  void createVelocityMarker(const autoware_msgs::lane& lane, const std::string& ns, const std_msgs::ColorRGBA& color,
                            visualization_msgs::MarkerArray& markers);
  void createVelocityMarker(const boost::circular_buffer<geometry_msgs::PoseStamped>& poses,
                            const boost::circular_buffer<geometry_msgs::TwistStamped>& twists, const std::string& ns,
                            const std_msgs::ColorRGBA& color, visualization_msgs::MarkerArray& markers);

  void createVelocityBarMarker(const std::vector<nav_msgs::Odometry>& waypoints, const std::string& ns,
                               const std_msgs::ColorRGBA& color, visualization_msgs::MarkerArray& markers);
  void createVelocityLineMarker(const std::vector<nav_msgs::Odometry>& waypoints, const std::string& ns,
                                const std_msgs::ColorRGBA& color, visualization_msgs::MarkerArray& markers);
  void createVelocityTextMarker(const std::vector<nav_msgs::Odometry>& waypoints, const std::string& ns,
                                const std_msgs::ColorRGBA& color, visualization_msgs::MarkerArray& markers);
};

WaypointVelocityVisualizer::WaypointVelocityVisualizer() : node_handle_(), private_node_handle_("~")
{
  private_node_handle_.param<bool>("use_bar_plot", use_bar_plot_, use_bar_plot_);
  private_node_handle_.param<bool>("use_line_plot", use_line_plot_, use_line_plot_);
  private_node_handle_.param<bool>("use_text_plot", use_text_plot_, use_text_plot_);

  private_node_handle_.param<int>("control_buffer_size", control_buffer_size_, control_buffer_size_);
  private_node_handle_.param<double>("plot_height_ratio", plot_height_ratio_, plot_height_ratio_);
  private_node_handle_.param<double>("plot_height_shift", plot_height_shift_, plot_height_shift_);
  private_node_handle_.param<double>("plot_metric_interval", plot_metric_interval_, plot_metric_interval_);

  private_node_handle_.param<std::vector<double> >("lane_waypoints_array_rgba", lane_waypoints_array_rgba_, lane_waypoints_array_rgba_);
  private_node_handle_.param<std::vector<double> >("final_waypoints_rgba", final_waypoints_rgba_,
                                                   final_waypoints_rgba_);
  private_node_handle_.param<std::vector<double> >("current_twist_rgba", current_twist_rgba_, current_twist_rgba_);
  private_node_handle_.param<std::vector<double> >("command_twist_rgba", command_twist_rgba_, command_twist_rgba_);

  lane_waypoints_array_color_ = vector2color(lane_waypoints_array_rgba_);
  final_waypoints_color_ = vector2color(final_waypoints_rgba_);
  current_twist_color_ = vector2color(current_twist_rgba_);
  command_twist_color_ = vector2color(command_twist_rgba_);

  previous_time_ = ros::Time::now();

  current_pose_buf_.set_capacity(control_buffer_size_);
  current_twist_buf_.set_capacity(control_buffer_size_);
  command_twist_buf_.set_capacity(control_buffer_size_);

  lane_waypoints_array_sub_ =
      node_handle_.subscribe("lane_waypoints_array", 1, &WaypointVelocityVisualizer::laneWaypointsArrayCallback, this);
  final_waypoints_sub_ =
      node_handle_.subscribe("final_waypoints", 1, &WaypointVelocityVisualizer::finalWaypointsCallback, this);

  current_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_handle_, "current_pose", 1);
  current_twist_sub_ =
      new message_filters::Subscriber<geometry_msgs::TwistStamped>(node_handle_, "current_velocity", 1);
  command_twist_sub_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(node_handle_, "twist_cmd", 1);
  control_sync_ = new message_filters::Synchronizer<ControlSyncPolicy>(ControlSyncPolicy(10), *current_pose_sub_,
                                                                       *current_twist_sub_, *command_twist_sub_);
  control_sync_->registerCallback(boost::bind(&WaypointVelocityVisualizer::controlCallback, this, _1, _2, _3));

  velocity_marker_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("waypoints_velocity", 10);
}

WaypointVelocityVisualizer::~WaypointVelocityVisualizer()
{
}

std_msgs::ColorRGBA WaypointVelocityVisualizer::vector2color(const std::vector<double>& v)
{
  std_msgs::ColorRGBA c;
  c.r = v[0];
  c.g = v[1];
  c.b = v[2];
  c.a = v[3];
  return c;
}

void WaypointVelocityVisualizer::deleteMarkers()
{
  velocity_marker_array_.markers.clear();
  visualization_msgs::Marker marker;
#ifndef ROS_KINETIC
  marker.action = visualization_msgs::Marker::DELETE;
#else
  marker.action = visualization_msgs::Marker::DELETEALL;
#endif
  velocity_marker_array_.markers.push_back(marker);
  velocity_marker_pub_.publish(velocity_marker_array_);
}

void WaypointVelocityVisualizer::resetBuffers()
{
  current_pose_buf_.clear();
  current_twist_buf_.clear();
  command_twist_buf_.clear();
}

void WaypointVelocityVisualizer::laneWaypointsArrayCallback(const autoware_msgs::LaneArray::ConstPtr& msg)
{
  lane_waypoints_array_marker_array_.markers.clear();
  for (size_t i = 0; i < msg->lanes.size(); ++i)
  {
    std::string ns = "lane_waypoints_" + std::to_string(i);
    createVelocityMarker(msg->lanes[i], ns, lane_waypoints_array_color_, lane_waypoints_array_marker_array_);
  }
  publishVelocityMarker();
}

void WaypointVelocityVisualizer::finalWaypointsCallback(const autoware_msgs::lane::ConstPtr& msg)
{
  final_waypoints_marker_array_.markers.clear();
  createVelocityMarker(*msg, "final_waypoints", final_waypoints_color_, final_waypoints_marker_array_);
  publishVelocityMarker();
}

void WaypointVelocityVisualizer::controlCallback(const geometry_msgs::PoseStamped::ConstPtr& current_pose_msg,
                                                 const geometry_msgs::TwistStamped::ConstPtr& current_twist_msg,
                                                 const geometry_msgs::TwistStamped::ConstPtr& command_twist_msg)
{
  // buffers are reset when time goes back, e.g. playback rosbag
  ros::Time current_time = ros::Time::now();
  if (previous_time_ > current_time)
  {
    ROS_WARN("Detected jump back in time of %.3fs. Clearing markers and buffers.",
             (previous_time_ - current_time).toSec());
    deleteMarkers();  // call 'DELETEALL'
    resetBuffers();   // clear circular buffers
  }
  previous_time_ = current_time;
  // if plot_metric_interval <= 0, velocity is plotted by each callback.
  if (plot_metric_interval_ > 0 && current_pose_buf_.size() > 0)
  {
    tf::Vector3 p1, p2;
    tf::pointMsgToTF(current_pose_buf_.back().pose.position, p1);
    tf::pointMsgToTF(current_pose_msg->pose.position, p2);
    if (!(p1.distance(p2) > plot_metric_interval_))
      return;  // skipping plot
  }
  current_pose_buf_.push_back(*current_pose_msg);
  current_twist_buf_.push_back(*current_twist_msg);
  command_twist_buf_.push_back(*command_twist_msg);
  current_twist_marker_array_.markers.clear();
  command_twist_marker_array_.markers.clear();
  createVelocityMarker(current_pose_buf_, current_twist_buf_, "current_velocity", current_twist_color_,
                       current_twist_marker_array_);
  createVelocityMarker(current_pose_buf_, command_twist_buf_, "twist_cmd", command_twist_color_,
                       command_twist_marker_array_);
  publishVelocityMarker();
}

void WaypointVelocityVisualizer::publishVelocityMarker()
{
  velocity_marker_array_.markers.clear();
  velocity_marker_array_.markers.insert(velocity_marker_array_.markers.end(),
                                        lane_waypoints_array_marker_array_.markers.begin(),
                                        lane_waypoints_array_marker_array_.markers.end());
  velocity_marker_array_.markers.insert(velocity_marker_array_.markers.end(),
                                        final_waypoints_marker_array_.markers.begin(),
                                        final_waypoints_marker_array_.markers.end());
  velocity_marker_array_.markers.insert(velocity_marker_array_.markers.end(),
                                        current_twist_marker_array_.markers.begin(),
                                        current_twist_marker_array_.markers.end());
  velocity_marker_array_.markers.insert(velocity_marker_array_.markers.end(),
                                        command_twist_marker_array_.markers.begin(),
                                        command_twist_marker_array_.markers.end());
  velocity_marker_pub_.publish(velocity_marker_array_);
}

void WaypointVelocityVisualizer::createVelocityMarker(const std::vector<nav_msgs::Odometry> waypoints,
                                                      const std::string& ns, const std_msgs::ColorRGBA& color,
                                                      visualization_msgs::MarkerArray& markers)
{
  if (use_bar_plot_)
    createVelocityBarMarker(waypoints, ns, color, markers);
  if (use_line_plot_)
    createVelocityLineMarker(waypoints, ns, color, markers);
  if (use_text_plot_)
    createVelocityTextMarker(waypoints, ns, color, markers);
}

void WaypointVelocityVisualizer::createVelocityMarker(const autoware_msgs::lane& lane, const std::string& ns,
                                                      const std_msgs::ColorRGBA& color,
                                                      visualization_msgs::MarkerArray& markers)
{
  std::vector<nav_msgs::Odometry> waypoints;
  for (const auto& wp : lane.waypoints)
  {
    nav_msgs::Odometry odom;
    odom.pose.pose = wp.pose.pose;
    odom.twist.twist = wp.twist.twist;
    waypoints.push_back(odom);
  }
  createVelocityMarker(waypoints, ns, color, markers);
}

void WaypointVelocityVisualizer::createVelocityMarker(const boost::circular_buffer<geometry_msgs::PoseStamped>& poses,
                                                      const boost::circular_buffer<geometry_msgs::TwistStamped>& twists,
                                                      const std::string& ns, const std_msgs::ColorRGBA& color,
                                                      visualization_msgs::MarkerArray& markers)
{
  assert(poses.size() == twists.size());
  std::vector<nav_msgs::Odometry> waypoints;
  for (unsigned int i = 0; i < poses.size(); ++i)
  {
    nav_msgs::Odometry odom;
    odom.pose.pose = poses[i].pose;
    odom.twist.twist = twists[i].twist;
    waypoints.push_back(odom);
  }
  createVelocityMarker(waypoints, ns, color, markers);
}

void WaypointVelocityVisualizer::createVelocityBarMarker(const std::vector<nav_msgs::Odometry>& waypoints,
                                                         const std::string& ns, const std_msgs::ColorRGBA& color,
                                                         visualization_msgs::MarkerArray& markers)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = ns + "/bar";
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color = color;

  unsigned int count = 0;
  for (const auto& wp : waypoints)
  {
    double h = plot_height_ratio_ * wp.twist.twist.linear.x;
    marker.id = count++;
    marker.pose = wp.pose.pose;
    marker.pose.position.z += h / 2.0;
    // When the the cylinder height is 0 or less, a warning occurs in RViz.
    marker.scale.z = fabs(h) + 1e-6;
    markers.markers.push_back(marker);
  }
}

void WaypointVelocityVisualizer::createVelocityLineMarker(const std::vector<nav_msgs::Odometry>& waypoints,
                                                          const std::string& ns, const std_msgs::ColorRGBA& color,
                                                          visualization_msgs::MarkerArray& markers)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = ns + "/line";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.color = color;

  for (const auto& wp : waypoints)
  {
    geometry_msgs::Point p = wp.pose.pose.position;
    p.z += plot_height_ratio_ * wp.twist.twist.linear.x;
    marker.points.push_back(p);
  }
  markers.markers.push_back(marker);
}

void WaypointVelocityVisualizer::createVelocityTextMarker(const std::vector<nav_msgs::Odometry>& waypoints,
                                                          const std::string& ns, const std_msgs::ColorRGBA& color,
                                                          visualization_msgs::MarkerArray& markers)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = ns + "/text";
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.z = 0.2;
  marker.color = color;

  unsigned int count = 0;
  for (const auto& wp : waypoints)
  {
    marker.id = count++;
    geometry_msgs::Point p = wp.pose.pose.position;
    p.z += plot_height_ratio_ * wp.twist.twist.linear.x + plot_height_shift_;
    marker.pose.position = p;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << mps2kmph(wp.twist.twist.linear.x);
    marker.text = oss.str();
    markers.markers.push_back(marker);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_velocity_visualizer");
  WaypointVelocityVisualizer node;
  ros::spin();
  return 0;
}
