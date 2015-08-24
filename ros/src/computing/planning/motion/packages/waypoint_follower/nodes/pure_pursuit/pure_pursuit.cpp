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
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "runtime_manager/ConfigLaneFollower.h"
#include "waypoint_follower/libwaypoint_follower.h"

static const int LOOP_RATE = 10; //Hz
static const double LOOK_AHEAD_THRESHOLD_CALC_RATIO = 3.0; // the next waypoint must be outside of this threshold.
static const double MINIMUM_LOOK_AHEAD_THRESHOLD = 4.0; // the next waypoint must be outside of this threshold.
static const double EVALUATION_THRESHOLD = 1.0; //meter
static const std::string MAP_FRAME = "map";

//define class
class PathPP: public Path
{
private:
  int next_waypoint_;
  int param_flag_; //0 = waypoint, 1 = Dialog
  double lookahead_threshold_; //meter
  double initial_velocity_; //km/h

public:
  PathPP()
  {
    param_flag_ = 0;
    lookahead_threshold_ = 4.0;
    initial_velocity_ = 5.0;
    next_waypoint_ = -1;
  }
  void setConfig(const runtime_manager::ConfigLaneFollowerConstPtr &config);
  double getCmdVelocity();
  double getLookAheadThreshold(int waypoint);
  int getNextWaypoint();
  double calcRadius(int waypoint);
  bool evaluateWaypoint(int next_waypoint);
  void displayTrajectory(tf::Vector3 center, double radius);
};
PathPP _path_pp;

static bool _sim_mode = false;

static geometry_msgs::PoseStamped _current_pose; // current pose by the global plane.
static double _current_velocity;
static double _prev_velocity = 0;

static ros::Publisher _vis_pub;
static ros::Publisher _traj_pub;
static ros::Publisher _stat_pub;
static bool _waypoint_set = false;
static bool _pose_set = false;

void PathPP::setConfig(const runtime_manager::ConfigLaneFollowerConstPtr &config)
{
  initial_velocity_ = config->velocity;
  param_flag_ = config->param_flag;
  lookahead_threshold_ = config->lookahead_threshold;
}

//get velocity of the closest waypoint or from config
double PathPP::getCmdVelocity()
{

  if (param_flag_)
  {
    ROS_INFO_STREAM("dialog : " << initial_velocity_ << " km/h (" << kmph2mps(initial_velocity_) << " m/s )");
    return kmph2mps(initial_velocity_);
  }
  if (current_path_.waypoints.empty())
  {
    ROS_INFO_STREAM("waypoint : not loaded path");
    return 0;
  }

  double velocity = current_path_.waypoints[closest_waypoint_].twist.twist.linear.x;
  ROS_INFO_STREAM("waypoint : " << mps2kmph(velocity) << " km/h ( " << velocity << "m/s )");
  return velocity;
}

double PathPP::getLookAheadThreshold(int waypoint)
{
  if (param_flag_)
    return lookahead_threshold_;

  double current_velocity_mps = current_path_.waypoints[waypoint].twist.twist.linear.x;
  if (current_velocity_mps * LOOK_AHEAD_THRESHOLD_CALC_RATIO < MINIMUM_LOOK_AHEAD_THRESHOLD)
    return MINIMUM_LOOK_AHEAD_THRESHOLD;
  else
    return current_velocity_mps * LOOK_AHEAD_THRESHOLD_CALC_RATIO;
}

/////////////////////////////////////////////////////////////////
// obtain the next "effective" waypoint.
// the vehicle drives itself toward this waypoint.
/////////////////////////////////////////////////////////////////
int PathPP::getNextWaypoint()
{
  // if waypoints are not given, do nothing.
  if (!getPathSize())
    return -1;

  if(next_waypoint_ == getPathSize() - 1)
    return next_waypoint_;

  double lookahead_threshold = getLookAheadThreshold(closest_waypoint_);
  //ROS_INFO_STREAM("threshold = " << lookahead_threshold);
  // look for the next waypoint.
  for (int i = closest_waypoint_; i < getPathSize(); i++)
  {
    if(i == getPathSize() -1)
        return i;

    //if threshold is  distance of previous waypoint
    if (next_waypoint_ > 0)
    {
      if (getDistance(next_waypoint_) > lookahead_threshold)
      {
        ROS_INFO_STREAM("threshold = " << lookahead_threshold);
        return next_waypoint_;
      }
    }

    // if there exists an effective waypoint
    if (getDistance(i) > lookahead_threshold)
    {

      //if param flag is waypoint
      if (!param_flag_)
      {

        if (evaluateWaypoint(i) == true)
        {
          ROS_INFO_STREAM("threshold = " << lookahead_threshold);
          next_waypoint_ = i;
          return i;
        }
        else
        {
          //restart search from closest_waypoint
          i = closest_waypoint_;

          //threshold shortening
          if (lookahead_threshold > MINIMUM_LOOK_AHEAD_THRESHOLD)
          {
            // std::cout << "threshold correction" << std::endl;
            lookahead_threshold -= lookahead_threshold / 10;
            ROS_INFO_STREAM("fixed threshold = " << lookahead_threshold);
          }
          else
          {
            lookahead_threshold = MINIMUM_LOOK_AHEAD_THRESHOLD;
          }
        }
      }
      else
      {
        ROS_INFO_STREAM("threshold = " << lookahead_threshold);
        next_waypoint_ = i;
        return i;
      }
    }
  }

  // if the program reaches here, it means we lost the waypoint.
  return -1;
}

bool PathPP::evaluateWaypoint(int next_waypoint)
{

  double radius = calcRadius(next_waypoint);
  // std::cout << "radius "<< radius << std::endl;
  if (radius < 0)
    radius = (-1) * radius;

  tf::Vector3 center;

  //calculate circle of trajectory
  if (transformWaypoint(next_waypoint).getY() > 0)
    center = tf::Vector3(0, 0 + radius, 0);
  else
    center = tf::Vector3(0, 0 - radius, 0);

  displayTrajectory(center, radius);

  //evaluation
  double evaluation = 0;
  for (int j = closest_waypoint_ + 1; j < next_waypoint; j++)
  {
    tf::Vector3 tf_waypoint = transformWaypoint(j);
    tf_waypoint.setZ(0);
    double dt_diff = fabs(tf::tfDistance(center, tf_waypoint) - fabs(radius));
    // std::cout << dt_diff << std::endl;
    if (dt_diff > evaluation)
      evaluation = dt_diff;
  }

  if (evaluation < EVALUATION_THRESHOLD)
    return true;
  else
    return false;

}

// display the trajectory by markers.
void PathPP::displayTrajectory(tf::Vector3 center, double radius)
{

  geometry_msgs::Point point;
  tf::Vector3 inv_center = transform_.inverse() * center;

  point.x = inv_center.getX();
  point.y = inv_center.getY();
  point.z = inv_center.getZ();

  visualization_msgs::Marker traj;
  traj.header.frame_id = MAP_FRAME;
  traj.header.stamp = ros::Time();
  traj.ns = "trajectory_marker";
  traj.id = 0;
  traj.type = visualization_msgs::Marker::SPHERE;
  traj.action = visualization_msgs::Marker::ADD;
  traj.pose.position = point;
  traj.scale.x = radius * 2;
  traj.scale.y = radius * 2;
  traj.scale.z = 1.0;
  traj.color.a = 0.3;
  traj.color.r = 1.0;
  traj.color.g = 0.0;
  traj.color.b = 0.0;
  traj.frame_locked = true;
  _traj_pub.publish(traj);
}

double PathPP::calcRadius(int waypoint)
{
  return pow(getDistance(waypoint), 2) / (2 * transformWaypoint(waypoint).getY());
}

static void ConfigCallback(const runtime_manager::ConfigLaneFollowerConstPtr config)
{
  _path_pp.setConfig(config);
}

static void OdometryPoseCallback(const nav_msgs::OdometryConstPtr &msg)
{
  //std::cout << "odometry callback" << std::endl;

  //
  // effective for testing.
  //
  if (_sim_mode)
  {
    _current_velocity = msg->twist.twist.linear.x;
    _current_pose.header = msg->header;
    _current_pose.pose = msg->pose.pose;

    tf::Transform inverse;
    tf::poseMsgToTF(msg->pose.pose, inverse);
    _path_pp.setTransform(inverse.inverse());
    _pose_set = true;
    //   std::cout << "transform2 (" << _transform2.getOrigin().x() << " " <<  _transform2.getOrigin().y() << " " <<  _transform2.getOrigin().z() << ")" << std::endl;
  }

}

static void NDTCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (!_sim_mode)
  {
    _current_pose.header = msg->header;
    _current_pose.pose = msg->pose;
    tf::Transform inverse;
    tf::poseMsgToTF(msg->pose, inverse);
    _path_pp.setTransform(inverse.inverse());
    _pose_set = true;
  }
}

static void estVelCallback(const std_msgs::Float32ConstPtr &msg)
{
  _current_velocity = kmph2mps(msg->data);
}

static void WayPointCallback(const waypoint_follower::laneConstPtr &msg)
{
  _path_pp.setPath(msg);
  _waypoint_set = true;
  ROS_INFO_STREAM("waypoint subscribed");
}

// display the next waypoint by markers.
static void displayNextWaypoint(int i)
{

  visualization_msgs::Marker marker;
  marker.header.frame_id = MAP_FRAME;
  marker.header.stamp = ros::Time();
  marker.ns = "next_waypoint_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = _path_pp.getWaypointPosition(i);
  marker.pose.orientation = _path_pp.getWaypointOrientation(i);
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.frame_locked = true;
  _vis_pub.publish(marker);
}

/////////////////////////////////////////////////////////////////
// obtain the linear/angular velocity toward the next waypoint.
/////////////////////////////////////////////////////////////////
static geometry_msgs::Twist calcTwist(int next_waypoint)
{
  //std::cout << "calculate" << std::endl;
  geometry_msgs::Twist twist;

  double radius = _path_pp.calcRadius(next_waypoint);
  twist.linear.x = _path_pp.getCmdVelocity();

  // double current_velocity = _current_velocity;
  double current_velocity = twist.linear.x;
  if (radius > 0 || radius < 0)
  {
    twist.angular.z = current_velocity / radius;
  }
  else
  {
    twist.angular.z = 0;
  }

  return twist;
}

/////////////////////////////////////////////////////////////////
// Safely stop the vehicle.
/////////////////////////////////////////////////////////////////
static geometry_msgs::Twist stopControl()
{
  geometry_msgs::Twist twist;

  double lookahead_distance = _path_pp.getDistance(_path_pp.getPathSize() - 1);

  double stop_interval = 3;
  if (lookahead_distance < stop_interval)
  {
    twist.linear.x = 0;
    twist.angular.z = 0;
    return twist;
  }

  twist.linear.x = DecelerateVelocity(lookahead_distance, _prev_velocity);

  if (twist.linear.x < 1.0)
  {
    twist.linear.x = 0;
  }

  double radius = _path_pp.calcRadius(_path_pp.getPathSize() - 1);

  if (radius > 0 || radius < 0)
  {
    twist.angular.z = twist.linear.x / radius;
  }
  else
  {
    twist.angular.z = 0;
  }
  return twist;

}

int main(int argc, char **argv)
{
  ROS_INFO_STREAM("pure pursuit start");

  // set up ros
  ros::init(argc, argv, "pure_pursuit");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // setting params
  private_nh.getParam("sim_mode", _sim_mode);
  ROS_INFO_STREAM("sim_mode : " << _sim_mode);

  //publish topic
  ros::Publisher cmd_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
  _vis_pub = nh.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
  _traj_pub = nh.advertise<visualization_msgs::Marker>("trajectory_mark", 0);
  _stat_pub = nh.advertise<std_msgs::Bool>("wf_stat", 0);

  //subscribe topic
  ros::Subscriber waypoint_subcscriber = nh.subscribe("safety_waypoint", 10, WayPointCallback);
  ros::Subscriber odometry_subscriber = nh.subscribe("odom_pose", 10, OdometryPoseCallback);
  ros::Subscriber ndt_subscriber = nh.subscribe("control_pose", 10, NDTCallback);
  ros::Subscriber estimated_vel_subscriber = nh.subscribe("estimated_vel", 10, estVelCallback);
  ros::Subscriber config_subscriber = nh.subscribe("config/waypoint_follower", 10, ConfigCallback);

  geometry_msgs::TwistStamped twist;
  ros::Rate loop_rate(LOOP_RATE); // by Hz
  bool endflag = false;
  while (ros::ok())
  {
    std_msgs::Bool wf_stat;

    ros::spinOnce();

    //check topic
    if (_waypoint_set == false || _pose_set == false)
    {
      ROS_INFO_STREAM("topic waiting...");
      loop_rate.sleep();
      continue;
    }

    //get closest waypoint
    int closest_waypoint = _path_pp.getClosestWaypoint();
    ROS_INFO_STREAM("closest waypoint = " << closest_waypoint);

    //if vehicle is not closed to final destination
    if (!endflag)
    {
      //if can get closest waypoint
      if (closest_waypoint > 0)
      {
        //get next waypoint
        int next_waypoint = _path_pp.getNextWaypoint();

        //if can get next waypoint
        if (next_waypoint > 0)
        {
          // obtain the linear/angular velocity.
          ROS_INFO_STREAM("next waypoint = " << next_waypoint << "/" << _path_pp.getPathSize() - 1);
          displayNextWaypoint(next_waypoint);
          wf_stat.data = true;
          _stat_pub.publish(wf_stat);
          twist.twist = calcTwist(next_waypoint);
        }
        else //if cannot get next
        {
          ROS_INFO_STREAM("lost waypoint! ");
          wf_stat.data = false;
          _stat_pub.publish(wf_stat);
          twist.twist.linear.x = 0;
          twist.twist.angular.z = 0;
        }

        //check whether vehicle is closed to final destination
        if (next_waypoint > _path_pp.getPathSize() - 5)
          endflag = true;

      }
      else //cannot get closest
      {
        ROS_INFO_STREAM("closest waypoint cannot detected !!");
        wf_stat.data = false;
        _stat_pub.publish(wf_stat);
        twist.twist.linear.x = 0;
        twist.twist.angular.z = 0;
      }

    }
    else //final sequence
    {
      twist.twist = stopControl();
      wf_stat.data = false;
      _stat_pub.publish(wf_stat);

      // after stopped or fed out, let's get ready for the restart.
      if (twist.twist.linear.x == 0)
      {
        ROS_INFO_STREAM("pure pursuit ended!!");
        wf_stat.data = false;
        _stat_pub.publish(wf_stat);
        endflag = false;
      }
    }

    ROS_INFO_STREAM(
        "twist(linear.x , angular.z) = ( " << twist.twist.linear.x << " , " << twist.twist.angular.z << " )");
    twist.header.stamp = ros::Time::now();
    cmd_velocity_publisher.publish(twist);
    _prev_velocity = twist.twist.linear.x;
    loop_rate.sleep();
  }

  return 0;
}
