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

#include "runtime_manager/ConfigLaneFollower.h"
#include "libwaypoint_follower.h"


#define LOOP_RATE 10 //Hz

//define class
class PathPP: public Path
{
private:
  int next_waypoint_;
  int param_flag_; //0 = waypoint, 1 = Dialog
  double lookahead_threshold_; //meter
  double initial_velocity_; //km/h

public:
  PathPP(){
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

// parameter servers
static std::string _mobility_frame = "/base_link"; // why is this default?
static std::string _current_pose_topic = "ndt";

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

  if(param_flag_){

    ROS_INFO_STREAM("dialog : " << initial_velocity_ << " km/h (" << kmph2mps(initial_velocity_) << " m/s )");
    return kmph2mps(initial_velocity_);
  }
  if (current_path_.waypoints.empty())
  {
    ROS_INFO_STREAM("waypoint : not loaded path");
    return 0;
  }

  double velocity = current_path_.waypoints[closest_waypoint_].twist.twist.linear.x;
  ROS_INFO_STREAM("waypoint : " <<  mps2kmph(velocity) << " km/h , " << velocity << "m/s");
  return velocity;
}

double PathPP::getLookAheadThreshold(int waypoint)
{
  if (param_flag_)
    return lookahead_threshold_;

  double ratio = 2.0;
  double current_velocity_mps = current_path_.waypoints[waypoint].twist.twist.linear.x;
  if (current_velocity_mps * ratio < 3)
    return 3.0;
  else
    return current_velocity_mps * ratio;
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

  // the next waypoint must be outside of this threshold.
  int minimum_th = 3;

  double lookahead_threshold = getLookAheadThreshold(closest_waypoint_);

  // look for the next waypoint.
  for (int i = closest_waypoint_; i < getPathSize(); i++)
  {

    //if threshold is  distance of previous waypoint
    if (next_waypoint_ > 0)
    {
      if (getDistance(next_waypoint_) > lookahead_threshold)
      {
        ROS_INFO_STREAM("threshold = " << lookahead_threshold);
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
          return i;
        }
        else
        {
          if (lookahead_threshold > minimum_th)
          {
            lookahead_threshold -= lookahead_threshold / 10;
          }
          else
          {
            lookahead_threshold = minimum_th;
          }
        }
      }
      else
      {
        ROS_INFO_STREAM("threshold = " << lookahead_threshold);
        return i;
      }
    }
  }

  // if the program reaches here, it means we lost the waypoint.
  return -1;
}

bool PathPP::evaluateWaypoint(int next_waypoint)
{
  double eval_threshold = 1.0;


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

  if (evaluation < eval_threshold)
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
    traj.header.frame_id = "map";
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
    _initial_velocity_kmh = config->velocity;
    _lookahead_threshold = config->lookahead_threshold;
    _param_flag = config->param_flag;
    // _param_set = true;
}

static void OdometryPoseCallback(const nav_msgs::OdometryConstPtr &msg)
{
    //std::cout << "odometry callback" << std::endl;
    _current_velocity = msg->twist.twist;

    //
    // effective for testing.
    //
    if (_current_pose_topic == "odometry") {
        _current_pose.header = msg->header;
        _current_pose.pose = msg->pose.pose;

        tf::Transform inverse;
        tf::poseMsgToTF(msg->pose.pose, inverse);
        _transform = inverse.inverse();
        _pose_set = true;
        //   std::cout << "transform2 (" << _transform2.getOrigin().x() << " " <<  _transform2.getOrigin().y() << " " <<  _transform2.getOrigin().z() << ")" << std::endl;
    }

}

static void NDTCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if (_current_pose_topic == "ndt") {
        _current_pose.header = msg->header;
        _current_pose.pose = msg->pose;
        tf::Transform inverse;
        tf::poseMsgToTF(msg->pose, inverse);
        _transform = inverse.inverse();
        _pose_set = true;
    }
}

static void WayPointCallback(const waypoint_follower::laneConstPtr &msg)
{
    _current_path = *msg;
    _waypoint_set = true;
    std::cout << "waypoint subscribed" << std::endl;
}

}

// display the next waypoint by markers.
static void DisplayTargetWaypoint(int i)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = PATH_FRAME;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = _current_path.waypoints[i].pose.pose.position;
    marker.pose.orientation = _current_path.waypoints[i].pose.pose.orientation;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration(5);
    marker.frame_locked = true;
    _vis_pub.publish(marker);
}

/////////////////////////////////////////////////////////////////
// obtain the linear/angular velocity toward the next waypoint.
/////////////////////////////////////////////////////////////////
static geometry_msgs::Twist CalculateCmdTwist()
{
    //std::cout << "calculate" << std::endl;
    geometry_msgs::Twist twist;

    double radius = CalcRadius(_next_waypoint);
    double set_velocity_ms = 0;
    if (!_param_flag)
        set_velocity_ms = GetWaypointVelocity();
    else
        set_velocity_ms = _initial_velocity_kmh / 3.6;

    twist.linear.x = set_velocity_ms;

    if (radius > 0 || radius < 0) {
        twist.angular.z = twist.linear.x / radius;
    } else {
        twist.angular.z = 0;
    }

    return twist;
}

/////////////////////////////////////////////////////////////////
// Safely stop the vehicle.
/////////////////////////////////////////////////////////////////
static geometry_msgs::Twist EndControl()
{
    std::cout << "end control" << std::endl;
    geometry_msgs::Twist twist;
    //  double minimum_velocity_kmh = 2.0;

    // std::cout << "End Distance = " << _end_distance << std::endl;

    double lookahead_distance = GetLookAheadDistance(_current_path.waypoints.size() - 1);
    std::cout << "Lookahead Distance = " << lookahead_distance << std::endl;

    double stop_interval = 5;
    if (lookahead_distance < stop_interval) {
        twist.linear.x = 0;
        twist.angular.z = 0;

        return twist;
    }

    twist.linear.x = DecelerateVelocity(lookahead_distance,_prev_velocity);

    if (twist.linear.x < 1.0) {
        twist.linear.x = 0;
    }

    double radius = CalcRadius(_current_path.waypoints.size() - 1);

    if (radius > 0 || radius < 0) {
        twist.angular.z = twist.linear.x / radius;
    } else {
        twist.angular.z = 0;
    }
    return twist;

}

int main(int argc, char **argv)
{
    std::cout << "pure pursuit start" << std::endl;

    // set up ros
    ros::init(argc, argv, "pure_pursuit");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // setting params
    private_nh.getParam("current_pose_topic", _current_pose_topic);
    std::cout << "current_pose_topic : " << _current_pose_topic << std::endl;

    private_nh.getParam("mobility_frame", _mobility_frame);
    std::cout << "mobility_frame : " << _mobility_frame << std::endl;


    //publish topic
    ros::Publisher cmd_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("twist_raw", 1000);

    _vis_pub = nh.advertise<visualization_msgs::Marker>("target_waypoint_mark", 0);
    _circle_pub = nh.advertise<visualization_msgs::Marker>("circle_mark", 0);
    _stat_pub = nh.advertise<std_msgs::Bool>("lf_stat", 0);

    //subscribe topic
    ros::Subscriber waypoint_subcscriber = nh.subscribe("path_waypoint", 1000, WayPointCallback);
    ros::Subscriber odometry_subscriber = nh.subscribe("odom_pose", 1000, OdometryPoseCallback);
    ros::Subscriber ndt_subscriber = nh.subscribe("control_pose", 1000, NDTCallback);
    ros::Subscriber config_subscriber = nh.subscribe("config/lane_follower", 1000, ConfigCallback);


    geometry_msgs::TwistStamped twist;
    ros::Rate loop_rate(LOOP_RATE); // by Hz
    bool endflag = false;
    while (ros::ok()) {
        ros::spinOnce();

        if (_waypoint_set == false || _pose_set == false) {
            std::cout << "topic waiting..." << std::endl;
            loop_rate.sleep();
            continue;
        }

        _closest_waypoint = GetClosestWaypoint(_transform,_current_path,_closest_waypoint);
        std::cout << "closest waypoint = " << _closest_waypoint << std::endl;

        if (_closest_waypoint > _prev_waypoint)
            _prev_waypoint = _closest_waypoint;
        // std::cout << "endflag = " << endflag << std::endl;

        if (endflag == false) {

            // get the waypoint.
            std::cout << "velocity : ";
            if (!_param_flag) {
                std::cout << "waypoint" << std::endl;
            } else {
                std::cout << "dialog" << std::endl;
            }

            if (_closest_waypoint < 0) {
                std::cout << "closest waypoint can not detected !!" << std::endl;
                twist.twist.linear.x = 0;
                twist.twist.angular.z = 0;
            } else {

                _next_waypoint = GetNextWayPoint();
                std::cout << "next waypoint = " << _next_waypoint << "/" << _current_path.waypoints.size() - 1 << std::endl;
              //  std::cout << "prev waypoint = " << _prev_waypoint << std::endl;

                if (_next_waypoint > 0) {
                    // obtain the linear/angular velocity.
                    twist.twist = CalculateCmdTwist();
                } else {
                    twist.twist.linear.x = 0;
                    twist.twist.angular.z = 0;
                }

                if (_next_waypoint > static_cast<int>(_current_path.waypoints.size()) - 5) {
                    endflag = true;
                    _next_waypoint = _current_path.waypoints.size() - 1;
                }
            }

        } else {
            twist.twist = EndControl();

         //   std::cout << "closest/next : " << _closest_waypoint << "/" << _next_waypoint << std::endl;

            // after stopped or fed out, let's get ready for the restart.
            if (twist.twist.linear.x == 0) {
                std::cout << "pure pursuit ended!!" << std::endl;
                endflag = false;

            }
        }
        std::cout << "set velocity (kmh) = " << twist.twist.linear.x * 3.6 << std::endl;
        std::cout << "twist.linear.x = " << twist.twist.linear.x << std::endl;
        std::cout << "twist.angular.z = " << twist.twist.angular.z << std::endl;
        std::cout << std::endl;

        twist.header.stamp = ros::Time::now();
        cmd_velocity_publisher.publish(twist);
        _prev_waypoint = _next_waypoint;
        _prev_velocity = twist.twist.linear.x;
        loop_rate.sleep();
    }

    return 0;
}
