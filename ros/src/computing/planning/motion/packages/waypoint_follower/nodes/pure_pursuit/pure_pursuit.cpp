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
#include "runtime_manager/ConfigWaypointFollower.h"
#include "waypoint_follower/libwaypoint_follower.h"
#include "vehicle_socket/CanInfo.h"

namespace
{
//#define LOG

const int LOOP_RATE = 30;  // Hz
const std::string MAP_FRAME = "map";
const int MODE_WAYPOINT = 0;
const int MODE_DIALOG = 1;
double ERROR = pow(10, -5);  // 0.00001

// parameter
bool g_linear_interpolate_mode = true;

geometry_msgs::PoseStamped _current_pose;  // current pose by the global plane.
double _current_velocity;

ros::Publisher _vis_pub;
ros::Publisher _stat_pub;
bool g_waypoint_set = false;
bool g_pose_set = false;

// config topic
int _param_flag = MODE_WAYPOINT;    // 0 = waypoint, 1 = Dialog
double _lookahead_threshold = 4.0;  // meter
double _initial_velocity = 5.0;     // km/h
double g_look_ahead_threshold_calc_ratio = 2.0;
double g_minimum_look_ahead_threshold = 6.0;  // the next waypoint must be outside of this threshold.
double g_displacement_threshold = 0.2;
double g_relative_angle_threshold = 10;

WayPoints _current_waypoints;
ros::Publisher _traj_circle_pub;
ros::Publisher _target_pub;
ros::Publisher _search_pub;
ros::Publisher g_cmd_velocity_publisher;
ros::Publisher _line_point_pub;

void ConfigCallback(const runtime_manager::ConfigWaypointFollowerConstPtr &config)
{
  _param_flag = config->param_flag;
  _lookahead_threshold = config->lookahead_threshold;
  _initial_velocity = config->velocity;
  g_look_ahead_threshold_calc_ratio = config->threshold_ratio;
  g_minimum_look_ahead_threshold = config->minimum_lookahead_threshold;
  g_displacement_threshold = config->displacement_threshold;
  g_relative_angle_threshold = config->relative_angle_threshold;
}

void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  _current_pose.header = msg->header;
  _current_pose.pose = msg->pose;
  g_pose_set = true;
}

void callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
  _current_velocity = msg->twist.linear.x;
}

void WayPointCallback(const waypoint_follower::laneConstPtr &msg)
{
  _current_waypoints.setPath(*msg);
  g_waypoint_set = true;
  // ROS_INFO_STREAM("waypoint subscribed");
}

// display the next waypoint by markers.
void displayNextWaypoint(int i)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = MAP_FRAME;
  marker.header.stamp = ros::Time();
  marker.ns = "next_waypoint_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = _current_waypoints.getWaypointPosition(i);
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

// display the nexttarget by markers.
void displayNextTarget(geometry_msgs::Point target)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = MAP_FRAME;
  marker.header.stamp = ros::Time();
  marker.ns = "next_target_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = target;
  std_msgs::ColorRGBA green;
  green.a = 1.0;
  green.b = 0.0;
  green.r = 0.0;
  green.g = 1.0;
  marker.color = green;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.frame_locked = true;
  _target_pub.publish(marker);
}

// display the locus of pure pursuit by markers.
void displayTrajectoryCircle(std::vector<geometry_msgs::Point> traj_circle_array)
{
  visualization_msgs::Marker traj_circle;
  traj_circle.header.frame_id = MAP_FRAME;
  traj_circle.header.stamp = ros::Time();
  traj_circle.ns = "trajectory_circle_marker";
  traj_circle.id = 0;
  traj_circle.type = visualization_msgs::Marker::LINE_STRIP;
  traj_circle.action = visualization_msgs::Marker::ADD;

  std_msgs::ColorRGBA white;
  white.a = 1.0;
  white.b = 1.0;
  white.r = 1.0;
  white.g = 1.0;
  //
  for (auto el : traj_circle_array)
    for (std::vector<geometry_msgs::Point>::iterator it = traj_circle_array.begin(); it != traj_circle_array.end();
         it++)
    {
      // traj_circle.points.push_back(*it);
      traj_circle.points.push_back(el);
      traj_circle.colors.push_back(white);
    }

  traj_circle.scale.x = 0.1;
  traj_circle.color.a = 0.3;
  traj_circle.color.r = 1.0;
  traj_circle.color.g = 0.0;
  traj_circle.color.b = 0.0;
  traj_circle.frame_locked = true;
  _traj_circle_pub.publish(traj_circle);
}

// display the search radius by markers.
void displaySearchRadius(double search_radius)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = MAP_FRAME;
  marker.header.stamp = ros::Time();
  marker.ns = "next_target_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = _current_pose.pose.position;
  marker.scale.x = search_radius * 2;
  marker.scale.y = search_radius * 2;
  marker.scale.z = 1.0;
  marker.color.a = 0.1;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.frame_locked = true;
  _search_pub.publish(marker);
}

// debug tool for interpolateNextTarget
void displayLinePoint(double a, double b, double c, geometry_msgs::Point target, geometry_msgs::Point target2,
                      geometry_msgs::Point target3)
{
  visualization_msgs::Marker line;
  line.header.frame_id = MAP_FRAME;
  line.header.stamp = ros::Time();
  line.ns = "line_marker";
  line.id = 0;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.action = visualization_msgs::Marker::ADD;

  std_msgs::ColorRGBA white;
  white.a = 1.0;
  white.b = 1.0;
  white.r = 1.0;
  white.g = 1.0;

  for (int i = -100000; i < 100000;)
  {
    geometry_msgs::Point p;
    if (fabs(a) < ERROR)  // linear equation y = n
    {
      p.y = (-1) * c / b;
      p.x = i;
    }
    else if (fabs(b) < ERROR)  // linear equation x = n
    {
      p.x = (-1) * c / a;
      p.y = i;
    }
    else
    {
      p.x = i;
      p.y = (-1) * (a * p.x + c) / b;
    }
    p.z = _current_pose.pose.position.z;
    line.points.push_back(p);
    i += 5000;
  }

  line.scale.x = 0.3;
  line.color = white;
  line.frame_locked = true;
  _line_point_pub.publish(line);

  visualization_msgs::Marker marker;
  marker.header.frame_id = MAP_FRAME;
  marker.header.stamp = ros::Time();
  marker.ns = "target_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  // marker.pose.position = target;
  marker.points.push_back(target);
  std_msgs::ColorRGBA green;
  green.a = 1.0;
  green.b = 0.0;
  green.r = 0.0;
  green.g = 1.0;
  marker.colors.push_back(green);
  marker.points.push_back(target2);
  std_msgs::ColorRGBA yellow;
  yellow.a = 1.0;
  yellow.b = 0.0;
  yellow.r = 1.0;
  yellow.g = 1.0;
  marker.colors.push_back(yellow);
  marker.points.push_back(target3);
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.b = 1.0;
  color.r = 0.0;
  color.g = 1.0;
  marker.colors.push_back(color);

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.frame_locked = true;
  _line_point_pub.publish(marker);
}

double getCmdVelocity(int waypoint)
{
  if (_param_flag == MODE_DIALOG)
  {
    ROS_INFO_STREAM("dialog : " << _initial_velocity << " km/h (" << kmph2mps(_initial_velocity) << " m/s )");
    return kmph2mps(_initial_velocity);
  }

  if (_current_waypoints.isEmpty())
  {
    ROS_INFO_STREAM("waypoint : not loaded path");
    return 0;
  }

  double velocity = _current_waypoints.getWaypointVelocityMPS(waypoint);
  // ROS_INFO_STREAM("waypoint : " << mps2kmph(velocity) << " km/h ( " << velocity << "m/s )");
  return velocity;
}

double getLookAheadThreshold(int waypoint)
{
  if (_param_flag == MODE_DIALOG)
    return _lookahead_threshold;

  // double current_velocity_mps = _current_waypoints.getWaypointVelocityMPS(waypoint);
  double current_velocity_mps = _current_velocity;

  if (current_velocity_mps * g_look_ahead_threshold_calc_ratio < g_minimum_look_ahead_threshold)
    return g_minimum_look_ahead_threshold;
  else
    return current_velocity_mps * g_look_ahead_threshold_calc_ratio;
}

double calcCurvature(geometry_msgs::Point target)
{
  double kappa;
  double denominator = pow(getPlaneDistance(target, _current_pose.pose.position), 2);
  double numerator = 2 * calcRelativeCoordinate(target, _current_pose.pose).y;

  if (denominator != 0)
    kappa = numerator / denominator;
  else
    kappa = 0;

  // ROS_INFO("kappa : %lf", kappa);
  return kappa;
}

double calcRadius(geometry_msgs::Point target)
{
  double radius;
  double denominator = 2 * calcRelativeCoordinate(target, _current_pose.pose).y;
  double numerator = pow(getPlaneDistance(target, _current_pose.pose.position), 2);

  if (denominator != 0)
    radius = numerator / denominator;
  else
    radius = 0;

  // ROS_INFO("radius : %lf", radius);
  return radius;
}

// linear interpolation of next target
bool interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target)
{
  int path_size = static_cast<int>(_current_waypoints.getSize());
  if (next_waypoint == path_size - 1)
  {
    *next_target = _current_waypoints.getWaypointPosition(next_waypoint);
    return true;
  }
  double search_radius = getLookAheadThreshold(0);
  geometry_msgs::Point zero_p;
  geometry_msgs::Point end = _current_waypoints.getWaypointPosition(next_waypoint);
  geometry_msgs::Point start = _current_waypoints.getWaypointPosition(next_waypoint - 1);

  // let the linear equation be "ax + by + c = 0"
  // if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
  double a = 0;
  double b = 0;
  double c = 0;
  double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
  if (!get_linear_flag)
    return false;

  // let the center of circle be "(x0,y0)", in my code , the center of circle is vehicle position
  // the distance  "d" between the foot of a perpendicular line and the center of circle is ...
  //    | a * x0 + b * y0 + c |
  // d = -------------------------------
  //          √( a~2 + b~2)
  double d = getDistanceBetweenLineAndPoint(_current_pose.pose.position, a, b, c);

  // ROS_INFO("a : %lf ", a);
  // ROS_INFO("b : %lf ", b);
  // ROS_INFO("c : %lf ", c);
  // ROS_INFO("distance : %lf ", d);

  if (d > search_radius)
    return false;

  // unit vector of point 'start' to point 'end'
  tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
  tf::Vector3 unit_v = v.normalize();

  // normal unit vectors of v
  tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90);   // rotate to counter clockwise 90 degree
  tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90);  // rotate to counter clockwise 90 degree

  // the foot of a perpendicular line
  geometry_msgs::Point h1;
  h1.x = _current_pose.pose.position.x + d * unit_w1.getX();
  h1.y = _current_pose.pose.position.y + d * unit_w1.getY();
  h1.z = _current_pose.pose.position.z;

  geometry_msgs::Point h2;
  h2.x = _current_pose.pose.position.x + d * unit_w2.getX();
  h2.y = _current_pose.pose.position.y + d * unit_w2.getY();
  h2.z = _current_pose.pose.position.z;

  // ROS_INFO("error : %lf", error);
  // ROS_INFO("whether h1 on line : %lf", h1.y - (slope * h1.x + intercept));
  // ROS_INFO("whether h2 on line : %lf", h2.y - (slope * h2.x + intercept));

  // check which of two foot of a perpendicular line is on the line equation
  geometry_msgs::Point h;
  if (fabs(a * h1.x + b * h1.y + c) < ERROR)
  {
    h = h1;
    //   ROS_INFO("use h1");
  }
  else if (fabs(a * h2.x + b * h2.y + c) < ERROR)
  {
    //   ROS_INFO("use h2");
    h = h2;
  }
  else
  {
    return false;
  }

  // get intersection[s]
  // if there is a intersection
  if (d == search_radius)
  {
    *next_target = h;
    return true;
  }
  else
  {
    // if there are two intersection
    // get intersection in front of vehicle
    double s = sqrt(pow(search_radius, 2) - pow(d, 2));
    geometry_msgs::Point target1;
    target1.x = h.x + s * unit_v.getX();
    target1.y = h.y + s * unit_v.getY();
    target1.z = _current_pose.pose.position.z;

    geometry_msgs::Point target2;
    target2.x = h.x - s * unit_v.getX();
    target2.y = h.y - s * unit_v.getY();
    target2.z = _current_pose.pose.position.z;

    // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
    // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
    displayLinePoint(a, b, c, target1, target2, h);  // debug tool

    // check intersection is between end and start
    double interval = getPlaneDistance(end, start);
    if (getPlaneDistance(target1, end) < interval)
    {
      // ROS_INFO("result : target1");
      *next_target = target1;
      return true;
    }
    else if (getPlaneDistance(target2, end) < interval)
    {
      // ROS_INFO("result : target2");
      *next_target = target2;
      return true;
    }
    else
    {
      // ROS_INFO("result : false ");
      return false;
    }
  }
}

static bool verifyFollowing()
{
  double a = 0;
  double b = 0;
  double c = 0;
  getLinearEquation(_current_waypoints.getWaypointPosition(1), _current_waypoints.getWaypointPosition(2), &a, &b, &c);
  double displacement = getDistanceBetweenLineAndPoint(_current_pose.pose.position, a, b, c);
  double relative_angle = getRelativeAngle(_current_waypoints.getWaypointPose(1), _current_pose.pose);
  // ROS_INFO("side diff : %lf , angle diff : %lf",displacement,relative_angle);
  if (displacement < g_displacement_threshold || relative_angle < g_relative_angle_threshold)
  {
    // ROS_INFO("Following : True");
    return true;
  }
  else
  {
    // ROS_INFO("Following : False");
    return false;
  }
}
geometry_msgs::Twist calcTwist(double curvature, double cmd_velocity)
{
  // verify whether vehicle is following the path
  bool following_flag = verifyFollowing();
  static double prev_angular_velocity = 0;

  geometry_msgs::Twist twist;
  twist.linear.x = cmd_velocity;
  if (!following_flag)
  {
    twist.angular.z = _current_velocity * curvature;
  }
  else
  {
    twist.angular.z = prev_angular_velocity;
  }

  prev_angular_velocity = twist.angular.z;
  return twist;
}

int getNextWaypoint()
{
  int path_size = static_cast<int>(_current_waypoints.getSize());
  double lookahead_threshold = getLookAheadThreshold(0);

  // if waypoints are not given, do nothing.
  if (path_size == 0)
  {
    return -1;
  }

  // look for the next waypoint.
  for (int i = 0; i < path_size; i++)
  {
    // if search waypoint is the last
    if (i == (path_size - 1))
    {
      ROS_INFO("search waypoint is the last");
      return i;
    }

    // if there exists an effective waypoint
    if (getPlaneDistance(_current_waypoints.getWaypointPosition(i), _current_pose.pose.position) > lookahead_threshold)
    {
      return i;
    }
  }

  // if this program reaches here , it means we lost the waypoint!
  return -1;
}

// generate the locus of pure pursuit
std::vector<geometry_msgs::Point> generateTrajectoryCircle(geometry_msgs::Point target)
{
  std::vector<geometry_msgs::Point> traj_circle_array;
  double radius = calcRadius(target);

  for (double i = 0; i < M_PI / 2; i += 0.1)
  {
    // calc a point of circumference
    geometry_msgs::Point p;
    p.x = radius * cos(i);
    p.y = radius * sin(i);

    // transform to (radius,0)
    geometry_msgs::Point relative_p;
    relative_p.x = p.x - radius;
    relative_p.y = p.y;

    // rotate -90°
    geometry_msgs::Point rotate_p = rotatePoint(relative_p, -90);

    // transform to vehicle plane
    geometry_msgs::Point tf_p = calcAbsoluteCoordinate(rotate_p, _current_pose.pose);

    traj_circle_array.push_back(tf_p);
  }

  // reverse vector
  std::reverse(traj_circle_array.begin(), traj_circle_array.end());

  for (double i = 0; i > (-1) * M_PI / 2; i -= 0.1)
  {
    // calc a point of circumference
    geometry_msgs::Point p;
    p.x = radius * cos(i);
    p.y = radius * sin(i);

    // transform to (radius,0)
    geometry_msgs::Point relative_p;
    relative_p.x = p.x - radius;
    relative_p.y = p.y;

    // rotate -90°
    geometry_msgs::Point rotate_p = rotatePoint(relative_p, -90);

    // transform to vehicle plane
    geometry_msgs::Point tf_p = calcAbsoluteCoordinate(rotate_p, _current_pose.pose);

    traj_circle_array.push_back(tf_p);
  }

  return traj_circle_array;
}

void publishFailed()
{
  geometry_msgs::TwistStamped twist;
  std_msgs::Bool wf_stat;

  wf_stat.data = false;
  _stat_pub.publish(wf_stat);
  twist.twist.linear.x = 0;
  twist.twist.angular.z = 0;
  twist.header.stamp = ros::Time::now();
  g_cmd_velocity_publisher.publish(twist);
}
void publishSuccess(geometry_msgs::Twist t)
{
  std_msgs::Bool wf_stat;
  geometry_msgs::TwistStamped twist;
  wf_stat.data = true;
  _stat_pub.publish(wf_stat);
  twist.twist = t;
  twist.header.stamp = ros::Time::now();
  g_cmd_velocity_publisher.publish(twist);
  return;
}

void doPurePursuit()
{
  bool interpolate_flag = false;

  // search next waypoint
  int next_waypoint = getNextWaypoint();
  if (next_waypoint == -1)
  {
    ROS_INFO("lost next waypoint");
    publishFailed();
    return;
  }

  displayNextWaypoint(next_waypoint);
  displaySearchRadius(getLookAheadThreshold(0));

  geometry_msgs::Point next_target;
  // if g_linear_interpolate_mode is false or next waypoint is first or last
  if (!g_linear_interpolate_mode || next_waypoint == 0 ||
      next_waypoint == (static_cast<int>(_current_waypoints.getSize() - 1)))
  {
    next_target = _current_waypoints.getWaypointPosition(next_waypoint);
    publishSuccess(calcTwist(calcCurvature(next_target), getCmdVelocity(0)));
    return;
  }

  // linear interpolation and calculate angular velocity
  interpolate_flag = interpolateNextTarget(next_waypoint, &next_target);

  if (!interpolate_flag)
  {
    ROS_INFO_STREAM("lost target! ");
    publishFailed();
    return;
  }

  // ROS_INFO("next_target : ( %lf , %lf , %lf)", next_target.x, next_target.y,next_target.z);
  displayNextTarget(next_target);
  displayTrajectoryCircle(generateTrajectoryCircle(next_target));

  publishSuccess(calcTwist(calcCurvature(next_target), getCmdVelocity(0)));

// ROS_INFO("linear : %lf, angular : %lf",twist.twist.linear.x,twist.twist.angular.z);

#ifdef LOG
  std::ofstream ofs("/tmp/pure_pursuit.log", std::ios::app);
  ofs << _current_waypoints.getWaypointPosition(next_waypoint).x << " "
      << _current_waypoints.getWaypointPosition(next_waypoint).y << " " << next_target.x << " " << next_target.y
      << std::endl;
#endif
}

}  // namespace

int main(int argc, char **argv)
{
  ROS_INFO_STREAM("pure pursuit start");

  // set up ros
  ros::init(argc, argv, "pure_pursuit");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // setting params
  private_nh.getParam("linear_interpolate_mode", g_linear_interpolate_mode);
  ROS_INFO_STREAM("linear_interpolate_mode : " << g_linear_interpolate_mode);

  // publish topic
  g_cmd_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
  _vis_pub = nh.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
  _stat_pub = nh.advertise<std_msgs::Bool>("wf_stat", 0);
  _target_pub = nh.advertise<visualization_msgs::Marker>("next_target_mark", 0);
  _search_pub = nh.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
  _line_point_pub = nh.advertise<visualization_msgs::Marker>("line_point_mark", 0);  // debug tool
  _traj_circle_pub = nh.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);

  // subscribe topic
  ros::Subscriber waypoint_subcscriber = nh.subscribe("final_waypoints", 10, WayPointCallback);
  ros::Subscriber ndt_subscriber = nh.subscribe("current_pose", 10, callbackFromCurrentPose);

  ros::Subscriber config_subscriber = nh.subscribe("config/waypoint_follower", 10, ConfigCallback);
  ros::Subscriber est_twist_subscriber = nh.subscribe("current_velocity", 10, callbackFromCurrentVelocity);

  ros::Rate loop_rate(LOOP_RATE);  // by Hz
  while (ros::ok())
  {
    ros::spinOnce();

    // check topic
    if (!g_waypoint_set || !g_pose_set)
    {
      ROS_INFO_STREAM("topic waiting...");
      loop_rate.sleep();
      continue;
    }
    else
    {
      // ROS_INFO("topic subscribed!");
    }
    doPurePursuit();
    loop_rate.sleep();
  }

  return 0;
}
