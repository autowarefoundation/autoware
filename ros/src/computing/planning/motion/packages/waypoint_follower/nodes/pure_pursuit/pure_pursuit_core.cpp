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

#include "pure_pursuit/pure_pursuit_core.h"

namespace waypoint_follower
{

void PurePursuit::callbackFromConfig(const runtime_manager::ConfigWaypointFollowerConstPtr &config)
{
  param_flag_ = config->param_flag;
  const_lookahead_distance_ = config->lookahead_distance;
  initial_velocity_ = config->velocity;
  lookahead_distance_calc_ratio_ = config->lookahead_ratio;
  minimum_lookahead_distance_ = config->minimum_lookahead_distance;
  displacement_threshold_ = config->displacement_threshold;
  relative_angle_threshold_ = config->relative_angle_threshold;
}

void PurePursuit::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose;
  pose_set_ = true;
}//processing frequency

void PurePursuit::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
  current_velocity_ = *msg;
  velocity_set_ = true;
}

void PurePursuit::callbackFromWayPoints(const waypoint_follower::laneConstPtr &msg)
{
  current_waypoints_.setPath(*msg);
  waypoint_set_ = true;
  // ROS_INFO_STREAM("waypoint subscribed");
}

double PurePursuit::getCmdVelocity(int waypoint) const
{
  if (param_flag_ == static_cast<int>(Mode::dialog))
  {
    ROS_INFO_STREAM("dialog : " << initial_velocity_ << " km/h (" << kmph2mps(initial_velocity_) << " m/s )");
    return kmph2mps(initial_velocity_);
  }

  if (current_waypoints_.isEmpty())
  {
    ROS_INFO_STREAM("waypoint : not loaded path");
    return 0;
  }

  double velocity = current_waypoints_.getWaypointVelocityMPS(waypoint);
  // ROS_INFO_STREAM("waypoint : " << mps2kmph(velocity) << " km/h ( " << velocity << "m/s )");
  return velocity;
}

void PurePursuit::calcLookaheadDistance(int waypoint)
{
  if (param_flag_ == static_cast<int>(Mode::dialog))
  {
    lookahead_distance_ = const_lookahead_distance_;
    return;
  }

  double current_velocity_mps = current_velocity_.twist.linear.x;
  double maximum_lookahead_distance =  current_velocity_mps * 10;
  double ld = current_velocity_mps * lookahead_distance_calc_ratio_;

  lookahead_distance_ = ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_
                      : ld > maximum_lookahead_distance ? maximum_lookahead_distance
                      : ld ;

  ROS_INFO("lookahead distance: %f",lookahead_distance_);

  return ;
}

double PurePursuit::calcCurvature(geometry_msgs::Point target) const
{
  double kappa;
  double denominator = pow(getPlaneDistance(target, current_pose_.pose.position), 2);
  double numerator = 2 * calcRelativeCoordinate(target, current_pose_.pose).y;

  if (denominator != 0)
    kappa = numerator / denominator;
  else
  {
    if(numerator > 0)
     kappa = KAPPA_MIN_;
    else
      kappa = -KAPPA_MIN_;
  }
  ROS_INFO("kappa : %lf", kappa);
  return kappa;
}

// linear interpolation of next target
bool PurePursuit::interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target) const
{
  constexpr double ERROR = pow(10, -5);  // 0.00001

  int path_size = static_cast<int>(current_waypoints_.getSize());
  if (next_waypoint == path_size - 1)
  {
    *next_target = current_waypoints_.getWaypointPosition(next_waypoint);
    return true;
  }
  double search_radius = lookahead_distance_;
  geometry_msgs::Point zero_p;
  geometry_msgs::Point end = current_waypoints_.getWaypointPosition(next_waypoint);
  geometry_msgs::Point start = current_waypoints_.getWaypointPosition(next_waypoint - 1);

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
  double d = getDistanceBetweenLineAndPoint(current_pose_.pose.position, a, b, c);

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
  h1.x = current_pose_.pose.position.x + d * unit_w1.getX();
  h1.y = current_pose_.pose.position.y + d * unit_w1.getY();
  h1.z = current_pose_.pose.position.z;

  geometry_msgs::Point h2;
  h2.x = current_pose_.pose.position.x + d * unit_w2.getX();
  h2.y = current_pose_.pose.position.y + d * unit_w2.getY();
  h2.z = current_pose_.pose.position.z;

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
    target1.z = current_pose_.pose.position.z;

    geometry_msgs::Point target2;
    target2.x = h.x - s * unit_v.getX();
    target2.y = h.y - s * unit_v.getY();
    target2.z = current_pose_.pose.position.z;

    // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
    // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
    //displayLinePoint(a, b, c, target1, target2, h);  // debug tool

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

bool PurePursuit::verifyFollowing() const
{
  double a = 0;
  double b = 0;
  double c = 0;
  getLinearEquation(current_waypoints_.getWaypointPosition(1), current_waypoints_.getWaypointPosition(2), &a, &b, &c);
  double displacement = getDistanceBetweenLineAndPoint(current_pose_.pose.position, a, b, c);
  double relative_angle = getRelativeAngle(current_waypoints_.getWaypointPose(1), current_pose_.pose);
  // ROS_INFO("side diff : %lf , angle diff : %lf",displacement,relative_angle);
  if (displacement < displacement_threshold_ || relative_angle < relative_angle_threshold_)
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
geometry_msgs::Twist PurePursuit::calcTwist(double curvature, double cmd_velocity) const
{
  // verify whether vehicle is following the path
  bool following_flag = verifyFollowing();
  static double prev_angular_velocity = 0;

  geometry_msgs::Twist twist;
  twist.linear.x = cmd_velocity;
  if (!following_flag)
  {
    twist.angular.z = current_velocity_.twist.linear.x * curvature;
  }
  else
  {
    twist.angular.z = prev_angular_velocity;
  }

  prev_angular_velocity = twist.angular.z;
  return twist;
}

void PurePursuit::getNextWaypoint()
{
  int path_size = static_cast<int>(current_waypoints_.getSize());

  // if waypoints are not given, do nothing.
  if (path_size == 0)
  {
    num_of_next_waypoint_ = -1;
    return;
  }

  // look for the next waypoint.
  for (int i = 0; i < path_size; i++)
  {
    // if search waypoint is the last
    if (i == (path_size - 1))
    {
      ROS_INFO("search waypoint is the last");
      num_of_next_waypoint_ = i;
      return;
    }

    // if there exists an effective waypoint
    if (getPlaneDistance(current_waypoints_.getWaypointPosition(i), current_pose_.pose.position) > lookahead_distance_)
    {
      num_of_next_waypoint_ = i;
      return;
    }
  }

  // if this program reaches here , it means we lost the waypoint!
  num_of_next_waypoint_ = -1;
  return;
}

geometry_msgs::TwistStamped PurePursuit::outputZero() const
{
  geometry_msgs::TwistStamped twist;
  twist.twist.linear.x = 0;
  twist.twist.angular.z = 0;
  twist.header.stamp = ros::Time::now();
  return twist;
}
geometry_msgs::TwistStamped PurePursuit::outputTwist(geometry_msgs::Twist t) const
{
  geometry_msgs::TwistStamped twist;
  twist.twist = t;
  twist.header.stamp = ros::Time::now();
  return twist;
}

geometry_msgs::TwistStamped PurePursuit::go()
{
  if(!pose_set_ || !waypoint_set_ || !velocity_set_){
    ROS_INFO("somethins is missing... ");
    return outputZero();

  }

  bool interpolate_flag = false;

  calcLookaheadDistance(1);
  // search next waypoint
  getNextWaypoint();
  if (num_of_next_waypoint_ == -1)
  {
    ROS_INFO("lost next waypoint");
    return outputZero();
  }

  // if g_linear_interpolate_mode is false or next waypoint is first or last
  if (!linear_interpolate_ || num_of_next_waypoint_ == 0 ||
      num_of_next_waypoint_ == (static_cast<int>(current_waypoints_.getSize() - 1)))
  {
    position_of_next_target_ = current_waypoints_.getWaypointPosition(num_of_next_waypoint_);
    return outputTwist(calcTwist(calcCurvature(position_of_next_target_), getCmdVelocity(0)));
  }

  // linear interpolation and calculate angular velocity
  interpolate_flag = interpolateNextTarget(num_of_next_waypoint_, &position_of_next_target_);

  if (!interpolate_flag)
  {
    ROS_INFO_STREAM("lost target! ");
    return outputZero();
  }

  // ROS_INFO("next_target : ( %lf , %lf , %lf)", next_target.x, next_target.y,next_target.z);

  return outputTwist(calcTwist(calcCurvature(position_of_next_target_), getCmdVelocity(0)));

// ROS_INFO("linear : %lf, angular : %lf",twist.twist.linear.x,twist.twist.angular.z);

#ifdef LOG
  std::ofstream ofs("/tmp/pure_pursuit.log", std::ios::app);
  ofs << _current_waypoints.getWaypointPosition(next_waypoint).x << " "
      << _current_waypoints.getWaypointPosition(next_waypoint).y << " " << next_target.x << " " << next_target.y
      << std::endl;
#endif
}
}
