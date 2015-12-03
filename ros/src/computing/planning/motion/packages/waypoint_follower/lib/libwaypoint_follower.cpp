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

#include "waypoint_follower/libwaypoint_follower.h"

int WayPoints::getSize() const
{
  if (current_waypoints_.waypoints.empty())
    return 0;
  else
    return current_waypoints_.waypoints.size();
}

double WayPoints::getInterval() const
{
  if (current_waypoints_.waypoints.empty())
    return 0;

  //interval between 2 waypoints
  tf::Vector3 v1(current_waypoints_.waypoints[0].pose.pose.position.x,
      current_waypoints_.waypoints[0].pose.pose.position.y, 0);

  tf::Vector3 v2(current_waypoints_.waypoints[1].pose.pose.position.x,
      current_waypoints_.waypoints[1].pose.pose.position.y, 0);
  return tf::tfDistance(v1, v2);
}

geometry_msgs::Point WayPoints::getWaypointPosition(int waypoint) const
{
  geometry_msgs::Point p;
  if(waypoint > getSize() - 1)
    return p;

  p = current_waypoints_.waypoints[waypoint].pose.pose.position;
  return p;
}

geometry_msgs::Quaternion WayPoints::getWaypointOrientation(int waypoint) const
{
  geometry_msgs::Quaternion q;
  if(waypoint > getSize() - 1)
    return q;

  //q = current_waypoints_.waypoints[waypoint].pose.pose.orientation;

  double waypoint_yaw;
  if (waypoint == static_cast<int>(current_waypoints_.waypoints.size()) - 1)
  {
    waypoint_yaw = atan2(
        current_waypoints_.waypoints[waypoint - 1].pose.pose.position.y - current_waypoints_.waypoints[waypoint].pose.pose.position.y,
        current_waypoints_.waypoints[waypoint - 1].pose.pose.position.x - current_waypoints_.waypoints[waypoint].pose.pose.position.x);
    waypoint_yaw -= M_PI;
  }
  else
  {
    waypoint_yaw = atan2(
        current_waypoints_.waypoints[waypoint + 1].pose.pose.position.y - current_waypoints_.waypoints[waypoint].pose.pose.position.y,
        current_waypoints_.waypoints[waypoint + 1].pose.pose.position.x - current_waypoints_.waypoints[waypoint].pose.pose.position.x);
  }
  q = tf::createQuaternionMsgFromYaw (waypoint_yaw);

  return q;
}

double WayPoints::getWaypointVelocityMPS(int waypoint) const
{
  if(waypoint > getSize() - 1)
    return 0;

  return current_waypoints_.waypoints[waypoint].twist.twist.linear.x;
}

bool WayPoints::isFront(int waypoint, geometry_msgs::Pose current_pose) const
{
  double x = calcRelativeCoordinate(current_waypoints_.waypoints[waypoint].pose.pose.position, current_pose).x;
  if (x < 0)
    return false;
  else
    return true;
}

bool WayPoints::isValid(int waypoint, geometry_msgs::Pose current_pose) const
{
  double angle_threshold = 90;
  tf::Vector3 waypoint_v(getWaypointPosition(waypoint).x,getWaypointPosition(waypoint).y,getWaypointPosition(waypoint).z);
  tf::Vector3 pose_v(current_pose.position.x,current_pose.position.y,current_pose.position.z);
  double angle = waypoint_v.angle(pose_v) * 180 / M_PI; //degree
  ROS_INFO("angle : %lf",angle);

  if (fabs(angle) > angle_threshold)
    return false;
  else
    return true;

}

double DecelerateVelocity(double distance, double prev_velocity)
{

  double decel_ms = 1.0; // m/s
  double decel_velocity_ms = sqrt(2 * decel_ms * distance);

  std::cout << "velocity/prev_velocity :" << decel_velocity_ms << "/" << prev_velocity << std::endl;
  if (decel_velocity_ms < prev_velocity)
  {
    return decel_velocity_ms;
  }
  else
  {
    return prev_velocity;
  }

}

//calculation relative coordinate of point from current_pose frame
geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point, geometry_msgs::Pose current_pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(current_pose, inverse);
  tf::Transform transform = inverse.inverse();

  tf::Vector3 v = point2vector(point);
  tf::Vector3 tf_v = transform * v;

  return vector2point(tf_v);
}

//calculation absolute coordinate of point on current_pose frame
geometry_msgs::Point calcAbsoluteCoordinate(geometry_msgs::Point point, geometry_msgs::Pose current_pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(current_pose, inverse);

  tf::Vector3 v = point2vector(point);
  tf::Vector3 tf_v = inverse * v;

  return vector2point(tf_v);
}

//distance between target 1 and target2 in 2-D
double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2)
{
  tf::Vector3 v1 = point2vector(target1);
  v1.setZ(0);
  tf::Vector3 v2 = point2vector(target2);
  v2.setZ(0);
  return tf::tfDistance(v1, v2);
}



//get closest waypoint from current pose
int getClosestWaypoint(const waypoint_follower::lane &current_path, geometry_msgs::Pose current_pose)
{
  WayPoints wp;
  wp.setPath(current_path);

  if (wp.isEmpty())
    return -1;

  //search closest candidate within a certain meter
  double search_distance = 5.0;
  std::vector<int> waypoint_candidates;
  for (int i = 1; i < wp.getSize(); i++)
  {
    if (getPlaneDistance(wp.getWaypointPosition(i), current_pose.position) > search_distance)
      continue;

    if (!wp.isFront(i, current_pose))
      continue;

    if(!wp.isValid(i,current_pose))
      continue;

    waypoint_candidates.push_back(i);
  }

  //get closest waypoint from candidates
  if (!waypoint_candidates.empty())
  {
    int waypoint_min = -1;
    double distance_min = DBL_MAX;
    for (auto el :waypoint_candidates)
    {
      //ROS_INFO("closest_candidates : %d",el);
      double d = getPlaneDistance(wp.getWaypointPosition(el), current_pose.position);
      if (d < distance_min)
      {
        waypoint_min = el;
        distance_min = d;
      }
    }
    return waypoint_min;

  }
  else
  {
    ROS_INFO("no candidate. search closest waypoint from all waypoints...");
    //if there is no candidate...
    int waypoint_min = -1;
    double distance_min = DBL_MAX;
    for (int i = 1; i < wp.getSize(); i++)
    {
      if (!wp.isFront(i, current_pose))
        continue;

      //if (!wp.isValid(i, current_pose))
      //  continue;

      double d = getPlaneDistance(wp.getWaypointPosition(i), current_pose.position);
      if (d < distance_min)
      {
        waypoint_min = i;
        distance_min = d;
      }
    }
    return waypoint_min;


  }

}

bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *slope, double *intercept)
{
  if ((start.x - end.x) == 0)
    return false;

  //get slope of segment end,start
  *slope = (start.y - end.y) / (start.x - end.x);

  //get intercept of segment end,start
  *intercept = (-1) * (*slope) * end.x + end.y;

  return true;
}

tf::Vector3 point2vector(geometry_msgs::Point point)
{
  tf::Vector3 vector(point.x, point.y, point.z);
  return vector;
}

geometry_msgs::Point vector2point(tf::Vector3 vector)
{
  geometry_msgs::Point point;
  point.x = vector.getX();
  point.y = vector.getY();
  point.z = vector.getZ();
  return point;
}

tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree)
{
  tf::Vector3 w1(cos(deg2rad(degree)) * unit_vector.getX() - sin(deg2rad(degree)) * unit_vector.getY(),
      sin(deg2rad(degree)) * unit_vector.getX() + cos(deg2rad(degree)) * unit_vector.getY(), 0);
  tf::Vector3 unit_w1 = w1.normalize();

  return unit_w1;
}

geometry_msgs::Point rotatePoint(geometry_msgs::Point point, double degree)
{
  geometry_msgs::Point rotate;
  rotate.x = cos(deg2rad(degree)) * point.x - sin(deg2rad(degree)) * point.y;
  rotate.y = sin(deg2rad(degree)) * point.x + cos(deg2rad(degree)) * point.y;

  return rotate;
}
