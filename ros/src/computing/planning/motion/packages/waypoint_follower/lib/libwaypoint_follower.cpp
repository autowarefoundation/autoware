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

//set subscribed waypoint
void Path::setPath(const waypoint_follower::laneConstPtr &msg)
{
	current_path_ = *msg;
	//  std::cout << "waypoint subscribed" << std::endl;
	std::cout << current_path_.waypoints.size() << std::endl;
}

int Path::getPathSize()
{
	if (current_path_.waypoints.empty() == true)
		return 0;
	else
		return current_path_.waypoints.size();
}

double Path::getInterval()
{
  if(current_path_.waypoints.empty())
    return 0;

	//interval between 2 waypoints
	tf::Vector3 v1(current_path_.waypoints[0].pose.pose.position.x, current_path_.waypoints[0].pose.pose.position.y, 0);

	tf::Vector3 v2(current_path_.waypoints[1].pose.pose.position.x, current_path_.waypoints[1].pose.pose.position.y, 0);
	return tf::tfDistance(v1, v2);
}

/////////////////////////////////////////////////////////////////
// obtain the distance to @waypoint. ignore z position
/////////////////////////////////////////////////////////////////
double Path::getDistance(int waypoint)
{
	// position of @waypoint.
	tf::Vector3 tf_v = transformWaypoint(waypoint);
	tf_v.setZ(0);

	return tf::tfDistance(origin_v_, tf_v);
}

/////////////////////////////////////////////////////////////////
// transform the waypoint to the vehicle plane.
/////////////////////////////////////////////////////////////////
tf::Vector3 Path::transformWaypoint(int waypoint)
{

	tf::Vector3 point(current_path_.waypoints[waypoint].pose.pose.position.x, current_path_.waypoints[waypoint].pose.pose.position.y,
			current_path_.waypoints[waypoint].pose.pose.position.z);
	tf::Vector3 tf_point = transform_ * point;

	return tf_point;
}

geometry_msgs::Point Path::getWaypointPosition(int waypoint)
{
	geometry_msgs::Point p;
	p.x = current_path_.waypoints[waypoint].pose.pose.position.x;
	p.y = current_path_.waypoints[waypoint].pose.pose.position.y;
	p.z = current_path_.waypoints[waypoint].pose.pose.position.z;
	return p;
}

geometry_msgs::Quaternion Path::getWaypointOrientation(int waypoint)
{
	geometry_msgs::Quaternion q;
	q.x = current_path_.waypoints[waypoint].pose.pose.orientation.x;
	q.y = current_path_.waypoints[waypoint].pose.pose.orientation.y;
	q.z = current_path_.waypoints[waypoint].pose.pose.orientation.z;
	q.w = current_path_.waypoints[waypoint].pose.pose.orientation.w;
	return q;
}

int Path::getClosestWaypoint()
{
	// std::cout << "==GetClosestWaypoint==" << std::endl;

	if (!getPathSize())
		return -1;

	int closest_threshold = 5;

	//decide search radius
	for (int ratio = 1; ratio < closest_threshold; ratio++) {

		double distance_threshold = 2 * ratio * getInterval(); //meter
			//	std::cout << "distance_threshold : " << distance_threshold << std::endl;

		std::vector<int> waypoint_candidates;

		//search closest candidate
		for (int i = 1; i < getPathSize(); i++) {

			//std::cout << waypoint << std::endl;

			//skip waypoint behind vehicle
			if (transformWaypoint(i).x() < 0)
				continue;

			if (getDistance(i) < distance_threshold) {
				waypoint_candidates.push_back(i);
				//std::cout << "waypoint = " << i << "  distance = " << getDistance(i)  << std::endl;
			}
		}

		if (waypoint_candidates.size() == 0) {
			continue;
		}

		//search substraction minimum between candidate and previous closest
		int substraction_minimum = 0;
		int decided_waypoint = 0;
		int initial_minimum = 0;

		//decide initial minimum
		for (unsigned int i = 0; i < waypoint_candidates.size(); i++) {
			substraction_minimum = waypoint_candidates[i] - closest_waypoint_;
			if (substraction_minimum < 0)
				continue;

			if (substraction_minimum >= 0) {
				decided_waypoint = waypoint_candidates[i];
				initial_minimum = i;
				break;
			}
		}

		//calc closest
		for (unsigned int i = initial_minimum; i < waypoint_candidates.size(); i++) {
			int sub = waypoint_candidates[i] - closest_waypoint_;
			//std::cout << "closest candidates : " << waypoint_candidates[i] << " sub : " << sub << std::endl;

			if (sub < 0)
				continue;

			if (sub < substraction_minimum) {
				decided_waypoint = waypoint_candidates[i];
				substraction_minimum = sub;
			}
		}

		if (decided_waypoint >= closest_waypoint_) {
			closest_waypoint_ = decided_waypoint;
			return decided_waypoint;
		}

	}
	return -1;
}

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

  p.x = current_waypoints_.waypoints[waypoint].pose.pose.position.x;
  p.y = current_waypoints_.waypoints[waypoint].pose.pose.position.y;
  p.z = current_waypoints_.waypoints[waypoint].pose.pose.position.z;
  return p;
}

geometry_msgs::Quaternion WayPoints::getWaypointOrientation(int waypoint) const
{
  geometry_msgs::Quaternion q;
  if(waypoint > getSize() - 1)
    return q;

  q.x = current_waypoints_.waypoints[waypoint].pose.pose.orientation.x;
  q.y = current_waypoints_.waypoints[waypoint].pose.pose.orientation.y;
  q.z = current_waypoints_.waypoints[waypoint].pose.pose.orientation.z;
  q.w = current_waypoints_.waypoints[waypoint].pose.pose.orientation.w;
  return q;
}

double WayPoints::getWaypointVelocityMPS(int waypoint) const
{
  if(waypoint > getSize() - 1)
    return 0;

  return current_waypoints_.waypoints[waypoint].twist.twist.linear.x;
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

  if (!current_path.waypoints.size())
  {
    return -1;
  }

  int waypoint_min = -1;
  double distance_min = DBL_MAX;
  //get closest waypoint
  for (int i = 1; i < static_cast<int>(current_path.waypoints.size()); i++)
  {
    //skip waypoint behind vehicle
    double x = calcRelativeCoordinate(current_path.waypoints[i].pose.pose.position, current_pose).x;
    //ROS_INFO("waypoint = %d, x = %lf",i,x);
    if (x < 0)
      continue;

    //calc waypoint angle
    double waypoint_yaw;
    if (i == static_cast<int>(current_path.waypoints.size()) - 1)
    {
      waypoint_yaw = atan2(
          current_path.waypoints[i - 1].pose.pose.position.y - current_path.waypoints[i].pose.pose.position.y,
          current_path.waypoints[i - 1].pose.pose.position.x - current_path.waypoints[i].pose.pose.position.x);
      waypoint_yaw -= M_PI;
    }
    else
    {
      waypoint_yaw = atan2(
          current_path.waypoints[i + 1].pose.pose.position.y - current_path.waypoints[i].pose.pose.position.y,
          current_path.waypoints[i + 1].pose.pose.position.x - current_path.waypoints[i].pose.pose.position.x);
    }
    if (waypoint_yaw < 0)
      waypoint_yaw += 2 * M_PI;

    //calc pose angle
    tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
        current_pose.orientation.w);
    double dummy1, dummy2, pose_yaw;
    tf::Matrix3x3(q).getRPY(dummy1, dummy2, pose_yaw);
    if (pose_yaw < 0)
      pose_yaw += 2 * M_PI;

    //skip waypoint which direction is reverse against current_pose
    double direction_sub = (waypoint_yaw - pose_yaw) * 180 / M_PI; //degree
    //ROS_INFO("waypoint = %d, waypoint_yaw = %lf, pose_yaw = %lf, direction sub = %lf",i,waypoint_yaw,pose_yaw,direction_sub);
    if (fabs(direction_sub) > 90)
      continue;

    double distance = getPlaneDistance(current_path.waypoints[i].pose.pose.position, current_pose.position);
    //ROS_INFO("waypoint = %d , distance = %lf , distance_min = %lf",i,distance,distance_min);
    if (distance_min > distance)
    {

      //waypoint_min = el;
      waypoint_min = i;
      distance_min = distance;
    }
  }

  //ROS_INFO("waypoint = %d",waypoint_min);
  return waypoint_min;
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
