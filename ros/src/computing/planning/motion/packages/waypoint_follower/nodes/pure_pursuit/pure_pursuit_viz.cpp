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

#include "pure_pursuit_viz.h"

namespace waypoint_follower
{
const std::string MAP_FRAME = "map";
// display the next waypoint by markers.
visualization_msgs::Marker displayNextWaypoint(geometry_msgs::Point position)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = MAP_FRAME;
  marker.header.stamp = ros::Time();
  marker.ns = "next_waypoint_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = position;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.frame_locked = true;
  return marker;
}

// display the next target by markers.
visualization_msgs::Marker displayNextTarget(geometry_msgs::Point target)
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
  return marker;
}

double calcRadius(geometry_msgs::Point target, geometry_msgs::Pose current_pose)
{
  double radius;
  double denominator = 2 * calcRelativeCoordinate(target, current_pose).y;
  double numerator = pow(getPlaneDistance(target, current_pose.position), 2);

  if (denominator != 0)
    radius = numerator / denominator;
  else
    radius = 0;

  // ROS_INFO("radius : %lf", radius);
  return radius;
}

// generate the locus of pure pursuit
std::vector<geometry_msgs::Point> generateTrajectoryCircle(geometry_msgs::Point target,
                                                           geometry_msgs::Pose current_pose)
{
  std::vector<geometry_msgs::Point> traj_circle_array;
  double radius = calcRadius(target, current_pose);
  double range = M_PI / 8;
  double increment = 0.01;

  for (double i = 0; i < range; i += increment)
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
    geometry_msgs::Point tf_p = calcAbsoluteCoordinate(rotate_p, current_pose);

    traj_circle_array.push_back(tf_p);
  }

  // reverse vector
  std::reverse(traj_circle_array.begin(), traj_circle_array.end());

  for (double i = 0; i > (-1) * range; i -= increment)
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
    geometry_msgs::Point tf_p = calcAbsoluteCoordinate(rotate_p, current_pose);

    traj_circle_array.push_back(tf_p);
  }

  return traj_circle_array;
}
// display the locus of pure pursuit by markers.
visualization_msgs::Marker displayTrajectoryCircle(std::vector<geometry_msgs::Point> traj_circle_array)
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
  return traj_circle;
}

// display the search radius by markers.
visualization_msgs::Marker displaySearchRadius(geometry_msgs::Point current_pose, double search_radius)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = MAP_FRAME;
  marker.header.stamp = ros::Time();
  marker.ns = "search_radius_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = current_pose;
  marker.scale.x = search_radius * 2;
  marker.scale.y = search_radius * 2;
  marker.scale.z = 1.0;
  marker.color.a = 0.1;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.frame_locked = true;
  return marker;
}
/*
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
 */
}
