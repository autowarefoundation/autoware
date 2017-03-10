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
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <runtime_manager/ConfigLatticeVelocitySet.h>
#include <iostream>

#include "waypoint_follower/lane.h"
#include "waypoint_follower/libwaypoint_follower.h"
#include "libvelocity_set.h"

namespace
{

const int LOOP_RATE = 10;

geometry_msgs::TwistStamped g_current_twist;
geometry_msgs::PoseStamped g_localizer_pose;  // pose of sensor
geometry_msgs::PoseStamped g_control_pose;  // pose of base_link
pcl::PointCloud<pcl::PointXYZ> g_vscan;

const std::string pedestrian_sound = "pedestrian";
bool g_pose_flag = false;
bool g_path_flag = false;
bool g_vscan_flag = false;
int g_obstacle_waypoint = -1;
double g_deceleration_search_distance = 30;
double g_search_distance = 60;
int g_closest_waypoint = -1;
double g_current_vel = 0.0;  // (m/s) subscribe estimated_vel
CrossWalk vmap;
ObstaclePoints g_obstacle;

/* Config Parameter */
double g_detection_range = 0;                   // if obstacle is in this range, stop
double g_deceleration_range = 1.8;              // if obstacle is in this range, decelerate
int g_threshold_points = 15;
double g_detection_height_top = 2.0;  // actually +2.0m
double g_detection_height_bottom = -2.0;
double g_others_distance = 8.0;            // meter: stopping distance from obstacles (using VSCAN)
double g_decel = 1.5;                      // (m/s) deceleration
double g_velocity_change_limit = 2.778;    // (m/s) about 10 km/h
double g_temporal_waypoints_size = 100.0;  // meter

// Publisher
ros::Publisher g_range_pub;
ros::Publisher g_deceleration_range_pub;
ros::Publisher g_sound_pub;
ros::Publisher g_safety_waypoint_pub;
ros::Publisher g_temporal_waypoints_pub;
ros::Publisher g_crosswalk_points_pub;
ros::Publisher g_obstacle_pub;

WayPoints g_path_dk;

class PathVset : public WayPoints
{
private:
  waypoint_follower::lane temporal_waypoints_;

public:
  void changeWaypoints(int stop_waypoint);
  void avoidSuddenBraking();
  void avoidSuddenAceleration();
  void setDeceleration();
  bool checkWaypoint(int num, const char *name) const;
  void setTemporalWaypoints();
  waypoint_follower::lane getTemporalWaypoints() const
  {
    return temporal_waypoints_;
  }
};
PathVset g_path_change;

//===============================
//       class function
//===============================

// check if waypoint number is valid
bool PathVset::checkWaypoint(int num, const char *name) const
{
  if (num < 0 || num >= getSize())
  {
    return false;
  }
  return true;
}

// set about '_temporal_waypoints_size' meter waypoints from closest waypoint
void PathVset::setTemporalWaypoints()
{
  if (g_closest_waypoint < 0)
    return;
  int size = (int)(g_temporal_waypoints_size / getInterval()) + 1;

  temporal_waypoints_.waypoints.clear();
  temporal_waypoints_.header = current_waypoints_.header;
  temporal_waypoints_.increment = current_waypoints_.increment;
  // push current pose
  waypoint_follower::waypoint current_point;

  current_point.pose = g_control_pose;
  current_point.twist = current_waypoints_.waypoints[g_closest_waypoint].twist;
  current_point.dtlane = current_waypoints_.waypoints[g_closest_waypoint].dtlane;
  temporal_waypoints_.waypoints.push_back(current_point);
  for (int i = 0; i < size; i++)
  {
    if (g_closest_waypoint + i >= getSize())
      return;
    temporal_waypoints_.waypoints.push_back(current_waypoints_.waypoints[g_closest_waypoint + i]);
  }

  return;
}

void PathVset::setDeceleration()
{
  int velocity_change_range = 5;
  double intervel = getInterval();
  double temp1 = g_current_vel * g_current_vel;
  double temp2 = 2 * g_decel * intervel;
  double deceleration_minimum = kmph2mps(4.0);

  for (int i = 0; i < velocity_change_range; i++)
  {
    if (!checkWaypoint(g_closest_waypoint + i, "setDeceleration"))
      continue;
    double waypoint_velocity = current_waypoints_.waypoints[g_closest_waypoint + i].twist.twist.linear.x;
    double changed_vel = temp1 - temp2;
    if (changed_vel < 0)
    {
      changed_vel = deceleration_minimum * deceleration_minimum;
    }
    if (sqrt(changed_vel) > waypoint_velocity || deceleration_minimum > waypoint_velocity)
      continue;
    if (sqrt(changed_vel) < deceleration_minimum)
    {
      current_waypoints_.waypoints[g_closest_waypoint + i].twist.twist.linear.x = deceleration_minimum;
      continue;
    }
    current_waypoints_.waypoints[g_closest_waypoint + i].twist.twist.linear.x = sqrt(changed_vel);
  }

  return;
}

void PathVset::avoidSuddenAceleration()
{
  double changed_vel;
  double interval = getInterval();
  double temp1 = g_current_vel * g_current_vel;
  double temp2 = 2 * g_decel * interval;
  double velocity_offset = 1.389; // m/s

  for (int i = 0;; i++)
  {
    if (!checkWaypoint(g_closest_waypoint + i, "avoidSuddenAceleration"))
      return;
    changed_vel = sqrt(temp1 + temp2 * (double)(i + 1)) + velocity_offset;
    if (changed_vel > current_waypoints_.waypoints[g_closest_waypoint + i].twist.twist.linear.x)
      return;
    current_waypoints_.waypoints[g_closest_waypoint + i].twist.twist.linear.x = changed_vel;
  }

  return;
}

void PathVset::avoidSuddenBraking()
{
  int i = 0;
  int fill_in_zero = 20;
  int fill_in_vel = 15;
  int examin_range = 1;  // need to change according to waypoint interval?
  int num;
  double interval = getInterval();
  double changed_vel;

  for (int j = -1; j < examin_range; j++)
  {
    if (!checkWaypoint(g_closest_waypoint + j, "avoidSuddenBraking"))
      return;
    if (getWaypointVelocityMPS(g_closest_waypoint + j) <
        g_current_vel - g_velocity_change_limit)  // we must change waypoints
      break;
    if (j == examin_range - 1)  // we don't have to change waypoints
      return;
  }

  // fill in waypoints velocity behind vehicle
  for (num = g_closest_waypoint - 1; fill_in_vel > 0; fill_in_vel--)
  {
    if (!checkWaypoint(num - fill_in_vel, "avoidSuddenBraking"))
      continue;
    current_waypoints_.waypoints[num - fill_in_vel].twist.twist.linear.x = g_current_vel;
  }

  // decelerate gradually
  double temp1 = (g_current_vel - g_velocity_change_limit + 1.389) * (g_current_vel - g_velocity_change_limit + 1.389);
  double temp2 = 2 * g_decel * interval;
  for (num = g_closest_waypoint - 1;; num++)
  {
    if (num >= getSize())
      return;
    if (!checkWaypoint(num, "avoidSuddenBraking"))
      continue;
    changed_vel = temp1 - temp2 * (double)i;  // sqrt(v^2 - 2*a*x)
    if (changed_vel <= 0)
      break;
    current_waypoints_.waypoints[num].twist.twist.linear.x = sqrt(changed_vel);

    i++;
  }

  for (int j = 0; j < fill_in_zero; j++)
  {
    if (!checkWaypoint(num + j, "avoidSuddenBraking"))
      continue;
    current_waypoints_.waypoints[num + j].twist.twist.linear.x = 0.0;
  }


  return;
}

void PathVset::changeWaypoints(int stop_waypoint)
{
  int i = 0;
  int close_waypoint_threshold = 4;
  int fill_in_zero = 20;
  double changed_vel;
  double interval = getInterval();

  // change waypoints to decelerate
  for (int num = stop_waypoint; num > g_closest_waypoint - close_waypoint_threshold; num--)
  {
    if (!checkWaypoint(num, "changeWaypoints"))
      continue;

    changed_vel = sqrt(2.0 * g_decel * (interval * i));  // sqrt(2*a*x)

    waypoint_follower::waypoint initial_waypoint = g_path_dk.getCurrentWaypoints().waypoints[num];
    if (changed_vel > initial_waypoint.twist.twist.linear.x)
    {  // avoid acceleration
      current_waypoints_.waypoints[num].twist.twist.linear.x = initial_waypoint.twist.twist.linear.x;
    }
    else
    {
      current_waypoints_.waypoints[num].twist.twist.linear.x = changed_vel;
    }

    i++;
  }

  // fill in 0
  for (int j = 1; j < fill_in_zero; j++)
  {
    if (!checkWaypoint(stop_waypoint + j, "changeWaypoints"))
      continue;
    current_waypoints_.waypoints[stop_waypoint + j].twist.twist.linear.x = 0.0;
  }


  return;
}

//===============================
//       class function
//===============================

//===============================
//          Callback
//===============================

void configCallback(const runtime_manager::ConfigLatticeVelocitySetConstPtr &config)
{
  g_others_distance = config->others_distance;
  g_detection_range = config->detection_range;
  g_threshold_points = config->threshold_points;
  g_detection_height_top = config->detection_height_top;
  g_detection_height_bottom = config->detection_height_bottom;
  g_decel = config->deceleration;
  g_velocity_change_limit = kmph2mps(config->velocity_change_limit);
  g_deceleration_range = config->deceleration_range;
  g_temporal_waypoints_size = config->temporal_waypoints_size;
}

void currentVelCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
  g_current_vel = msg->twist.linear.x;
}

void baseWaypointCallback(const waypoint_follower::laneConstPtr &msg)
{
  g_path_dk.setPath(*msg);
  g_path_change.setPath(*msg);
  if (g_path_flag == false)
  {
    g_path_flag = true;
  }
}

void objPoseCallback(const visualization_msgs::MarkerConstPtr &msg)
{
  //ROS_INFO("subscribed obj_pose\n");
}

void vscanCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ> vscan_raw;
  pcl::fromROSMsg(*msg, vscan_raw);

  g_vscan.clear();
  for (const auto &v : vscan_raw)
  {
    if (v.x == 0 && v.y == 0)
      continue;
    if (v.z > g_detection_height_top || v.z < g_detection_height_bottom)
      continue;
    g_vscan.push_back(v);
  }

  if (g_vscan_flag == false)
  {
    g_vscan_flag = true;
  }
}

void controlCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (!g_pose_flag)
    g_pose_flag = true;

  g_control_pose.header = msg->header;
  g_control_pose.pose = msg->pose;
}

void localizerCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  g_localizer_pose.header = msg->header;
  g_localizer_pose.pose = msg->pose;
}


//===============================
//          Callback
//===============================

void displayObstacle(const EControl &kind)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = g_obstacle.getObstaclePoint(kind);
  if (kind == OTHERS)
    marker.pose.position = g_obstacle.getPreviousDetection();
  marker.pose.orientation = g_localizer_pose.pose.orientation;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 2.0;
  marker.color.a = 0.7;
  if (kind == STOP)
  {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  else
  {
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }
  marker.lifetime = ros::Duration(0.1);
  marker.frame_locked = true;

  g_obstacle_pub.publish(marker);
}

void displayDetectionRange(const int &crosswalk_id, const int &num, const EControl &kind)
{
  // set up for marker array
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker crosswalk_marker;
  visualization_msgs::Marker waypoint_marker_stop;
  visualization_msgs::Marker waypoint_marker_decelerate;
  visualization_msgs::Marker stop_line;
  crosswalk_marker.header.frame_id = "/map";
  crosswalk_marker.header.stamp = ros::Time();
  crosswalk_marker.id = 0;
  crosswalk_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  crosswalk_marker.action = visualization_msgs::Marker::ADD;
  waypoint_marker_stop = crosswalk_marker;
  waypoint_marker_decelerate = crosswalk_marker;
  stop_line = crosswalk_marker;
  stop_line.type = visualization_msgs::Marker::CUBE;

  // set each namespace
  crosswalk_marker.ns = "Crosswalk Detection";
  waypoint_marker_stop.ns = "Stop Detection";
  waypoint_marker_decelerate.ns = "Decelerate Detection";
  stop_line.ns = "Stop Line";

  // set scale and color
  double scale = 2 * g_detection_range;
  waypoint_marker_stop.scale.x = scale;
  waypoint_marker_stop.scale.y = scale;
  waypoint_marker_stop.scale.z = scale;
  waypoint_marker_stop.color.a = 0.2;
  waypoint_marker_stop.color.r = 0.0;
  waypoint_marker_stop.color.g = 1.0;
  waypoint_marker_stop.color.b = 0.0;
  waypoint_marker_stop.frame_locked = true;

  scale = 2 * (g_detection_range + g_deceleration_range);
  waypoint_marker_decelerate.scale.x = scale;
  waypoint_marker_decelerate.scale.y = scale;
  waypoint_marker_decelerate.scale.z = scale;
  waypoint_marker_decelerate.color.a = 0.15;
  waypoint_marker_decelerate.color.r = 1.0;
  waypoint_marker_decelerate.color.g = 1.0;
  waypoint_marker_decelerate.color.b = 0.0;
  waypoint_marker_decelerate.frame_locked = true;

  if (g_obstacle_waypoint > -1)
  {
    stop_line.pose.position = g_path_dk.getWaypointPosition(g_obstacle_waypoint);
    stop_line.pose.orientation = g_path_dk.getWaypointOrientation(g_obstacle_waypoint);
  }
  stop_line.pose.position.z += 1.0;
  stop_line.scale.x = 0.1;
  stop_line.scale.y = 15.0;
  stop_line.scale.z = 2.0;
  stop_line.color.a = 0.3;
  stop_line.color.r = 1.0;
  stop_line.color.g = 0.0;
  stop_line.color.b = 0.0;
  stop_line.lifetime = ros::Duration(0.1);
  stop_line.frame_locked = true;

  if (crosswalk_id > 0)
    scale = vmap.getDetectionPoints(crosswalk_id).width;
  crosswalk_marker.scale.x = scale;
  crosswalk_marker.scale.y = scale;
  crosswalk_marker.scale.z = scale;
  crosswalk_marker.color.a = 0.5;
  crosswalk_marker.color.r = 0.0;
  crosswalk_marker.color.g = 1.0;
  crosswalk_marker.color.b = 0.0;
  crosswalk_marker.frame_locked = true;

  // set marker points coordinate
  for (int i = 0; i < g_search_distance; i++)
  {
    if (num < 0 || i + num > g_path_dk.getSize() - 1)
      break;

    geometry_msgs::Point point;
    point = g_path_dk.getWaypointPosition(num + i);

    waypoint_marker_stop.points.push_back(point);

    if (i > g_deceleration_search_distance)
      continue;
    waypoint_marker_decelerate.points.push_back(point);
  }

  if (crosswalk_id > 0)
  {
    for (const auto &p : vmap.getDetectionPoints(crosswalk_id).points)
      crosswalk_marker.points.push_back(p);
  }

  // publish marker
  marker_array.markers.push_back(crosswalk_marker);
  marker_array.markers.push_back(waypoint_marker_stop);
  marker_array.markers.push_back(waypoint_marker_decelerate);
  if (kind == STOP)
    marker_array.markers.push_back(stop_line);
  g_range_pub.publish(marker_array);
  marker_array.markers.clear();
}

int findCrossWalk()
{
  if (!vmap.set_points || g_closest_waypoint < 0)
    return -1;

  double find_distance = 2.0 * 2.0;      // meter
  double ignore_distance = 20.0 * 20.0;  // meter
  static std::vector<int> bdid = vmap.getBDID();
  // Find near cross walk
  for (int num = g_closest_waypoint; num < g_closest_waypoint + g_search_distance; num++)
  {
    geometry_msgs::Point waypoint = g_path_dk.getWaypointPosition(num);
    waypoint.z = 0.0;  // ignore Z axis
    for (const auto &i : bdid)
    {
      // ignore far crosswalk
      geometry_msgs::Point crosswalk_center = vmap.getDetectionPoints(i).center;
      crosswalk_center.z = 0.0;
      if (calcSquareOfLength(crosswalk_center, waypoint) > ignore_distance)
        continue;

      for (auto p : vmap.getDetectionPoints(i).points)
      {
        p.z = waypoint.z;
        if (calcSquareOfLength(p, waypoint) < find_distance)
        {
          vmap.setDetectionCrossWalkID(i);
          return num;
        }
      }
    }
  }

  vmap.setDetectionCrossWalkID(-1);
  return -1;  // no near crosswalk
}

EControl crossWalkDetection(const int &crosswalk_id)
{
  double search_radius = vmap.getDetectionPoints(crosswalk_id).width / 2;

  // Search each calculated points in the crosswalk
  for (const auto &p : vmap.getDetectionPoints(crosswalk_id).points)
  {
    geometry_msgs::Point detection_point = calcRelativeCoordinate(p, g_localizer_pose.pose);
    tf::Vector3 detection_vector = point2vector(detection_point);
    detection_vector.setZ(0.0);

    int stop_count = 0;  // the number of points in the detection area
    for (const auto &vscan : g_vscan)
    {
      tf::Vector3 vscan_vector(vscan.x, vscan.y, 0.0);
      double distance = tf::tfDistance(vscan_vector, detection_vector);
      if (distance < search_radius)
      {
        stop_count++;
        geometry_msgs::Point vscan_temp;
        vscan_temp.x = vscan.x;
        vscan_temp.y = vscan.y;
        vscan_temp.z = vscan.z;
	g_obstacle.setStopPoint(calcAbsoluteCoordinate(vscan_temp, g_localizer_pose.pose));
      }
      if (stop_count > g_threshold_points)
        return STOP;
    }

    g_obstacle.clearStopPoints();
  }

  return KEEP;  // find no obstacles
}

EControl vscanDetection()
{
  if (g_vscan.empty() == true || g_closest_waypoint < 0)
    return KEEP;

  int decelerate_or_stop = -10000;
  int decelerate2stop_waypoints = 15;

  for (int i = g_closest_waypoint; i < g_closest_waypoint + g_search_distance; i++)
  {
    g_obstacle.clearStopPoints();
    if (!g_obstacle.isDecided())
      g_obstacle.clearDeceleratePoints();

    decelerate_or_stop++;
    if (decelerate_or_stop > decelerate2stop_waypoints || (decelerate_or_stop >= 0 && i >= g_path_dk.getSize() - 1) ||
        (decelerate_or_stop >= 0 && i == g_closest_waypoint + g_search_distance - 1))
      return DECELERATE;
    if (i > g_path_dk.getSize() - 1)
      return KEEP;

    // Detection for cross walk
    if (i == vmap.getDetectionWaypoint())
    {
      if (crossWalkDetection(vmap.getDetectionCrossWalkID()) == STOP)
      {
        g_obstacle_waypoint = i;
        return STOP;
      }
    }

    // waypoint seen by vehicle
    geometry_msgs::Point waypoint = calcRelativeCoordinate(g_path_dk.getWaypointPosition(i), g_localizer_pose.pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    int stop_point_count = 0;
    int decelerate_point_count = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = g_vscan.begin(); item != g_vscan.end(); item++)
    {
      tf::Vector3 vscan_vector((double)item->x, (double)item->y, 0);

      // 2D distance between waypoint and vscan points(obstacle)
      // ---STOP OBSTACLE DETECTION---
      double dt = tf::tfDistance(vscan_vector, tf_waypoint);
      if (dt < g_detection_range)
      {
        stop_point_count++;
        geometry_msgs::Point vscan_temp;
        vscan_temp.x = item->x;
        vscan_temp.y = item->y;
        vscan_temp.z = item->z;
	g_obstacle.setStopPoint(calcAbsoluteCoordinate(vscan_temp, g_localizer_pose.pose));
      }
      if (stop_point_count > g_threshold_points)
      {
        g_obstacle_waypoint = i;
        return STOP;
      }

      // without deceleration range
      if (g_deceleration_range < 0.01)
        continue;
      // deceleration search runs "decelerate_search_distance" waypoints from closest
      if (i > g_closest_waypoint + g_deceleration_search_distance || decelerate_or_stop >= 0)
        continue;

      // ---DECELERATE OBSTACLE DETECTION---
      if (dt > g_detection_range && dt < g_detection_range + g_deceleration_range)
      {
        bool count_flag = true;

        // search overlaps between DETECTION range and DECELERATION range
        for (int waypoint_search = -5; waypoint_search <= 5; waypoint_search++)
        {
          if (i + waypoint_search < 0 || i + waypoint_search >= g_path_dk.getSize() || !waypoint_search)
            continue;
          geometry_msgs::Point temp_waypoint =
              calcRelativeCoordinate(g_path_dk.getWaypointPosition(i + waypoint_search), g_localizer_pose.pose);
          tf::Vector3 waypoint_vector = point2vector(temp_waypoint);
          waypoint_vector.setZ(0);
          // if there is a overlap, give priority to DETECTION range
          if (tf::tfDistance(vscan_vector, waypoint_vector) < g_detection_range)
          {
            count_flag = false;
            break;
          }
        }
        if (count_flag)
        {
          decelerate_point_count++;
          geometry_msgs::Point vscan_temp;
          vscan_temp.x = item->x;
          vscan_temp.y = item->y;
          vscan_temp.z = item->z;
	  g_obstacle.setDeceleratePoint(calcAbsoluteCoordinate(vscan_temp, g_localizer_pose.pose));
        }
      }

      // found obstacle to DECELERATE
      if (decelerate_point_count > g_threshold_points)
      {
        g_obstacle_waypoint = i;
        decelerate_or_stop = 0;  // for searching near STOP obstacle
        g_obstacle.setDecided(true);
      }
    }
  }

  return KEEP;  // no obstacles
}

  /*
void soundPlay()
{
  std_msgs::String string;
  string.data = pedestrian_sound;
  g_sound_pub.publish(string);
}
  */

EControl obstacleDetection()
{
  static int false_count = 0;
  static EControl prev_detection = KEEP;

  EControl vscan_result = vscanDetection();
  displayDetectionRange(vmap.getDetectionCrossWalkID(), g_closest_waypoint, vscan_result);

  if (prev_detection == KEEP)
  {
    if (vscan_result != KEEP)
    {  // found obstacle
      displayObstacle(vscan_result);
      prev_detection = vscan_result;
      // SoundPlay();
      false_count = 0;
      return vscan_result;
    }
    else
    {  // no obstacle
      prev_detection = KEEP;
      return vscan_result;
    }
  }
  else
  {  // prev_detection = STOP or DECELERATE
    if (vscan_result != KEEP)
    {  // found obstacle
      displayObstacle(vscan_result);
      prev_detection = vscan_result;
      false_count = 0;
      return vscan_result;
    }
    else
    {  // no obstacle
      false_count++;

      // fail-safe
      if (false_count >= LOOP_RATE / 2)
      {
        g_obstacle_waypoint = -1;
        false_count = 0;
        prev_detection = KEEP;
        return vscan_result;
      }
      else
      {
        displayObstacle(OTHERS);
        return prev_detection;
      }
    }
  }
}

void changeWaypoint(EControl detection_result)
{
  int obs = g_obstacle_waypoint;

  if (detection_result == STOP)
  {  // STOP for obstacle
    // stop_waypoint is about g_others_distance meter away from obstacles
    int stop_waypoint = obs - ((int)(g_others_distance / g_path_change.getInterval()));
    // change waypoints to stop by the stop_waypoint
    g_path_change.changeWaypoints(stop_waypoint);
    g_path_change.avoidSuddenBraking();
    g_path_change.setTemporalWaypoints();
    g_temporal_waypoints_pub.publish(g_path_change.getTemporalWaypoints());
  }
  else if (detection_result == DECELERATE)
  {  // DECELERATE for obstacles
    g_path_change.setPath(g_path_dk.getCurrentWaypoints());
    g_path_change.setDeceleration();
    g_path_change.setTemporalWaypoints();
    g_temporal_waypoints_pub.publish(g_path_change.getTemporalWaypoints());
  }
  else
  {  // ACELERATE or KEEP
    g_path_change.setPath(g_path_dk.getCurrentWaypoints());
    g_path_change.avoidSuddenAceleration();
    g_path_change.avoidSuddenBraking();
    g_path_change.setTemporalWaypoints();
    g_temporal_waypoints_pub.publish(g_path_change.getTemporalWaypoints());
  }

  return;
}

} // end namespace

//======================================
//                 main
//======================================

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lattice_velocity_set");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  bool use_crosswalk_detection;
  private_nh.param<bool>("use_crosswalk_detection", use_crosswalk_detection, true);

  ros::Subscriber localizer_sub = nh.subscribe("localizer_pose", 1, localizerCallback);
  ros::Subscriber control_pose_sub = nh.subscribe("current_pose", 1, controlCallback);
  ros::Subscriber vscan_sub = nh.subscribe("vscan_points", 1, vscanCallback);
  ros::Subscriber base_waypoint_sub = nh.subscribe("base_waypoints", 1, baseWaypointCallback);
  ros::Subscriber obj_pose_sub = nh.subscribe("obj_pose", 1, objPoseCallback);
  ros::Subscriber current_vel_sub = nh.subscribe("current_velocity", 1, currentVelCallback);
  ros::Subscriber config_sub = nh.subscribe("config/lattice_velocity_set", 10, configCallback);

  //------------------ Vector Map ----------------------//
  ros::Subscriber sub_dtlane = nh.subscribe("vector_map_info/cross_walk", 1, &CrossWalk::crossWalkCallback, &vmap);
  ros::Subscriber sub_area = nh.subscribe("vector_map_info/area", 1, &CrossWalk::areaCallback, &vmap);
  ros::Subscriber sub_line = nh.subscribe("vector_map_info/line", 1, &CrossWalk::lineCallback, &vmap);
  ros::Subscriber sub_point = nh.subscribe("vector_map_info/point", 1, &CrossWalk::pointCallback, &vmap);
  //----------------------------------------------------//

  g_range_pub = nh.advertise<visualization_msgs::MarkerArray>("detection_range", 0);
  g_sound_pub = nh.advertise<std_msgs::String>("sound_player", 10);
  g_temporal_waypoints_pub = nh.advertise<waypoint_follower::lane>("temporal_waypoints", 1000, true);
  ros::Publisher closest_waypoint_pub;
  closest_waypoint_pub = nh.advertise<std_msgs::Int32>("closest_waypoint", 1000);
  g_obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 0);

  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    ros::spinOnce();

    if (vmap.loaded_all && !vmap.set_points)
      vmap.setCrossWalkPoints();

    if (g_pose_flag == false || g_path_flag == false)
    {
      loop_rate.sleep();
      continue;
    }

    g_closest_waypoint = getClosestWaypoint(g_path_change.getCurrentWaypoints(), g_control_pose.pose);

    std_msgs::Int32 closest_waypoint;
    closest_waypoint.data = g_closest_waypoint;
    closest_waypoint_pub.publish(closest_waypoint);

    if (use_crosswalk_detection)
      vmap.setDetectionWaypoint(findCrossWalk());

    EControl detection_result = obstacleDetection();

    changeWaypoint(detection_result);

    g_vscan.clear();

    loop_rate.sleep();
  }

  return 0;
}
