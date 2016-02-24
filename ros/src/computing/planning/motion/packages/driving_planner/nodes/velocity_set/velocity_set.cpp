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
#include <runtime_manager/ConfigVelocitySet.h>
#include <iostream>

#include "waypoint_follower/lane.h"
#include "waypoint_follower/libwaypoint_follower.h"
#include "libvelocity_set.h"

static const int LOOP_RATE = 10;

static geometry_msgs::TwistStamped _current_twist;
static geometry_msgs::PoseStamped _current_pose; // pose of sensor
static geometry_msgs::PoseStamped _control_pose; // pose of base_link
static geometry_msgs::PoseStamped _sim_ndt_pose;
static pcl::PointCloud<pcl::PointXYZ> _vscan;

static const std::string pedestrian_sound = "pedestrian";
static bool _pose_flag = false;
static bool _path_flag = false;
static bool _vscan_flag = false;
static int _obstacle_waypoint = -1;
static double _deceleration_search_distance = 30;
static double _search_distance = 60;
static int _closest_waypoint = -1;
static double _current_vel = 0.0;       // subscribe estimated_vel_mps
static CrossWalk vmap;
static ObstaclePoints g_obstacle;
static bool g_sim_mode;

/* Config Parameter */
static double _detection_range = 0; // if obstacle is in this range, stop
static double _deceleration_range = 1.8; // if obstacle is in this range, decelerate
static double _deceleration_minimum = kmph2mps(4.0); // until this speed
static int _threshold_points = 15;
static double _detection_height_top = 2.0; //actually +2.0m
static double _detection_height_bottom = -2.0;
static double _cars_distance = 15.0; // meter: stopping distance from cars (using DPM)
static double _pedestrians_distance = 10.0; // meter: stopping distance from pedestrians (using DPM)
static double _others_distance = 8.0;    // meter: stopping distance from obstacles (using VSCAN)
static double _decel = 1.5;           // (m/s) deceleration
static double _velocity_change_limit = 2.778;   // (m/s) about 10 km/h
static double _velocity_limit = 12.0; //(m/s) limit velocity for waypoints
static double _temporal_waypoints_size = 100.0; // meter
static double _velocity_offset = 1.389; // (m/s)


//Publisher
static ros::Publisher _range_pub;
static ros::Publisher _deceleration_range_pub;
static ros::Publisher _sound_pub;
static ros::Publisher _safety_waypoint_pub;
static ros::Publisher _temporal_waypoints_pub;
static ros::Publisher _crosswalk_points_pub;
static ros::Publisher _obstacle_pub;

static WayPoints _path_dk;

class PathVset: public WayPoints{
private:
  waypoint_follower::lane temporal_waypoints_;
public:
  void changeWaypoints(int stop_waypoint);
  void avoidSuddenBraking();
  void avoidSuddenAceleration();
  void setDeceleration();
  bool checkWaypoint(int num, const char *name) const;
  void setTemporalWaypoints();
  waypoint_follower::lane getTemporalWaypoints() const { return temporal_waypoints_; }
};
static PathVset _path_change;



//===============================
//       class function
//===============================


// check if waypoint number is valid
bool PathVset::checkWaypoint(int num, const char *name) const
{
  if (num < 0 || num >= getSize()){
    std::cout << name << ": invalid waypoint number" << std::endl;
    return false;
  }
  return true;
}

// set about '_temporal_waypoints_size' meter waypoints from closest waypoint
void PathVset::setTemporalWaypoints()
{
  if (_closest_waypoint < 0)
    return;
  int size = (int)(_temporal_waypoints_size/getInterval())+1;

  temporal_waypoints_.waypoints.clear();
  temporal_waypoints_.header = current_waypoints_.header;
  temporal_waypoints_.increment = current_waypoints_.increment;
  // push current pose
  waypoint_follower::waypoint current_point;

  current_point.pose = _control_pose;
  if (g_sim_mode)
    current_point.pose = _current_pose;
  current_point.twist = current_waypoints_.waypoints[_closest_waypoint].twist;
  current_point.dtlane = current_waypoints_.waypoints[_closest_waypoint].dtlane;
  temporal_waypoints_.waypoints.push_back(current_point);
  for (int i = 0; i < size; i++) {
    if (_closest_waypoint+i >= getSize())
      return;
    temporal_waypoints_.waypoints.push_back(current_waypoints_.waypoints[_closest_waypoint+i]);
  }

  return;
}

void PathVset::setDeceleration()
{
  int velocity_change_range = 5;
  double intervel = getInterval();
  double temp1 = _current_vel*_current_vel;
  double temp2 = 2*_decel*intervel;

  for (int i = 0; i < velocity_change_range; i++) {
    if (!checkWaypoint(_closest_waypoint+i, "setDeceleration"))
      continue;
    double waypoint_velocity = current_waypoints_.waypoints[_closest_waypoint+i].twist.twist.linear.x;
    double changed_vel = temp1 - temp2;
    if (changed_vel < 0) {
      changed_vel = _deceleration_minimum * _deceleration_minimum;
    }
    if (sqrt(changed_vel) > waypoint_velocity || _deceleration_minimum > waypoint_velocity)
      continue;
    if (sqrt(changed_vel) < _deceleration_minimum) {
      current_waypoints_.waypoints[_closest_waypoint+i].twist.twist.linear.x = _deceleration_minimum;
      continue;
    }
    current_waypoints_.waypoints[_closest_waypoint+i].twist.twist.linear.x = sqrt(changed_vel);
  }

  return;
}

void PathVset::avoidSuddenAceleration()
{
  double changed_vel;
  double interval = getInterval();
  double temp1 = _current_vel*_current_vel;
  double temp2 = 2*_decel*interval;

  for (int i = 0; ; i++) {
    if (!checkWaypoint(_closest_waypoint+i, "avoidSuddenAceleration"))
      return;
    changed_vel = sqrt(temp1 + temp2*(double)(i+1)) + _velocity_offset;
    if (changed_vel > current_waypoints_.waypoints[_closest_waypoint+i].twist.twist.linear.x)
      return;
    current_waypoints_.waypoints[_closest_waypoint+i].twist.twist.linear.x = changed_vel;
  }

  return;
}


void PathVset::avoidSuddenBraking()
{
  int i = 0;
  int fill_in_zero = 20;
  int fill_in_vel = 15;
  int examin_range = 1; // need to change according to waypoint interval?
  int num;
  double interval = getInterval();
  double changed_vel;

  for (int j = -1; j < examin_range; j++) {
    if (!checkWaypoint(_closest_waypoint+j, "avoidSuddenBraking"))
      return;
    if (getWaypointVelocityMPS(_closest_waypoint+j) < _current_vel - _velocity_change_limit) // we must change waypoints
      break;
    if (j == examin_range-1) // we don't have to change waypoints
      return;
  }


  std::cout << "====avoid sudden braking====" << std::endl;
  std::cout << "vehicle is decelerating..." << std::endl;
  std::cout << "closest_waypoint: " << _closest_waypoint << std::endl;


  // fill in waypoints velocity behind vehicle
  for (num = _closest_waypoint-1; fill_in_vel > 0; fill_in_vel--) {
    if (!checkWaypoint(num-fill_in_vel, "avoidSuddenBraking"))
      continue;
    current_waypoints_.waypoints[num-fill_in_vel].twist.twist.linear.x = _current_vel;
  }

  // decelerate gradually
  double temp1 = (_current_vel-_velocity_change_limit+1.389)*(_current_vel-_velocity_change_limit+1.389);
  double temp2 = 2*_decel*interval;
  for (num = _closest_waypoint-1; ; num++) {
    if (num >= getSize())
      return;
    if (!checkWaypoint(num, "avoidSuddenBraking"))
      continue;
    changed_vel = temp1 - temp2*(double)i; // sqrt(v^2 - 2*a*x)
    if (changed_vel <= 0)
      break;
    current_waypoints_.waypoints[num].twist.twist.linear.x = sqrt(changed_vel);

    i++;
  }

  for (int j = 0; j < fill_in_zero; j++) {
    if (!checkWaypoint(num+j, "avoidSuddenBraking"))
      continue;
    current_waypoints_.waypoints[num+j].twist.twist.linear.x = 0.0;
  }

  std::cout << "====changed waypoints====" << std::endl;

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
  for (int num = stop_waypoint; num > _closest_waypoint - close_waypoint_threshold; num--){
    if (!checkWaypoint(num, "changeWaypoints"))
      continue;

    changed_vel = sqrt(2.0*_decel*(interval*i)); // sqrt(2*a*x)

    //std::cout << "changed_vel[" << num << "]: " << mps2kmph(changed_vel) << " (km/h)";
    //std::cout << "   distance: " << (_obstacle_waypoint-num)*interval << " (m)";
    //std::cout << "   current_vel: " << mps2kmph(_current_vel) << std::endl;

    waypoint_follower::waypoint initial_waypoint = _path_dk.getCurrentWaypoints().waypoints[num];
    if (changed_vel > _velocity_limit || //
	changed_vel > initial_waypoint.twist.twist.linear.x){ // avoid acceleration
      //std::cout << "too large velocity!!" << std::endl;
      current_waypoints_.waypoints[num].twist.twist.linear.x = initial_waypoint.twist.twist.linear.x;
    } else {
      current_waypoints_.waypoints[num].twist.twist.linear.x = changed_vel;
    }

    i++;
  }

  // fill in 0
  for (int j = 1; j < fill_in_zero; j++){
    if (!checkWaypoint(stop_waypoint+j, "changeWaypoints"))
      continue;
    current_waypoints_.waypoints[stop_waypoint+j].twist.twist.linear.x = 0.0;
  }


  std::cout << "---changed waypoints---" << std::endl;

  return;
}


//===============================
//       class function
//===============================





//===============================
//          Callback
//===============================

void ConfigCallback(const runtime_manager::ConfigVelocitySetConstPtr &config)
{
  _velocity_limit = kmph2mps(config->velocity_limit);
  _others_distance = config->others_distance;
  _cars_distance = config->cars_distance;
  _pedestrians_distance = config->pedestrians_distance;
  _detection_range = config->detection_range;
  _threshold_points = config->threshold_points;
  _detection_height_top = config->detection_height_top;
  _detection_height_bottom = config->detection_height_bottom;
  _decel = config->deceleration;
  _velocity_change_limit = kmph2mps(config->velocity_change_limit);
  _velocity_offset = kmph2mps(config->velocity_offset);
  _deceleration_range = config->deceleration_range;
  _deceleration_minimum = kmph2mps(config->deceleration_minimum);
  _temporal_waypoints_size = config->temporal_waypoints_size;
}

void EstimatedVelCallback(const std_msgs::Float32ConstPtr &msg)
{
  if (g_sim_mode)
    return;

  _current_vel = msg->data;
}

void BaseWaypointCallback(const waypoint_follower::laneConstPtr &msg)
{
  ROS_INFO("subscribed base_waypoint\n");
  _path_dk.setPath(*msg);
  _path_change.setPath(*msg);
  if (_path_flag == false) {
    std::cout << "waypoint subscribed" << std::endl;
    _path_flag = true;
  }

}


void ObjPoseCallback(const visualization_msgs::MarkerConstPtr &msg)
{
  ROS_INFO("subscribed obj_pose\n");
}




static void VscanCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ> vscan_raw;
    pcl::fromROSMsg(*msg, vscan_raw);

    _vscan.clear();
    for (const auto &v : vscan_raw) {
      if (v.x == 0 && v.y == 0)
	continue;
      if (v.z > _detection_height_top || v.z < _detection_height_bottom)
	continue;
      _vscan.push_back(v);
    }

    if (_vscan_flag == false) {
        std::cout << "vscan subscribed" << std::endl;
        _vscan_flag = true;
    }

}

static void ControlCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
        _control_pose.header = msg->header;
        _control_pose.pose = msg->pose;
}

static void LocalizerCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (g_sim_mode) {
    _sim_ndt_pose.header = msg->header;
    _sim_ndt_pose.pose = msg->pose;
    return;
  }

        _current_pose.header = msg->header;
        _current_pose.pose = msg->pose;
        if (_pose_flag == false) {
            std::cout << "pose subscribed" << std::endl;
            _pose_flag = true;
        }
}

static void OdometryPoseCallback(const nav_msgs::OdometryConstPtr &msg)
{

  //
  // effective for testing.
  //
  if (!g_sim_mode)
   return;

    _current_pose.header = msg->header;
    _current_pose.pose = msg->pose.pose;
    _current_vel = msg->twist.twist.linear.x;
    if (_pose_flag == false) {
      std::cout << "pose subscribed" << std::endl;
      _pose_flag = true;
    }

}

//===============================
//          Callback
//===============================




static void DisplayObstacle(const EControl &kind)
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
    marker.pose.orientation = _current_pose.pose.orientation;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.7;
    if (kind == STOP) {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    marker.lifetime = ros::Duration(0.1);
    marker.frame_locked = true;

    _obstacle_pub.publish(marker);
}


static void DisplayDetectionRange(const int &crosswalk_id, const int &num, const EControl &kind)
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
    double scale = 2*_detection_range;
    waypoint_marker_stop.scale.x = scale;
    waypoint_marker_stop.scale.y = scale;
    waypoint_marker_stop.scale.z = scale;
    waypoint_marker_stop.color.a = 0.2;
    waypoint_marker_stop.color.r = 0.0;
    waypoint_marker_stop.color.g = 1.0;
    waypoint_marker_stop.color.b = 0.0;
    waypoint_marker_stop.frame_locked = true;

    scale = 2*(_detection_range + _deceleration_range);
    waypoint_marker_decelerate.scale.x = scale;
    waypoint_marker_decelerate.scale.y = scale;
    waypoint_marker_decelerate.scale.z = scale;
    waypoint_marker_decelerate.color.a = 0.15;
    waypoint_marker_decelerate.color.r = 1.0;
    waypoint_marker_decelerate.color.g = 1.0;
    waypoint_marker_decelerate.color.b = 0.0;
    waypoint_marker_decelerate.frame_locked = true;

    if (_obstacle_waypoint > -1) {
      stop_line.pose.position = _path_dk.getWaypointPosition(_obstacle_waypoint);
      stop_line.pose.orientation = _path_dk.getWaypointOrientation(_obstacle_waypoint);
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
    for (int i = 0; i < _search_distance; i++) {
      if (num < 0 || i+num > _path_dk.getSize() - 1)
	break;

      geometry_msgs::Point point;
      point = _path_dk.getWaypointPosition(num+i);

      waypoint_marker_stop.points.push_back(point);

      if (i > _deceleration_search_distance)
	continue;
      waypoint_marker_decelerate.points.push_back(point);
    }

    if (crosswalk_id > 0) {
      for (const auto &p : vmap.getDetectionPoints(crosswalk_id).points)
	crosswalk_marker.points.push_back(p);
    }


    // publish marker
    marker_array.markers.push_back(crosswalk_marker);
    marker_array.markers.push_back(waypoint_marker_stop);
    marker_array.markers.push_back(waypoint_marker_decelerate);
    if (kind == STOP)
      marker_array.markers.push_back(stop_line);
    _range_pub.publish(marker_array);
    marker_array.markers.clear();
}

static int FindCrossWalk()
{
  if (!vmap.set_points || _closest_waypoint < 0)
    return -1;

  double find_distance = 2.0*2.0; // meter
  double ignore_distance = 20.0*20.0; // meter
  // Find near cross walk
  for (int num = _closest_waypoint; num < _closest_waypoint+_search_distance; num++) {
    geometry_msgs::Point waypoint = _path_dk.getWaypointPosition(num);
    waypoint.z = 0.0; // ignore Z axis
    for (int i = 1; i < vmap.getSize(); i++) {
      // ignore far crosswalk
      geometry_msgs::Point crosswalk_center = vmap.getDetectionPoints(i).center;
      crosswalk_center.z = 0.0;
      if (CalcSquareOfLength(crosswalk_center, waypoint) > ignore_distance)
	continue;

      for (auto p : vmap.getDetectionPoints(i).points) {
	p.z = waypoint.z;
	if (CalcSquareOfLength(p, waypoint) < find_distance) {
	  vmap.setDetectionCrossWalkID(i);
	  return num;
	}
      }

    }
  }

  vmap.setDetectionCrossWalkID(-1);
  return -1; // no near crosswalk
}

static EControl CrossWalkDetection(const int &crosswalk_id)
{
  double search_radius = vmap.getDetectionPoints(crosswalk_id).width / 2;

  // Search each calculated points in the crosswalk
  for (const auto &p : vmap.getDetectionPoints(crosswalk_id).points) {
    geometry_msgs::Point detection_point = calcRelativeCoordinate(p, _current_pose.pose);
    if (g_sim_mode)
      detection_point = calcRelativeCoordinate(p, _sim_ndt_pose.pose);
    tf::Vector3 detection_vector = point2vector(detection_point);
    detection_vector.setZ(0.0);

    int stop_count = 0; // the number of points in the detection area
    for (const auto &vscan : _vscan) {
      tf::Vector3 vscan_vector(vscan.x, vscan.y, 0.0);
      double distance = tf::tfDistance(vscan_vector, detection_vector);
      if (distance < search_radius) {
	stop_count++;
	geometry_msgs::Point vscan_temp;
	vscan_temp.x = vscan.x;
	vscan_temp.y = vscan.y;
	vscan_temp.z = vscan.z;
	if (g_sim_mode)
	  g_obstacle.setStopPoint(calcAbsoluteCoordinate(vscan_temp, _sim_ndt_pose.pose));
	else
	  g_obstacle.setStopPoint(calcAbsoluteCoordinate(vscan_temp, _current_pose.pose));
      }
      if (stop_count > _threshold_points)
	return STOP;
    }

    g_obstacle.clearStopPoints();
  }

  return KEEP; // find no obstacles
}

static EControl vscanDetection()
{

    if (_vscan.empty() == true || _closest_waypoint < 0)
        return KEEP;

    int decelerate_or_stop = -10000;
    int decelerate2stop_waypoints = 15;


    for (int i = _closest_waypoint; i < _closest_waypoint + _search_distance; i++) {
        g_obstacle.clearStopPoints();
	if (!g_obstacle.isDecided())
	  g_obstacle.clearDeceleratePoints();

        decelerate_or_stop++;
        if (decelerate_or_stop > decelerate2stop_waypoints ||
	    (decelerate_or_stop >= 0 && i >= _path_dk.getSize()-1) ||
	    (decelerate_or_stop >= 0 && i == _closest_waypoint+_search_distance-1))
	    return DECELERATE;
        if (i > _path_dk.getSize() - 1 )
            return KEEP;


	// Detection for cross walk
	if (i == vmap.getDetectionWaypoint()) {
	  if (CrossWalkDetection(vmap.getDetectionCrossWalkID()) == STOP) {
	    _obstacle_waypoint = i;
	    return STOP;
	  }
	}


	// waypoint seen by vehicle
	geometry_msgs::Point waypoint = calcRelativeCoordinate(_path_dk.getWaypointPosition(i),
							       _current_pose.pose);
	tf::Vector3 tf_waypoint = point2vector(waypoint);
        tf_waypoint.setZ(0);

        int stop_point_count = 0;
	int decelerate_point_count = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = _vscan.begin(); item != _vscan.end(); item++) {

	    tf::Vector3 vscan_vector((double) item->x, (double) item->y, 0);
	    // for simulation
	    if (g_sim_mode) {
	      tf::Transform transform;
	      tf::poseMsgToTF(_sim_ndt_pose.pose, transform);
	      geometry_msgs::Point world2vscan = vector2point(transform * vscan_vector);
	      vscan_vector = point2vector(calcRelativeCoordinate(world2vscan, _current_pose.pose));
	      vscan_vector.setZ(0);
	    }

	    // 2D distance between waypoint and vscan points(obstacle)
	    // ---STOP OBSTACLE DETECTION---
            double dt = tf::tfDistance(vscan_vector, tf_waypoint);
            if (dt < _detection_range) {
	      stop_point_count++;
	      geometry_msgs::Point vscan_temp;
	      vscan_temp.x = item->x;
	      vscan_temp.y = item->y;
	      vscan_temp.z = item->z;
	      if (g_sim_mode)
		g_obstacle.setStopPoint(calcAbsoluteCoordinate(vscan_temp, _sim_ndt_pose.pose));
	      else
		g_obstacle.setStopPoint(calcAbsoluteCoordinate(vscan_temp, _current_pose.pose));
	    }
            if (stop_point_count > _threshold_points) {
	      _obstacle_waypoint = i;
	      return STOP;
	    }


	    // without deceleration range
	    if (_deceleration_range < 0.01)
	      continue;
	    // deceleration search runs "decelerate_search_distance" waypoints from closest
	    if (i > _closest_waypoint+_deceleration_search_distance || decelerate_or_stop >= 0)
	      continue;


	    // ---DECELERATE OBSTACLE DETECTION---
            if (dt > _detection_range && dt < _detection_range + _deceleration_range) {
	      bool count_flag = true;

	      // search overlaps between DETECTION range and DECELERATION range
	      for (int waypoint_search = -5; waypoint_search <= 5; waypoint_search++) {
		if (i+waypoint_search < 0 || i+waypoint_search >= _path_dk.getSize() || !waypoint_search)
		  continue;
		geometry_msgs::Point temp_waypoint  = calcRelativeCoordinate(
						      _path_dk.getWaypointPosition(i+waypoint_search),
						      _current_pose.pose);
		tf::Vector3 waypoint_vector = point2vector(temp_waypoint);
		waypoint_vector.setZ(0);
		// if there is a overlap, give priority to DETECTION range
		if (tf::tfDistance(vscan_vector, waypoint_vector) < _detection_range) {
		  count_flag = false;
		  break;
		}
	      }
	      if (count_flag) {
		decelerate_point_count++;
		geometry_msgs::Point vscan_temp;
		vscan_temp.x = item->x;
		vscan_temp.y = item->y;
		vscan_temp.z = item->z;
		if (g_sim_mode)
		  g_obstacle.setDeceleratePoint(calcAbsoluteCoordinate(vscan_temp, _sim_ndt_pose.pose));
		else
		  g_obstacle.setDeceleratePoint(calcAbsoluteCoordinate(vscan_temp, _current_pose.pose));
	      }
	    }

	    // found obstacle to DECELERATE
            if (decelerate_point_count > _threshold_points) {
	      _obstacle_waypoint = i;
	      decelerate_or_stop = 0; // for searching near STOP obstacle
	      g_obstacle.setDecided(true);
	    }

        }
    }

    return KEEP; //no obstacles
}

static void SoundPlay()
{
    std_msgs::String string;
    string.data = pedestrian_sound;
    _sound_pub.publish(string);
}

static EControl ObstacleDetection()
{
    static int false_count = 0;
    static EControl prev_detection = KEEP;

    std::cout << "closest_waypoint : " << _closest_waypoint << std::endl;
    std::cout << "current_velocity : " << mps2kmph(_current_vel) << std::endl;
    EControl vscan_result = vscanDetection();
    DisplayDetectionRange(vmap.getDetectionCrossWalkID(), _closest_waypoint, vscan_result);


    if (prev_detection == KEEP) {
      if (vscan_result != KEEP) { // found obstacle
	DisplayObstacle(vscan_result);
	std::cout << "obstacle waypoint : " << _obstacle_waypoint << std::endl << std::endl;
	prev_detection = vscan_result;
	//SoundPlay();
	false_count = 0;
	return vscan_result;
      } else {                  // no obstacle
	prev_detection = KEEP;
	return vscan_result;
      }
    } else { //prev_detection = STOP or DECELERATE
      if (vscan_result != KEEP) { // found obstacle
	DisplayObstacle(vscan_result);
	std::cout << "obstacle waypoint : " << vscan_result << std::endl << std::endl;
	prev_detection = vscan_result;
	false_count = 0;
	return vscan_result;
      } else {                  // no obstacle
	false_count++;
	std::cout << "false_count : "<< false_count << std::endl;

	//fail-safe
	if (false_count >= LOOP_RATE/2) {
	  _obstacle_waypoint = -1;
	  false_count = 0;
	  prev_detection = KEEP;
	  return vscan_result;
	} else {
	  std::cout << "obstacle waypoint : " << _obstacle_waypoint << std::endl << std::endl;
	  DisplayObstacle(OTHERS);
	  return prev_detection;
	}
      }
    }

}


static void ChangeWaypoint(EControl detection_result)
{

  int obs = _obstacle_waypoint;

  if (obs != -1){
    std::cout << "====got obstacle waypoint====" << std::endl;
    std::cout << "=============================" << std::endl;
  }

  if (detection_result == STOP){ // STOP for obstacle
    // stop_waypoint is about _others_distance meter away from obstacles
    int stop_waypoint = obs - ((int)(_others_distance / _path_change.getInterval()));
    std::cout << "stop_waypoint: " << stop_waypoint << std::endl;
    // change waypoints to stop by the stop_waypoint
    _path_change.changeWaypoints(stop_waypoint);
    _path_change.avoidSuddenBraking();
    _path_change.setTemporalWaypoints();
    _temporal_waypoints_pub.publish(_path_change.getTemporalWaypoints());
  } else if (detection_result == DECELERATE) { // DECELERATE for obstacles
    _path_change.setPath(_path_dk.getCurrentWaypoints());
    _path_change.setDeceleration();
    _path_change.setTemporalWaypoints();
    _temporal_waypoints_pub.publish(_path_change.getTemporalWaypoints());
  } else {               // ACELERATE or KEEP
    _path_change.setPath(_path_dk.getCurrentWaypoints());
    _path_change.avoidSuddenAceleration();
    _path_change.avoidSuddenBraking();
    _path_change.setTemporalWaypoints();
    _temporal_waypoints_pub.publish(_path_change.getTemporalWaypoints());
  }


    return;
}



//======================================
//                 main
//======================================

int main(int argc, char **argv)
{
    int i = 0;///
    ros::init(argc, argv, "velocity_set");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Subscriber localizer_sub = nh.subscribe("localizer_pose", 1, LocalizerCallback);
    ros::Subscriber control_pose_sub = nh.subscribe("control_pose", 1, ControlCallback);
    ros::Subscriber odometry_subscriber = nh.subscribe("odom_pose", 10, OdometryPoseCallback);
    ros::Subscriber vscan_sub = nh.subscribe("vscan_points", 1, VscanCallback);
    ros::Subscriber base_waypoint_sub = nh.subscribe("base_waypoints", 1, BaseWaypointCallback);
    ros::Subscriber obj_pose_sub = nh.subscribe("obj_pose", 1, ObjPoseCallback);
    ros::Subscriber estimated_vel_sub = nh.subscribe("estimated_vel_mps", 1, EstimatedVelCallback);
    ros::Subscriber config_sub = nh.subscribe("config/velocity_set", 10, ConfigCallback);

    //------------------ Vector Map ----------------------//
    ros::Subscriber sub_dtlane = nh.subscribe("vector_map_info/cross_walk", 1,
					      &CrossWalk::CrossWalkCallback, &vmap);
    ros::Subscriber sub_area   = nh.subscribe("vector_map_info/area_class", 1,
					      &CrossWalk::AreaclassCallback, &vmap);
    ros::Subscriber sub_line   = nh.subscribe("vector_map_info/line_class", 1,
					      &CrossWalk::LineclassCallback, &vmap);
    ros::Subscriber sub_point  = nh.subscribe("vector_map_info/point_class", 1,
					      &CrossWalk::PointclassCallback, &vmap);
    //----------------------------------------------------//

    _range_pub = nh.advertise<visualization_msgs::MarkerArray>("detection_range", 0);
    _sound_pub = nh.advertise<std_msgs::String>("sound_player", 10);
    _temporal_waypoints_pub = nh.advertise<waypoint_follower::lane>("temporal_waypoints", 1000, true);
    static ros::Publisher closest_waypoint_pub;
    closest_waypoint_pub = nh.advertise<std_msgs::Int32>("closest_waypoint", 1000);
    _obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 0);


    private_nh.param<bool>("sim_mode", g_sim_mode, false);
    ROS_INFO_STREAM("sim_mode : " << g_sim_mode);


    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok()) {
        ros::spinOnce();

	if (vmap.loaded_all && !vmap.set_points)
	  vmap.setCrossWalkPoints();

        if (_pose_flag == false || _path_flag == false) {
	  std::cout << "\rtopic waiting          \rtopic waiting";
	  for (int j = 0; j < i; j++) {std::cout << ".";}
	  i++;
	  i = i%10;
	  std::cout << std::flush;
	  loop_rate.sleep();
	  continue;
        }

	_closest_waypoint = getClosestWaypoint(_path_change.getCurrentWaypoints(), _current_pose.pose);
	closest_waypoint_pub.publish(_closest_waypoint);

	vmap.setDetectionWaypoint(FindCrossWalk());

        EControl detection_result = ObstacleDetection();

	ChangeWaypoint(detection_result);


        loop_rate.sleep();
    }


    return 0;
}
