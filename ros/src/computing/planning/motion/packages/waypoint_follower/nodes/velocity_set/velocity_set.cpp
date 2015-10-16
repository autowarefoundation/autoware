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
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <runtime_manager/ConfigVelocitySet.h>

#include <iostream>

#include "waypoint_follower/lane.h"
#include "waypoint_follower/libwaypoint_follower.h"

static const int LOOP_RATE = 10;

static geometry_msgs::TwistStamped _current_twist;
static geometry_msgs::PoseStamped _current_pose; // current pose by the global plane.
static pcl::PointCloud<pcl::PointXYZ> _vscan;

static std::string _current_pose_topic = "ndt";
static const std::string pedestrian_sound = "pedestrian";
static std::string _linelist_frame = "/velodyne";
static bool _pose_flag = false;
static bool _path_flag = false;
static bool _vscan_flag = false;

static double _detection_range = 0;
static int _obstacle_waypoint = -1;
static int _threshold_points = 15;
static double _detection_height_top = 2.0; //actually +2.0m
static double _detection_height_bottom = -2.0;
static double _search_distance = 60;
static double _cars_distance = 15.0; // meter: stopping distance from cars (using DPM)
static double _pedestrians_distance = 10.0; // meter: stopping distance from pedestrians (using DPM)
static double _others_distance = 8.0;    // meter: stopping distance from obstacles (using VSCAN)
static int _closest_waypoint = -1;
static double _current_vel = 0.0;       // subscribe estimated_vel_mps
static double _decel = 1.5;           // (m/s) deceleration
static double _decel_limit = 2.778;   // (m/s) about 10 km/h
static double _velocity_limit = 12.0; //(m/s) limit velocity for waypoints
static double _accel_bias = 1.389; // (m/s)
static visualization_msgs::Marker _linelist; // for obstacle's vscan linelist
static tf::Transform _transform;

//Publisher
static ros::Publisher _vis_pub;
static ros::Publisher _range_pub;
static ros::Publisher _sound_pub;
static ros::Publisher _safety_waypoint_pub;
static ros::Publisher _linelist_pub;

Path _path_dk;

class PathVset: public Path{
private:
public:
  const waypoint_follower::lane& getPathVset(){ return current_path_; }
  void setPathVset(const waypoint_follower::lane& current_path){ current_path_ = current_path;}
  void changeWaypoints(int stop_waypoint);
  double getVel(int num);
  void avoidSuddenBraking();
  void avoidSuddenAceleration();
  bool checkWaypoint(int num, const char *name);
};
PathVset _path_change, _path_subscribe;



//===============================
//       class function
//===============================


// check if waypoint number is valid
bool PathVset::checkWaypoint(int num, const char *name)
{
  if (num < 0 || num >= getPathSize()){
    std::cout << name << ": invalid waypoint number" << std::endl;
    return false;
  }
  return true;
}


double PathVset::getVel(int num)
{
  if (current_path_.waypoints.empty() || num < 0 || num >= getPathSize()){
    std::cout << "getVel: invalid waypoint" << std::endl;
    return 0.0;
  }
  return current_path_.waypoints[num].twist.twist.linear.x;
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
    changed_vel = sqrt(temp1 + temp2*(double)(i+1)) + _accel_bias;
    if (changed_vel > current_path_.waypoints[_closest_waypoint+i].twist.twist.linear.x)
      return;
    current_path_.waypoints[_closest_waypoint+i].twist.twist.linear.x = changed_vel;
  }

  return;
}


void PathVset::avoidSuddenBraking()
{
  int i = 0;
  int fill_in_zero = 20;
  int fill_in_vel = 15;
  int examin_range = 2; // need to change according to waypoint interval?
  int num;
  double interval = getInterval();
  double changed_vel;

  for (int j = -1; j < examin_range; j++) {
    if (!checkWaypoint(_closest_waypoint+j, "avoidSuddenBraking"))
      return;
    if (getVel(_closest_waypoint+j) < _current_vel - _decel_limit) // we must change waypoints
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
    current_path_.waypoints[num-fill_in_vel].twist.twist.linear.x = _current_vel;
  }

  // decelerate gradually
  double temp1 = (_current_vel-_decel_limit+1.389)*(_current_vel-_decel_limit+1.389);
  double temp2 = 2*_decel*interval;
  for (num = _closest_waypoint-1; ; num++) {
    if (num >= getPathSize())
      return;
    if (!checkWaypoint(num, "avoidSuddenBraking"))
      continue;
    changed_vel = temp1 - temp2*(double)i; // sqrt(v^2 - 2*a*x)
    if (changed_vel <= 0)
      break;
    current_path_.waypoints[num].twist.twist.linear.x = sqrt(changed_vel);

    i++;
  }

  for (int j = 0; j < fill_in_zero; j++) {
    if (!checkWaypoint(num+j, "avoidSuddenBraking"))
      continue;
    current_path_.waypoints[num+j].twist.twist.linear.x = 0.0;
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

    waypoint_follower::waypoint initial_waypoint = _path_dk.getCurrentPath().waypoints[num];
    if (changed_vel > _velocity_limit || //
	changed_vel > initial_waypoint.twist.twist.linear.x){ // avoid acceleration
      //std::cout << "too large velocity!!" << std::endl;
      current_path_.waypoints[num].twist.twist.linear.x = initial_waypoint.twist.twist.linear.x;
    } else {
      current_path_.waypoints[num].twist.twist.linear.x = changed_vel;
    }

    i++;
  }

  // fill in 0
  for (int j = 1; j < fill_in_zero; j++){
    if (!checkWaypoint(stop_waypoint+j, "changeWaypoints"))
      continue;
    current_path_.waypoints[stop_waypoint+j].twist.twist.linear.x = 0.0;
  }


  avoidSuddenBraking(); // examine close waypoints to avoid sudden braking
  _safety_waypoint_pub.publish(current_path_);// publish new waypoints
  std::cout << "---published waypoints---" << std::endl;

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
  _decel_limit = kmph2mps(config->decel_change_limit);
  _accel_bias = kmph2mps(config->accel_bias);
}

void EstimatedVelCallback(const std_msgs::Float32ConstPtr &msg)
{
  _current_vel = msg->data;
}

void BaseWaypointCallback(const waypoint_follower::laneConstPtr &msg)
{
  ROS_INFO("subscribed base_waypoint\n");
  _path_dk.setPath(msg);
  _path_change.setPath(msg); //++
  _path_subscribe.setPath(msg); //++
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
    pcl::fromROSMsg(*msg, _vscan);
    if (_vscan_flag == false) {
        std::cout << "vscan subscribed" << std::endl;
        _vscan_flag = true;
    }

}

static void NDTCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
        _current_pose.header = msg->header;
        _current_pose.pose = msg->pose;
        tf::Transform inverse;
        tf::poseMsgToTF(msg->pose, inverse);
        _path_dk.setTransform(inverse.inverse());
	_path_change.setTransform(inverse.inverse());//++
        if (_pose_flag == false) {
            std::cout << "pose subscribed" << std::endl;
            _pose_flag = true;
        }
}

//===============================
//          Callback
//===============================





// display by markers.
static void DisplayObstacleWaypoint(int i)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = _path_dk.getWaypointPosition(i);
    marker.pose.orientation = _current_pose.pose.orientation;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 2.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration(0.1);
    marker.frame_locked = true;

    _vis_pub.publish(marker);
}

// display by markers.
static void DisplayDetectionRange(int i)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    for (int j = 0; j < _search_distance; j++) {
        if(i+j > _path_dk.getPathSize() - 1)
            break;

        geometry_msgs::Point point;
        point = _path_dk.getWaypointPosition(j+i);
        marker.points.push_back(point);
    }
    marker.scale.x = 2 * _detection_range;
    marker.scale.y = 2 * _detection_range;
    marker.scale.z = _detection_height_top;
    marker.color.a = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.frame_locked = true;

    _range_pub.publish(marker);
    marker.points.clear();
}

static int vscanDetection(int closest_waypoint)
{

    if (_vscan.empty() == true)
        return -1;

    for (int i = closest_waypoint + 1; i < closest_waypoint + _search_distance; i++) {

        if(i > _path_dk.getPathSize() - 1 )
            return -1;

        tf::Vector3 tf_waypoint = _path_dk.transformWaypoint(i); // waypoint seen by vehicle
        tf_waypoint.setZ(0);

        int point_count = 0;
	geometry_msgs::Point vscan_point;
	_linelist.points.clear();
        for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = _vscan.begin(); item != _vscan.end(); item++) {
            if ((item->x == 0 && item->y == 0)) {
	      continue;
	    }

            tf::Vector3 point((double) item->x, (double) item->y, 0);

	    // 2D distance between waypoint and vscan points(obstacle)
            double dt = tf::tfDistance(point, tf_waypoint);
            if (dt < _detection_range) {
	      vscan_point.x = item->x;
	      vscan_point.y = item->y;
	      vscan_point.z = item->z;
	      _linelist.points.push_back(vscan_point);
	      if (item->z > _detection_height_top || item->z < _detection_height_bottom)
		continue;
	      point_count++;
	    }

	    // return closest waypoint from obstacles
            if (point_count > _threshold_points){
	      //_linelist_pub.publish(_linelist);
	      _linelist.points.clear();
	      return i;
	    }
        }
    }

    return -1; //no obstacles
}

static void SoundPlay()
{
    std_msgs::String string;
    string.data = pedestrian_sound;
    _sound_pub.publish(string);
}

static bool ObstacleDetection()
{
    static int false_count = 0;
    static bool prev_detection = false;

    std::cout << "closest_waypoint : " << _closest_waypoint << std::endl;
    DisplayDetectionRange(_closest_waypoint + 1);
    int vscan_result = vscanDetection(_closest_waypoint);

    if (prev_detection == false) {
      if (vscan_result != -1) { // found obstacle
	DisplayObstacleWaypoint(vscan_result);
	std::cout << "obstacle waypoint : " << vscan_result << std::endl << std::endl;
	prev_detection = true;
	_obstacle_waypoint = vscan_result;
	SoundPlay();
	false_count = 0;
	return true;
      } else {                  // no obstacle
	prev_detection = false;
	return false;
      }
    } else { //prev_detection = true
      if (vscan_result != -1) { // found obstacle
	DisplayObstacleWaypoint(vscan_result);
	std::cout << "obstacle waypoint : " << vscan_result << std::endl << std::endl;
	prev_detection = true;
	_obstacle_waypoint = vscan_result;
	false_count = 0;

	return true;
      } else {                  // no obstacle
	false_count++;
	std::cout << "false_count : "<< false_count << std::endl;
      }

      //fail-safe
      if (false_count >= LOOP_RATE/2) {
	_obstacle_waypoint = -1;
	false_count = 0;
	prev_detection = false;
	return false;
      } else {
	std::cout << "obstacle waypoint : " << _obstacle_waypoint << std::endl << std::endl;
	DisplayObstacleWaypoint(_obstacle_waypoint);
	prev_detection = true;

	return true;
      }
    }

}


// publish obstacles as RED linelist
static void linelistInit()
{

  _linelist.header.frame_id = _linelist_frame;
  _linelist.header.stamp = ros::Time(0);
  _linelist.ns = "vscan_linelist";
  _linelist.id = 0;
  _linelist.type = visualization_msgs::Marker::LINE_LIST;
  _linelist.action = visualization_msgs::Marker::ADD;
  _linelist.scale.x = 0.15;
  _linelist.color.a = 0.5;
  _linelist.color.r = 1.0;
  _linelist.color.g = 0.0;
  _linelist.color.b = 0.0;

}


static void ChangeWaypoint(bool detection_result)
{

  int obs = _obstacle_waypoint;
  waypoint_follower::lane lane;

  if (obs != -1){
    std::cout << "====got obstacle waypoint====" << std::endl;
    //lane = _path_change.getCurrentPath();
    //std::cout << "waypoint[" << obs << "] velocity: " << lane.waypoints[obs].twist.twist.linear.x << std::endl;
    //std::cout << "getDistance: " << _path_change.getDistance(obs) << std::endl;
    std::cout << "=============================" << std::endl;
  }

  if (detection_result){ // DECELERATE
    // stop_waypoint is about _others_distance meter away from obstacle
    int stop_waypoint = obs - ((int)(_others_distance / _path_change.getInterval()));
    std::cout << "stop_waypoint: " << stop_waypoint << std::endl;
    // change waypoints to stop by the stop_waypoint
    _path_change.changeWaypoints(stop_waypoint);
  } else {               // ACELERATE or KEEP
    _path_change.setPathVset(_path_subscribe.getPathVset());
    _path_change.avoidSuddenBraking();
    _path_change.avoidSuddenAceleration();
    _safety_waypoint_pub.publish(_path_change.getCurrentPath());
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
    ros::Subscriber ndt_sub = nh.subscribe("control_pose", 1, NDTCallback);
    ros::Subscriber vscan_sub = nh.subscribe("vscan_points", 1, VscanCallback);
    ros::Subscriber base_waypoint_sub = nh.subscribe("base_waypoint", 1, BaseWaypointCallback);
    ros::Subscriber obj_pose_sub = nh.subscribe("obj_pose", 1, ObjPoseCallback);
    ros::Subscriber estimated_vel_sub = nh.subscribe("estimated_vel_mps", 1, EstimatedVelCallback);
    ros::Subscriber config_sub = nh.subscribe("config/velocity_set", 10, ConfigCallback);

    _vis_pub = nh.advertise<visualization_msgs::Marker>("obstaclewaypoint_mark", 0);
    _range_pub = nh.advertise<visualization_msgs::Marker>("detection_range", 0);
    _sound_pub = nh.advertise<std_msgs::String>("sound_player", 10);
    _safety_waypoint_pub = nh.advertise<waypoint_follower::lane>("safety_waypoint", 1000, true);
    _linelist_pub = nh.advertise<visualization_msgs::Marker>("vscan_linelist", 10);



    private_nh.getParam("detection_range", _detection_range);
    std::cout << "detection_range : " << _detection_range << std::endl;

    private_nh.getParam("threshold_points", _threshold_points);
    std::cout << "threshold_points : " << _threshold_points << std::endl;

    private_nh.getParam("others_distance", _others_distance);
    std::cout << "others_distance : " << _others_distance << std::endl;

    private_nh.getParam("cars_distance", _cars_distance);
    std::cout << "cars_distance : " << _cars_distance << std::endl;

    private_nh.getParam("pedestrians_distance", _pedestrians_distance);
    std::cout << "pedestrians_distance : " << _pedestrians_distance << std::endl;

    private_nh.getParam("detection_height_top", _detection_height_top);
    std::cout << "detection_height_top : " << _detection_height_top << std::endl;

    private_nh.getParam("detection_height_bottom", _detection_height_bottom);
    std::cout << "detection_height_bottom : " << _detection_height_bottom << std::endl;

    private_nh.getParam("current_pose_topic", _current_pose_topic);
    std::cout << "current_pose_topic : " << _current_pose_topic << std::endl;

    private_nh.getParam("velocity_limit", _velocity_limit);
    std::cout << "velocity_limit : " << _velocity_limit << std::endl;

    private_nh.getParam("accel_bias", _accel_bias);
    std::cout << "accel_bias : " << _accel_bias << std::endl;

    linelistInit();

    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok()) {
        ros::spinOnce();

        if (_pose_flag == false || _path_flag == false) {
	  std::cout << "\rtopic waiting          \rtopic waiting";
	  for (int j = 0; j < i; j++) {std::cout << ".";}
	  i++;
	  i = i%10;
	  std::cout << std::flush;
	  loop_rate.sleep();
	  continue;
        }

	_closest_waypoint = _path_dk.getClosestWaypoint();
        bool detection_result = ObstacleDetection();

	ChangeWaypoint(detection_result);


        loop_rate.sleep();
    }


    return 0;
}
