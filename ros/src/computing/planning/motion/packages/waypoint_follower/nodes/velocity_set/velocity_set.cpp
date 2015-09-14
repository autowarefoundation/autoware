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

#include <iostream>

#include "waypoint_follower/lane.h"
#include "waypoint_follower/libwaypoint_follower.h"

#define LOOP_RATE 10

static geometry_msgs::TwistStamped _current_twist;
static geometry_msgs::PoseStamped _current_pose; // current pose by the global plane.
static pcl::PointCloud<pcl::PointXYZ> _vscan;

static std::string _current_pose_topic = "ndt";
static const std::string pedestrian_sound = "pedestrian";
static std::string _linelist_frame = "/velodyne";
//static bool _twist_flag = false;
static bool _pose_flag = false;
static bool _path_flag = false;
static bool _vscan_flag = false;
static bool _changepath_flag = false;

static double _detection_range = 0;
static int _obstacle_waypoint = -1;
static int _threshold_points = 15;
static double _detection_height_top = 2.0; //actually +2.0m
static double _detection_height_bottom = -2.0;
static double _search_distance = 70;
static int _stop_interval = 5;
static double _car_distance = 8.0;  // meter :distance from a front car when stopped
static int _closest_waypoint = -1;
static double _current_vel = 0;     // subscribe estimated_vel_mps
static double _decel = 1.5;         // (m/s) deceleration
static double _decel_limit = 2.778; // (m/s) about 10 km/h
static double _velocity_limit = 1.0; //(m/x) limit velocity for waypoints
static visualization_msgs::Marker _linelist; // for obstacle's vscan linelist
static tf::Transform _transform;

//Publisher
//static ros::Publisher _twist_pub;
static ros::Publisher _vis_pub;
static ros::Publisher _range_pub;
static ros::Publisher _sound_pub;
static ros::Publisher _safety_waypoint_pub;
static ros::Publisher _linelist_pub;

Path _path_dk;

class PathVset: public Path{
private:
public:
  void changeWaypoints(int stop_waypoint);
  double getVel(int num);
  void avoidSuddenBraking();
};
PathVset _path_change;



//===============================
//       class function
//===============================


double PathVset::getVel(int num){

  if (current_path_.waypoints.empty() || num < 0){
    std::cout << "getVel: invalid waypoint" << std::endl;
    return 0.0;
  }

  return current_path_.waypoints[num].twist.twist.linear.x;
}



void PathVset::avoidSuddenBraking(){
  int i = 0;
  int path_size = getPathSize();
  int close_waypoint_threshold = 5;
  int fill_in_zero = 5;
  int fill_in_vel = 15;
  int num;
  double temp;
  double interval = getInterval();

  std::cout << "====avoid sudden braking====" << std::endl;
  std::cout << "vehicle is decelerating..." << std::endl;
  _closest_waypoint = getClosestWaypoint();
  std::cout << "closest_waypoint: " << _closest_waypoint << std::endl;
  for (num = _closest_waypoint - close_waypoint_threshold; fill_in_vel > 0; fill_in_vel--){
    if (num-fill_in_vel < 0 || num >= path_size){
      std::cout << "avoidSuddenBraking: invalid waypoint number" << std::endl;
      continue;
    }
    current_path_.waypoints[num-fill_in_vel].twist.twist.linear.x = _current_vel;
  }

  for (num = _closest_waypoint - close_waypoint_threshold; num > -1; num++){
    if (num < 0 || num >= path_size){
      std::cout << "avoidSuddenBraking: invalid waypoint number" << std::endl;
      return;
    }
    temp = _current_vel*_current_vel - 2*_decel*i*interval; // sqrt(v^2 - 2*a*x)
    if (temp > 0){
      //if (sqrt(temp) > getVel(num)){current_path_.waypoints[num].twist.twist.linear.x = getVel(num);}
      current_path_.waypoints[num].twist.twist.linear.x = sqrt(temp);//
      //std::cout << "waypoint[" << num << "] vel: " << mps2kmph(sqrt(temp)) << std::endl;
    } else {
      break;
    }
    i++;
  }

  if (num < 0){
    std::cout << "avoidSuddenBraking: invalid waypoint number" << std::endl;
    return;
  }
  for (int j = 0; j < fill_in_zero; j++){current_path_.waypoints[num+j].twist.twist.linear.x = 0.0;}

  _safety_waypoint_pub.publish(current_path_);// publish new waypoints
  std::cout << "====published waypoints====" << std::endl;

  return;
}


void PathVset::changeWaypoints(int stop_waypoint){
  int i = 0;
  int path_size = getPathSize();
  int close_waypoint_threshold = 4;
  int fill_in_zero = 10;
  double changed_vel;
  double interval = getInterval();


  _closest_waypoint = getClosestWaypoint();
  if (_closest_waypoint < 0){
    std::cout << "changeWaypoints: invalid waypoint number(1)" << std::endl;
    return;
  }

  // avoid sudden braking at close waypoints
  for (int num = _closest_waypoint + close_waypoint_threshold; num > _closest_waypoint-5; num--){
    if (num < 0 || num >= path_size){
      std::cout << "changeWaypoints: invalid waypoint number(2)" << std::endl;
      break;
    }
    if (getVel(num) < _current_vel - _decel_limit){
      avoidSuddenBraking();
      return;
    }
  }

  // change waypoints to decelerate
  for (int num = stop_waypoint; num > _closest_waypoint - close_waypoint_threshold; num--){
    if (num < 0 || num >= path_size){
      std::cout << "changeWaypoints: invalid waypoint number(3)" << std::endl;
      break;
    }
    changed_vel = sqrt(2.0*_decel*(interval*i)); // sqrt(2*a*x)

    std::cout << "changed_vel[" << num << "]: " << mps2kmph(changed_vel) << " (km/h)";
    std::cout << "   distance: " << (_obstacle_waypoint-num)*interval << " (m)";
    std::cout << "   current_vel: " << mps2kmph(_current_vel) << std::endl;

    // avoid sudden braking at close waypoints
    if (num < _closest_waypoint + close_waypoint_threshold){
      if (changed_vel < _current_vel - _decel_limit ||
	  _velocity_limit < _current_vel - _decel_limit){
	avoidSuddenBraking();
	return;
      }
    }

    if (changed_vel > _velocity_limit ||
	changed_vel > _path_dk.getCurrentPath().waypoints[num].twist.twist.linear.x){ // avoid acceleration
      std::cout << "too large velocity!!" << std::endl;
      current_path_.waypoints[num].twist.twist.linear.x = current_path_.waypoints[num+1].twist.twist.linear.x;
    } else {
      current_path_.waypoints[num].twist.twist.linear.x = changed_vel; // set waypoint velocity to decelerate
    }

    i++;
  }

  // fill in 0
  for (int j = 1; j < fill_in_zero; j++){
    // if obstacle is near a vehicle, we must avoid sudden braking
    if (stop_waypoint+j < _closest_waypoint+close_waypoint_threshold &&
	_current_vel > _decel_limit){
      avoidSuddenBraking();
      return;
    }
    current_path_.waypoints[stop_waypoint+j].twist.twist.linear.x = 0;
  }


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

void EstimatedVelCallback(const std_msgs::Float32ConstPtr &msg){

  _current_vel = msg->data;
}

void BaseWaypointCallback(const waypoint_follower::laneConstPtr &msg){

  ROS_INFO("subscribed safety_waypoint\n");
  _path_dk.setPath(msg);
  _path_change.setPath(msg); //++
  if (_path_flag == false) {
    std::cout << "waypoint subscribed" << std::endl;
    _path_flag = true;
  }

  _safety_waypoint_pub.publish(msg);

}


void ObjPoseCallback(const visualization_msgs::MarkerConstPtr &msg){

  ROS_INFO("subscribed obj_pose\n");

}


/*
static void TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    _current_twist = *msg;

    if (_twist_flag == false) {
        std::cout << "twist subscribed" << std::endl;
        _twist_flag = true;
    }
}
*/

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
    marker.scale.z = _detection_height_top;
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
        //std::cout << "waypoint : "<< tf_waypoint.getX()  << " "<< tf_waypoint.getY() << std::endl;

        int point_count = 0;
	geometry_msgs::Point vscan_point;
	_linelist.points.clear();
        for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = _vscan.begin(); item != _vscan.end(); item++) {
	    // out of detection range
            if ((item->x == 0 && item->y == 0) /*|| item->z > _detection_height_top || 
						 item->z < _detection_height_bottom*/){
	      continue;
	    }

            tf::Vector3 point((double) item->x, (double) item->y, 0);

	    // 2D distance between waypoint and vscan point(obstacle)
            double dt = tf::tfDistance(point, tf_waypoint);
            if (dt < _detection_range) {
	      vscan_point.x = item->x;
	      vscan_point.y = item->y;
	      vscan_point.z = item->z;
	      _linelist.points.push_back(vscan_point);
	      if (item->z > _detection_height_top || item->z < _detection_height_bottom){continue;}
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

    // no obstacles
    return -1;
}

static void SoundPlay(){
    std_msgs::String string;
    string.data = pedestrian_sound;
    _sound_pub.publish(string);
}

static bool ObstacleDetection()
{
    static int false_count = 0;
    static bool prev_detection = false;

    int closest_waypoint = _path_change.getClosestWaypoint(); //
    _closest_waypoint = closest_waypoint;//
    std::cout << "closest_waypoint : " << closest_waypoint << std::endl;
    DisplayDetectionRange(closest_waypoint + 1);
    int vscan_result = vscanDetection(closest_waypoint);

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
      if (false_count >= LOOP_RATE * 2) {
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

/*
static double Decelerate()
{
    //calculate distance from my position to waypoint
    //tf::Vector3 tf_waypoint = TransformWaypoint(_transform,_current_path.waypoints[_obstacle_waypoint].pose.pose);
    tf::Vector3 tf_waypoint = _path_dk.transformWaypoint(_obstacle_waypoint);
    tf::Vector3 origin_v;
    origin_v.setZero();
  double distance = tf::tfDistance(origin_v, tf_waypoint);
   // std::cout << "distance " << distance << std::endl;

    //if distance is within stop_interval param, publish 0km/h
    if(distance < _stop_interval){
        return 0;
    }

    double decel_velocity_ms = DecelerateVelocity(distance,_current_twist.twist.linear.x);

    if(decel_velocity_ms < 1.0){
        decel_velocity_ms = 0;
    }

    return decel_velocity_ms;

}
*/

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
    lane = _path_change.getCurrentPath();
    std::cout << "waypoint[" << obs << "] velocity: " << lane.waypoints[obs].twist.twist.linear.x << std::endl;
    std::cout << "getDistance: " << _path_change.getDistance(obs) << std::endl;
    std::cout << "=============================" << std::endl;
  }

  _closest_waypoint = _path_change.getClosestWaypoint();
  if (detection_result){ // DECELERATE
    // if obstacle is behind a vehicle, return
    if (obs < _closest_waypoint){
      std::cout << "ChangeWaypoint: invalid obstacle waypoint" << std::endl;
      return;
    }
    // *stop_waypoint is about _car_distance meter away from obstacle*
    int stop_waypoint = obs - (((int)_car_distance / _path_change.getInterval()));
    std::cout << "stop_waypoint: " << stop_waypoint << std::endl;
    if (stop_waypoint < 0){
      std::cout << "ChangeWaypoint: invalid stop_waypoint!" << std::endl;
      return;
    }
    _path_change.changeWaypoints(stop_waypoint);
    _changepath_flag = true;
  } else {               // ACELERATE or KEEP
    if (_changepath_flag){
      _safety_waypoint_pub.publish(_path_dk.getCurrentPath());///
      _changepath_flag = false;
    }
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
    //ros::Subscriber twist_sub = nh.subscribe("twist_raw", 1, TwistCmdCallback);
    ros::Subscriber ndt_sub = nh.subscribe("control_pose", 1, NDTCallback);
    ros::Subscriber vscan_sub = nh.subscribe("vscan_points", 1, VscanCallback);
    ros::Subscriber base_waypoint_sub = nh.subscribe("base_waypoint", 1, BaseWaypointCallback);
    ros::Subscriber obj_pose_sub = nh.subscribe("obj_pose", 1, ObjPoseCallback);
    ros::Subscriber estimated_vel_sub = nh.subscribe("estimated_vel_mps", 1, EstimatedVelCallback);

    //_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 1000);
    _vis_pub = nh.advertise<visualization_msgs::Marker>("obstaclewaypoint_mark", 0);
    _range_pub = nh.advertise<visualization_msgs::Marker>("detection_range", 0);
    _sound_pub = nh.advertise<std_msgs::String>("sound_player", 10);
    _safety_waypoint_pub = nh.advertise<waypoint_follower::lane>("safety_waypoint", 1000);
    _linelist_pub = nh.advertise<visualization_msgs::Marker>("vscan_linelist", 10);



    private_nh.getParam("detection_range", _detection_range);
    std::cout << "detection_range : " << _detection_range << std::endl;

    private_nh.getParam("threshold_points", _threshold_points);
    std::cout << "threshold_points : " << _threshold_points << std::endl;

    private_nh.getParam("stop_interval", _stop_interval);
    std::cout << "stop_interval : " << _stop_interval << std::endl;

    private_nh.getParam("detection_height_top", _detection_height_top);
    std::cout << "detection_height_top : " << _detection_height_top << std::endl;

    private_nh.getParam("detection_height_bottom", _detection_height_bottom);
    std::cout << "detection_height_bottom : " << _detection_height_bottom << std::endl;

    private_nh.getParam("current_pose_topic", _current_pose_topic);
    std::cout << "current_pose_topic : " << _current_pose_topic << std::endl;

    private_nh.getParam("velocity_limit", _velocity_limit);
    std::cout << "velocity_limit : " << _velocity_limit << std::endl;


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
	
        bool detection_result = ObstacleDetection();


	/* change waypoints to avoid collision */
	/* if detection_result is true,and then it becomes false,we have to Accelerate */
	ChangeWaypoint(detection_result);
	

	/*
        if (_twist_flag == true) {
            geometry_msgs::TwistStamped twist;
            if (detection_result == true) {
                //decelerate
                std::cout << "twist deceleration..." << std::endl;
                twist.twist.linear.x = Decelerate();
                twist.twist.angular.z = _current_twist.twist.angular.z;
            } else {
                //through
                std::cout << "twist through" << std::endl;
                twist.twist = _current_twist.twist;
            }
            std::cout << "twist.linear.x = " << twist.twist.linear.x << std::endl;
            std::cout << "twist.angular.z = " << twist.twist.angular.z << std::endl;
            std::cout << std::endl;

            twist.header.stamp = _current_twist.header.stamp;
            _twist_pub.publish(twist);
        } else {
            std::cout << "no twist topic" << std::endl;
        }
	*/

        loop_rate.sleep();
    }


    return 0;
}
