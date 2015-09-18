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
#include "runtime_manager/ConfigLaneFollower.h"
#include "waypoint_follower/libwaypoint_follower.h"

static const int LOOP_RATE = 30; //Hz
static const double LOOK_AHEAD_THRESHOLD_CALC_RATIO = 3.0; // the next waypoint must be outside of this threshold.
static const double MINIMUM_LOOK_AHEAD_THRESHOLD = 4.0; // the next waypoint must be outside of this threshold.
static const double EVALUATION_THRESHOLD = 1.0; //meter
static const std::string MAP_FRAME = "map";
static const int MODE_WAYPOINT = 0;
static const int MODE_DIALOG = 1;

static bool _sim_mode = false;
static geometry_msgs::PoseStamped _current_pose; // current pose by the global plane.
static double _current_velocity;
static double _prev_velocity = 0;

static ros::Publisher _vis_pub;
static ros::Publisher _stat_pub;
static bool _waypoint_set = false;
static bool _pose_set = false;

static int _param_flag = 0; //0 = waypoint, 1 = Dialog
static double _lookahead_threshold = 4.0; //meter
static double _initial_velocity = 5.0; //km/h
static WayPoints _current_waypoints;
static ros::Publisher _locus_pub;
static ros::Publisher _target_pub;
static ros::Publisher _search_pub;
static ros::Publisher _line_point_pub;

static void ConfigCallback(const runtime_manager::ConfigLaneFollowerConstPtr config)
{
  _param_flag = config->param_flag;
  _lookahead_threshold = config->lookahead_threshold;
  _initial_velocity = config->velocity;
}

static void OdometryPoseCallback(const nav_msgs::OdometryConstPtr &msg)
{
  //std::cout << "odometry callback" << std::endl;

  //
  // effective for testing.
  //
  if (_sim_mode)
  {
    _current_velocity = msg->twist.twist.linear.x;
    _current_pose.header = msg->header;
    _current_pose.pose = msg->pose.pose;

    tf::Transform inverse;
    tf::poseMsgToTF(msg->pose.pose, inverse);
    _pose_set = true;
  }

}

static void NDTCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (!_sim_mode)
  {
    _current_pose.header = msg->header;
    _current_pose.pose = msg->pose;
    _pose_set = true;
  }
}

static void estVelCallback(const std_msgs::Float32ConstPtr &msg)
{
  _current_velocity = kmph2mps(msg->data);
}

static void WayPointCallback(const waypoint_follower::laneConstPtr &msg)
{
	_current_waypoints.setPath(*msg);
  _waypoint_set = true;
  ROS_INFO_STREAM("waypoint subscribed");
}

// display the next waypoint by markers.
static void displayNextWaypoint(int i)
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
static void displayNextTarget(geometry_msgs::Point target)
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
void displayLocus(std::vector<geometry_msgs::Point>locus_array)
{
  visualization_msgs::Marker locus;
  locus.header.frame_id = MAP_FRAME;
  locus.header.stamp = ros::Time();
  locus.ns = "locus_marker";
  locus.id = 0;
  locus.type = visualization_msgs::Marker::LINE_STRIP;
  locus.action = visualization_msgs::Marker::ADD;

  std_msgs::ColorRGBA white;
  white.a = 1.0;
  white.b = 1.0;
  white.r = 1.0;
  white.g = 1.0;

  for(std::vector<geometry_msgs::Point>::iterator it = locus_array.begin() ; it != locus_array.end();it++){
    locus.points.push_back(*it);
    locus.colors.push_back(white);
  }

  locus.scale.x = 0.5;
  locus.color.a = 0.3;
  locus.color.r = 1.0;
  locus.color.g = 0.0;
  locus.color.b = 0.0;
  locus.frame_locked = true;
  _locus_pub.publish(locus);
}

// display the search radius by markers.
static void displaySearchRadius(double search_radius)
{

  visualization_msgs::Marker marker;
  marker.header.frame_id = MAP_FRAME;
  marker.header.stamp = ros::Time();
  marker.ns = "next_target_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position =  _current_pose.pose.position;
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
void displayLinePoint(double slope,double intercept,geometry_msgs::Point target , geometry_msgs::Point target2,geometry_msgs::Point target3)
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

  for(int i = -1000000;i < 1000000 ; ){
    geometry_msgs::Point p;
    p.x = i;
    p.y = slope * i + intercept;
    p.z = _current_pose.pose.position.z;
    line.points.push_back(p);
    i+=10000;
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
  //marker.pose.position = target;
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

//rotation point by degree
geometry_msgs::Point rotatePoint(double degree, geometry_msgs::Point point)
{
  geometry_msgs::Point rotate;
  rotate.x = cos(deg2rad(degree)) * point.x - sin(deg2rad(degree)) * point.y;
  rotate.y = sin(deg2rad(degree)) * point.x + cos(deg2rad(degree)) * point.y;

  return rotate;
}

double getCmdVelocity(int waypoint)
{

  if (_param_flag)
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
  ROS_INFO_STREAM("waypoint : " << mps2kmph(velocity) << " km/h ( " << velocity << "m/s )");
  return velocity;
}

double getLookAheadThreshold(int waypoint)
{
  if (_param_flag)
    return _lookahead_threshold;

  double current_velocity_mps = _current_waypoints.getWaypointVelocityMPS(waypoint);

  if (current_velocity_mps * LOOK_AHEAD_THRESHOLD_CALC_RATIO < MINIMUM_LOOK_AHEAD_THRESHOLD)
    return MINIMUM_LOOK_AHEAD_THRESHOLD;
  else
    return current_velocity_mps * LOOK_AHEAD_THRESHOLD_CALC_RATIO;
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
  ROS_INFO("kappa : %lf", kappa);
  return kappa;

}

double calcRadius(geometry_msgs::Point target)
{
  double radius;
  double denominator = 2 * calcRelativeCoordinate(target, _current_pose.pose).y;
  double numerator = pow(getPlaneDistance(target,_current_pose.pose.position), 2);

  if (denominator != 0)
    radius = numerator / denominator;
  else
    radius = 0;
  ROS_INFO("radius : %lf", radius);
  return radius;
}

static void shorteningThreshold(double *lookahead_threshold)
{
  //threshold shortening
  if (*lookahead_threshold > MINIMUM_LOOK_AHEAD_THRESHOLD)
  {
    // std::cout << "threshold correction" << std::endl;
   *lookahead_threshold -= *lookahead_threshold / 10;
  }
  else
  {
    *lookahead_threshold = MINIMUM_LOOK_AHEAD_THRESHOLD;
  }
  ROS_INFO_STREAM("fixed threshold = " << *lookahead_threshold);
}

//evaluate score between locus and path
bool evaluateLocusFitness(int closest_waypoint,int next_waypoint)
{
	geometry_msgs::Point next_waypoint_position = _current_waypoints.getWaypointPosition(next_waypoint);
  double radius = calcRadius(next_waypoint_position);

  if (radius < 0)
    radius = (-1) * radius;

  tf::Vector3 center;

  //calculate circle of locus
  if (calcRelativeCoordinate(next_waypoint_position,_current_pose.pose).y > 0)
    center = tf::Vector3(0, 0 + radius, 0);
  else
    center = tf::Vector3(0, 0 - radius, 0);

  //evaluation
  double evaluation = 0;
  for (int j = closest_waypoint + 1; j < next_waypoint; j++)
  {
   geometry_msgs::Point tf_p = calcRelativeCoordinate(_current_waypoints.getWaypointPosition(j),_current_pose.pose);
   tf::Vector3 tf_v = point2vector(tf_p);
    tf_v.setZ(0);
    double dt_diff = fabs(tf::tfDistance(center, tf_v) - fabs(radius));
     //std::cout << dt_diff << std::endl;
    if (dt_diff > evaluation)
      evaluation = dt_diff;
  }

  if (evaluation < EVALUATION_THRESHOLD)
    return true;
  else
    return false;

}

//linear interpolation of next target
bool interpolateNextTarget(int next_waypoint, double search_radius, geometry_msgs::Point *next_target)
{
   geometry_msgs::Point zero_p;
   geometry_msgs::Point end = _current_waypoints.getWaypointPosition(next_waypoint);
   geometry_msgs::Point start =_current_waypoints.getWaypointPosition(next_waypoint - 1);

   //get slope of segment end,start
   double slope = (start.y -end.y)/(start.x - end.x);
  // ROS_INFO("slope : %lf ",slope);
   //get intercept of segment end,start
   double intercept = (-1) * slope * end.x + end.y;
 //  ROS_INFO("intercept : %lf ",intercept);

   //distance between the foot of a perpendicular line and the center of circle
   double d = fabs(_current_pose.pose.position.y - slope * _current_pose.pose.position.x - intercept) / sqrt(1 + pow(slope,2));
  // ROS_INFO("distance : %lf ",d);

   if(d > search_radius)
     return false;

   //unit vector of point 'start' to point 'end'
   tf::Vector3 v((end.x - start.x),(end.y-start.y),0);
   tf::Vector3 unit_v = v.normalize();

   //normal unit vector of v
   double deg1 = 90;
   tf::Vector3 w1(cos(deg2rad(deg1)) * v.getX() - sin(deg2rad(deg1)) * v.getY(),sin(deg2rad(deg1)) * v.getX() + cos(deg2rad(deg1)) * v.getY(),0);
   tf::Vector3 unit_w1 = w1.normalize();

   double deg2 = -90;
   tf::Vector3 w2(cos(deg2rad(deg2)) * v.getX() - sin(deg2rad(deg2)) * v.getY(),sin(deg2rad(deg2)) * v.getX() + cos(deg2rad(deg2)) * v.getY(),0);
   tf::Vector3 unit_w2 = w2.normalize();

   //the foot of a perpendicular line
   geometry_msgs::Point h1;
   h1.x = _current_pose.pose.position.x + d * unit_w1.getX();
   h1.y = _current_pose.pose.position.y + d * unit_w1.getY();
   h1.z = _current_pose.pose.position.z;

   geometry_msgs::Point h2;
   h2.x = _current_pose.pose.position.x + d * unit_w2.getX();
   h2.y = _current_pose.pose.position.y + d * unit_w2.getY();
   h2.z = _current_pose.pose.position.z;

   double error = pow(10,-5); //0.00001
  // ROS_INFO("error : %lf",error);
  // ROS_INFO("whether h1 on line : %lf", h1.y - (slope * h1.x + intercept) );
 //  ROS_INFO("whether h2 on line : %lf",h2.y - (slope * h2.x + intercept) );

   geometry_msgs::Point h;
   if(fabs(h1.y - (slope * h1.x + intercept)) < error){
     h = h1;
  //   ROS_INFO("use h1");
   }else if(fabs(h2.y - (slope * h2.x + intercept)) < error ){
  //   ROS_INFO("use h2");
     h = h2;
   }else
   {
     return false;
   }

   if(d == search_radius)
   {
     *next_target = h;
     return true;
   }else{

     double s = sqrt(pow(search_radius,2) - pow(d,2));
     geometry_msgs::Point target1;
     target1.x = h.x + s* unit_v.getX();
     target1.y = h.y + s* unit_v.getY();
     target1.z = _current_pose.pose.position.z;
  //   ROS_INFO("target1 : ( %lf , %lf , %lf)",target1.x,target1.y,target1.z);
     geometry_msgs::Point target2;
     target2.x = h.x - s* unit_v.getX();
     target2.y = h.y - s* unit_v.getY();
     target2.z = _current_pose.pose.position.z;
 //    ROS_INFO("target2 : ( %lf , %lf , %lf)",target2.x,target2.y,target2.z);

     displayLinePoint(slope,intercept,target1,target2 ,h); //debug tool

     if(calcRelativeCoordinate(target1,_current_pose.pose).x >0)
     {


   //    ROS_INFO("result : target1");
       *next_target = target1;
       return true;
     }
     else if(calcRelativeCoordinate(target2,_current_pose.pose).x > 0)
     {
   //    ROS_INFO("result : target2");

       *next_target = target2;
       return true;
     }
     else
     {
   //    ROS_INFO("result : false ");
       return false;
     }
   }
}

geometry_msgs::Point getNextTarget(double closest_waypoint)
{
  static int next_waypoint = -1;
  int path_size = static_cast<int>(_current_waypoints.getSize());
  geometry_msgs::Point next_target;
  geometry_msgs::Point point_zero;
  double lookahead_threshold = getLookAheadThreshold(closest_waypoint);
  //ROS_INFO_STREAM("threshold = " << lookahead_threshold);


  // if waypoints are not given, do nothing.
  if (_current_waypoints.isEmpty()){
    next_waypoint = -1;
    ROS_INFO_STREAM("next waypoint = " << next_waypoint << "/" << path_size - 1);
    return point_zero;
  }

  // look for the next waypoint.
  int i = closest_waypoint;
  while(i < path_size)
  {

    //if search waypoint is last
    if(i == (path_size -1)){
      next_waypoint = i;
      break;
    }

    //if threshold is  distance of previous waypoint
    if (next_waypoint > 0 && _param_flag == MODE_WAYPOINT && 
        getPlaneDistance(_current_waypoints.getWaypointPosition(next_waypoint), _current_pose.pose.position) > lookahead_threshold)
        break;

    // if there exists an effective waypoint
    if (getPlaneDistance(_current_waypoints.getWaypointPosition(i), _current_pose.pose.position) > lookahead_threshold)
    {
      //param flag is waypoint
      if (_param_flag  == MODE_DIALOG || evaluateLocusFitness(closest_waypoint,i))
      {
        next_waypoint = i;
        break;
      }

      //threshold shortening
      shorteningThreshold(&lookahead_threshold);
      if(lookahead_threshold == MINIMUM_LOOK_AHEAD_THRESHOLD){
        next_waypoint = i;
        break;       
      }

      //restart search from closest_waypoint
      i = closest_waypoint;
    }
    i++;
  }

  if(next_waypoint != -1){

    ROS_INFO_STREAM("next waypoint = " << next_waypoint << "/" << path_size - 1);
    displayNextWaypoint(next_waypoint);


    if(next_waypoint  == (path_size -1))
      return _current_waypoints.getWaypointPosition(next_waypoint);

    displaySearchRadius(lookahead_threshold);
    interpolateNextTarget(next_waypoint, lookahead_threshold, &next_target);

#if 1 /* log */
      std::ofstream ofs("/tmp/pure_pursuit.log", std::ios::app);
      ofs << _current_waypoints.getWaypointPosition(next_waypoint).x << " "  << _current_waypoints.getWaypointPosition(next_waypoint).y << " " <<  next_target.x << " " << next_target.y <<  std::endl;
#endif

    return next_target;
  }

  // if the program reaches here, it means we lost the waypoint.
  return point_zero;
}

geometry_msgs::Twist calcTwist(double curvature, double cmd_velocity)
{
  geometry_msgs::Twist twist;

  twist.linear.x = cmd_velocity;
  twist.angular.z = _current_velocity * curvature;
  //twist.angular.z = cmd_velocity * curvature;
  return twist;
}

//generate the locus of pure pursuit
std::vector<geometry_msgs::Point> generateLocus(geometry_msgs::Point target)
{
  std::vector<geometry_msgs::Point> locus_array;
  double radius = calcRadius (target);
  //if(radius < 0)
  //  radius = radius * (-1);
  double theta;
  if (radius != 0)
    theta = 2 * asin (getPlaneDistance (target, _current_pose.pose.position) / (2 * radius));
  else
    theta = 0;

  //ROS_INFO("radius : %lf", radius);
  //ROS_INFO("theta : %lf", theta);
  int step = 100;
  for (double i = theta/step; fabs(i) < fabs(theta); i+=theta/step)
  {
    //   ROS_INFO("loop : %d", i);
    //calc a point of circumference
    geometry_msgs::Point p;
    p.x = radius * cos (i);
    p.y = radius * sin (i);
    //    ROS_INFO("p : (%lf , %lf )", p.x, p.y);

    //transform to (radius,0)
    geometry_msgs::Point relative_p;
    relative_p.x = p.x - radius;
    relative_p.y = p.y;

    //rotate -90°
    geometry_msgs::Point rotate_p = rotatePoint (-90, relative_p);

    //    ROS_INFO("relative point : (%lf , %lf )", rotate_p.x, rotate_p.y);

    //transform to vehicle plane
    geometry_msgs::Point tf_p = calcAbsoluteCoordinate (rotate_p, _current_pose.pose);
    //   ROS_INFO("locus : (%lf , %lf )", tf_p.x, tf_p.y);

    locus_array.push_back (tf_p);

  }
  return locus_array;

}

int main(int argc, char **argv)
{
  ROS_INFO_STREAM("pure pursuit start");

  // set up ros
  ros::init(argc, argv, "pure_pursuit");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // setting params
  private_nh.getParam("sim_mode", _sim_mode);
  ROS_INFO_STREAM("sim_mode : " << _sim_mode);

  //publish topic
  ros::Publisher cmd_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
  _vis_pub = nh.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
  _stat_pub = nh.advertise<std_msgs::Bool>("wf_stat", 0);
  _target_pub = nh.advertise<visualization_msgs::Marker>("next_target_mark", 0);
  _search_pub = nh.advertise<visualization_msgs::Marker>("search_circle", 0);
  _line_point_pub = nh.advertise<visualization_msgs::Marker>("line_point_mark", 0); //debug tool
  _locus_pub = nh.advertise<visualization_msgs::Marker>("locus_mark", 0);

  //subscribe topic
  ros::Subscriber waypoint_subcscriber = nh.subscribe("safety_waypoint", 10, WayPointCallback);
  ros::Subscriber odometry_subscriber = nh.subscribe("odom_pose", 10, OdometryPoseCallback);
  ros::Subscriber ndt_subscriber = nh.subscribe("control_pose", 10, NDTCallback);
  ros::Subscriber estimated_vel_subscriber = nh.subscribe("estimated_vel_mps", 10, estVelCallback);
  ros::Subscriber config_subscriber = nh.subscribe("config/waypoint_follower", 10, ConfigCallback);

  geometry_msgs::TwistStamped twist;
  ros::Rate loop_rate(LOOP_RATE); // by Hz
 // bool endflag = false;

  while (ros::ok())
  {
    std_msgs::Bool wf_stat;

    ros::spinOnce();

    //check topic
    if (_waypoint_set == false || _pose_set == false)
    {
      ROS_INFO_STREAM("topic waiting...");
      loop_rate.sleep();
      continue;
    }

    //get closest waypoint
    int closest_waypoint = getClosestWaypoint(_current_waypoints.getCurrentWaypoints(),_current_pose.pose);
    ROS_INFO_STREAM("closest waypoint = " << closest_waypoint);

      //if can get closest waypoint
    if (closest_waypoint > 0)
    {
      //get next target
      geometry_msgs::Point next_target = getNextTarget(closest_waypoint);
      ROS_INFO("next_target : ( %lf , %lf , %lf)", next_target.x, next_target.y, next_target.z);

      if (next_target.x == 0 && next_target.y == 0 && next_target.z == 0)
      {
        ROS_INFO_STREAM("lost waypoint! ");
        wf_stat.data = false;
        _stat_pub.publish(wf_stat);
        twist.twist.linear.x = 0;
        twist.twist.angular.z = 0;
      }
      else
      {
        displayNextTarget(next_target);
        std::vector<geometry_msgs::Point> locus_array = generateLocus(next_target);
        displayLocus(locus_array);
        twist.twist = calcTwist(calcCurvature(next_target), getCmdVelocity(closest_waypoint));
        wf_stat.data = true;
        _stat_pub.publish(wf_stat);
      }
    }
    else //cannot get closest
    {
      ROS_INFO_STREAM("closest waypoint cannot detected !!");
      wf_stat.data = false;
      _stat_pub.publish(wf_stat);
      twist.twist.linear.x = 0;
      twist.twist.angular.z = 0;
    }

    ROS_INFO_STREAM(
        "twist(linear.x , angular.z) = ( " << twist.twist.linear.x << " , " << twist.twist.angular.z << " )");
    twist.header.stamp = ros::Time::now();
    cmd_velocity_publisher.publish(twist);
    _prev_velocity = twist.twist.linear.x;
    loop_rate.sleep();
  }

  return 0;
}
