/*
 *  trajectory_generator.cpp
 *  Navigation via Cubic Spline Generation 
 *
 *  Created by Matthew O'Kelly on 7/17/15.
 *  Copyright (c) 2015 Matthew O'Kelly. All rights reserved.
 *  mokelly@seas.upenn.edu
 *  
*/

/*
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
 *
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
#include <std_msgs/Float64MultiArray.h>
#include "runtime_manager/ConfigLaneFollower.h"
#include "waypoint_follower/libwaypoint_follower.h"
#include "libtraj_gen.h"
#include "vehicle_socket/CanInfo.h"

#define DEBUG_TRAJECTORY_GEN
#define WHEEL_ANGLE_MAX (31.28067)
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX/WHEEL_ANGLE_MAX)
#define STEERING_ANGLE_MAX (666.00000)
#define WHEEL_BASE (2.70)

#define DEBUG_TRAJECTORY_GEN

static const int LOOP_RATE = 10; //Hz
static const double LOOK_AHEAD_THRESHOLD_CALC_RATIO = 3.0; // the next waypoint must be outside of this threshold.
static const double MINIMUM_LOOK_AHEAD_THRESHOLD = 4.0; // the next waypoint must be outside of this threshold.
static const double EVALUATION_THRESHOLD = 1.0; //meter
static const std::string MAP_FRAME = "map";
static const std::string SIM_BASE_FRAME = "sim_base_link";
static const std::string BASE_FRAME = "base_link";
static ros::Publisher marker_pub;
static int SPLINE_INDEX=0;

//define class
class PathPP: public Path
{
private:
  int next_waypoint_;

  int param_flag_; //0 = waypoint, 1 = Dialog
  double lookahead_threshold_; //meter
  double initial_velocity_; //km/h

public:
  PathPP()
  {
    param_flag_ = 0;
    lookahead_threshold_ = 4.0;
    initial_velocity_ = 5.0;
    next_waypoint_ = -1;
  }
  void setConfig(const runtime_manager::ConfigLaneFollowerConstPtr &config);
  double getCmdVelocity();
  double getLookAheadThreshold(int waypoint);
  int getNextWaypoint();
  double calcRadius(int waypoint);
  bool evaluateWaypoint(int next_waypoint);
  void displayTrajectory(tf::Vector3 center, double radius);
  double getNextWaypointVelocity(int next_waypoint);
  union Spline waypointTrajectory(union State veh, union State goal, union Spline curvature, int next_waypoint);
  union State computeWaypointGoal(int next_waypoint);
  union State computeVeh(int old_time, double old_theta, int next_waypoint);
  void drawSpline(union Spline curvature, union State veh, int flag, int selected);


};
PathPP _path_pp;

static bool _sim_mode = false;
static geometry_msgs::PoseStamped _current_pose; // current pose by the global plane.
static double _current_velocity;
static double _current_angular_velocity;
static double _can_info_curvature;
static double _prev_velocity = 0;
static ros::Publisher _vis_pub;
static ros::Publisher _traj_pub;
static ros::Publisher _stat_pub;
static bool _waypoint_set = false;
static bool _pose_set = false;

void PathPP::setConfig(const runtime_manager::ConfigLaneFollowerConstPtr &config)
{
  initial_velocity_ = config->velocity;
  param_flag_ = config->param_flag;
  lookahead_threshold_ = config->lookahead_threshold;
}

//get velocity of the closest waypoint or from config
double PathPP::getCmdVelocity()
{

  if (param_flag_)
  {
    ROS_INFO_STREAM("dialog : " << initial_velocity_ << " km/h (" << kmph2mps(initial_velocity_) << " m/s )");
    return kmph2mps(initial_velocity_);
  }
  if (current_path_.waypoints.empty())
  {
    return 0;
  }

  double velocity = current_path_.waypoints[closest_waypoint_].twist.twist.linear.x;
  return velocity;
}

double PathPP::getNextWaypointVelocity(int next_waypoint)
{
  if (current_path_.waypoints.empty())
  {
    ROS_INFO_STREAM("waypoint : not loaded path");
    return 0;
  }

  double velocity = current_path_.waypoints[next_waypoint].twist.twist.linear.x;
  return velocity;
}

double PathPP::getLookAheadThreshold(int waypoint)
{
  if (param_flag_)
    return lookahead_threshold_;

  double current_velocity_mps = current_path_.waypoints[waypoint].twist.twist.linear.x;
  if (current_velocity_mps * LOOK_AHEAD_THRESHOLD_CALC_RATIO < MINIMUM_LOOK_AHEAD_THRESHOLD)
    return MINIMUM_LOOK_AHEAD_THRESHOLD;
  else if (getDistance(waypoint) < 0.5)
    return current_velocity_mps * LOOK_AHEAD_THRESHOLD_CALC_RATIO;
  else
    return current_velocity_mps * LOOK_AHEAD_THRESHOLD_CALC_RATIO + getDistance(waypoint) * 3.0;
}

/////////////////////////////////////////////////////////////////
// obtain the next "effective" waypoint.
// the vehicle drives itself toward this waypoint.
/////////////////////////////////////////////////////////////////
int PathPP::getNextWaypoint()
{
  // if waypoints are not given, do nothing.
  if (!getPathSize())
    return -1;

  double lookahead_threshold = getLookAheadThreshold(closest_waypoint_);

  //ROS_INFO_STREAM("threshold = " << lookahead_threshold);
  // look for the next waypoint.
  for (int i = closest_waypoint_; i < getPathSize(); i++)
  {

    //if threshold is  distance of previous waypoint
    if (next_waypoint_ > 0)
    {
      if (getDistance(next_waypoint_) > lookahead_threshold)
      {
        //ROS_INFO_STREAM("threshold = " << lookahead_threshold);
        return next_waypoint_;
      }
    }

    // if there exists an effective waypoint
    if (getDistance(i) > lookahead_threshold)
    {

      //if param flag is waypoint
      if (!param_flag_)
      {

        if (evaluateWaypoint(i) == true)
        {
          //ROS_INFO_STREAM("threshold = " << lookahead_threshold);
          next_waypoint_ = i;
          return i;
        }
        else
        {
          //restart search from closest_waypoint
          i = closest_waypoint_;

          //threshold shortening
          if (lookahead_threshold > MINIMUM_LOOK_AHEAD_THRESHOLD)
          {
            // std::cout << "threshold correction" << std::endl;
            lookahead_threshold -= lookahead_threshold / 10;
            //ROS_INFO_STREAM("fixed threshold = " << lookahead_threshold);
          }
          else
          {
            lookahead_threshold = MINIMUM_LOOK_AHEAD_THRESHOLD;
          }
        }
      }
      else
      {
        //ROS_INFO_STREAM("threshold = " << lookahead_threshold);
        next_waypoint_ = i;
        return i;
      }
    }
  }

  // if the program reaches here, it means we lost the waypoint.
  return -1;
}

bool PathPP::evaluateWaypoint(int next_waypoint)
{

  double radius = calcRadius(next_waypoint);
  // std::cout << "radius "<< radius << std::endl;
  if (radius < 0)
    radius = (-1) * radius;

  tf::Vector3 center;

  //calculate circle of trajectory
  if (transformWaypoint(next_waypoint).getY() > 0)
    center = tf::Vector3(0, 0 + radius, 0);
  else
    center = tf::Vector3(0, 0 - radius, 0);

  displayTrajectory(center, radius);

  //evaluation
  double evaluation = 0;
  for (int j = closest_waypoint_ + 1; j < next_waypoint; j++)
  {
    tf::Vector3 tf_waypoint = transformWaypoint(j);
    tf_waypoint.setZ(0);
    double dt_diff = fabs(tf::tfDistance(center, tf_waypoint) - fabs(radius));
    // std::cout << dt_diff << std::endl;
    if (dt_diff > evaluation)
      evaluation = dt_diff;
  }

  if (evaluation < EVALUATION_THRESHOLD)
    return true;
  else
    return false;

}

// display the trajectory by markers.
void PathPP::displayTrajectory(tf::Vector3 center, double radius)
{

  geometry_msgs::Point point;
  tf::Vector3 inv_center = transform_.inverse() * center;

  point.x = inv_center.getX();
  point.y = inv_center.getY();
  point.z = inv_center.getZ();

  visualization_msgs::Marker traj;
  traj.header.frame_id = MAP_FRAME;
  traj.header.stamp = ros::Time();
  traj.ns = "trajectory_marker";
  traj.id = 0;
  traj.type = visualization_msgs::Marker::SPHERE;
  traj.action = visualization_msgs::Marker::ADD;
  traj.pose.position = point;
  traj.scale.x = radius * 2;
  traj.scale.y = radius * 2;
  traj.scale.z = 1.0;
  traj.color.a = 0.3;
  traj.color.r = 1.0;
  traj.color.g = 0.0;
  traj.color.b = 0.0;
  traj.frame_locked = true;
  _traj_pub.publish(traj);
}

double PathPP::calcRadius(int waypoint)
{
  return pow(getDistance(waypoint), 2) / (2 * transformWaypoint(waypoint).getY());
}

static void ConfigCallback(const runtime_manager::ConfigLaneFollowerConstPtr config)
{
  _path_pp.setConfig(config);
}

static void OdometryPoseCallback(const nav_msgs::OdometryConstPtr &msg)
{
  if (_sim_mode)
  {
    _current_velocity = msg->twist.twist.linear.x;
    _current_angular_velocity = msg->twist.twist.angular.z;
    _current_pose.header = msg->header;
    _current_pose.pose = msg->pose.pose;

    tf::Transform inverse;
    tf::poseMsgToTF(msg->pose.pose, inverse);
    _path_pp.setTransform(inverse.inverse());
    _pose_set = true;
    //   std::cout << "transform2 (" << _transform2.getOrigin().x() << " " <<  _transform2.getOrigin().y() << " " <<  _transform2.getOrigin().z() << ")" << std::endl;
  }

}

static void NDTCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (!_sim_mode)
  {
    _current_pose.header = msg->header;
    _current_pose.pose = msg->pose;
    tf::Transform inverse;
    tf::poseMsgToTF(msg->pose, inverse);
    _path_pp.setTransform(inverse.inverse());
    _pose_set = true;
  }
}

static void estVelCallback(const std_msgs::Float32ConstPtr &msg)
{
  //_current_velocity = msg->data;
}

static void WayPointCallback(const waypoint_follower::laneConstPtr &msg)
{
  _path_pp.setPath(msg);
  _waypoint_set = true;
  ROS_INFO_STREAM("waypoint subscribed");
}

static void canInfoCallback(const vehicle_socket::CanInfoConstPtr &msg)
{
  double steering_wheel_angle = msg->angle;
  _current_velocity = (msg->speed)*(1000.00/3600);
  steering_wheel_angle = steering_wheel_angle*(3.1496/180.00);
  _can_info_curvature = (steering_wheel_angle / (double) WHEEL_TO_STEERING) / WHEEL_BASE;
  ROS_INFO_STREAM("Steering Wheel Angle: "<<steering_wheel_angle);
  ROS_INFO_STREAM("Curvature from CAN: "<<_can_info_curvature);
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
  marker.pose.position = _path_pp.getWaypointPosition(i);
  marker.pose.orientation = _path_pp.getWaypointOrientation(i);
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

/////////////////////////////////////////////////////////////////
// Safely stop the vehicle.
/////////////////////////////////////////////////////////////////
static geometry_msgs::Twist stopControl()
{
  geometry_msgs::Twist twist;

  double lookahead_distance = _path_pp.getDistance(_path_pp.getPathSize() - 1);

  double stop_interval = 3;
  if (lookahead_distance < stop_interval)
  {
    twist.linear.x = 0;
    twist.angular.z = 0;
    return twist;
  }

  twist.linear.x = DecelerateVelocity(lookahead_distance, _prev_velocity);

  if (twist.linear.x < 1.0)
  {
    twist.linear.x = 0;
  }

  double radius = _path_pp.calcRadius(_path_pp.getPathSize() - 1);

  if (radius > 0 || radius < 0)
  {
    twist.angular.z = twist.linear.x / radius;
  }
  else
  {
    twist.angular.z = 0;
  }
  return twist;

}

//////////////////////////////////////////////////////////////////////////////////////////////////
// M. O'Kelly code begins here, 
// Suggested: need to clean up unused functions above... don't have time.
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
// Compute the goal state of the vehicle
/////////////////////////////////////////////////////////////////
union State computeWaypointGoal(int next_waypoint)
{
    union State l_goal;

    // Get the next waypoint position with respect to the vehicles frame
    l_goal.sx = _path_pp.transformWaypoint(next_waypoint).getX();
    l_goal.sy = _path_pp.transformWaypoint(next_waypoint).getY();

    // Get the next next waypoint position
    double waypoint_lookahead_1_x = _path_pp.transformWaypoint(next_waypoint+1).getX();
    double waypoint_lookahead_1_y = _path_pp.transformWaypoint(next_waypoint+1).getY();

    // Get the next next next waypoint
    double waypoint_lookahead_2_x = _path_pp.transformWaypoint(next_waypoint+2).getX();
    double waypoint_lookahead_2_y = _path_pp.transformWaypoint(next_waypoint+2).getY();

    // Compute dX and dY relative to lookahead
    double dX_1 =  waypoint_lookahead_1_x - l_goal.sx;
    double dY_1 =  waypoint_lookahead_1_y - l_goal.sy;

    // Compute dX and dY relative to lookahead and next lookahead
    double dX_2 =  waypoint_lookahead_2_x - waypoint_lookahead_1_x;
    double dY_2 =  waypoint_lookahead_2_y - waypoint_lookahead_1_y;

    // Estimate desired orientation of the vehicle at next waypoint
    l_goal.theta = atan(dY_1/dX_1);

    // Should we use lookahead and next waypoint 
    // or current and next as below:
    //l_goal.theta = atan(l_goal.sy/l_goal.sx);


    // Estimate the desired orientation at the next next waypoint
    double theta_lookahead = atan(dY_2/dX_2);

    // Estimate the arc length
    double ds = sqrt(pow(waypoint_lookahead_1_x - waypoint_lookahead_2_x, 2) + pow(waypoint_lookahead_1_x - waypoint_lookahead_2_y, 2));
    
    // Angle
    double angle = (theta_lookahead-l_goal.theta)/2.00;

    // Estimate Kappa
    double estimate = 2.00*sin(angle)/ds;
    
    l_goal.kappa = estimate;

    // Note we limit kappa from being too extreme
    // 10.0 was arbitrary, we really need a better curvature estimate
    l_goal.kappa = min((double)kmax/10.0, l_goal.kappa);

    l_goal.kappa = max ((double)kmin/10.0, l_goal.kappa); 
  
    // Get the desired velocity at the next waypoint
    l_goal.v = _path_pp.getNextWaypointVelocity(next_waypoint);

    return l_goal;
}

/////////////////////////////////////////////////////////////////
// Compute current state of the vehicle
/////////////////////////////////////////////////////////////////
union State computeVeh(int old_time, double old_theta, int next_waypoint)
{
    union State l_veh;

    // Goal is computed relative to vehicle coordinate frame
    l_veh.sx=0.0;
    l_veh.sy=0.0;

    // Compute yaw (orientation) relative to the world coordinate frame
    // Necessary to compute curvature, but note that in the local coordinate frame yaw is still 0.0
    tf::Quaternion q(_current_pose.pose.orientation.x,_current_pose.pose.orientation.y,_current_pose.pose.orientation.z,_current_pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    // Check this later:
    //double local_theta=yaw;

    // Because we are on the coordinate system of the base frame
    l_veh.theta = 0.0;

    // Get the current velocity of the vehicle
    l_veh.v = _current_velocity;

    // Get the desired velocity
    l_veh.vdes = _path_pp.getNextWaypointVelocity(next_waypoint);
    ROS_INFO_STREAM("Desired Velocity: "<< l_veh.vdes);

    // Not using timing related stuff for now...
    // In order to estimate the curvature we need to compute the arc length we have travelled on
    // Thus we need to estimate dt
    // l_veh.timestamp = _current_pose.header.stamp.nsec;
    //double localElapsedTime=((double)l_veh.timestamp-old_time)/1000000000.00;

    // ds is the change in position along the arc
    //double ds = localElapsedTime*veh.v;

    // Should get the current steering angle instead
    // Steering angle ~= kappa 
    // Ask Ohta-san?
    double w = _current_angular_velocity;
    ROS_INFO_STREAM("Current omega: " <<w);
    l_veh.kappa = w/l_veh.vdes;


    if (!_sim_mode)
    {
     l_veh.kappa = _can_info_curvature;
     ROS_INFO_STREAM("Current kappa: " <<l_veh.kappa);
    }

    return l_veh;
}

/////////////////////////////////////////////////////////////////
// Compute trajectory
/////////////////////////////////////////////////////////////////
union Spline waypointTrajectory(union State veh, union State goal, union Spline curvature, int next_waypoint)
{
    curvature.success=TRUE;  
    bool convergence=FALSE;
    int iteration = 0;
    union State veh_next;
    double dt = step_size;
    veh.v=goal.v;

    // While loop for computing trajectory parameters
    while(convergence == FALSE && iteration<4)
    {
        // Set time horizon
        double horizon = curvature.s/veh.vdes;
        ROS_INFO_STREAM("vdes: " << veh.vdes);
        ROS_INFO_STREAM("horizon: " << horizon);

        // Run motion model
        veh_next = motionModel(veh, goal, curvature, dt, horizon, 0);
        
        // Determine convergence criteria
        convergence = checkConvergence(veh_next, goal);

        // If the motion model doesn't get us to the goal compute new parameters
        if(convergence==FALSE)
        {
            // Update parameters
            curvature = generateCorrection(veh, veh_next, goal, curvature, dt, horizon);
            iteration++;

            // Escape route for poorly conditioned Jacobian
            if(curvature.success==FALSE)
            {
                ROS_INFO_STREAM("Init State: sx "<<veh.sx<<" sy " <<veh.sy<<" theta "<<veh.theta<<" kappa "<<veh.kappa<<" v "<<veh.v);
                ROS_INFO_STREAM("Goal State: sx "<<goal.sx<<" sy " <<goal.sy<<" theta "<<goal.theta<<" kappa "<<goal.kappa<<" v"<<goal.v);
                break;
            }
        }    
    }

    if(convergence==FALSE)
    {
      ROS_INFO_STREAM("Next State: sx "<<veh_next.sx<<" sy " <<veh_next.sy<<" theta "<<veh_next.theta<<" kappa "<<veh_next.kappa<<" v "<<veh_next.v);
      ROS_INFO_STREAM("Init State: sx "<<veh.sx<<" sy " <<veh.sy<<" theta "<<veh.theta<<" kappa "<<veh.kappa);
      ROS_INFO_STREAM("Goal State: sx "<<goal.sx<<" sy " <<goal.sy<<" theta "<<goal.theta<<" kappa "<<goal.kappa);
      curvature.success= FALSE;
    }

    else
    {
        ROS_INFO_STREAM("Converged in "<<iteration<<" iterations");

        #ifdef LOG_OUTPUT
        // Set time horizon
         double horizon = curvature.s/v_0;
        // Run motion model and log data for plotting
        veh_next = motionModel(veh, goal, curvature, 0.1, horizon, 1);
        fmm_sx<<"0.0 \n";
        fmm_sy<<"0.0 \n";
        #endif
    }

    return curvature;
}

/////////////////////////////////////////////////////////////////
// Draw Spline
/////////////////////////////////////////////////////////////////
void drawSpline(union Spline curvature, union State veh, int flag, int selected)
{
  static double vdes=veh.vdes;
  // Setup up line strips
  visualization_msgs:: Marker line_strip;
  if (!_sim_mode)
  {
    line_strip.header.frame_id = BASE_FRAME;
  }
  else
  {
    line_strip.header.frame_id = SIM_BASE_FRAME;
  }
  line_strip.header.stamp = ros::Time();
  line_strip.action = visualization_msgs::Marker::ADD;


  // Define message id and scale (thickness)
  line_strip.id = flag;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  

  // Set the color and transparency (blue and solid) if not selected
  if(selected>0)
  {
    line_strip.scale.x = 0.08;
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;
  }

  // Set the color and transparency (green and solid) if selected  
  else
  {
    line_strip.scale.x = 0.1;
    line_strip.color.g= 1.0;
    line_strip.color.a = 1.0;
  }


  // Init temp state for storing results of genLineStrip
  union State temp;

  // Init time
  double sim_time = 0.0;

  // Figure out sim time horizon
  double horizon = curvature.s/veh.vdes;

  // Init points
  geometry_msgs::Point p;

  // Create veritices
  while(sim_time<horizon && curvature.success==TRUE)
  {
    temp = genLineStrip(veh, curvature, vdes, sim_time);
    p.x = temp.sx;
    p.y = temp.sy;
    p.z = 0.0;
    line_strip.points.push_back(p);
    veh= temp;
    sim_time = sim_time+ plot_step_size;
  }

  // Publish trajectory line strip (to RViz)
  marker_pub.publish(line_strip);

}

/////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  // These are local variables for keeping track of the previous
  // timestamp and orientation. Used in the estimate of curvature...
  int old_time=0;
  double old_theta=0.0;

  // Write to console that we are starting trajectory generation
  ROS_INFO_STREAM("Trajectory Generation Begins: ");

  // Set up ROS, TO DO: change to proper name (same with rest of the file)
  ros::init(argc, argv, "pure_pursuit");

  // Create node handles 
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Set the parameters for the node
  private_nh.getParam("sim_mode", _sim_mode);
  ROS_INFO_STREAM("sim_mode : " << _sim_mode);

  // Publish the following topics: 
  // Velocity commands, waypoint markers, trajectory marker, and lf_stat (? MOK not sure what this is)
  // NOTE: MOK commented out velocity command publisher, because that will be done in command_converter now
  // ros::Publisher cmd_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
  _vis_pub = nh.advertise<visualization_msgs::Marker>("next_waypoint_mark", 1);
  _traj_pub = nh.advertise<visualization_msgs::Marker>("trajectory_mark", 0);
  _stat_pub = nh.advertise<std_msgs::Bool>("wf_stat", 0);

  // Publish the curvature information:
  ros::Publisher spline_parameters_pub = nh.advertise<std_msgs::Float64MultiArray>("spline", 10);
  ros::Publisher state_parameters_pub = nh.advertise<std_msgs::Float64MultiArray>("state", 10);

  // Publish the trajectory visualization
  marker_pub = nh.advertise<visualization_msgs::Marker>("cubic_splines_viz", 10);

  // Subscribe to the following topics: 
  // waypoints, odometry, localization, velocity estimate, lane follower (MOK not sure of the purpose)
  //ros::Subscriber waypoint_subcscriber = nh.subscribe("base_waypoint", 10, WayPointCallback);
  ros::Subscriber waypoint_subcscriber = nh.subscribe("safety_waypoint", 10, WayPointCallback);
  ros::Subscriber odometry_subscriber = nh.subscribe("odom_pose", 10, OdometryPoseCallback);
  ros::Subscriber ndt_subscriber = nh.subscribe("control_pose", 10, NDTCallback);
  ros::Subscriber estimated_vel_subscriber = nh.subscribe("estimated_vel", 10, estVelCallback);
  //ros::Subscriber config_subscriber = nh.subscribe("config/lane_follower", 10, ConfigCallback);
  ros::Subscriber config_subscriber = nh.subscribe("config/waypoint_follower", 10, ConfigCallback);
  ros::Subscriber can_info = nh.subscribe("can_info", 10, canInfoCallback);

  // Local variable for geometry messages
  geometry_msgs::TwistStamped twist;

  // Set the loop rate unit is Hz
  ros::Rate loop_rate(LOOP_RATE); 

  // Initialize endflag to false
  bool endflag = false;

  // Set up arrays for perturb and flag to enable easy
  // parallelization via OpenMP pragma
  double perturb[30];
  perturb[0]=-3.00;

  int flag[30];
  flag[0]=1;

  for(int i=1; i<30; i++)
  {
    perturb[i] = perturb[i-1] + 0.2;
    flag[i-1] = flag[i]+1;
  }
  bool initFlag = FALSE;
  union Spline prev_curvature;
  union State veh_fmm;

  // Here we go....
  while (ros::ok())
  {
    std_msgs::Bool _lf_stat;

    ros::spinOnce();

    // Wait for waypoints (in Runtime Manager) and pose to be set (in RViz)
    if (_waypoint_set == false || _pose_set == false)
    {
      ROS_INFO_STREAM("topic waiting...");
      loop_rate.sleep();
      continue;
    }

    // Get the closest waypoinmt 
    int closest_waypoint = _path_pp.getClosestWaypoint();
    static int prev_closest_waypoint = 0;
    if (closest_waypoint < 0)
    {
      closest_waypoint = prev_closest_waypoint+5;
      prev_closest_waypoint = closest_waypoint;
    }
    
    
    ROS_INFO_STREAM("closest waypoint = " << closest_waypoint);

    // Check to make sure that the vehicle has not reached the end
    // of the mission
    if (!endflag)
    {
      // If the current  waypoint has a valid index
      if (closest_waypoint > 0)
      {
        // Return the next waypoint
        int next_waypoint = _path_pp.getNextWaypoint();
        ROS_INFO_STREAM("Next waypoint: "<<next_waypoint);

        // If the next waypoiont also has a valid index
        if (next_waypoint > 0)
        {
          // Display and publish waypoint information 
          displayNextWaypoint(next_waypoint);
          _lf_stat.data = true;
          _stat_pub.publish(_lf_stat);

          // Determine the desired state of the vehicle at the next waypoint 
          union State goal = computeWaypointGoal(next_waypoint);
          
          // Estimate the current state of the vehicle
          union State veh = computeVeh(old_time, old_theta, next_waypoint);

          if(initFlag==TRUE && prev_curvature.success==TRUE)
          {
            veh_fmm = nextState(veh, prev_curvature, veh.vdes, 0.2, 0);
            //veh.kappa = veh_fmm.kappa;
            ROS_INFO_STREAM("est kappa: " <<veh_fmm.kappa);
          }
        
          // Initialize the estimate for the curvature
          union Spline curvature = initParams(veh, goal);

          // Generate a cubic spline (trajectory) for the vehicle to follow
          curvature = waypointTrajectory(veh, goal, curvature, next_waypoint);
          prev_curvature = curvature;
          initFlag=TRUE;

          // Check that we got a result and publish it or stream expletive to screen
          if(curvature.success==TRUE)
          { 

            std_msgs::Float64MultiArray spline;
  	        spline.data.clear();

            for(int i = 0; i < 6;i++)
            {
              spline.data.push_back(curvature.spline_value[i]);
            }

          spline_parameters_pub.publish(spline);
          }
          else 
          {
            ROS_INFO_STREAM("DAMNIT");
          }

          // Also publish the state at the time of the result for curvature...
          std_msgs::Float64MultiArray state;
          state.data.clear();
          for(int i = 0; i < 7; i++)
          {
            state.data.push_back(veh.state_value[i]);
          }

          state_parameters_pub.publish(state);

          // If the velocity is nonzero (would preclude horizon calc) publish trajectory viz
          if(veh.v>0)
          {
            drawSpline(curvature, veh, 0,0);
            SPLINE_INDEX++;
            ROS_INFO_STREAM("Spline published to RVIZ");
          }
          
            // This is a messy for loop which generates extra trajectories for visualization
            // Likely will change when valid cost map arrives.
            // Note: pragma indicates parallelization for OpenMP

            // Setup variables
            union State tempGoal= goal;
            int i;
            union Spline extra;
            
            // Tell OpenMP how to parallelize (keep i private because it is a counter)
            #pragma omp parallel for private(i)

            // Index through all the predefined perturbations from waypoint
            for(i=1; i<31; i++)
            {
              // Shift the y-coordinate of the goal
              tempGoal.sy = tempGoal.sy + perturb[i-1];

              // Compute new spline 
              extra= waypointTrajectory(veh, tempGoal, curvature, next_waypoint);

              // Display trajectory
              if(veh.v>5.00)
              {
                drawSpline(extra, veh, i,1);
              }
            }


          // Update the elapsed time 
          //double elapsedTime=(veh.timestamp-old_time)/1000000000.00;

          // Update previous time and orientation measurements
          old_time= veh.timestamp;
          old_theta=veh.theta;

        }

        // If the next way point is not available 
        else 
        {
          ROS_INFO_STREAM("Lost waypoint!");
          _lf_stat.data = false;
          _stat_pub.publish(_lf_stat);
          twist.twist.linear.x = 0;
          twist.twist.angular.z = 0;
        }

        // Check whether vehicle is close to the final destination
        if (next_waypoint > _path_pp.getPathSize() - 5)
        {
            endflag = true;
        }
      }

      // Else if the closest waypoint is not detected
      /*else 
      {
        ROS_INFO_STREAM("Closest waypoint cannot be detected!");
        _lf_stat.data = false;
        _stat_pub.publish(_lf_stat);
        twist.twist.linear.x = 0;
        twist.twist.angular.z = 0;
      }*/

    }

    // If endflag is true
    else 
    {
      twist.twist = stopControl();
      _lf_stat.data = false;
      _stat_pub.publish(_lf_stat);

      // After stopped or fed out, let's get ready for the restart.
      if (twist.twist.linear.x == 0)
      {
        ROS_INFO_STREAM("Trajectory generation ended!");
        _lf_stat.data = false;
        _stat_pub.publish(_lf_stat);
        endflag = false;
      }
    }
    _prev_velocity = twist.twist.linear.x;
    loop_rate.sleep();
  }

  return 0;
}
