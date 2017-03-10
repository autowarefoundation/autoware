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
#include "runtime_manager/ConfigWaypointFollower.h"
#include "waypoint_follower/libwaypoint_follower.h"
#include "libtraj_gen.h"
#include "vehicle_socket/CanInfo.h"
//#include <dbw_mkz_msgs/SteeringReport.h>


#define DEBUG_TRAJECTORY_GEN

#define WHEEL_ANGLE_MAX (31.28067)
#define STEERING_ANGLE_MAX (666.00000)
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX/WHEEL_ANGLE_MAX)
#define WHEEL_BASE (2.70)

#define WHEEL_BASE_MKZ (2.84988)
#define WHEEL_TO_STEERING_MKZ (22.00)

static const int LOOP_RATE = 10; //Hz

static const std::string MAP_FRAME = "map";
static const std::string SIM_BASE_FRAME = "sim_base_link";
static const std::string BASE_FRAME = "base_link";

static ros::Publisher g_marker_pub;
static ros::Publisher g_vis_pub;
static ros::Publisher g_stat_pub;

static bool g_prius_mode = FALSE;
static bool g_mkz_mode = FALSE;
static bool g_sim_mode = false;
static geometry_msgs::PoseStamped g_current_pose; // current pose by the global plane.
static double g_current_velocity;
static double g_can_info_curvature;
static double g_prev_velocity = 0;
static ros::Publisher _traj_pub;
static ros::Publisher _stat_pub;
static bool g_waypoint_set = false;
static bool g_pose_set = false;

static double g_current_angular_velocity;
static double g_mkz_info_curvature;

static int SPLINE_INDEX=0;

//config topic
static int g_param_flag = 0; //0 = waypoint, 1 = Dialog
static double g_lookahead_threshold = 4.0; //meter
static double g_initial_velocity = 5.0; //km/h
static double g_look_ahead_threshold_calc_ratio = 2.0;
static double g_minimum_look_ahead_threshold = 6.0; // the next waypoint must be outside of this threshold.

static WayPoints g_current_waypoints;

static void ConfigCallback(const runtime_manager::ConfigWaypointFollowerConstPtr &config)
{
  g_param_flag = config->param_flag;
  g_lookahead_threshold = config->lookahead_distance;
  g_initial_velocity = config->velocity;
  g_look_ahead_threshold_calc_ratio = config->lookahead_ratio;
  g_minimum_look_ahead_threshold = config->minimum_lookahead_distance;
}



static void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  g_current_pose = *msg;
  g_pose_set = true;
}

static void currentVelCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
  g_current_velocity = msg->twist.linear.x;
}

static void WayPointCallback(const waypoint_follower::laneConstPtr &msg)
{
  g_current_waypoints.setPath(*msg);
  g_waypoint_set = true;
  ROS_INFO_STREAM("waypoint subscribed");
}

/*static double getCmdVelocity(int waypoint)
{

  if (g_param_flag)
  {
    ROS_INFO_STREAM("dialog : " << g_initial_velocity << " km/h (" << kmph2mps(g_initial_velocity) << " m/s )");
    return kmph2mps(g_initial_velocity);
  }

  if (g_current_waypoints.isEmpty())
  {
    ROS_INFO_STREAM("waypoint : not loaded path");
    return 0;
  }

  double velocity = g_current_waypoints.getWaypointVelocityMPS(waypoint);
  ROS_INFO_STREAM("waypoint : " << mps2kmph(velocity) << " km/h ( " << velocity << "m/s )");
  return velocity;
}
*/
static double getLookAheadThreshold(int waypoint)
{
  if (g_param_flag)
    return g_lookahead_threshold;

  // double current_velocity_mps = _current_waypoints.getWaypointVelocityMPS(waypoint);
  double current_velocity_mps = g_current_velocity;

  if (current_velocity_mps * g_look_ahead_threshold_calc_ratio < g_minimum_look_ahead_threshold)
    return g_minimum_look_ahead_threshold;
  else
    return current_velocity_mps * g_look_ahead_threshold_calc_ratio;
}

static void canInfoCallback(const vehicle_socket::CanInfoConstPtr &msg)
{
  double steering_wheel_angle = msg->angle;
  //g_current_velocity = (msg->speed)*(1000.00/3600);
  steering_wheel_angle = steering_wheel_angle*(3.1496/180.00);
  g_can_info_curvature = (steering_wheel_angle / (double) WHEEL_TO_STEERING) / WHEEL_BASE;
  ROS_INFO_STREAM("Steering Wheel Angle: "<<steering_wheel_angle);
  ROS_INFO_STREAM("Curvature from CAN: "<<g_can_info_curvature);
}

/*static void mkzInfoCallback(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg)
{
  double steering_wheel_angle = msg->steering_wheel_angle;
  _current_velocity = (msg->speed);
  _can_info_curvature = (steering_wheel_angle / (double) WHEEL_TO_STEERING_MKZ) / WHEEL_BASE_MKZ;
  ROS_INFO_STREAM("Steering Wheel Angle: "<<steering_wheel_angle);
  ROS_INFO_STREAM("Curvature from CAN: "<<_mkz_info_curvature);
}
*/

static int getNextWaypoint(int closest_waypoint)
{
  // if waypoints are not given, do nothing.
  if (g_current_waypoints.getSize() == 0)
    return -1;

  double lookahead_threshold = getLookAheadThreshold(closest_waypoint);

  //ROS_INFO_STREAM("threshold = " << lookahead_threshold);
  // look for the next waypoint.
  for (int i = closest_waypoint; i < g_current_waypoints.getSize(); i++)
  {
    //skip waypoint behind vehicle
    if (calcRelativeCoordinate(g_current_waypoints.getCurrentWaypoints().waypoints[i].pose.pose.position,
        g_current_pose.pose).x < 0)
      continue;

    // if there exists an effective waypoint
    if (getPlaneDistance(g_current_waypoints.getCurrentWaypoints().waypoints[i].pose.pose.position,
        g_current_pose.pose.position) > lookahead_threshold)
      return i;
  }

  // if the program reaches here, it means we lost the waypoint.
  return -1;
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
  marker.pose.position = g_current_waypoints.getWaypointPosition(i);
  marker.pose.orientation = g_current_waypoints.getWaypointOrientation(i);
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.frame_locked = true;
  g_vis_pub.publish(marker);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// M. O'Kelly code begins here, 
// Suggested: need to clean up unused functions above... don't have time.
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
// Compute the goal state of the vehicle
/////////////////////////////////////////////////////////////////
static union State computeWaypointGoal(int next_waypoint)
{
    union State l_goal;

    // Get the next waypoint position with respect to the vehicles frame
    //l_goal.sx = _path_pp.transformWaypoint(next_waypoint).getX();
    //l_goal.sy = _path_pp.transformWaypoint(next_waypoint).getY();

    l_goal.sx = calcRelativeCoordinate(g_current_waypoints.getWaypointPosition(next_waypoint),g_current_pose.pose).x;
    l_goal.sy = calcRelativeCoordinate(g_current_waypoints.getWaypointPosition(next_waypoint),g_current_pose.pose).y;

    // Get the next next waypoint position
    //double waypoint_lookahead_1_x = _path_pp.transformWaypoint(next_waypoint+1).getX();
   // double waypoint_lookahead_1_y = _path_pp.transformWaypoint(next_waypoint+1).getY();
    double waypoint_lookahead_1_x = calcRelativeCoordinate(g_current_waypoints.getWaypointPosition(next_waypoint+1),g_current_pose.pose).x;
    double waypoint_lookahead_1_y = calcRelativeCoordinate(g_current_waypoints.getWaypointPosition(next_waypoint+1),g_current_pose.pose).y;

    // Get the next next next waypoint
   // double waypoint_lookahead_2_x = _path_pp.transformWaypoint(next_waypoint+2).getX();
    //double waypoint_lookahead_2_y = _path_pp.transformWaypoint(next_waypoint+2).getY();
    double waypoint_lookahead_2_x = calcRelativeCoordinate(g_current_waypoints.getWaypointPosition(next_waypoint+2),g_current_pose.pose).x;
    double waypoint_lookahead_2_y = calcRelativeCoordinate(g_current_waypoints.getWaypointPosition(next_waypoint+2),g_current_pose.pose).y;

    // Compute dX and dY relative to lookahead
    double dX_1 =  waypoint_lookahead_1_x - l_goal.sx;
    double dY_1 =  waypoint_lookahead_1_y - l_goal.sy;

    // Compute dX and dY relative to lookahead and next lookahead
    double dX_2 =  waypoint_lookahead_2_x - waypoint_lookahead_1_x;
    double dY_2 =  waypoint_lookahead_2_y - waypoint_lookahead_1_y;

    // Estimate desired orientation of the vehicle at next waypoint
    l_goal.theta = atan(dY_1/dX_1);

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
  
    // Get the desired velocity at the closest waypoint
    l_goal.v = g_current_waypoints.getWaypointVelocityMPS(next_waypoint);

    return l_goal;
}

/////////////////////////////////////////////////////////////////
// Compute current state of the vehicle
/////////////////////////////////////////////////////////////////
static union State computeVeh(int old_time, double old_theta, int next_waypoint)
{
    union State l_veh;

    // Goal is computed relative to vehicle coordinate frame
    l_veh.sx=0.0;
    l_veh.sy=0.0;

    // Compute yaw (orientation) relative to the world coordinate frame
    // Necessary to compute curvature, but note that in the local coordinate frame yaw is still 0.0
    tf::Quaternion q(g_current_pose.pose.orientation.x,g_current_pose.pose.orientation.y,g_current_pose.pose.orientation.z,g_current_pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    // Because we are on the coordinate system of the base frame
    l_veh.theta = 0.0;

    // Get the current velocity of the vehicle
    l_veh.v = g_current_velocity;

    // Get the desired velocity
    l_veh.vdes = g_current_waypoints.getWaypointVelocityMPS(next_waypoint);
    ROS_INFO_STREAM("Desired Velocity: "<< l_veh.vdes);

    // Not using timing related stuff for now...
    double w = g_current_angular_velocity;
    ROS_INFO_STREAM("Current omega: " <<w);
    l_veh.kappa = w/l_veh.vdes;


    if (!g_sim_mode && g_prius_mode)
    {
     l_veh.kappa = g_can_info_curvature;
     ROS_INFO_STREAM("Current kappa (prius): " <<l_veh.kappa);
    }

    else if(!g_sim_mode && g_mkz_mode)
    {
      l_veh.kappa = g_mkz_info_curvature;
      ROS_INFO_STREAM("Current kappa (mkz): " <<l_veh.kappa);
    }

    else
    {
      double w = g_current_angular_velocity;
      l_veh.kappa = w/l_veh.vdes;
      ROS_INFO_STREAM("Current kappa (sim): " <<l_veh.kappa);
    }

    return l_veh;
}

/////////////////////////////////////////////////////////////////
// Compute trajectory
/////////////////////////////////////////////////////////////////
static union Spline waypointTrajectory(union State veh, union State goal, union Spline curvature, int next_waypoint)
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
static void drawSpline(union Spline curvature, union State veh, int flag, int selected)
{
  static double vdes=veh.vdes;
  // Setup up line strips
  visualization_msgs:: Marker line_strip;
  if (!g_sim_mode)
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
  g_marker_pub.publish(line_strip);

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
  ros::init(argc, argv, "lattice_trajectory_gen");

  // Create node handles 
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Set the parameters for the node
  private_nh.getParam("sim_mode", g_sim_mode);
  private_nh.getParam("prius_mode", g_prius_mode);
  private_nh.getParam("mkz_mode", g_mkz_mode);
  ROS_INFO_STREAM("sim_mode : " << g_sim_mode);
  ROS_INFO_STREAM("prius_mode : " << g_prius_mode);
  ROS_INFO_STREAM("mkz_mode : " << g_mkz_mode);

  // Publish the following topics: 
  g_vis_pub = nh.advertise<visualization_msgs::Marker>("next_waypoint_mark", 1);
  g_stat_pub = nh.advertise<std_msgs::Bool>("wf_stat", 0);
  // Publish the curvature information:
  ros::Publisher spline_parameters_pub = nh.advertise<std_msgs::Float64MultiArray>("spline", 10);
  ros::Publisher state_parameters_pub = nh.advertise<std_msgs::Float64MultiArray>("state", 10);
  // Publish the trajectory visualization
  g_marker_pub = nh.advertise<visualization_msgs::Marker>("cubic_splines_viz", 10);

  // Subscribe to the following topics: 
  ros::Subscriber waypoint_subcscriber = nh.subscribe("final_waypoints", 1, WayPointCallback);
  ros::Subscriber current_pose_subscriber = nh.subscribe("current_pose", 1, currentPoseCallback);
  ros::Subscriber current_vel_subscriber = nh.subscribe("current_velocity", 1, currentVelCallback);
  ros::Subscriber config_subscriber = nh.subscribe("config/waypoint_follower", 1, ConfigCallback);
  ros::Subscriber sub_steering;
  ros::Subscriber can_info;

  if(g_prius_mode)
  {
    can_info = nh.subscribe("can_info", 1, canInfoCallback);
  }

  
/*  else if(_mkz_mode)
    {
    ROS_INFO_STREAM("********************mkz_mode ON");
    sub_steering = nh.subscribe("/vehicle/steering_report", 1, mkzInfoCallback);
    }
  */

  // Local variable for geometry messages
  geometry_msgs::TwistStamped twist;

  // Set the loop rate unit is Hz
  ros::Rate loop_rate(LOOP_RATE); 

  // Set up arrays for perturb and flag to enable easy parallelization via OpenMP pragma
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
    if (g_waypoint_set == false || g_pose_set == false)
    {
      ROS_INFO_STREAM("topic waiting...");
      loop_rate.sleep();
      continue;
    }

    // Get the closest waypoinmt
    int closest_waypoint = getClosestWaypoint(g_current_waypoints.getCurrentWaypoints(), g_current_pose.pose);
    ROS_INFO_STREAM("closest waypoint = " << closest_waypoint);

      // If the current  waypoint has a valid index
      if (closest_waypoint > 0)
      {
        // Return the next waypoint
        int next_waypoint = getNextWaypoint(closest_waypoint);
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
            ROS_INFO_STREAM("SPLINE FAIL");
          }

          // Also publish the state at the time of the result for curvature...
          std_msgs::Float64MultiArray state;
          state.data.clear();
          for(int i = 0; i < 7; i++)
          {
            state.data.push_back(veh.state_value[i]);
          }

          state_parameters_pub.publish(state);

          // Need to hold back on extra trajectories until CPU utilization is figured out...
          // Need cost map etc...
          if(g_sim_mode)
          {
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
          }

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

      }

    g_prev_velocity = twist.twist.linear.x;
    loop_rate.sleep();
  }

  return 0;
}
