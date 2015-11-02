/*
 *  command_converter.cpp
 *  Command Converter for Cubic Spline Trajectory Generation
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
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include "runtime_manager/ConfigWaypointFollower.h"
#include "waypoint_follower/libwaypoint_follower.h"
#include "libtraj_gen.h"
#include <vector>


/////////////////////////////////////////////////////////////////
// Global variables
/////////////////////////////////////////////////////////////////

// Set update rate
static const int LOOP_RATE = 15; //Hz

// Next state time difference
static const double next_time = 1.00/LOOP_RATE;

// Global vairable to hold curvature
union Spline curvature;

// Global variable to hold vehicle state
union State veh; 

// Global var
union State veh_temp;

// Global variable to keep track of when the last message was recieved:
double start_time;

// Global variable keep track of current time in this node:
double current_time;

// Global flag to indicate a new state
bool newState = FALSE;

// Global flag to indicate a new spline
bool newSpline =FALSE;


/////////////////////////////////////////////////////////////////
// Callback function declarations
/////////////////////////////////////////////////////////////////

// Callback function to get control parameters 
void splineCallback(const std_msgs::Float64MultiArray::ConstPtr& spline);

// Callback function to get state parameters 
void stateCallback(const std_msgs::Float64MultiArray::ConstPtr& state);


/////////////////////////////////////////////////////////////////
// Main loop
/////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  //fmm_omega.open("fmm_omega.dat");

  ROS_INFO_STREAM("Command converter begin: ");
  ROS_INFO_STREAM("Loop Rate: " << next_time);

  // Set up ROS
  ros::init(argc, argv, "command_converter");

  // Make handles
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Publish the following topics:
  // Commands
  ros::Publisher cmd_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);

  // Subscribe to the following topics:
  // Curvature parameters and state parameters
  ros::Subscriber spline_parameters = nh.subscribe("spline", 1, splineCallback);
  ros::Subscriber state_parameters = nh.subscribe("state", 1, stateCallback);

  // Setup message to hold commands
  geometry_msgs::TwistStamped twist;

  // Setup the loop rate in Hz
  ros::Rate loop_rate(LOOP_RATE);

  bool endflag = false;
  static  double vdes;

  while (ros::ok())
  {
    std_msgs::Bool _lf_stat;
    

    ros::spinOnce();
    current_time = ros::Time::now().toSec();
    double elapsedTime = current_time - start_time;

    // Make sure we haven't finished the mission yet
    if(endflag==FALSE)
    {
          if(newState == TRUE)
          {
            vdes = veh.vdes;
            // This computes the next command
            veh_temp = nextState(veh, curvature, vdes, next_time, elapsedTime + 0.1);
          }

          // Set velocity
          twist.twist.linear.x=vdes;
          
          // Ensure kappa is reasonable
          veh_temp.kappa = min(kmax, veh_temp.kappa);
          veh_temp.kappa = max (kmin, veh_temp.kappa); 

          // Set angular velocity
          twist.twist.angular.z=vdes*veh_temp.kappa;
    }

    // If we have finished the mission clean up 
    else
    {
      ROS_INFO_STREAM("End mission");
    }
    
    // Publish messages
    cmd_velocity_publisher.publish(twist);
    loop_rate.sleep();
  }
  return 0;
}


/////////////////////////////////////////////////////////////////
// Call back function for state update
/////////////////////////////////////////////////////////////////

void stateCallback(const std_msgs::Float64MultiArray::ConstPtr& state)
{
    int i = 0;

    for(std::vector<double>::const_iterator it = state->data.begin(); it != state->data.end(); ++it)
    {
        veh.state_value[i] = *it;
        i++;
    }

    newState = TRUE;

    return;
}

/////////////////////////////////////////////////////////////////
// Callback function for spline update
/////////////////////////////////////////////////////////////////

void splineCallback(const std_msgs::Float64MultiArray::ConstPtr& spline)
{
    if(ros::ok())
    {
      start_time= ros::Time::now().toSec();
    }

    // Reset elapsed time to 0, if called...
    int i = 0;

    for(std::vector<double>::const_iterator it = spline->data.begin(); it != spline->data.end(); ++it)
    {
        curvature.spline_value[i] = *it;
        i++;
    }

    if(newState == TRUE)
    {
      newSpline = TRUE;
    }
    
    return;
}
