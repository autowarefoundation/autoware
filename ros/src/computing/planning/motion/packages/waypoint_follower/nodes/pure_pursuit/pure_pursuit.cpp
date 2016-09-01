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

#include "pure_pursuit/pure_pursuit_core.h"
#include "pure_pursuit/pure_pursuit_viz.h"

constexpr int LOOP_RATE = 30; //processing frequency


int main(int argc, char **argv)
{


  // set up ros
  ros::init(argc, argv, "pure_pursuit");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  bool linear_interpolate_mode;
  private_nh.param("linear_interpolate_mode", linear_interpolate_mode, bool(true));
  ROS_INFO_STREAM("linear_interpolate_mode : " << linear_interpolate_mode);

  waypoint_follower::PurePursuit pp(linear_interpolate_mode);

  ROS_INFO("set publisher...");
  // publish topic
  ros::Publisher cmd_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
  //ros::Publisher stat_pub = nh.advertise<std_msgs::Bool>("wf_stat", 0);
  ros::Publisher target_pub = nh.advertise<visualization_msgs::Marker>("next_target_mark", 0);
  ros::Publisher search_pub = nh.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
  ros::Publisher line_point_pub = nh.advertise<visualization_msgs::Marker>("line_point_mark", 0);  // debug tool
  ros::Publisher traj_circle_pub = nh.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);

  ROS_INFO("set subscriber...");
  // subscribe topic
  ros::Subscriber waypoint_subscriber =
      nh.subscribe("final_waypoints", 10, &waypoint_follower::PurePursuit::callbackFromWayPoints, &pp);
  ros::Subscriber ndt_subscriber =
      nh.subscribe("current_pose", 10, &waypoint_follower::PurePursuit::callbackFromCurrentPose, &pp);
  ros::Subscriber config_subscriber =
      nh.subscribe("config/waypoint_follower", 10, &waypoint_follower::PurePursuit::callbackFromConfig, &pp);
  ros::Subscriber est_twist_subscriber =
      nh.subscribe("current_velocity", 10, &waypoint_follower::PurePursuit::callbackFromCurrentVelocity, &pp);

  ROS_INFO_STREAM("pure pursuit start");
  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    ros::spinOnce();
    cmd_velocity_publisher.publish(pp.go());

    // for visualization with Rviz
    vis_pub.publish(waypoint_follower::displayNextWaypoint(pp.getPoseOfNextWaypoint()));
    search_pub.publish(waypoint_follower::displaySearchRadius(pp.getCurrentPose().position, pp.getLookaheadDistance()));
    target_pub.publish(waypoint_follower::displayNextTarget(pp.getPoseOfNextTarget()));
    traj_circle_pub.publish(waypoint_follower::displayTrajectoryCircle(
        waypoint_follower::generateTrajectoryCircle(pp.getPoseOfNextTarget(), pp.getCurrentPose())));
    loop_rate.sleep();
  }

  return 0;
}
