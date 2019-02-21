/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include <gtest/gtest.h>
#include <ros/connection_manager.h>
#include <ros/ros.h>

#include "lane_select_core.h"

namespace lane_planner {

class LaneSelectTestClass {
public:
  LaneSelectTestClass() {}

  LaneSelectNode *lsn;
  autoware_msgs::VehicleLocation cb_vehicle_location;

  ros::NodeHandle nh;
  ros::Publisher traffic_waypoints_array_pub =
      nh.advertise<autoware_msgs::LaneArray>("/traffic_waypoints_array", 0);
  ros::Publisher current_pose_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 0);
  ros::Publisher current_velocity_pub =
      nh.advertise<geometry_msgs::TwistStamped>("/current_velocity", 0);
  ros::Subscriber vehicle_location_sub =
      nh.subscribe("/vehicle_location", 1,
                   &LaneSelectTestClass::vehicleLocationCallback, this);

  int32_t lane_array_id_ = 6587651;

  void accessPublishVehicleLocation(int32_t clst_wp, int32_t larray_id) {
    lsn->publishVehicleLocation(clst_wp, larray_id);
  }

  void publishTrafficWaypointsArray() {
    autoware_msgs::LaneArray pub_msg;
    autoware_msgs::Lane pub_lane;
    for (int idx = 0; idx < 100; idx++) {
      static autoware_msgs::Waypoint wp;
      wp.gid = idx;
      wp.lid = idx;
      wp.pose.pose.position.x = 0.0 + (double)idx;
      wp.pose.pose.position.y = 0.0;
      wp.pose.pose.position.z = 0.0;
      wp.twist.twist.linear.x = 5.0;
      wp.twist.twist.angular.z = 0.0;

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);
      quaternionTFToMsg(quaternion, wp.pose.pose.orientation);

      pub_lane.waypoints.push_back(wp);
    }
    pub_msg.lanes.push_back(pub_lane);
    pub_msg.id = lane_array_id_;

    traffic_waypoints_array_pub.publish(pub_msg);
  }

  void publishCurrentPose(double x, double y, double yaw) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);
    quaternionTFToMsg(quaternion, pose.pose.orientation);
    current_pose_pub.publish(pose);
  }

  void publishCurrentVelocity(double vel) {
    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = vel;
    current_velocity_pub.publish(twist);
  }

  void
  vehicleLocationCallback(const autoware_msgs::VehicleLocationConstPtr &msg) {
    cb_vehicle_location = *msg;
    std::cout << "vehicleLocationCallback: "
              << cb_vehicle_location.waypoint_index << ", "
              << cb_vehicle_location.lane_array_id << std::endl;
  }

  void lsnSpinOnce() { lsn->spinOnce(); }
};

} // namespace lane_planner
