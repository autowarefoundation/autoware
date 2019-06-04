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

#include <ros/ros.h>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>

#include <autoware_msgs/Lane.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_msgs/ControlCommand.h>
#include <amathutils_lib/amathutils.hpp>
#include "amathutils_lib/amathutils.hpp"
#include "mpc_follower/mpc_utils.h"
#include "mpc_follower/mpc_follower_core.h"

class TestSuite : public ::testing::Test
{
public:
    TestSuite() : nh_(""), pnh_("~")
    {
        pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 1);
        pub_vs_ = nh_.advertise<autoware_msgs::VehicleStatus>("/vehicle_status", 1);
        pub_lane_ = nh_.advertise<autoware_msgs::Lane>("/base_waypoints", 1);
        pub_estimate_twist_ = nh_.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 1);
        sub_twist_ = nh_.subscribe("/twist_raw", 1, &TestSuite::callbackTwistRaw, this);
        sub_ctrl_cmd_ = nh_.subscribe("/ctrl_cmd", 1, &TestSuite::callbackCtrlCmd, this);
        spin_duration_ = 0.05;
        spin_loopnum_ = 10;
    }
    ~TestSuite() {}

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_twist_, sub_ctrl_cmd_;
    ros::Publisher pub_pose_, pub_vs_, pub_lane_, pub_estimate_twist_;
    geometry_msgs::TwistStamped twist_raw_;
    autoware_msgs::ControlCommandStamped ctrl_cmd_;
    double spin_duration_;
    int spin_loopnum_;

    void callbackTwistRaw(const geometry_msgs::TwistStamped &twist)
    {
        twist_raw_ = twist;
    }
    void callbackCtrlCmd(const autoware_msgs::ControlCommandStamped &cmd)
    {
        ctrl_cmd_ = cmd;
    }

    void publishEstimateTwist()
    {
        geometry_msgs::TwistStamped twist;
        twist.header.frame_id = "base_link";
        for (int i = 0; i < spin_loopnum_; ++i)
        {
            twist.header.stamp = ros::Time::now();
            pub_estimate_twist_.publish(twist);
            ros::spinOnce();
            ros::Duration(spin_duration_).sleep();
        }
    }

    void publishMsgs(geometry_msgs::PoseStamped &pose, autoware_msgs::VehicleStatus &vs, double wp_vx,
                     double wp_wz, double wp_dt, double wp_x_ini, double wp_y_ini, double wp_yaw_ini)
    {
        double x = wp_x_ini;
        double y = wp_y_ini;
        double yaw = wp_yaw_ini;
        autoware_msgs::Lane lane;
        autoware_msgs::Waypoint wp;
        for (int i = 0; i < 50; ++i)
        {
            wp.pose.pose.position.x = x;
            wp.pose.pose.position.y = y;
            wp.pose.pose.orientation = amathutils::getQuaternionFromYaw(yaw);
            wp.twist.twist.linear.x = wp_vx;
            wp.twist.twist.angular.z = wp_wz;
            lane.waypoints.push_back(wp);
            x += wp_vx * std::cos(yaw) * wp_dt;
            y += wp_vx * std::sin(yaw) * wp_dt;
            yaw += wp_wz * wp_dt;
        }

        for (int i = 0; i < spin_loopnum_; ++i)
        {
            ros::Time current_time = ros::Time::now();
            pose.header.stamp = current_time;
            vs.header.stamp = current_time;
            lane.header.stamp = current_time;
            pub_pose_.publish(pose);
            pub_vs_.publish(vs);
            pub_lane_.publish(lane);
            ros::spinOnce();
            ros::Duration(spin_duration_).sleep();
        }
    }

    void testTurningLeft()
    {
        MPCFollower mpc_follower;

        geometry_msgs::PoseStamped current_pose;
        current_pose.pose.position.y = 1.0; // vehicle position is on the RIGHT side of the path -> turning LEFT
        current_pose.pose.orientation.w = 1.0;

        autoware_msgs::VehicleStatus vs;
        vs.speed = 0.0;
        vs.angle = 0.0;

        // autoware_msgs::Lane lane;
        const double vx = 1.0;
        const double wz = 0.1;
        const double dt = 1.0;
        double x(0.0), y(0.0), yaw(0.0);

        publishMsgs(current_pose, vs, vx, wz, dt, x, y, yaw);

        ASSERT_EQ(true, twist_raw_.twist.linear.x > 0.1) << "going forward";
        ASSERT_EQ(true, ctrl_cmd_.cmd.linear_velocity > 0.1) << "going forward";
        ASSERT_LT(ctrl_cmd_.cmd.steering_angle, 0.0) << "vehicle is turning left, negative steering is expected";
    }

    void testTurningRight()
    {
        MPCFollower mpc_follower;

        geometry_msgs::PoseStamped current_pose;
        current_pose.pose.position.y = -1.0; // vehicle position is on the LEFT side of the path -> turning RIGHT
        current_pose.pose.orientation.w = 1.0;

        autoware_msgs::VehicleStatus vs;
        vs.speed = 0.0;
        vs.angle = 0.0;

        // autoware_msgs::Lane lane;
        const double vx = 1.0;
        const double wz = -0.1;
        const double dt = 1.0;
        double x(0.0), y(0.0), yaw(0.0);

        publishMsgs(current_pose, vs, vx, wz, dt, x, y, yaw);

        ASSERT_GT(twist_raw_.twist.linear.x, 0.1) << "going forward";
        ASSERT_GT(ctrl_cmd_.cmd.linear_velocity, 0.1) << "going forward";
        ASSERT_GT(ctrl_cmd_.cmd.steering_angle, 0.0) << "vehicle is turning right, positive steering is expected";
    }
};


TEST_F(TestSuite, TestMPCFollower)
{
    /* TestMPCFollowerInvalidPath */
    { 
        MPCFollower mpc_follower;

        publishEstimateTwist();

        geometry_msgs::PoseStamped current_pose;
        current_pose.pose.position.y = 1.0;
        current_pose.pose.orientation.w = 1.0;

        autoware_msgs::VehicleStatus vs;
        vs.speed = 0.0;
        vs.angle = 0.0;

        // autoware_msgs::Lane lane;
        const double vx = 1.0;
        const double wz = 0.1;
        const double dt = 1.0;
        double x(0.0), y(0.0), yaw(0.0);

        // first publish valid values
        publishMsgs(current_pose, vs, vx, wz, dt, x, y, yaw);

        // then, publish invalid path 
        autoware_msgs::Lane empty_lane;
        for (int i = 0; i < spin_loopnum_; ++i)
        {
            pub_lane_.publish(empty_lane);
            ros::spinOnce();
            ros::Duration(spin_duration_).sleep();
        }

        ASSERT_TRUE(std::isfinite(twist_raw_.twist.linear.x)) << "expected keepping old path and publish finite values";
        ASSERT_TRUE(std::isfinite(twist_raw_.twist.angular.z)) << "expected keepping old path and publish finite values";
        ASSERT_TRUE(std::isfinite(ctrl_cmd_.cmd.linear_velocity)) << "expected keepping old path and publish finite values";
        ASSERT_TRUE(std::isfinite(ctrl_cmd_.cmd.steering_angle)) << "expected keepping old path and publish finite values";
    }


    /*  == TestMPCFollowerInvalidPose == */
    {
        MPCFollower mpc_follower;

        geometry_msgs::PoseStamped current_pose;
        current_pose.pose.position.y = 1.0; // first publish valid value
        current_pose.pose.orientation.w = 1.0;

        autoware_msgs::VehicleStatus vs;
        vs.speed = 0.0; 
        vs.speed = 0.0; 
        vs.speed = 0.0; 
        vs.angle = 0.0;

        // autoware_msgs::Lane lane;
        const double vx = 1.0;
        const double wz = 0.1;
        const double dt = 1.0;
        double x(0.0), y(0.0), yaw(0.0);

        // first publish valid values
        twist_raw_.twist.linear.x = -9.99;
        publishMsgs(current_pose, vs, vx, wz, dt, x, y, yaw);

        ASSERT_GT(twist_raw_.twist.linear.x, 0.1);


        // then publish invalid pose
        current_pose.pose.position.y = NAN;
        vs.speed = 0.0;
        publishMsgs(current_pose, vs, vx, wz, dt, x, y, yaw);
        ASSERT_DOUBLE_EQ(0.0, twist_raw_.twist.linear.x) << "emergency stop, zero speed command is expected";
        ASSERT_DOUBLE_EQ(0.0, ctrl_cmd_.cmd.linear_velocity) << "emergency stop, zero speed command is expected";
    }

    /* == TestMPCFollowerInvalidVehicleStatus == */
    {
        pnh_.setParam("vehicle_model_type", "kinematics"); // set as default. kinematics_no_delay & dynamics does not use vehicle status.
        pnh_.setParam("qp_solver_type", "unconstraint_fast");

        MPCFollower mpc_follower;

        geometry_msgs::PoseStamped current_pose;
        current_pose.pose.position.y = 1.0; 
        current_pose.pose.position.y = 1.0; 
        current_pose.pose.position.y = 1.0; 
        current_pose.pose.orientation.w = 1.0;

        autoware_msgs::VehicleStatus vs;
        vs.speed = 0.0; 
        vs.speed = 0.0; 
        vs.speed = 0.0; 
        vs.angle = 0.0; // first publish valid value

        // autoware_msgs::Lane lane;
        const double vx = 3.28;
        const double wz = 0.1;
        const double dt = 1.0;
        double x(0.0), y(0.0), yaw(0.0);

        // first publish valid values
        publishMsgs(current_pose, vs, vx, wz, dt, x, y, yaw);
        ASSERT_GT(twist_raw_.twist.linear.x, 0.1);

        // then publish invalid vehicle status
        current_pose.pose.position.y = 1.0;
        vs.angle = NAN;
        publishMsgs(current_pose, vs, vx, wz, dt, x, y, yaw);
        ASSERT_DOUBLE_EQ(0.0, twist_raw_.twist.linear.x) << "emergency stop, zero speed command is expected";
        ASSERT_DOUBLE_EQ(0.0, ctrl_cmd_.cmd.linear_velocity) << "emergency stop, zero speed command is expected";
    }

    /* == TestMPCFollowerNoMessageCase == */
    {
        MPCFollower mpc_follower;
        for (int i = 0; i < spin_loopnum_; ++i)
        {
            ros::spinOnce();
            ros::Duration(spin_duration_).sleep();
        }
        ASSERT_DOUBLE_EQ(0.0, twist_raw_.twist.linear.x) << "no messages published, zero speed command is expected";
        ASSERT_DOUBLE_EQ(0.0, twist_raw_.twist.angular.z) << "no messages published, zero speed command is expected";
        ASSERT_DOUBLE_EQ(0.0, ctrl_cmd_.cmd.linear_velocity) << "no messages published, zero speed command is expected";
        ASSERT_DOUBLE_EQ(0.0, ctrl_cmd_.cmd.steering_angle) << "no messages published, zero speed command is expected";
    }

    /* == TestMPCFollowerAlgorithmOptions == */
    {
        std::string vehicle_mode_type_array[] = {"kinematics", "kinematics_no_delay", "dynamics"};
        std::string qp_solver_type_array[] = {"unconstraint", "unconstraint_fast"};

        for (const auto vehicle_model_type : vehicle_mode_type_array)
        {
            for (const auto qp_solver_type : qp_solver_type_array)
            {
                // ROS_ERROR("%s, %s", vehicle_model_type.c_str(), qp_solver_type.c_str());
                pnh_.setParam("vehicle_model_type", vehicle_model_type);
                pnh_.setParam("qp_solver_type", qp_solver_type);
                testTurningRight();
                testTurningLeft();
            }
        } 
    }

    /* == TestMPCFollowerDebugOptions == */
    {
        pnh_.setParam("show_debug_info", true);
        pnh_.setParam("publish_debug_values", true);
        testTurningRight();

        pnh_.setParam("show_debug_info", false);
        pnh_.setParam("publish_debug_values", false);
        testTurningRight();
    }


}



int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "TestNode");
    return RUN_ALL_TESTS();
}
