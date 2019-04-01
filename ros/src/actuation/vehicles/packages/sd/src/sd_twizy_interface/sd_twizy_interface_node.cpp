/*
 * Copyright (C) 2019 StreetDrone Limited - All rights reserved
 * Author(s): Efimia Panagiotaki, Chris Whimpenny
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ros/ros.h>
#include "autoware_msgs/VehicleCmd.h"
#include <can_msgs/Frame.h>
#include <boost/thread/thread.hpp>
#include <queue>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include "sd_twizy_interface_h.h"

can_msgs::Frame check_automation;
can_msgs::Frame received_can;
ros::Publisher sent_msgs_pub;
ros::Publisher current_twist_pub;

double target_twist_linear_mps;
double target_twist_angular_degps;
double current_vel_mph = 0.0;
double vehicle_velocity[2] = {0,0};
double previous_data[2] = {0,0};

using namespace std;

void received_can_callback(const can_msgs::Frame::ConstPtr& msg)
{
    check_automation = *msg.get();
    received_can = *msg.get();
}

void twist_cmd_callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    double twist_angular = msg->twist.angular.z;
    double twist_linear = msg->twist.linear.x;

    twist_angular = 57.296 * twist_angular;

    target_twist_angular_degps = twist_angular;
    target_twist_linear_mps = twist_linear;
}

void current_vel_callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    current_vel_mph = msg->twist.linear.x * 3.6; //mps
}

void subscribe_func(int* subscribe_rate){

    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
    // subscriber
    ros::Subscriber received_can_sub = nh->subscribe("received_messages", 10000, received_can_callback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber current_velocity_sub = nh->subscribe<geometry_msgs::TwistStamped>("current_velocity", 1, current_vel_callback);
    ros::Subscriber twist_cmd_sub = nh->subscribe<geometry_msgs::TwistStamped>("twist_cmd", 1000, twist_cmd_callback);

    ros::Rate loop_rate(*subscribe_rate);
    current_twist_pub = nh->advertise<geometry_msgs::TwistStamped>("sd_current_twist", 1000);

    while (ros::ok())
    {
        sd::TranslateCANData(received_can, current_twist_pub, vehicle_velocity[0], vehicle_velocity[1]);

        loop_rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char **argv)
{

    ros::init(argc ,argv, "sd_twizy_interface_node") ;

    std::cout << "SD Twizy interface" << std::endl;
    int publish_rate = 200;
    int subscribe_rate = 8000;
    int pid_rate = 10;

    boost::thread thread_b(subscribe_func, &subscribe_rate);
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

    //publisher
    sent_msgs_pub = nh->advertise<can_msgs::Frame>("sent_messages", 1000);
    ros::Rate loop_rate(publish_rate);

    uint8_t count = 0;
    bool automationOK = false;
    bool automationGIVEN = false;

    while(ros::ok())
    {
        can_msgs::Frame command_data;

        automationOK = sd::CheckAuto(check_automation, automationOK);
        if (automationOK){
            sd::SetAutoCommands(command_data, previous_data);
        }else{
            sd::Reset(command_data, previous_data[0], previous_data[1]);
        }

        automationGIVEN = sd::CheckGiven(check_automation, automationGIVEN);
        if (automationGIVEN){
            sd::SteerTorqueControl(command_data, target_twist_linear_mps, target_twist_angular_degps, previous_data[0], previous_data[1], vehicle_velocity[0]);
        }

        count++;
        sd::SetCrc(command_data, count);

        sent_msgs_pub.publish(command_data);

        loop_rate.sleep();
        ros::spinOnce();
    }

    thread_b.join();
    return 0;
}
