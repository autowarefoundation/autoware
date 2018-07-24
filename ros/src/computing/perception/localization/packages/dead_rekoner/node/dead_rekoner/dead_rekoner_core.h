/*
 *  Copyright (c) 2017, Tier IV, Inc.
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

#ifndef DEAD_REKONER_CORE_H
#define DEAD_REKONER_CORE_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>

#include "dead_rekoner/libdead_rekoner.h"

class DeadRekoner
{
    public:
        DeadRekoner(ros::NodeHandle nh, ros::NodeHandle private_nh);

    private:
        template <class T> void update(const boost::shared_ptr<T const> msg_ptr);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg_ptr);
        void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg_ptr);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg_ptr);

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Publisher odom_pub_;
        ros::Subscriber odom_sub_;
        ros::Subscriber twist_sub_;
        ros::Subscriber imu_sub_;

        tf::TransformListener tf_listener_;

        LibDeadRekoner dead_rekoner_;
};

template <class T>
void DeadRekoner::update(const boost::shared_ptr<T const> msg_ptr)
{
    dead_rekoner_.updateOdometry(*msg_ptr);

    static int seq = 0;
    nav_msgs::Odometry odom_msg = dead_rekoner_.getOdometryMsg();

    odom_msg.header.stamp = msg_ptr->header.stamp;
    odom_msg.header.seq = seq;
    odom_msg.header.frame_id = "/odom";
    odom_msg.child_frame_id = "/base_link";
    odom_pub_.publish(odom_msg);
    ++seq;
}

#endif
