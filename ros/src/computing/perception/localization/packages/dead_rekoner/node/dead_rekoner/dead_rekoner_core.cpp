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

#include "dead_rekoner_core.h"

DeadRekoner::DeadRekoner(ros::NodeHandle nh, ros::NodeHandle private_nh)
    :nh_(nh)
    ,private_nh_(private_nh)
{
    bool use_imu = false;
    private_nh_.getParam("use_imu", use_imu);
    dead_rekoner_.setUseImuFlag(use_imu);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
    //twist_sub_ = nh.subscribe("/can_velocity", 10, &DeadRekoner::twistCallback, this);
    odom_sub_ = nh.subscribe("/vehicle/odom", 10, &DeadRekoner::odomCallback, this);
    //velocity_sub_ = nh.subscribe("/localizer_velocity", 10, &DeadRekoner::velocityCallback, this);
    if(use_imu)
        imu_sub_ = nh.subscribe("/imu_raw", 10, &DeadRekoner::imuCallback, this);
}

void DeadRekoner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg_ptr)
{
    update(odom_msg_ptr);
}

void DeadRekoner::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg_ptr)
{
    update(twist_msg_ptr);
}

void DeadRekoner::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg_ptr)
{
    // sensor_msgs::Imu imu_msg_base_link;
    // try {
    //     tf_listener_.waitForTransform("/base_link", imu_msg_ptr->header.frame_id, ros::Time(0), ros::Duration(0.1));
    //
    //     geometry_msgs::Vector3Stamped vin, vout;
    //     vin.header = imu_msg_ptr->header;
    //     vin.header.stamp = ros::Time(0);
    //     vin.vector = imu_msg_ptr->linear_acceleration;
    //     tf_listener_.transformVector("/base_link", vin, vout);
    //     imu_msg_base_link.linear_acceleration = vout.vector;
    //
    //     vin.vector = imu_msg_ptr->angular_velocity;
    //     tf_listener_.transformVector("/base_link", vin, vout);
    //     imu_msg_base_link.angular_velocity = vout.vector;
    //
    //     tf::Stamped<tf::Quaternion> qin, qout;
    //     geometry_msgs::QuaternionStamped qmin, qmout;
    //     qmin.header = imu_msg_ptr->header;
    //     qmin.quaternion = imu_msg_ptr->orientation;
    //     tf::quaternionStampedMsgToTF(qmin, qin);
    //
    //     auto axis = qin.getAxis();
    //     auto angle = qin.getAngle();
    //     tf::Stamped<tf::Vector3> axis1, axis2;
    //     axis1.setData(axis);
    //     axis1.stamp_ = ros::Time(0);
    //     axis1.frame_id_ = qin.frame_id_;
    //     tf_listener_.transformVector("/base_link", axis1, axis2);
    //
    //     qout.setData(tf::Quaternion(axis2, angle));
    //     qout.stamp_ = qin.stamp_;
    //     qout.frame_id_ = "/base_link";
    //
    //     tf::quaternionStampedMsgToTF(qout, qmout);
    //     imu_msg_base_link.orientation = qmout.quaternion;
    // }
    // catch(tf::TransformException &e)
    // {
    //     ROS_ERROR("%s", e.what());
    // }
    update(imu_msg_ptr);
}
