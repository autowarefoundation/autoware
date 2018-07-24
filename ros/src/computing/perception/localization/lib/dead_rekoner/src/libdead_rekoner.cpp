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

#include "dead_rekoner/libdead_rekoner.h"

#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

LibDeadRekoner::LibDeadRekoner()
    :use_imu_(false)
{
    odom_.pose.pose.position.x = 0;
    odom_.pose.pose.position.y = 0;
    odom_.pose.pose.position.z = 0;
    odom_.pose.pose.orientation.x = 0;
    odom_.pose.pose.orientation.y = 0;
    odom_.pose.pose.orientation.z = 0;
    odom_.pose.pose.orientation.w = 1;
}

nav_msgs::Odometry odom_msg;
geometry_msgs::Quaternion imu_orientation;


Eigen::Quaternionf toEigen(const geometry_msgs::Quaternion &a)
{
    return Eigen::Quaternionf(a.w, a.x, a.y, a.z);
}

void LibDeadRekoner::updateOdometry(const nav_msgs::Odometry& arg_odom)
{
    if(use_imu_)
        twist_.twist.linear = arg_odom.twist.twist.linear;
    else
        twist_.twist = arg_odom.twist.twist;
    odom_msg = arg_odom;
    updateOdometryCommon(arg_odom.header.stamp.toSec());
}

void LibDeadRekoner::updateOdometry(const geometry_msgs::TwistStamped& arg_twist)
{
    if(use_imu_)
        twist_.twist.linear = arg_twist.twist.linear;
    else
        twist_ = arg_twist;

    updateOdometryCommon(arg_twist.header.stamp.toSec());
}

void LibDeadRekoner::updateOdometry(const sensor_msgs::Imu& imu)
{
    if(use_imu_)
      twist_.twist.angular = imu.angular_velocity;

    static bool is_first = true;
    static geometry_msgs::Quaternion imu_orientation_first = imu.orientation;

    double fr = 0, fp = 0, fa = 0;
    tf::Quaternion ftf_quaternion;
    tf::quaternionMsgToTF(imu_orientation_first, ftf_quaternion);
    tf::Matrix3x3(ftf_quaternion).getRPY(fr, fp, fa);

    double r = 0, p = 0, a = 0;
    tf::Quaternion tf_quaternion;
    tf::quaternionMsgToTF(imu.orientation, tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(r, p, a);
    imu_orientation = tf::createQuaternionMsgFromRollPitchYaw(r-fr, p-fp, a-fa);

    updateOdometryCommon(imu.header.stamp.toSec());
}

void LibDeadRekoner::updateOdometryCommon(const double current_time_sec)
{
    static double previous_time_sec = current_time_sec;
    const double time_diff_sec = current_time_sec - previous_time_sec;
    previous_time_sec = current_time_sec;

    double r = 0, p = 0, a = 0;
    tf::Quaternion tf_quaternion;
    tf::quaternionMsgToTF(odom_.pose.pose.orientation, tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(r, p, a);

    r += twist_.twist.angular.x * time_diff_sec;
    p += twist_.twist.angular.y * time_diff_sec;
    a += twist_.twist.angular.z * time_diff_sec;

    // odom_.pose.pose.position.x += twist_.twist.linear.x * time_diff_sec;
    // odom_.pose.pose.position.y += twist_.twist.linear.y * time_diff_sec;
    // odom_.pose.pose.position.z += twist_.twist.linear.z * time_diff_sec;

    // const double x = twist_.twist.linear.x * time_diff_sec;
    // const double y = twist_.twist.linear.y * time_diff_sec;
    // const double z = twist_.twist.linear.z * time_diff_sec;

    static nav_msgs::Odometry prev_odom_msg = odom_msg;
    const double x = odom_msg.pose.pose.position.x - prev_odom_msg.pose.pose.position.x;
    const double y = odom_msg.pose.pose.position.y - prev_odom_msg.pose.pose.position.y;
    const double z = odom_msg.pose.pose.position.z - prev_odom_msg.pose.pose.position.z;
    prev_odom_msg = odom_msg;
    //TODO
    double vx = x*std::cos(p)*std::cos(a) - y*std::cos(p)*std::sin(a) + z*std::sin(p);
    double vy = x*(std::cos(r)*std::sin(a)+std::sin(r)*std::sin(p)*std::cos(a)) + y*(std::cos(r)*std::cos(a)-std::sin(r)*std::sin(y)*std::sin(a)) - z*std::sin(r)*std::cos(p);
    double vz = x*(std::sin(r)*std::sin(a)-std::cos(r)*std::sin(p)*std::cos(a)) + y*(std::sin(r)*std::cos(a)+std::cos(r)*std::sin(y)*std::sin(a)) - z*std::cos(r)*std::cos(p);
    // odom_.pose.pose.position.x += x*std::cos(p)*std::cos(a) - y*std::cos(p)*std::sin(a) + z*std::sin(p);
    // odom_.pose.pose.position.y += x*(std::cos(r)*std::sin(a)+std::sin(r)*std::sin(p)*std::cos(a)) + y*(std::cos(r)*std::cos(a)-std::sin(r)*std::sin(y)*std::sin(a)) - z*std::sin(r)*std::cos(p);
    // odom_.pose.pose.position.z += x*(std::sin(r)*std::sin(a)-std::cos(r)*std::sin(p)*std::cos(a)) + y*(std::sin(r)*std::cos(a)+std::cos(r)*std::sin(y)*std::sin(a)) - z*std::cos(r)*std::cos(p);
    //odom_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, a);
    odom_.twist.twist = twist_.twist;

    Eigen::Vector3f diff(x, y, z);
    Eigen::Vector3f v = toEigen(imu_orientation) * toEigen(odom_msg.pose.pose.orientation).inverse() * diff;

    odom_.pose.pose.position.x += v.x();
    odom_.pose.pose.position.y += v.y();
    odom_.pose.pose.position.z += v.z();
    odom_.pose.pose.orientation = imu_orientation;

    std::cout << vx << " "
              << vy << " "
              << vz << std::endl;

    std::cout << v.x() << " "
              << v.y() << " "
              << v.z() << std::endl;

    std::cout << std::endl;
}
