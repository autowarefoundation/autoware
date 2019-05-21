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

#include <iostream>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "ekf_localizer/ekf_localizer.h"

class EKFLocalizerTestSuite : public ::testing::Test
{
public:
    EKFLocalizerTestSuite() {}
    ~EKFLocalizerTestSuite() {}

    ros::NodeHandle nh_;

    EKFLocalizer obj_;
    std::string frame_id_a_ = "world";
    std::string frame_id_b_ = "base_link";

    ros::Timer tiemr_ = nh_.createTimer(ros::Duration(0.1), &EKFLocalizerTestSuite::timerCallback, this);

    void timerCallback(const ros::TimerEvent &e)
    {
        /* !!! this should be defined before sendTransform() !!! */
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped sended;

        ros::Time current_time = ros::Time::now();

        sended.header.stamp = current_time;
        sended.header.frame_id = frame_id_a_;
        sended.child_frame_id = frame_id_b_;
        sended.transform.translation.x = -7.11;
        sended.transform.translation.y = 0.0;
        sended.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0.5);
        sended.transform.rotation.x = q.x();
        sended.transform.rotation.y = q.y();
        sended.transform.rotation.z = q.z();
        sended.transform.rotation.w = q.w();

        br.sendTransform(sended);
    };

    void getX(Eigen::MatrixXd &X)
    {
        obj_.ekf_.getLatestX(X);
    }

    void getMeasuredPose(const geometry_msgs::PoseStamped *pose)
    {
        if (obj_.current_pose_ptr_ == nullptr)
        {
            ROS_ERROR("pose pointer is null");
            pose = nullptr;
        }
        else
        {
            pose = obj_.current_pose_ptr_.get();
        }
    }

    bool getTransformFromTF_true_expected()
    {
        geometry_msgs::TransformStamped ts;
        return obj_.getTransformFromTF(frame_id_a_, frame_id_b_, ts);
    };
    bool getTransformFromTF_false_expected()
    {
        geometry_msgs::TransformStamped ts;
        return obj_.getTransformFromTF("bad_frame1", "bad_frame2", ts);
    };
};

/*
 * this test fails due to tf_listener specifications
TEST_F(EKFLocalizerTestSuite, getTransformFromTF)
{
    for (int i = 0; i < 10; ++i)
    {
        // to run timer callback 
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ASSERT_EQ(true, getTransformFromTF_true_expected()) << "appropriate transform, true expected";
    ASSERT_EQ(false, getTransformFromTF_false_expected()) << "inappropriate transform, false expected";
}
*/

TEST_F(EKFLocalizerTestSuite, measurementUpdatePose)
{
    ros::Publisher pub_pose = nh_.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1);
    geometry_msgs::PoseStamped ndt_pose;
    ndt_pose.header.frame_id = "world";
    ndt_pose.pose.position.x = 1.0;
    ndt_pose.pose.position.y = 2.0;
    ndt_pose.pose.position.z = 3.0;
    ndt_pose.pose.orientation.x = 0.0;
    ndt_pose.pose.orientation.y = 0.0;
    ndt_pose.pose.orientation.z = 0.0;
    ndt_pose.pose.orientation.w = 1.0;
    Eigen::MatrixXd X;

    /* test for valid value */
    const double pos_x = 12.3;
    ndt_pose.pose.position.x = pos_x; // for vaild value

    for (int i = 0; i < 10; ++i)
    {
        ndt_pose.header.stamp = ros::Time::now();
        pub_pose.publish(ndt_pose);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    getX(X);
    double estimated_pos = X(0, 0);
    bool is_succeeded = !(isnan(X.array()).any() || isinf(X.array()).any());
    ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";  
    ASSERT_TRUE((estimated_pos - pos_x) < 0.1) << "pos x should be close to " << pos_x;


    /* test for invalid value */
    ndt_pose.pose.position.x = NAN; // check for invalid values
    for (int i = 0; i < 10; ++i)
    {
        ndt_pose.header.stamp = ros::Time::now();
        pub_pose.publish(ndt_pose);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    getX(X);
    is_succeeded = !(isnan(X.array()).any() || isinf(X.array()).any());
    ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";  
}

TEST_F(EKFLocalizerTestSuite, measurementUpdateTwist)
{
    ros::Publisher pub_twist = nh_.advertise<geometry_msgs::TwistStamped>("/can_twist", 1);
    geometry_msgs::TwistStamped can_twist;
    can_twist.header.frame_id = "base_link";
    Eigen::MatrixXd X;

    /* test for valid value */
    const double vx = 12.3;
    can_twist.twist.linear.x = vx; // for vaild value
    for (int i = 0; i < 10; ++i)
    {
        can_twist.header.stamp = ros::Time::now();
        pub_twist.publish(can_twist);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    getX(X);
    double estimated_vx = X(4, 0);
    bool is_succeeded = !(isnan(X.array()).any() || isinf(X.array()).any());
    ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";  
    ASSERT_TRUE((estimated_vx - vx) < 0.1) << "vel x should be close to " << vx;


    /* test for invalid value */
    can_twist.twist.linear.x = NAN; // check for invalid values
    for (int i = 0; i < 10; ++i)
    {
        can_twist.header.stamp = ros::Time::now();
        pub_twist.publish(can_twist);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    getX(X);
    is_succeeded = !(isnan(X.array()).any() || isinf(X.array()).any());
    ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";  
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "EKFLocalizerTestSuite");

    return RUN_ALL_TESTS();
}