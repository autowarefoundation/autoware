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
#include "mpc_follower/mpc_utils.h"

class TestSuite: public ::testing::Test {
public:
	TestSuite(){}
	~TestSuite(){}
};

TEST(TestSuite, ConvertEulerAngleIntoPiToMinusPi){

    ASSERT_EQ(MPCUtils::intoSemicircle(0.0), 0.0);
    ASSERT_EQ(MPCUtils::intoSemicircle(0.5), 0.5);
    ASSERT_EQ(MPCUtils::intoSemicircle(-0.5), -0.5);
    ASSERT_EQ(MPCUtils::intoSemicircle(2.0 * M_PI), 0.0);
    ASSERT_EQ(MPCUtils::intoSemicircle(M_PI), M_PI);
    ASSERT_EQ(MPCUtils::intoSemicircle(-M_PI), -M_PI);

}

TEST(TestSuite, InterpolationTest){

    std::vector<double> idx = {0.0, 1.0, 2.0, 3.0};
    
    std::vector<double> value = {-2.0, 0.0, 2.0, 4.0};
    double ref = 1.0;
    double ret = 0.0;
    
    std::vector<double> idx_bad = {0.0, 1.0, 0.0, 3.0};
    bool res = MPCUtils::interp1dX<std::vector<double>, std::vector<double>>(idx_bad, value, ref, ret);
    ASSERT_EQ(res, false);

    ref = -10.0;
    MPCUtils::interp1dX<std::vector<double>, std::vector<double>>(idx, value, ref, ret);
    ASSERT_EQ(ret, -2.0);

    ref = 10.0;
    MPCUtils::interp1dX<std::vector<double>, std::vector<double>>(idx, value, ref, ret);
    ASSERT_EQ(ret, 4.0);

    ref = 0.3;
    MPCUtils::interp1dX<std::vector<double>, std::vector<double>>(idx, value, ref, ret);
    ASSERT_EQ(ret, -1.4);

}


TEST(TestSuite, TestYawQuaternion){

    geometry_msgs::Quaternion q;
    q = MPCUtils::getQuaternionFromYaw(0.0);
    ASSERT_EQ(q.x, 0.0);
    ASSERT_EQ(q.y, 0.0);
    ASSERT_EQ(q.z, 0.0);
    ASSERT_EQ(q.w, 1.0);
}


TEST(TestSuite, TestCalcNearestPose){

    MPCTrajectory traj;
    /*              x    y    z     yaw        k    vx  time */
    traj.push_back(0.0, 0.0, 0.0, M_PI / 4.0, 0.0, 0.0, 0.0);
    traj.push_back(1.0, 1.0, 0.0, M_PI / 4.0, 1.0, 0.0, 1.0);
    traj.push_back(2.0, 2.0, 0.0, M_PI / 4.0, 1.0, 0.0, 2.0);

    geometry_msgs::Pose self_pose;
    geometry_msgs::Pose nearest_pose;
    unsigned int nearest_index;
    double min_dist_error, nearest_yaw_error, nearest_time;

    self_pose.position.x = 0.3;
    self_pose.position.y = 0.3;
    self_pose.position.z = 0.0;
    self_pose.orientation = MPCUtils::getQuaternionFromYaw(M_PI / 3.0);
    MPCUtils::calcNearestPose(traj, self_pose, nearest_pose, nearest_index, min_dist_error, nearest_yaw_error, nearest_time);
    EXPECT_EQ(nearest_index, 0);
    EXPECT_EQ(min_dist_error, std::sqrt(0.3 * 0.3 + 0.3 * 0.3));
    EXPECT_EQ(nearest_yaw_error, M_PI / 3.0 - M_PI / 4.0);
    EXPECT_EQ(nearest_time, 0.0);

    self_pose.position.x = 0.0;
    self_pose.position.y = 0.0;
    self_pose.position.z = 0.1;
    self_pose.orientation = MPCUtils::getQuaternionFromYaw(M_PI / 4.0);
    MPCUtils::calcNearestPose(traj, self_pose, nearest_pose, nearest_index, min_dist_error, nearest_yaw_error, nearest_time);
    EXPECT_EQ(nearest_index, 0);
    EXPECT_EQ(min_dist_error, 0.0);
    EXPECT_EQ(std::fabs(nearest_yaw_error) < 1.0E-5, true);
    EXPECT_EQ(nearest_time, 0.0);


    self_pose.position.x = 0.3;
    self_pose.position.y = 0.3;
    self_pose.position.z = 0.0;
    self_pose.orientation = MPCUtils::getQuaternionFromYaw(M_PI / 4.0);
    MPCUtils::calcNearestPoseInterp(traj, self_pose, nearest_pose, nearest_index, min_dist_error, nearest_yaw_error, nearest_time);
    EXPECT_EQ(nearest_index, 0);
    EXPECT_EQ(min_dist_error, 0.0);
    EXPECT_EQ(std::fabs(nearest_yaw_error) < 1.0E-5, true);
    EXPECT_EQ(std::fabs(nearest_time - 0.3) < 1.0E-5, true);

    self_pose.position.x = 0.3;
    self_pose.position.y = 0.3;
    self_pose.position.z = 0.0;
    self_pose.orientation = MPCUtils::getQuaternionFromYaw(M_PI / 4.0);
    MPCUtils::calcNearestPoseInterp(traj, self_pose, nearest_pose, nearest_index, min_dist_error, nearest_yaw_error, nearest_time);
    EXPECT_EQ(nearest_index, 0);
    EXPECT_EQ(min_dist_error, 0.0) << "min_dist_error = " << min_dist_error;
    EXPECT_EQ(std::fabs(nearest_yaw_error) < 1.0E-5, true) << "nearest_yaw_error = " << nearest_yaw_error;
    EXPECT_EQ(std::fabs(nearest_time - 0.3) < 1.0E-5, true) << "nearest_time = " << nearest_time;

    self_pose.position.x = -1.0;
    self_pose.position.y = 0.0;
    self_pose.position.z = 0.0;
    self_pose.orientation = MPCUtils::getQuaternionFromYaw(M_PI / 4.0);
    MPCUtils::calcNearestPoseInterp(traj, self_pose, nearest_pose, nearest_index, min_dist_error, nearest_yaw_error, nearest_time);
    EXPECT_EQ(nearest_index, 0);
    EXPECT_EQ(std::fabs(min_dist_error - sqrt(2.0)/2.0) < 1.0E-5, true) << "min_dist_error = " << min_dist_error;
    EXPECT_EQ(std::fabs(nearest_yaw_error) < 1.0E-5, true) << "nearest_yaw_error = " << nearest_yaw_error;
    EXPECT_EQ(std::fabs(nearest_time - (-0.5)) < 1.0E-5, true) << "nearest_time = " << nearest_time;

}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "TestNode");
	return RUN_ALL_TESTS();
}