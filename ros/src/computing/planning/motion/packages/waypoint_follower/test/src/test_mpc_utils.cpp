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
#include <amathutils_lib/amathutils.hpp>
#include "mpc_follower/mpc_utils.h"

class TestSuite: public ::testing::Test {
public:
	TestSuite(){}
	~TestSuite(){}
};


TEST(TestSuite, TestConvertEulerAngleToMonotonic){
    std::vector<double> yaw;
    for (int i = -5; i < 5; ++i) {
        double tmp = amathutils::normalizeRadian((double)i * M_PI);
        yaw.push_back(tmp);
    }
    ASSERT_DOUBLE_EQ(-M_PI, yaw.front());
    ASSERT_DOUBLE_EQ(0.0, yaw.back());
    double diff = yaw.back() - yaw.front();

    MPCUtils::convertEulerAngleToMonotonic(yaw);
    ASSERT_DOUBLE_EQ(-M_PI, yaw.front());
    ASSERT_DOUBLE_EQ(-M_PI + diff, yaw.back());
}



TEST(TestSuite, InterpolationTest){

    std::vector<double> idx = {0.0, 1.0, 2.0, 3.0};
    
    std::vector<double> value = {-2.0, 0.0, 2.0, 4.0};
    double ref = 1.0;
    double ret = 0.0;
    
    std::vector<double> idx_bad = {0.0, 1.0, 0.0, 3.0};
    bool res = MPCUtils::interp1d<std::vector<double>, std::vector<double>>(idx_bad, value, ref, ret);
    ASSERT_EQ(false, res);

    ref = -10.0;
    MPCUtils::interp1d<std::vector<double>, std::vector<double>>(idx, value, ref, ret);
    ASSERT_DOUBLE_EQ(-2.0, ret);

    ref = 10.0;
    MPCUtils::interp1d<std::vector<double>, std::vector<double>>(idx, value, ref, ret);
    ASSERT_DOUBLE_EQ(4.0, ret);

    ref = 0.3;
    MPCUtils::interp1d<std::vector<double>, std::vector<double>>(idx, value, ref, ret);
    ASSERT_DOUBLE_EQ(-1.4, ret);
}


TEST(TestSuite, TestCalcTrajectoryYawFromXY) {

    MPCTrajectory traj;
    /*              x    y    z   yaw   vx   k  time */
    traj.push_back(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    traj.push_back(1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0);
    traj.push_back(2.0, 2.0, 0.0, 0.0, 1.0, 0.0, 2.0);
    MPCUtils::calcTrajectoryYawFromXY(traj);
    ASSERT_DOUBLE_EQ(M_PI / 4.0, traj.yaw[0]);
    ASSERT_DOUBLE_EQ(M_PI / 4.0, traj.yaw[1]);
    ASSERT_DOUBLE_EQ(M_PI / 4.0, traj.yaw[2]);

    traj.clear();
    /*              x    y    z   yaw   vx   k  time */
    traj.push_back(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    traj.push_back(0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0);
    traj.push_back(0.0, 2.0, 0.0, 0.0, 1.0, 0.0, 2.0);
    MPCUtils::calcTrajectoryYawFromXY(traj);
    ASSERT_DOUBLE_EQ(M_PI / 2.0, traj.yaw[0]);
    ASSERT_DOUBLE_EQ(M_PI / 2.0, traj.yaw[1]);
    ASSERT_DOUBLE_EQ(M_PI / 2.0, traj.yaw[2]);

    traj.clear();
    /*              x    y    z   yaw   vx   k  time */
    traj.push_back(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    traj.push_back(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0);
    traj.push_back(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 2.0);
    MPCUtils::calcTrajectoryYawFromXY(traj);
    ASSERT_DOUBLE_EQ(0.0, traj.yaw[0]);
    ASSERT_DOUBLE_EQ(0.0, traj.yaw[1]);
    ASSERT_DOUBLE_EQ(0.0, traj.yaw[2]);
}

TEST(TestSuite, TestCalcTrajectoryCurvature)
{
    MPCTrajectory traj;
    double radius = 0.5;
    for (double theta = 0; theta < M_PI; theta += 0.1)
    {
        double x = radius * std::cos(theta);
        double y = radius * std::sin(theta);
        traj.push_back(x, y, 0, 0, 0, 0, theta);
    }
    MPCUtils::calcTrajectoryCurvature(traj, 1);
    ASSERT_NEAR(1 / radius, traj.k[0], 1.0E-5);
    ASSERT_NEAR(1 / radius, traj.k[5], 1.0E-5);
    ASSERT_NEAR(1 / radius, traj.k.back(), 1.0E-5);

    MPCUtils::calcTrajectoryCurvature(traj, 3);
    ASSERT_NEAR(1 / radius, traj.k[0], 1.0E-5);
    ASSERT_NEAR(1 / radius, traj.k[5], 1.0E-5);
    ASSERT_NEAR(1 / radius, traj.k.back(), 1.0E-5);

    traj.clear();
    radius = 10.0;
    for (double theta = 0; theta < M_PI; theta += 0.1)
    {
        double x = radius * std::cos(theta);
        double y = radius * std::sin(theta);
        traj.push_back(x, y, 0, 0, 0, 0, theta);
    }

    MPCUtils::calcTrajectoryCurvature(traj, 1);
    ASSERT_NEAR(1 / radius, traj.k[0], 1.0E-5);
    ASSERT_NEAR(1 / radius, traj.k[5], 1.0E-5);
    ASSERT_NEAR(1 / radius, traj.k.back(), 1.0E-5);

    MPCUtils::calcTrajectoryCurvature(traj, 3);
    ASSERT_NEAR(1 / radius, traj.k[0], 1.0E-5);
    ASSERT_NEAR(1 / radius, traj.k[5], 1.0E-5);
    ASSERT_NEAR(1 / radius, traj.k.back(), 1.0E-5);
}

TEST(TestSuite, TestCalcNearestPose){

    MPCTrajectory traj;
    /*              x    y    z     yaw        vx   k  time */
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
    self_pose.orientation = amathutils::getQuaternionFromYaw(M_PI / 3.0);
    MPCUtils::calcNearestPose(traj, self_pose, nearest_pose, nearest_index, min_dist_error, nearest_yaw_error, nearest_time);
    ASSERT_EQ(0, nearest_index);
    ASSERT_DOUBLE_EQ(std::sqrt(0.3 * 0.3 + 0.3 * 0.3), min_dist_error);
    ASSERT_DOUBLE_EQ(M_PI / 3.0 - M_PI / 4.0, nearest_yaw_error);
    ASSERT_DOUBLE_EQ(0.0, nearest_time);

    self_pose.position.x = 0.0;
    self_pose.position.y = 0.0;
    self_pose.position.z = 0.1;
    self_pose.orientation = amathutils::getQuaternionFromYaw(M_PI / 4.0);
    MPCUtils::calcNearestPose(traj, self_pose, nearest_pose, nearest_index, min_dist_error, nearest_yaw_error, nearest_time);
    ASSERT_EQ(0, nearest_index);
    ASSERT_DOUBLE_EQ(0.0, min_dist_error);
    ASSERT_EQ(true, std::fabs(nearest_yaw_error) < 1.0E-5);
    ASSERT_DOUBLE_EQ(0.0, nearest_time);


    self_pose.position.x = 0.3;
    self_pose.position.y = 0.3;
    self_pose.position.z = 0.0;
    self_pose.orientation = amathutils::getQuaternionFromYaw(M_PI / 4.0);
    MPCUtils::calcNearestPoseInterp(traj, self_pose, nearest_pose, nearest_index, min_dist_error, nearest_yaw_error, nearest_time);
    ASSERT_EQ(0, nearest_index);
    ASSERT_DOUBLE_EQ(0.0, min_dist_error);
    ASSERT_EQ(true, std::fabs(nearest_yaw_error) < 1.0E-5);
    ASSERT_EQ(true, std::fabs(nearest_time - 0.3) < 1.0E-5);

    self_pose.position.x = 0.3;
    self_pose.position.y = 0.3;
    self_pose.position.z = 0.0;
    self_pose.orientation = amathutils::getQuaternionFromYaw(M_PI / 4.0);
    MPCUtils::calcNearestPoseInterp(traj, self_pose, nearest_pose, nearest_index, min_dist_error, nearest_yaw_error, nearest_time);
    ASSERT_EQ(0, nearest_index);
    ASSERT_DOUBLE_EQ(0.0, min_dist_error) << "min_dist_error = " << min_dist_error;
    ASSERT_EQ(true, std::fabs(nearest_yaw_error) < 1.0E-5) << "nearest_yaw_error = " << nearest_yaw_error;
    ASSERT_EQ(true, std::fabs(nearest_time - 0.3) < 1.0E-5) << "nearest_time = " << nearest_time;

    self_pose.position.x = -1.0;
    self_pose.position.y = 0.0;
    self_pose.position.z = 0.0;
    self_pose.orientation = amathutils::getQuaternionFromYaw(M_PI / 4.0);
    MPCUtils::calcNearestPoseInterp(traj, self_pose, nearest_pose, nearest_index, min_dist_error, nearest_yaw_error, nearest_time);
    ASSERT_EQ(0, nearest_index);
    ASSERT_EQ(true, std::fabs(min_dist_error - sqrt(2.0)/2.0) < 1.0E-5) << "min_dist_error = " << min_dist_error;
    ASSERT_EQ(true, std::fabs(nearest_yaw_error) < 1.0E-5) << "nearest_yaw_error = " << nearest_yaw_error;
    ASSERT_EQ(true, std::fabs(nearest_time - (-0.5)) < 1.0E-5) << "nearest_time = " << nearest_time;

}

TEST(TestSuite, TestInterp1dMPCTraj){

    MPCTrajectory traj, traj_result;
    /*              x    y    z    yaw   vx   k  time */
    traj.push_back(0.0, 0.0, 0.0,  0.2, 0.0, 0.0, 0.0);
    traj.push_back(1.0, 2.0, 0.0,  0.5, 1.0, 0.0, 1.0);
    traj.push_back(2.0, 3.0, 0.0, -0.2, 1.0, 0.0, 2.0);

    /*                               0     1    2    3    4    5    6  */
    std::vector<double> index_time{-0.1, 0.0, 0.7, 1.0, 1.5, 2.0, 2.2};
    bool res = MPCUtils::interp1dMPCTraj(traj.relative_time, traj, index_time, traj_result);
    ASSERT_EQ(true, res);
    ASSERT_DOUBLE_EQ(index_time[0], traj_result.relative_time[0]);
    ASSERT_DOUBLE_EQ(traj.x[0], traj_result.x[0]);
    ASSERT_DOUBLE_EQ(traj.y[0], traj_result.y[0]);
    ASSERT_DOUBLE_EQ(traj.z[0], traj_result.z[0]);
    ASSERT_DOUBLE_EQ(traj.yaw[0], traj_result.yaw[0]);
    ASSERT_DOUBLE_EQ(traj.k[0], traj_result.k[0]);
    ASSERT_DOUBLE_EQ(traj.vx[0], traj_result.vx[0]);

    ASSERT_DOUBLE_EQ(0.0, traj_result.relative_time[1]);
    ASSERT_DOUBLE_EQ(traj.x[0], traj_result.x[1]);
    ASSERT_DOUBLE_EQ(traj.y[0], traj_result.y[1]);
    ASSERT_DOUBLE_EQ(traj.z[0], traj_result.z[1]);
    ASSERT_DOUBLE_EQ(traj.yaw[0], traj_result.yaw[1]);
    ASSERT_DOUBLE_EQ(traj.k[0], traj_result.k[1]);
    ASSERT_DOUBLE_EQ(traj.vx[0], traj_result.vx[1]);

    ASSERT_DOUBLE_EQ(0.7, traj_result.relative_time[2]);
    ASSERT_DOUBLE_EQ(0.7, traj_result.x[2]);
    ASSERT_DOUBLE_EQ(1.4, traj_result.y[2]);
    ASSERT_DOUBLE_EQ(0.0, traj_result.z[2]);
    ASSERT_DOUBLE_EQ(0.5*0.7+0.2*0.3, traj_result.yaw[2]);
    ASSERT_DOUBLE_EQ(0.0, traj_result.k[2]);
    ASSERT_DOUBLE_EQ(0.7, traj_result.vx[2]);

    ASSERT_DOUBLE_EQ(index_time[5], traj_result.relative_time[5]);
    ASSERT_DOUBLE_EQ(traj.x[2], traj_result.x[5]);
    ASSERT_DOUBLE_EQ(traj.y[2], traj_result.y[5]);
    ASSERT_DOUBLE_EQ(traj.z[2], traj_result.z[5]);
    ASSERT_DOUBLE_EQ(traj.yaw[2], traj_result.yaw[5]);
    ASSERT_DOUBLE_EQ(traj.k[2], traj_result.k[5]);
    ASSERT_DOUBLE_EQ(traj.vx[2], traj_result.vx[5]);

    ASSERT_DOUBLE_EQ(index_time[6], traj_result.relative_time[6]);
    ASSERT_DOUBLE_EQ(traj.x[2], traj_result.x[6]);
    ASSERT_DOUBLE_EQ(traj.y[2], traj_result.y[6]);
    ASSERT_DOUBLE_EQ(traj.z[2], traj_result.z[6]);
    ASSERT_DOUBLE_EQ(traj.yaw[2], traj_result.yaw[6]);
    ASSERT_DOUBLE_EQ(traj.k[2], traj_result.k[6]);
    ASSERT_DOUBLE_EQ(traj.vx[2], traj_result.vx[6]);

    MPCTrajectory bad_traj;
    /*                 x    y    z    yaw   k    vx  time */
    bad_traj.push_back(0.0, 0.0, 0.0,  0.2, 0.0, 0.0, 0.0);
    bad_traj.push_back(1.0, 2.0, 0.0,  0.5, 1.0, 0.0, -1.0);
    bad_traj.push_back(2.0, 3.0, 0.0, -0.2, 1.0, 0.0, 2.0);

    /*                                      0     1    2    3    4    5    6  */
    std::vector<double> bad_index_time = {-0.1, 0.0, -0.7, 1.0, 1,5, 2.0, 2.2};
    ASSERT_EQ(false, MPCUtils::interp1dMPCTraj(bad_traj.relative_time, traj, index_time, traj_result));
    ASSERT_EQ(false, MPCUtils::interp1dMPCTraj(traj.relative_time, traj, bad_index_time, traj_result));
    ASSERT_EQ(false, MPCUtils::interp1dMPCTraj(bad_traj.relative_time, traj, bad_index_time, traj_result));
}

TEST(TestSuite, TestMPCTrajSize)
{
    MPCTrajectory traj;
    traj.push_back(0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0);
    ASSERT_EQ(1, traj.size()) << "invalid size, zero expected";

    traj.z.push_back(0.0);
    ASSERT_EQ(0, traj.size()) << "invalid size, zero expected";
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "TestNode");
	return RUN_ALL_TESTS();
}