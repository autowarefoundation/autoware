/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
#include <gtest/gtest.h>

#include "astar_search/astar_util.h"

#include "test_class.h"

//class TestSuite2: public ::testing::Test {
//public:
//	TestSuite2(){}
//	~TestSuite2(){}
//};

TEST_F(TestSuite, CheckGreaterThanOperator){

	int x = 0;
	int y = 0;
	int theta = 0;
	double gc = 0;
	double hc = 10;

	SimpleNode simple_node1;
	SimpleNode simple_node2(x, y, theta, gc, hc);

	ASSERT_TRUE(simple_node2 > simple_node1) << "Should be true";
	ASSERT_TRUE(!(simple_node1 > simple_node2)) << "Should be false";
}

TEST_F(TestSuite, CalculateDistanceBetween2Points){
	// Point 1
	double x1 = 0.0;
	double y1 = 0.0;
	// Point 2
	double x2 = 3.0;
	double y2 = 4.0;
	ASSERT_EQ(calcDistance(x1, y1, x2, y2), sqrt(x2*x2 + y2*y2)) << "Distance should be " << sqrt(x2*x2 + y2*y2);
}

TEST_F(TestSuite, CheckThetaWrapAround){
	double theta = -90; //Degrees
	double theta_new = (theta+360)*M_PI/180;
	ASSERT_DOUBLE_EQ(modifyTheta(theta*M_PI/180), theta_new) << "Angle should be " << theta_new;

	theta = 400;
	theta_new = (theta-360)*M_PI/180;
	ASSERT_DOUBLE_EQ(modifyTheta(theta*M_PI/180), theta_new) << "Angle should be " << theta_new;

	theta = 60;
	theta_new = theta*M_PI/180;
	ASSERT_DOUBLE_EQ(modifyTheta(theta*M_PI/180), theta_new) << "Angle should be " << theta_new;
}

TEST_F(TestSuite, CheckTransformPose){

	// Check translation of 1 along X axis
	tf::Quaternion q(0,0,0,1);
	tf::Vector3 v(1,0,0);
	geometry_msgs::Pose in_pose, out_pose, expected_pose;

	in_pose.position.x = 0;
	in_pose.position.y = 0;
	in_pose.position.z = 0;
	in_pose.orientation.x = 0;
	in_pose.orientation.y = 0;
	in_pose.orientation.z = 0;
	in_pose.orientation.w = 1;
	expected_pose.position.x = 1;
	expected_pose.position.y = 0;
	expected_pose.position.z = 0;
	expected_pose.orientation.x = 0;
	expected_pose.orientation.y = 0;
	expected_pose.orientation.z = 0;
	expected_pose.orientation.w = 1;
	tf::Transform translation(q, v);

	out_pose = transformPose(in_pose, translation);

	ASSERT_DOUBLE_EQ(out_pose.position.x, expected_pose.position.x) << "X Coordinate should be " << expected_pose.position.x;
	ASSERT_DOUBLE_EQ(out_pose.position.y, expected_pose.position.y) << "Y Coordinate should be " << expected_pose.position.y;
	ASSERT_DOUBLE_EQ(out_pose.position.z, expected_pose.position.z) << "Z Coordinate should be " << expected_pose.position.z;
	ASSERT_DOUBLE_EQ(out_pose.orientation.x, expected_pose.orientation.x) << "X Quaternion should be " << expected_pose.orientation.x;
	ASSERT_DOUBLE_EQ(out_pose.orientation.y, expected_pose.orientation.y) << "Y Quaternion should be " << expected_pose.orientation.y;
	ASSERT_DOUBLE_EQ(out_pose.orientation.z, expected_pose.orientation.z) << "Z Quaternion should be " << expected_pose.orientation.z;
	ASSERT_DOUBLE_EQ(out_pose.orientation.w, expected_pose.orientation.w) << "W Quaternion should be " << expected_pose.orientation.w;

}

TEST_F(TestSuite, CheckWaveFrontNodeConstruct){

	WaveFrontNode node;
	ASSERT_EQ(node.index_x, 0) << "index_x should be " << 0;
	ASSERT_EQ(node.index_y, 0) << "index_y should be " << 0;
	ASSERT_EQ(node.hc, 0) << "hc should be " << 0;

	int x = 0;
	int y = 0;
	double cost = 10;
	node = getWaveFrontNode(x, y, cost);
	ASSERT_EQ(node.index_x, x) << "index_x should be " << x;
	ASSERT_EQ(node.index_y, y) << "index_y should be " << y;
	ASSERT_EQ(node.hc, cost) << "hc should be " << cost;
}

TEST_F(TestSuite, CheckRelativeCoordinate){

	geometry_msgs::Pose in_pose;
	in_pose.position.x = 1;
	in_pose.position.y = -4;
	in_pose.position.z = 8;
	in_pose.orientation.x = 0;
	in_pose.orientation.y = 0;
	in_pose.orientation.z = 0;
	in_pose.orientation.w = 1;

	tf::Point point = tf::Point(0,0,0);

	geometry_msgs::Point relative_point = calcRelativeCoordinate(in_pose, point);
	geometry_msgs::Point expected_point;
	expected_point.x = -in_pose.position.x;
	expected_point.y = -in_pose.position.y;
	expected_point.z = -in_pose.position.z;

	ASSERT_EQ(relative_point.x, expected_point.x) << "X coord should be " << expected_point.x;
	ASSERT_EQ(relative_point.y, expected_point.y) << "Y coord should be " << expected_point.y;
	ASSERT_EQ(relative_point.z, expected_point.z) << "Z coord should be " << expected_point.z;
}

TEST_F(TestSuite, CheckRadianDifference){

	// Diff < 180 degrees
	double a = 0*M_PI/180;
	double b = 90*M_PI/180;
	ASSERT_DOUBLE_EQ(calcDiffOfRadian(a, b), 90*M_PI/180) << "diff should be " << 90*M_PI/180;

	// 180 degrees < Diff < 360 degrees
	a = 0*M_PI/180;
	b = 190*M_PI/180;
	ASSERT_DOUBLE_EQ(calcDiffOfRadian(a, b), 170*M_PI/180) << "diff should be " << 170*M_PI/180;

	// Diff > 360 degrees
	a = 0*M_PI/180;
	b = 400*M_PI/180;
	ASSERT_DOUBLE_EQ(calcDiffOfRadian(a, b), 40*M_PI/180) << "diff should be " << 40*M_PI/180;
}
