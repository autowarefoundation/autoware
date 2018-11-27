#include <ros/ros.h>
#include <gtest/gtest.h>

#include "astar_search/astar_util.h"

class TestSuite: public ::testing::Test {
public:
	TestSuite(){}
	~TestSuite(){}
};

TEST(TestSuite, CalculateDistanceBetween2Points){
	// Point 1
	double x1 = 0.0;
	double y1 = 0.0;
	// Point 2
	double x2 = 3.0;
	double y2 = 4.0;
	ASSERT_EQ(calcDistance(x1, y1, x2, y2), sqrt(x2*x2 + y2*y2)) << "Distance should be " << sqrt(x2*x2 + y2*y2);
}

TEST(TestSuite, CheckThetaWrapAround){
	double theta = -90; //Degrees
	double thetaNew = (theta+360)*M_PI/180;
	ASSERT_DOUBLE_EQ(modifyTheta(theta*M_PI/180), thetaNew) << "Angle should be " << thetaNew;

	theta = 400;
	thetaNew = (theta-360)*M_PI/180;
	ASSERT_DOUBLE_EQ(modifyTheta(theta*M_PI/180), thetaNew) << "Angle should be " << thetaNew;
}

TEST(TestSuite, CheckTransformPose){

	// Check translation of 1 along X axis
	tf::Quaternion q(0,0,0,1);
	tf::Vector3 v(1,0,0);
	geometry_msgs::Pose inPose, outPose, expectedPose;
	inPose.position.x = 0;
	inPose.position.y = 0;
	inPose.position.z = 0;
	inPose.orientation.x = 0;
	inPose.orientation.y = 0;
	inPose.orientation.z = 0;
	inPose.orientation.w = 1;
	expectedPose.position.x = 1;
	expectedPose.position.y = 0;
	expectedPose.position.z = 0;
	expectedPose.orientation.x = 0;
	expectedPose.orientation.y = 0;
	expectedPose.orientation.z = 0;
	expectedPose.orientation.w = 1;
	tf::Transform translation(q, v);

	outPose = transformPose(inPose, translation);

	ASSERT_DOUBLE_EQ(outPose.position.x, expectedPose.position.x) << "X Coordinate should be " << expectedPose.position.x;
	ASSERT_DOUBLE_EQ(outPose.position.y, expectedPose.position.y) << "Y Coordinate should be " << expectedPose.position.y;
	ASSERT_DOUBLE_EQ(outPose.position.z, expectedPose.position.z) << "Z Coordinate should be " << expectedPose.position.z;
	ASSERT_DOUBLE_EQ(outPose.orientation.x, expectedPose.orientation.x) << "X Quaternion should be " << expectedPose.orientation.x;
	ASSERT_DOUBLE_EQ(outPose.orientation.y, expectedPose.orientation.y) << "Y Quaternion should be " << expectedPose.orientation.y;
	ASSERT_DOUBLE_EQ(outPose.orientation.z, expectedPose.orientation.z) << "Z Quaternion should be " << expectedPose.orientation.z;
	ASSERT_DOUBLE_EQ(outPose.orientation.w, expectedPose.orientation.w) << "W Quaternion should be " << expectedPose.orientation.w;


	// 90deg rotation
//	tf::Quaternion q1(-sqrt(2)/2,0,0,sqrt(2)/2);
//	tf::Vector3 v1(0,0,0);
//	inPose.position.x = 0;
//	inPose.position.y = 0;
//	inPose.position.z = 0;
//	inPose.orientation.x = 0;
//	inPose.orientation.y = 0;
//	inPose.orientation.z = 0;
//	inPose.orientation.w = 1;
//	expectedPose.position.x = 0;
//	expectedPose.position.y = 0;
//	expectedPose.position.z = 0;
//	expectedPose.orientation.x = -sqrt(2)/2;
//	expectedPose.orientation.y = 0;
//	expectedPose.orientation.z = 0;
//	expectedPose.orientation.w = sqrt(2)/2;
//	tf::Transform rotation(q1, v1);
//
//	outPose = transformPose(inPose, rotation);
//
//	ASSERT_DOUBLE_EQ(outPose.position.x, expectedPose.position.x) << "X Coordinate should be " << expectedPose.position.x;
//	ASSERT_DOUBLE_EQ(outPose.position.y, expectedPose.position.y) << "Y Coordinate should be " << expectedPose.position.y;
//	ASSERT_DOUBLE_EQ(outPose.position.z, expectedPose.position.z) << "Z Coordinate should be " << expectedPose.position.z;
//	ASSERT_DOUBLE_EQ(outPose.orientation.x, expectedPose.orientation.x) << "X Quaternion should be " << expectedPose.orientation.x;
//	ASSERT_DOUBLE_EQ(outPose.orientation.y, expectedPose.orientation.y) << "Y Quaternion should be " << expectedPose.orientation.y;
//	ASSERT_DOUBLE_EQ(outPose.orientation.z, expectedPose.orientation.z) << "Z Quaternion should be " << expectedPose.orientation.z;
//	ASSERT_DOUBLE_EQ(outPose.orientation.w, expectedPose.orientation.w) << "W Quaternion should be " << expectedPose.orientation.w;

}

TEST(TestSuite, CheckWaveFrontNodeConstruct){
	int x = 0;
	int y = 0;
	double cost = 10;
	WaveFrontNode node = getWaveFrontNode(x, y, cost);
	ASSERT_EQ(node.index_x, x) << "index_x should be " << x;
	ASSERT_EQ(node.index_y, x) << "index_y should be " << y;
	ASSERT_EQ(node.hc, cost) << "hc should be " << cost;
}


//geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Pose pose, tf::Point point)
//double calcDiffOfRadian(double a, double b)
//geometry_msgs::Pose xytToPoseMsg(double x, double y, double theta)

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "TestNode");
	return RUN_ALL_TESTS();
}

