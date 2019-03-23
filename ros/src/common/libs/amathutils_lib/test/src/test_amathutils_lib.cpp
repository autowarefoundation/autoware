#include "amathutils_lib/amathutils.hpp"
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

class TestSuite: public ::testing::Test {
public:
	TestSuite(){}
	~TestSuite(){}
};

TEST(TestSuite, Rad2Deg){
	double radAngle = 1;
	ASSERT_EQ(amathutils::rad2deg(radAngle), radAngle*180/M_PI) << "Angle in degrees should be " << radAngle*180/M_PI;
}

TEST(TestSuite, Deg2Rad){
	double degAngle = 10;
	ASSERT_EQ(amathutils::deg2rad(degAngle), degAngle*M_PI/180) << "Angle in radians should be " << degAngle*M_PI/180;
}

// Value from https://www.google.com/search?client=ubuntu&channel=fs&q=mps+to+kph&ie=utf-8&oe=utf-8
TEST(TestSuite, Transform_mps2kph){
	double mpsValue = 1;
	ASSERT_DOUBLE_EQ(amathutils::mps2kmph(mpsValue), mpsValue*3.6) << "Speed should be " << mpsValue*3.6 << "kmph";
}

// Value from https://www.google.com/search?client=ubuntu&channel=fs&q=kph+to+mps&ie=utf-8&oe=utf-8
TEST(TestSuite, Transform_kmph2mps){
	double kmphValue = 1;
	ASSERT_DOUBLE_EQ(amathutils::kmph2mps(kmphValue), kmphValue/3.6) << "Speed should be " << kmphValue/3.6 << "mps";
}

TEST(TestSuite, GetGravityAcceleration){
	double accel = 10.8;
	ASSERT_DOUBLE_EQ(amathutils::getGravityAcceleration(accel), accel/G_MPSS) << "Acceleration should be " << accel/G_MPSS << "g";
}

TEST(TestSuite, GetAcceleration){
	double accel = 5.5;
	double t = 2;
	double v0 = 2.5;
	double v = v0 + accel*t;
	double x = v0*t + 0.5*accel*t*t;
	ASSERT_DOUBLE_EQ(amathutils::getAcceleration(v0, v, x), accel) << "Acceleration should be " << accel;
}

TEST(TestSuite, GetTimeFromAcceleration){
	double accel = 0.5;
	double t = 2.8;
	double v0 = 2;
	double v = v0 + accel*t;
	ASSERT_DOUBLE_EQ(amathutils::getTimefromAcceleration(v0, v, accel), t) << "Time should be " << t;
}

// Values taken from https://gerardnico.com/linear_algebra/closest_point_line - Example 5.4
TEST(TestSuite, GetNearPointOnLine2D){
	geometry_msgs::Point a, b, p, nearPOut, nearP;

	a.x = 0;
	a.y = 0;
	a.z = 0;

	b.x = 6;
	b.y = 2;
	b.z = 0;

	p.x = 2;
	p.y = 4;
	p.z = 0;

	nearP.x = 3;
	nearP.y = 1;
	nearP.z = 0;

	nearPOut = amathutils::getNearPtOnLine(p, a, b);

	ASSERT_DOUBLE_EQ(nearPOut.x, nearP.x) << "nearPoint coordinate X should be " << nearP.x;
	ASSERT_DOUBLE_EQ(nearPOut.y, nearP.y) << "nearPoint coordinate X should be " << nearP.y;
	ASSERT_DOUBLE_EQ(nearPOut.z, nearP.z) << "nearPoint coordinate X should be " << nearP.z;

}

// Values taken from https://math.stackexchange.com/questions/13176/how-to-find-a-point-on-a-line-closest-to-another-given-point
TEST(TestSuite, GetNearPointOnLine3D){
	geometry_msgs::Point a, b, p, nearPOut, nearP;
	double threshold = 0.00000001;

	a.x = -2;
	a.y = -4;
	a.z = 5;

	b.x = 0;
	b.y = 0;
	b.z = 1;

	p.x = 1;
	p.y = 1;
	p.z = 1;

	nearP.x = 1.0/3.0;
	nearP.y = 2.0/3.0;
	nearP.z = 1.0/3.0;

	nearPOut = amathutils::getNearPtOnLine(p, a, b);

	ASSERT_NEAR(nearPOut.x, nearP.x, threshold) << "nearPoint coordinate X should be " << nearP.x;
	ASSERT_NEAR(nearPOut.y, nearP.y, threshold) << "nearPoint coordinate X should be " << nearP.y;
	ASSERT_NEAR(nearPOut.z, nearP.z, threshold) << "nearPoint coordinate X should be " << nearP.z;
}

// Values from http://www.math.usm.edu/lambers/mat169/fall09/lecture17.pdf - The Distance Formula
TEST(TestSuite, GetDistance){

	geometry_msgs::Point ptA, ptB;
	geometry_msgs::Pose poseA, poseB;

	ptA.x = 2;
	ptA.y = 3;
	ptA.z = 1;

	ptB.x = 8;
	ptB.y = -5;
	ptB.z = 0;

	poseA.position.x = 2;
	poseA.position.y = 3;
	poseA.position.z = 1;

	poseB.position.x = 8;
	poseB.position.y = -5;
	poseB.position.z = 0;

	ASSERT_DOUBLE_EQ(amathutils::find_distance(ptA, ptB), sqrt(101)) << "Distance between points should be " << sqrt(101);
	ASSERT_DOUBLE_EQ(amathutils::find_distance(poseA, poseB), sqrt(101)) << "Distance between poses should be " << sqrt(101);
}

// Values from https://math.stackexchange.com/questions/707673/find-angle-in-degrees-from-one-point-to-another-in-2d-space
TEST(TestSuite, GetAngle){

	geometry_msgs::Point ptA, ptB;

	ptA.x = 0;
	ptA.y = 10;
	ptA.z = 0;

	ptB.x = 10;
	ptB.y = 20;
	ptB.z = 0;

	ASSERT_DOUBLE_EQ(amathutils::find_angle(ptA, ptB), 45) << "Angle should be 45deg";
	ASSERT_DOUBLE_EQ(amathutils::find_angle(ptB, ptA), 225) << "Angle should be -45deg";
}

// Values from https://www.mathopenref.com/coordintersection.html
TEST(TestSuite, LineIntersect){

	geometry_msgs::Point l1_p1, l1_p2, l2_p1, l2_p2;
	l1_p1.x = 29;
	l1_p1.y = 5;
	l1_p1.z = 0;
	l1_p2.x = 51;
	l1_p2.y = 15;
	l1_p2.z = 0;
	l2_p1.x = 15;
	l2_p1.y = 10;
	l2_p1.z = 0;
	l2_p2.x = 58;
	l2_p2.y = 10;
	l2_p2.z = 0;

	ASSERT_TRUE(amathutils::isIntersectLine(l1_p1, l1_p2, l2_p1, l2_p2)) << "Lines intersect";
}

// Values from https://www.mathopenref.com/coordintersection.html
TEST(TestSuite, ParallelLines){

	geometry_msgs::Point l1_p1, l1_p2, l2_p1, l2_p2;
	l1_p1.x = 29;
	l1_p1.y = 5;
	l1_p1.z = 0;
	l1_p2.x = 51;
	l1_p2.y = 15;
	l1_p2.z = 0;
	l2_p1.x = 15;
	l2_p1.y = 10;
	l2_p1.z = 0;
	l2_p2.x = 49;
	l2_p2.y = 25;
	l2_p2.z = 0;

	ASSERT_TRUE(!amathutils::isIntersectLine(l1_p1, l1_p2, l2_p1, l2_p2)) << "Parallel lines";
}

#define LEFT 1
#define RIGHT -1
#define ONLINE 0

// 45degree angle line through the origin (0,0)
TEST(TestSuite, PointOnTheLeft){

	geometry_msgs::Point p1, line_p1, line_p2;
	p1.x = 1;
	p1.y = 10;
	p1.z = 0;
	line_p1.x = 1;
	line_p1.y = 1;
	line_p1.z = 0;
	line_p2.x = 10;
	line_p2.y = 10;
	line_p2.z = 0;

	ASSERT_EQ(amathutils::isPointLeftFromLine(p1, line_p1, line_p2), LEFT) << "Point is on the left";
}

// 45degree angle line through the origin (0,0)
TEST(TestSuite, PointOnTheRight){

	geometry_msgs::Point p1, line_p1, line_p2;
	p1.x = 10;
	p1.y = 1;
	p1.z = 0;
	line_p1.x = 1;
	line_p1.y = 1;
	line_p1.z = 0;
	line_p2.x = 10;
	line_p2.y = 10;
	line_p2.z = 0;

	ASSERT_EQ(amathutils::isPointLeftFromLine(p1, line_p1, line_p2), RIGHT) << "Point is on the right";
}

// 45degree angle line through the origin (0,0)
TEST(TestSuite, PointOnTheLine){

	geometry_msgs::Point p1, line_p1, line_p2;
	p1.x = -5;
	p1.y = -5;
	p1.z = 0;
	line_p1.x = 1;
	line_p1.y = 1;
	line_p1.z = 0;
	line_p2.x = 10;
	line_p2.y = 10;
	line_p2.z = 0;

	ASSERT_EQ(amathutils::isPointLeftFromLine(p1, line_p1, line_p2), ONLINE) << "Point is on the line ";
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "TestNode");
	return RUN_ALL_TESTS();
}
