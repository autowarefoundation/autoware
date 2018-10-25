#include "amathutils_lib/amathutils.hpp"
#include <ros/ros.h>
#include <gtest/gtest.h>

class TestSuite: public ::testing::Test {
public:
	TestSuite(){}
	~TestSuite(){}
};

TEST(TestSuite, AssignOrigin){
	amathutils::point originPt;

	ASSERT_EQ(originPt.x, 0) << "X coordinate should be 0";
	ASSERT_EQ(originPt.y, 0) << "Y coordinate should be 0";
	ASSERT_EQ(originPt.z, 0) << "Z coordinate should be 0";
}

TEST(TestSuite, AssignOther){
	double xCoord = 0.5;
	double yCoord = -1.3;
	double zCoord = 6.8;
	amathutils::point otherPt(xCoord, yCoord, zCoord);

	ASSERT_DOUBLE_EQ(otherPt.x, xCoord) << "X coordinate should be "<< xCoord;
	ASSERT_DOUBLE_EQ(otherPt.y, yCoord) << "Y coordinate should be "<< yCoord;
	ASSERT_DOUBLE_EQ(otherPt.z, zCoord) << "Z coordinate should be "<< zCoord;
}

// Values from http://www.math.usm.edu/lambers/mat169/fall09/lecture17.pdf - The Distance Formula
TEST(TestSuite, GetDistance){

	double xCoordA = 2;
	double yCoordA = 3;
	double zCoordA = 1;
	double xCoordB = 8;
	double yCoordB = -5;
	double zCoordB = 0;
	amathutils::point ptA(xCoordA, yCoordA, zCoordA);
	amathutils::point ptB(xCoordB, yCoordB, zCoordB);



	ASSERT_DOUBLE_EQ(amathutils::find_distance(&ptA, &ptB), sqrt(101)) << "Distance should be sqrt(101)";
	ASSERT_DOUBLE_EQ(amathutils::find_distance(ptA, ptB), sqrt(101)) << "Distance should be sqrt(101)";
}

// Values from https://math.stackexchange.com/questions/707673/find-angle-in-degrees-from-one-point-to-another-in-2d-space
TEST(TestSuite, GetAngle){

	double xCoordA = 0;
	double yCoordA = 10;
	double zCoordA = 0;
	double xCoordB = 10;
	double yCoordB = 20;
	double zCoordB = 0;
	amathutils::point ptA(xCoordA, yCoordA, zCoordA);
	amathutils::point ptB(xCoordB, yCoordB, zCoordB);

	ASSERT_DOUBLE_EQ(amathutils::find_angle(&ptA, &ptB), 45) << "Angle should be 45deg";
	ASSERT_DOUBLE_EQ(amathutils::find_angle(&ptB, &ptA), 225) << "Angle should be -45deg";
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

// Values from https://www.mathopenref.com/coordintersection.html
TEST(TestSuite, LineIntersect){

	double p1x = 29;
	double p1y = 5;
	double p2x = 51;
	double p2y = 15;
	double p3x = 15;
	double p3y = 10;
	double p4x = 58;
	double p4y = 10;

	ASSERT_TRUE(amathutils::isIntersectLine(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y)) << "Lines intersect";
}

// Values from https://www.mathopenref.com/coordintersection.html
TEST(TestSuite, ParallelLines){

	double p1x = 29;
	double p1y = 5;
	double p2x = 51;
	double p2y = 15;
	double p3x = 15;
	double p3y = 10;
	double p4x = 49;
	double p4y = 25;

	ASSERT_TRUE(!amathutils::isIntersectLine(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y)) << "Parallel lines";
}

#define LEFT 1
#define RIGHT -1
#define ONLINE 0

// 45degree angle line through the origin (0,0)
TEST(TestSuite, PointOnTheLeft){

	double p1x = 1;
	double p1y = 10;
	double line_p1x = 1;
	double line_p1y = 1;
	double line_p2x = 10;
	double line_p2y = 10;

	ASSERT_EQ(amathutils::isPointLeftFromLine(p1x, p1y, line_p1x, line_p1y, line_p2x, line_p2y), LEFT) << "Point is on the left";
}

// 45degree angle line through the origin (0,0)
TEST(TestSuite, PointOnTheRight){

	double p1x = 10;
	double p1y = 1;
	double line_p1x = 1;
	double line_p1y = 1;
	double line_p2x = 10;
	double line_p2y = 10;

	ASSERT_EQ(amathutils::isPointLeftFromLine(p1x, p1y, line_p1x, line_p1y, line_p2x, line_p2y), RIGHT) << "Point is on the right";
}

// 45degree angle line through the origin (0,0)
TEST(TestSuite, PointOnTheLine){

	double p1x = -5;
	double p1y = -5;
	double line_p1x = 1;
	double line_p1y = 1;
	double line_p2x = 10;
	double line_p2y = 10;

	ASSERT_EQ(amathutils::isPointLeftFromLine(p1x, p1y, line_p1x, line_p1y, line_p2x, line_p2y), ONLINE) << "Point is on the line ";
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "TestNode");
	return RUN_ALL_TESTS();
}
