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
	double rad_angle = 1;
	ASSERT_EQ(amathutils::rad2deg(rad_angle), rad_angle*180/M_PI) << "Angle in degrees should be " << rad_angle*180/M_PI;
}

TEST(TestSuite, Deg2Rad){
	double deg_angle = 10;
	ASSERT_EQ(amathutils::deg2rad(deg_angle), deg_angle*M_PI/180) << "Angle in radians should be " << deg_angle*M_PI/180;
}

// Value from https://www.google.com/search?client=ubuntu&channel=fs&q=mps+to+kph&ie=utf-8&oe=utf-8
TEST(TestSuite, Transform_mps2kph){
	double mps_value = 1;
	ASSERT_DOUBLE_EQ(amathutils::mps2kmph(mps_value), mps_value*3.6) << "Speed should be " << mps_value*3.6 << "kmph";
}

// Value from https://www.google.com/search?client=ubuntu&channel=fs&q=kph+to+mps&ie=utf-8&oe=utf-8
TEST(TestSuite, Transform_kmph2mps){
	double kmph_value = 1;
	ASSERT_DOUBLE_EQ(amathutils::kmph2mps(kmph_value), kmph_value/3.6) << "Speed should be " << kmph_value/3.6 << "mps";
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
	geometry_msgs::Point a, b, p, near_point_out, near_point;

	a.x = 0;
	a.y = 0;
	a.z = 0;

	b.x = 6;
	b.y = 2;
	b.z = 0;

	p.x = 2;
	p.y = 4;
	p.z = 0;

	near_point.x = 3;
	near_point.y = 1;
	near_point.z = 0;

	near_point_out = amathutils::getNearPtOnLine(p, a, b);

	ASSERT_DOUBLE_EQ(near_point_out.x, near_point.x) << "near_pointoint coordinate X should be " << near_point.x;
	ASSERT_DOUBLE_EQ(near_point_out.y, near_point.y) << "near_pointoint coordinate X should be " << near_point.y;
	ASSERT_DOUBLE_EQ(near_point_out.z, near_point.z) << "near_pointoint coordinate X should be " << near_point.z;

}

// Values taken from https://math.stackexchange.com/questions/13176/how-to-find-a-point-on-a-line-closest-to-another-given-point
TEST(TestSuite, GetNearPointOnLine3D){
	geometry_msgs::Point a, b, p, near_point_out, near_point;
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

	near_point.x = 1.0/3.0;
	near_point.y = 2.0/3.0;
	near_point.z = 1.0/3.0;

	near_point_out = amathutils::getNearPtOnLine(p, a, b);

	ASSERT_NEAR(near_point_out.x, near_point.x, threshold) << "near_pointoint coordinate X should be " << near_point.x;
	ASSERT_NEAR(near_point_out.y, near_point.y, threshold) << "near_pointoint coordinate X should be " << near_point.y;
	ASSERT_NEAR(near_point_out.z, near_point.z, threshold) << "near_pointoint coordinate X should be " << near_point.z;
}

// Values from http://www.math.usm.edu/lambers/mat169/fall09/lecture17.pdf - The Distance Formula
TEST(TestSuite, GetDistance){

	geometry_msgs::Point point_a, point_b;
	geometry_msgs::Pose poseA, poseB;

	point_a.x = 2;
	point_a.y = 3;
	point_a.z = 1;

	point_b.x = 8;
	point_b.y = -5;
	point_b.z = 0;

	poseA.position.x = 2;
	poseA.position.y = 3;
	poseA.position.z = 1;

	poseB.position.x = 8;
	poseB.position.y = -5;
	poseB.position.z = 0;

	ASSERT_DOUBLE_EQ(amathutils::find_distance(point_a, point_b), sqrt(101)) << "Distance between points should be " << sqrt(101);
	ASSERT_DOUBLE_EQ(amathutils::find_distance(poseA, poseB), sqrt(101)) << "Distance between poses should be " << sqrt(101);
}

// Values from https://math.stackexchange.com/questions/707673/find-angle-in-degrees-from-one-point-to-another-in-2d-space
TEST(TestSuite, GetAngle){

	geometry_msgs::Point point_a, point_b;

	point_a.x = 0;
	point_a.y = 10;
	point_a.z = 0;

	point_b.x = 10;
	point_b.y = 20;
	point_b.z = 0;

	ASSERT_DOUBLE_EQ(amathutils::find_angle(point_a, point_b), 45) << "Angle should be 45deg";
	ASSERT_DOUBLE_EQ(amathutils::find_angle(point_b, point_a), 225) << "Angle should be -45deg";
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

// pose_new: 45degrees around Z quaternion (https://www.andre-gaschler.com/rotationconverter/)
TEST(TestSuite, CheckYawAngleFromPose){

	geometry_msgs::Pose pose_origin, pose_new;
	pose_origin.position.x = 0;
	pose_origin.position.y = 0;
	pose_origin.position.z = 0;
	pose_origin.orientation.x = 0;
	pose_origin.orientation.y = 0;
	pose_origin.orientation.z = 0;
	pose_origin.orientation.w = 1;

	pose_new.position.x = 0;
	pose_new.position.y = 0;
	pose_new.position.z = 0;
	pose_new.orientation.x = 0;
	pose_new.orientation.y = 0;
	pose_new.orientation.z = 0.3826834;
	pose_new.orientation.w = 0.9238795;

	double epsilon = 0.0001; //Error threshold [degrees]

	ASSERT_NEAR(amathutils::getPoseYawAngle(pose_origin)*180/M_PI, 0, epsilon) << "Yaw angle should be 0degrees";
	ASSERT_NEAR(amathutils::getPoseYawAngle(pose_new)*180/M_PI, 45, epsilon) << "Yaw angle should be 45degrees";
}

TEST(TestSuite, CheckNormalizedRadian){

	double angle;

	// Angle < 180 degrees
	angle = 60*M_PI/180;
	ASSERT_DOUBLE_EQ(amathutils::radianNormalize(angle), angle) << "Normalized angle < 180 degrees should be " << angle;

	// Angle > 180 degrees
	angle = 210*M_PI/180;
	ASSERT_DOUBLE_EQ(amathutils::radianNormalize(angle), -150*M_PI/180) << "Normalized angle > 180 degrees should be " << -150*M_PI/180;

	// Angle > 360 degrees
	angle = 400*M_PI/180;
	ASSERT_DOUBLE_EQ(amathutils::radianNormalize(angle), 40*M_PI/180) << "Normalized angle > 360 degrees should be " << 40*M_PI/180;
}

TEST(TestSuite, CheckYawAngleDiffs){

	double epsilon = 0.0001; //Error threshold [degrees]
	geometry_msgs::Pose pose_origin, pose_new;
	pose_origin.position.x = 0;
	pose_origin.position.y = 0;
	pose_origin.position.z = 0;
	pose_origin.orientation.x = 0;
	pose_origin.orientation.y = 0;
	pose_origin.orientation.z = 0;
	pose_origin.orientation.w = 1;

	// Diff < 180 degrees
	// pose_new: 45degrees around Z quaternion (https://www.andre-gaschler.com/rotationconverter/)
	pose_new.position.x = 0;
	pose_new.position.y = 0;
	pose_new.position.z = 0;
	pose_new.orientation.x = 0;
	pose_new.orientation.y = 0;
	pose_new.orientation.z = 0.3826834;
	pose_new.orientation.w = 0.9238795;

	ASSERT_NEAR(amathutils::calcPosesAngleDiffRaw(pose_origin, pose_new), -45*M_PI/180, epsilon) << "DiffRaw angle < 180 degrees should be " << -45*M_PI/180 << " radians";
	ASSERT_NEAR(amathutils::calcPosesAngleDiffDeg(pose_origin, pose_new), -45, epsilon) << "DiffDeg angle < 180 degrees should be " << -45 << " degrees";
	ASSERT_NEAR(amathutils::calcPosesAngleDiffRad(pose_origin, pose_new), -45*M_PI/180, epsilon) << "DiffRad angle < 180 degrees should be " << -45*M_PI/180 << " radians";

	// Diff > 180 degrees
	// pose_new: 210degrees around Z quaternion (https://www.andre-gaschler.com/rotationconverter/)
	pose_new.position.x = 0;
	pose_new.position.y = 0;
	pose_new.position.z = 0;
	pose_new.orientation.x = 0;
	pose_new.orientation.y = 0;
	pose_new.orientation.z = 0.9659258;
	pose_new.orientation.w = -0.258819;

	ASSERT_NEAR(amathutils::calcPosesAngleDiffRaw(pose_origin, pose_new), 150*M_PI/180, epsilon) << "DiffRaw angle > 180 degrees should be " << 150*M_PI/180 << " radians";
	ASSERT_NEAR(amathutils::calcPosesAngleDiffDeg(pose_origin, pose_new), 150, epsilon) << "DiffDeg angle > 180 degrees should be " << 150 << " degrees";
	ASSERT_NEAR(amathutils::calcPosesAngleDiffRad(pose_origin, pose_new), 150*M_PI/180, epsilon) << "DiffRad angle > 180 degrees should be " << 150*M_PI/180 << " radians";
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "TestNode");
	return RUN_ALL_TESTS();
}
