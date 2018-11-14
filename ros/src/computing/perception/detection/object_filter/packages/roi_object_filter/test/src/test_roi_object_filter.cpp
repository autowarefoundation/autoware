#include <ros/ros.h>
#include <gtest/gtest.h>

#include "roi_object_filter/roi_object_filter.h"

class TestSuite: public ::testing::Test {
public:
	TestSuite(){}
	~TestSuite(){}
};

class TestClass{
public:
	TestClass(){};

	RosRoiObjectFilterApp app;

	geometry_msgs::Point TransformPoint(const geometry_msgs::Point& in_point, const tf::Transform& in_tf);
	bool CheckPointInGrid(const grid_map::GridMap& in_grid_map, const cv::Mat& in_grid_image,
	                                            const geometry_msgs::Point& in_point);
};

geometry_msgs::Point TestClass::TransformPoint(const geometry_msgs::Point& in_point, const tf::Transform& in_tf)
{
	return app.TransformPoint(in_point, in_tf);
}

bool TestClass::CheckPointInGrid(const grid_map::GridMap& in_grid_map, const cv::Mat& in_grid_image,
                                            const geometry_msgs::Point& in_point)
{
	return app.CheckPointInGrid(in_grid_map, in_grid_image, in_point);
}


TEST(TestSuite, CheckTransformPoint){

	TestClass testObj;

	// Check translation of 1 along X axis
	tf::Quaternion q(0,0,0,1);
	tf::Vector3 v(1,0,0);
	geometry_msgs::Point inPt, outPt, expectedPt;

	inPt.x = 0;
	inPt.y = 0;
	inPt.z = 0;
	expectedPt.x = 1;
	expectedPt.y = 0;
	expectedPt.z = 0;

	tf::Transform translation(q, v);

	outPt = testObj.TransformPoint(inPt, translation);

	ASSERT_EQ(outPt.x, expectedPt.x) << "X Coordinate should be " << expectedPt.x;
	ASSERT_EQ(outPt.y, expectedPt.y) << "Y Coordinate should be " << expectedPt.y;
	ASSERT_EQ(outPt.z, expectedPt.z) << "Z Coordinate should be " << expectedPt.z;

}



int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "TestNode");
	return RUN_ALL_TESTS();
}

