#include <ros/ros.h>
#include <gtest/gtest.h>

#include "roi_object_filter/roi_object_filter.h"

class TestSuite: public ::testing::Test {
public:
	TestSuite(){}
	~TestSuite(){}
};

TEST(TestSuite, CheckConstructor){

	RosRoiObjectFilterApp app;

	// Check boolean
	ASSERT_TRUE(app.getGridmap_ready()) << "Boolean should be " << true;

}



int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "TestNode");
	return RUN_ALL_TESTS();
}

