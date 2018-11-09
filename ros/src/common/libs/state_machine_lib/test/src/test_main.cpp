#include <ros/ros.h>
#include <gtest/gtest.h>

class TestSuite: public ::testing::Test {
public:
	TestSuite(){}
	~TestSuite(){}
};

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "TestNode");
	return RUN_ALL_TESTS();
}
