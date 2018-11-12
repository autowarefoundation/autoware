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



int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "TestNode");
	return RUN_ALL_TESTS();
}

