#include <ros/ros.h>
#include <gtest/gtest.h>
#include "test_class.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <tf/transform_broadcaster.h>

#include "waypoint_planner/astar_avoid/astar_avoid.h"

class TestSuite: public ::testing::Test {
public:
	TestSuite(){}
	~TestSuite(){}
};

TEST(TestSuite, CheckInitialization){

	TestClass testObj;

	// Check false
	ASSERT_TRUE(!testObj.checkInitialized()) << "Should be " << false;

	testObj.initializeAstarAvoid(testObj.occGrid, testObj.currPose, testObj.currVelocity, testObj.baseWaypoints, testObj.closestWpIdx, testObj.obstacleWpIdx);
	ASSERT_TRUE(testObj.checkInitialized()) << "Should be " << true;

	autoware_msgs::Lane avoid_waypoints;
	int end_of_avoid_index = -1;
	AstarAvoid::State state = AstarAvoid::STATE::PLANNING;
	ros::WallTime start_avoid_time = ros::WallTime::now();


	//	tf::TransformBroadcaster br;
	//	tf::Transform transform;
	//	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	//	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
	//	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));

	//	testObj.planAvoidWaypoints(testObj.baseWaypoints, avoid_waypoints, end_of_avoid_index);
	//	testObj.planWorker(baseWaypoints, avoid_waypoints, end_of_avoid_index, state, start_avoid_time);

	rosbag::Bag bag;
	bag.open("/home/autoware/shared_dir/2018-11-23-09-42-01_0.bag");

	for(rosbag::MessageInstance const m: rosbag::View(bag))
	{
		std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
		if (i != NULL)
			std::cerr << i->data << std::endl;
	}

	bag.close();

}



int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "TestNode");
	return RUN_ALL_TESTS();
}
