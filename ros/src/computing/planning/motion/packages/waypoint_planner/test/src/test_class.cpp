#include "test_class.h"
#include "std_msgs/Int8.h"

void generateTestOccGrid(nav_msgs::OccupancyGrid &occGrid, const ros::Time &stamp)
{

	occGrid.header.seq = 0;
	occGrid.header.stamp = stamp;
	occGrid.header.frame_id = "map";
	occGrid.info.map_load_time = stamp;
	occGrid.info.resolution = 1;
	occGrid.info.width = 4;
	occGrid.info.height = 4;
	occGrid.info.origin.position.x = (-1)*float(occGrid.info.width)/2 * occGrid.info.resolution ;
	occGrid.info.origin.position.y = occGrid.info.height*occGrid.info.resolution;
	occGrid.info.origin.position.z = 0;
	occGrid.info.origin.orientation.x = 0;
	occGrid.info.origin.orientation.y = 0;
	occGrid.info.origin.orientation.z = 0;
	occGrid.info.origin.orientation.w = 1;

	//	std::cerr << "Data " << occGrid << std::endl;
	for (int idx = 0; idx < occGrid.info.width*occGrid.info.height; ++idx) {

		int row = idx/occGrid.info.width;
		int col = idx%occGrid.info.width;

		//		std::cerr << row << ":" << int((float(occGrid.info.width)/2-1)) << "|" ;
		//		std::cerr << col << ":" << int((float(occGrid.info.height)/2+1)) << std::endl;

		if (row > int((float(occGrid.info.width)/2-1)) && col < int((float(occGrid.info.height)/2+1))
				&& row < int(occGrid.info.width-1))
		{
			occGrid.data.push_back(100);
		}
		else
		{
			occGrid.data.push_back(0);
		}
		//		if (idx/occGrid.info.width)
		//			occGrid.data.push_back(10);
	}


	int counter = 0;
	for (int idx = 0; idx < occGrid.info.height; ++idx)
	{
		for (int idx2 = 0; idx2 < occGrid.info.width; ++idx2)
		{
			std::cerr << float(occGrid.data[counter]) << " ";
			++counter;
		}
		std::cerr << std::endl;
	}

}

void testWaypoints(autoware_msgs::Lane &lane, const ros::Time &stamp)
{

	for (int idx = 0; idx < 4; ++idx) {
		autoware_msgs::Waypoint wp;
		wp.gid = idx;
		wp.lid = idx;

		wp.pose.header.seq = 0;
		wp.pose.header.stamp = stamp;
		wp.pose.header.frame_id = "map";
		wp.pose.pose.position.x = idx*1;
		wp.pose.pose.position.y = 0;
		wp.pose.pose.position.z = 0;
		wp.pose.pose.orientation.x = 0;
		wp.pose.pose.orientation.y = 0;
		wp.pose.pose.orientation.z = 0;
		wp.pose.pose.orientation.w = 1;

		wp.twist.header.seq = 0;
		wp.twist.header.stamp = stamp;
		wp.twist.header.frame_id = "map";
		wp.twist.twist.linear.x = 1;
		wp.twist.twist.linear.y = 0;
		wp.twist.twist.linear.z = 0;
		wp.twist.twist.angular.x = 0;
		wp.twist.twist.angular.y = 0;
		wp.twist.twist.angular.z = 0;

		wp.dtlane.dist = 0;
		wp.dtlane.dir = 0;
		wp.dtlane.apara = 0;
		wp.dtlane.r = 0;
		wp.dtlane.slope = 0;
		wp.dtlane.cant = 0;
		wp.dtlane.lw = 0;
		wp.dtlane.rw = 0;

		wp.change_flag = 0;

		wp.wpstate.aid = 0;
		wp.wpstate.lanechange_state = wp.wpstate.NULLSTATE;
		wp.wpstate.steering_state = wp.wpstate.STR_STRAIGHT;
		wp.wpstate.accel_state = 0;
		wp.wpstate.stopline_state = wp.wpstate.TYPE_NULL;
		wp.wpstate.event_state = 0;

		wp.lane_id = 0;
		wp.left_lane_id = 0;
		wp.right_lane_id = 0;
		wp.stop_line_id = 0;
		wp.cost = 0;
		wp.time_cost = 0;

		wp.direction = 0;
		lane.waypoints.push_back(wp);
	}
}

TestClass::TestClass()
{

	stamp = ros::Time::now();

	// Initialize occupancy grid
	generateTestOccGrid(occGrid, stamp);

	// Initialize current pose
	currPose.header.seq = 0;
	currPose.header.stamp = stamp;
	currPose.header.frame_id = "map";
	currPose.pose.position.x = -2;
	currPose.pose.position.y = 0;
	currPose.pose.position.z = 0;
	currPose.pose.orientation.x = 0;
	currPose.pose.orientation.y = 0;
	currPose.pose.orientation.z = 0;
	currPose.pose.orientation.w = 1;

	// Initialize current velocity
	currVelocity.header.seq = 0;
	currVelocity.header.stamp = stamp;
	currVelocity.header.frame_id = "map";
	currVelocity.twist.linear.x = 1;
	currVelocity.twist.linear.y = 0;
	currVelocity.twist.linear.z = 0;
	currVelocity.twist.angular.x = 0;
	currVelocity.twist.angular.y = 0;
	currVelocity.twist.angular.z = 0;

	// Initialize base waypoints
	baseWaypoints.header.seq = 0;
	baseWaypoints.header.stamp = stamp;
	baseWaypoints.header.frame_id = "map";
	baseWaypoints.increment = 0.1;
	baseWaypoints.lane_id = 0;
	testWaypoints(baseWaypoints, stamp);
	baseWaypoints.lane_index = 0;
	baseWaypoints.cost = 0;
	baseWaypoints.closest_object_distance = 1000;
	baseWaypoints.closest_object_velocity = 0;
	baseWaypoints.is_blocked  = false;

	// Initialize closest waypoint
	closestWpIdx.data = getClosestWaypoint(baseWaypoints, currPose.pose);
//	std::cerr << "baseWaypoints : " << baseWaypoints << std::endl;
	std::cerr << "Closest : " << closestWpIdx << std::endl;

	// Initialize obstacle waypoint
	obstacleWpIdx.data = 2;
}

void TestClass::initializeAstarAvoid(const nav_msgs::OccupancyGrid& occGrid,
		const geometry_msgs::PoseStamped& currPose,
		const geometry_msgs::TwistStamped& currVelocity,
		const autoware_msgs::Lane& baseWp,
		const std_msgs::Int32& closestWpIdx,
		const std_msgs::Int32& obstacleWpIdx)
{
	astarAvoidObj.costmapCallback(occGrid);
	astarAvoidObj.currentPoseCallback(currPose);
	astarAvoidObj.currentVelocityCallback(currVelocity);
	astarAvoidObj.baseWaypointsCallback(baseWp);
	astarAvoidObj.closestWaypointCallback(closestWpIdx);
	astarAvoidObj.obstacleWaypointCallback(obstacleWpIdx);
	astarAvoidObj.enable_avoidance_ = true;
}

bool TestClass::checkInitialized()
{
	return astarAvoidObj.checkInitialized();
}

void TestClass::startPlanThread(const autoware_msgs::Lane& current_waypoints, autoware_msgs::Lane& avoid_waypoints, int& end_of_avoid_index, ros::WallTime& start_avoid_time)
{
	astarAvoidObj.startPlanThread(current_waypoints, avoid_waypoints, end_of_avoid_index, start_avoid_time);
}

void TestClass::planWorker(const autoware_msgs::Lane& current_waypoints, autoware_msgs::Lane& avoid_waypoints, int& end_of_avoid_index, AstarAvoid::State& state, ros::WallTime& start_avoid_time)
{
	astarAvoidObj.planWorker(current_waypoints, avoid_waypoints, end_of_avoid_index, state, start_avoid_time);
}

bool TestClass::planAvoidWaypoints(const autoware_msgs::Lane& current_waypoints, autoware_msgs::Lane& avoid_waypoints, int& end_of_avoid_index)
{
	return astarAvoidObj.planAvoidWaypoints(current_waypoints, avoid_waypoints, end_of_avoid_index);
}

void TestClass::mergeAvoidWaypoints(const nav_msgs::Path& path,const autoware_msgs::Lane& current_waypoints, autoware_msgs::Lane& avoid_waypoints, int& end_of_avoid_index)
{
	astarAvoidObj.mergeAvoidWaypoints(path, current_waypoints, avoid_waypoints, end_of_avoid_index);
}

void TestClass::publishWaypoints(const autoware_msgs::Lane& base_waypoints)
{
	astarAvoidObj.publishWaypoints(base_waypoints);
}

tf::Transform TestClass::getTransform(const std::string& from, const std::string& to)
{
	return astarAvoidObj.getTransform(from, to);
}
