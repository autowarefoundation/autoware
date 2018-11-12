#include <ros/ros.h>
#include "waypoint_planner/astar_avoid/astar_avoid.h"

//void generateTestOccGrid(nav_msgs::OccupancyGrid &occGrid, const ros::Time &stamp);
//void testWaypoints(autoware_msgs::Lane &lane, const ros::Time &stamp);

class TestClass {
public:

	TestClass();

	// Member variables
	AstarAvoid astarAvoidObj;

	ros::Time stamp;

	// Initialize occupancy grid
	nav_msgs::OccupancyGrid occGrid;

	// Initialize current pose
	geometry_msgs::PoseStamped currPose;

	// Initialize current velocity
	geometry_msgs::TwistStamped currVelocity;

	// Initialize base waypoints
	autoware_msgs::Lane baseWaypoints;

	// Initialize closest waypoint
	std_msgs::Int32 closestWpIdx;

	// Initialize obstacle waypoint
	std_msgs::Int32 obstacleWpIdx;

	// Member functions
	void initializeAstarAvoid(const nav_msgs::OccupancyGrid& occGrid,
			const geometry_msgs::PoseStamped& currPose,
			const geometry_msgs::TwistStamped& currVelocity,
			const autoware_msgs::Lane& baseWp,
			const std_msgs::Int32& closestWpIdx,
			const std_msgs::Int32& obstacleWpIdx);

	bool checkInitialized();

	void startPlanThread(const autoware_msgs::Lane& current_waypoints, autoware_msgs::Lane& avoid_waypoints, int& end_of_avoid_index, ros::WallTime& start_avoid_time);

	void planWorker(const autoware_msgs::Lane& current_waypoints, autoware_msgs::Lane& avoid_waypoints, int& end_of_avoid_index, AstarAvoid::State& state, ros::WallTime& start_avoid_time);

	bool planAvoidWaypoints(const autoware_msgs::Lane& current_waypoints, autoware_msgs::Lane& avoid_waypoints, int& end_of_avoid_index);

	void mergeAvoidWaypoints(const nav_msgs::Path& path,const autoware_msgs::Lane& current_waypoints, autoware_msgs::Lane& avoid_waypoints, int& end_of_avoid_index);

	void publishWaypoints(const autoware_msgs::Lane& base_waypoints);

	tf::Transform getTransform(const std::string& from, const std::string& to);
};
