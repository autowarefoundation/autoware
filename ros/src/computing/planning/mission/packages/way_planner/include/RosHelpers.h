/*
 * RosHelpers.h
 *
 *  Created on: Jun 30, 2016
 *      Author: ai-driver
 */

#ifndef ROSHELPERS_H_
#define ROSHELPERS_H_

#include <ros/ros.h>

#include <vector_map_msgs/PointArray.h>
#include <vector_map_msgs/LaneArray.h>
#include <vector_map_msgs/NodeArray.h>
#include <vector_map_msgs/StopLineArray.h>
#include <vector_map_msgs/DTLaneArray.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

//#include <jsk_recognition_msgs/BoundingBox.h>
//#include <jsk_recognition_msgs/BoundingBoxArray.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>

#include "waypoint_follower/libwaypoint_follower.h"
#include "waypoint_follower/LaneArray.h"

#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "RoadNetwork.h"

namespace WayPlannerNS
{

enum MSG_TYPE{COMMAND_MSG = 0, CONFIRM_MSG = 1, OPTIONS_MSG = 2, UNKNOWN_MSG = 5};

class HMI_MSG
{
public:
	MSG_TYPE type;
	std::vector<PlannerHNS::ACTION_TYPE> options;
	bool bErr;
	std::string err_msg;
	HMI_MSG()
	{
		type = OPTIONS_MSG;
		options.push_back(PlannerHNS::FORWARD_ACTION);
		bErr = false;
	}
};

class RosHelpers
{
public:
	RosHelpers();
	virtual ~RosHelpers();
	static void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);
	static void ConvertFromPlannerHToAutowarePathFormat(const std::vector<PlannerHNS::WayPoint>& path,
				waypoint_follower::LaneArray& laneArray);

	static void ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<PlannerHNS::WayPoint>& curr_path,
			const std::vector<std::vector<PlannerHNS::WayPoint> >& paths,
				visualization_msgs::MarkerArray& markerArray);

	static void ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<std::vector<PlannerHNS::WayPoint> >& globalPaths,
				visualization_msgs::MarkerArray& markerArray);

	static void ConvertFromRoadNetworkToAutowareVisualizeMapFormat(const PlannerHNS::RoadNetwork& map,	visualization_msgs::MarkerArray& markerArray);

	static void createGlobalLaneArrayMarker(std_msgs::ColorRGBA color, const waypoint_follower::LaneArray &lane_waypoints_array, visualization_msgs::MarkerArray& markerArray);

	static void createGlobalLaneArrayVelocityMarker(const waypoint_follower::LaneArray &lane_waypoints_array
			, visualization_msgs::MarkerArray& markerArray);

	static void createGlobalLaneArrayOrientationMarker(const waypoint_follower::LaneArray &lane_waypoints_array
			, visualization_msgs::MarkerArray& markerArray);

	static void ConvertFromPlannerHPointsToAutowarePathFormat(const std::vector<PlannerHNS::GPSPoint>& path,
			waypoint_follower::LaneArray& laneArray);

	static void FindIncommingBranches(const std::vector<std::vector<PlannerHNS::WayPoint> >& globalPaths, const PlannerHNS::WayPoint& currPose, const double& min_distance,
			std::vector<PlannerHNS::WayPoint>& branches);
};

}
#endif /* ROSHELPERS_H_ */
