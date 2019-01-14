/*
 * RosHelpers.h
 *
 *  Created on: Jun 30, 2016
 *      Author: Hatem Darweesh
 */

#ifndef ROSHELPERS_H_
#define ROSHELPERS_H_

#include <ros/ros.h>

#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/LaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/StopLineArray.h"
#include "vector_map_msgs/DTLaneArray.h"

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "waypoint_follower/libwaypoint_follower.h"
#include "autoware_msgs/LaneArray.h"

#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "op_planner/RoadNetwork.h"

namespace WayPlannerNS
{

enum MSG_TYPE{COMMAND_MSG = 0, CONFIRM_MSG = 1, OPTIONS_MSG = 2, CURR_OPTION_MSG = 3, UNKNOWN_MSG = 5};

class HMI_MSG
{
public:
	MSG_TYPE type;
	std::vector<PlannerHNS::ACTION_TYPE> options;
	PlannerHNS::ACTION_TYPE current;
	int currID;
	bool bErr;
	std::string err_msg;
	HMI_MSG()
	{
		currID = -1;
		type = OPTIONS_MSG;
		current = PlannerHNS::FORWARD_ACTION;
		bErr = false;
	}

	static HMI_MSG FromString(std::string msg)
	{
		HMI_MSG recieved_msg;
		std::vector<std::string> sections = SplitString(msg, ",");
		if (sections.size() == 6)
		{
			int type_str = atoi(sections.at(0).c_str());
			switch (type_str)
			{
			case 0:
				recieved_msg.type = COMMAND_MSG;
				break;
			case 1:
				recieved_msg.type = CONFIRM_MSG;
				break;
			case 2:
				recieved_msg.type = OPTIONS_MSG;
				break;
			case 3:
				recieved_msg.type = CURR_OPTION_MSG;
				break;
			default:
				recieved_msg.type = UNKNOWN_MSG;
				break;
			}

			std::vector<std::string> directions = SplitString(sections.at(1), ";");
			for (unsigned int i = 0; i < directions.size(); i++)
			{
				int idirect = atoi(directions.at(i).c_str());
				if(idirect == 0)
					recieved_msg.options.push_back(PlannerHNS::FORWARD_ACTION);
				else if (idirect == 3)
					recieved_msg.options.push_back(PlannerHNS::LEFT_TURN_ACTION);
				else if (idirect == 4)
					recieved_msg.options.push_back(PlannerHNS::RIGHT_TURN_ACTION);
			}
			recieved_msg.currID = atoi(sections.at(3).c_str());
			recieved_msg.bErr = atoi(sections.at(4).c_str());
			recieved_msg.err_msg = sections.at(5);
		}
		return recieved_msg;
	}

	static std::vector<std::string> SplitString(const std::string& str, const std::string& token)
	{
		std::vector<std::string> str_parts;
		int iFirstPart = 0;
		int iSecondPart = str.find(token, iFirstPart);

		while (iSecondPart > 0 && iSecondPart < str.size())
		{
			str_parts.push_back(str.substr(iFirstPart, iSecondPart- iFirstPart));
			iFirstPart = iSecondPart+1;
			iSecondPart = str.find(token, iFirstPart);
		}

		return str_parts;
	}
};

class RosHelpers
{
public:
	RosHelpers();
	virtual ~RosHelpers();
	static void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);
	static void ConvertFromPlannerHToAutowarePathFormat(const std::vector<PlannerHNS::WayPoint>& path,
				autoware_msgs::LaneArray& laneArray);

	static void ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<PlannerHNS::WayPoint>& curr_path,
			const std::vector<std::vector<PlannerHNS::WayPoint> >& paths,
				visualization_msgs::MarkerArray& markerArray);

	static void ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<std::vector<PlannerHNS::WayPoint> >& globalPaths,
				visualization_msgs::MarkerArray& markerArray);

	static void ConvertFromRoadNetworkToAutowareVisualizeMapFormat(const PlannerHNS::RoadNetwork& map,	visualization_msgs::MarkerArray& markerArray);

	static void createGlobalLaneArrayMarker(std_msgs::ColorRGBA color, const autoware_msgs::LaneArray &lane_waypoints_array, visualization_msgs::MarkerArray& markerArray);

	static void createGlobalLaneArrayVelocityMarker(const autoware_msgs::LaneArray &lane_waypoints_array
			, visualization_msgs::MarkerArray& markerArray);

	static void createGlobalLaneArrayOrientationMarker(const autoware_msgs::LaneArray &lane_waypoints_array
			, visualization_msgs::MarkerArray& markerArray);

	static void ConvertFromPlannerHPointsToAutowarePathFormat(const std::vector<PlannerHNS::GPSPoint>& path,
			autoware_msgs::LaneArray& laneArray);

	static void FindIncommingBranches(const std::vector<std::vector<PlannerHNS::WayPoint> >& globalPaths, const PlannerHNS::WayPoint& currPose, const double& min_distance,
			std::vector<PlannerHNS::WayPoint*>& branches, PlannerHNS::WayPoint* currOptions);
};

}
#endif /* ROSHELPERS_H_ */
