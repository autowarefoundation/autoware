/*
 * PlannerXInterface.h
 *
 *  Created on: Jul 11, 2016
 *      Author: ai-driver
 */

#ifndef PLANNERXINTERFACE_H_
#define PLANNERXINTERFACE_H_

#include <ros/ros.h>
#include <cv_tracker/obj_label.h>
#include <runtime_manager/traffic_light.h>

#include <map_file/PointClassArray.h>
#include <map_file/LaneArray.h>
#include <map_file/NodeArray.h>
#include <map_file/StopLineArray.h>
#include <map_file/DTLaneArray.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "RosHelpers.h"
#include "waypoint_follower/libwaypoint_follower.h"
#include "waypoint_follower/LaneArray.h"
#include <visualization_msgs/MarkerArray.h>
#include "geo_pos_conv.hh"



namespace PlannerXNS
{

class AutowareRoadNetwork
{
public:
	map_file::PointClassArray 		points;
	std::vector<map_file::Lane> 	lanes;
	std::vector<map_file::Node> 	nodes;
	std::vector<map_file::StopLine> stoplines;
	std::vector<map_file::DTLane> 	dtlanes; //center lines
	bool bPoints;
	bool bLanes;
	bool bNodes;
	bool bStopLines;
	bool bDtLanes;

	AutowareRoadNetwork()
	{
		bPoints 	= false;
		bLanes  	= false;
		bStopLines 	= false;
		bDtLanes  	= false;
		bNodes 		= false;
	}
};

enum AUTOWARE_STATE_TYPE {AW_INITIAL_STATE, AW_WAITING_STATE, AW_FORWARD_STATE, AW_STOPPING_STATE, AW_EMERGENCY_STATE,
	AW_TRAFFIC_LIGHT_STOP_STATE, AW_STOP_SIGN_STOP_STATE, AW_FOLLOW_STATE, AW_LANE_CHANGE_STATE, AW_OBSTACLE_AVOIDANCE_STATE, AW_FINISH_STATE};
enum AUTOWARE_LIGHT_INDICATOR {AW_INDICATOR_LEFT, AW_INDICATOR_RIGHT, AW_INDICATOR_BOTH , AW_INDICATOR_NONE};
enum AUTOWARE_SHIFT_POS {AW_SHIFT_POS_PP = 0x60, AW_SHIFT_POS_RR = 0x40, AW_SHIFT_POS_NN = 0x20,
	AW_SHIFT_POS_DD = 0x10, AW_SHIFT_POS_BB = 0xA0, AW_SHIFT_POS_SS = 0x0f, AW_SHIFT_POS_UU = 0xff };

class AutowareBehaviorState
{
public:
	AUTOWARE_STATE_TYPE state;
	double maxVelocity;
	double minVelocity;
	double stopDistance;
	double followVelocity;
	double followDistance;
	AUTOWARE_LIGHT_INDICATOR indicator;

	AutowareBehaviorState()
	{
		state = AW_INITIAL_STATE;
		maxVelocity = 0;
		minVelocity = 0;
		stopDistance = 0;
		followVelocity = 0;
		followDistance = 0;
		indicator  = AW_INDICATOR_NONE;
	}
};

class AutowareVehicleState
{
public:
	double speed;
	double steer;
	AUTOWARE_SHIFT_POS shift;

	AutowareVehicleState()
	{
		speed = 0;
		steer = 0;
		shift = AW_SHIFT_POS_NN;
	}

};

class PlannerX_Interface
{
public:
	PlannerX_Interface();
	virtual ~PlannerX_Interface();
	static PlannerX_Interface* CreatePlannerInstance(const std::string& plannerName);

	virtual void UpdateRoadMap(const AutowareRoadNetwork& map) = 0;

	virtual void UpdateOriginTransformationPoint(const geometry_msgs::Pose& originPoint) = 0;

	virtual void UpdateGlobalGoalPosition(const geometry_msgs::Pose& goalPose) = 0;

	virtual void UpdatePredefinedPath(const std::vector<int>& predefinedPath) = 0;

	virtual bool GeneratePlan(const geometry_msgs::Pose& currentPose, const cv_tracker::obj_label& detectedObstacles,
			const runtime_manager::traffic_light& trafficLight, const AutowareVehicleState& carState,
			AutowareBehaviorState& behaviorState, visualization_msgs::MarkerArray& pathToVisualize, waypoint_follower::LaneArray& pathToFollow) = 0;

protected:
	bool bMap;
	bool bGoal;
	bool bOriginPoint;
	bool bPredefinedPath;
};

} /* namespace PlannerHNS */

#endif /* PLANNERXINTERFACE_H_ */
