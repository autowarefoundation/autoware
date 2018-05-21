/*
 * RosHelpers.h
 *
 *  Created on: Jun 30, 2016
 *      Author: ai-driver
 */

#ifndef ROSHELPERS_H_
#define ROSHELPERS_H_

#include <ros/ros.h>
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/LocalPlannerH.h"

#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/LaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/StopLineArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/LineArray.h"
#include "vector_map_msgs/AreaArray.h"
#include "vector_map_msgs/SignalArray.h"
#include "vector_map_msgs/StopLine.h"
#include "vector_map_msgs/VectorArray.h"

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "autoware_msgs/CloudClusterArray.h"

#include "waypoint_follower/libwaypoint_follower.h"
#include "autoware_msgs/LaneArray.h"

#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

namespace PlannerXNS
{

class AutowareRoadNetwork
{
public:
	vector_map_msgs::PointArray 	points;
	vector_map_msgs::LaneArray	 	lanes;
	vector_map_msgs::NodeArray 		nodes;
	vector_map_msgs::StopLineArray 	stoplines;
	vector_map_msgs::DTLaneArray 	dtlanes; //center lines
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

class AutowarePlanningParams
{
public:
	double 	maxSpeed;
	double 	minSpeed;
	double 	planningDistance;
	double 	microPlanDistance;
	double 	carTipMargin;
	double 	rollInMargin;
	double 	rollInSpeedFactor;
	double 	pathDensity;
	double 	rollOutDensity;
	int 	rollOutNumber;
	double 	horizonDistance;
	double 	minFollowingDistance;
	double 	maxFollowingDistance;
	double 	minDistanceToAvoid;
	double 	speedProfileFactor;

	bool 	enableLaneChange;
	bool 	enableSwerving;
	bool 	enableFollowing;
	bool 	enableHeadingSmoothing;
	bool 	enableTrafficLightBehavior;

	AutowarePlanningParams()
	{
		maxSpeed 						= 3;
		minSpeed 						= 0;
		planningDistance 				= 10000;
		microPlanDistance 				= 35;
		carTipMargin					= 8.0;
		rollInMargin					= 20.0;
		rollInSpeedFactor				= 0.25;
		pathDensity						= 0.25;
		rollOutDensity					= 0.5;
		rollOutNumber					= 4;
		horizonDistance					= 120;
		minFollowingDistance			= 35;
		maxFollowingDistance			= 40;
		minDistanceToAvoid				= 15;
		speedProfileFactor				= 1.0;

		enableHeadingSmoothing			= false;
		enableSwerving 					= false;
		enableFollowing					= false;
		enableTrafficLightBehavior		= false;
		enableLaneChange 				= false;
	}
};

class RosHelpers
{
public:
	RosHelpers();
	virtual ~RosHelpers();
	static void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);
	static void ConvertFromPlannerHToAutowarePathFormat(const std::vector<PlannerHNS::WayPoint>& path, const int& iStart,
				autoware_msgs::lane & trajectory);

	static void ConvertFromPlannerHRectangleToAutowareRviz(const std::vector<PlannerHNS::GPSPoint>& safety_rect,
			visualization_msgs::Marker& marker);

	static void ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<PlannerHNS::WayPoint>& curr_path,
			const std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& paths, const PlannerHNS::LocalPlannerH& localPlanner,
				visualization_msgs::MarkerArray& markerArray);

	static void ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<std::vector<PlannerHNS::WayPoint> >& globalPaths,
			  visualization_msgs::MarkerArray& markerArray);

	static void ConvertFromRoadNetworkToAutowareVisualizeMapFormat(const PlannerHNS::RoadNetwork& map,	visualization_msgs::MarkerArray& markerArray);

	static void ConvertFromAutowareBoundingBoxObstaclesToPlannerH(const jsk_recognition_msgs::BoundingBoxArray& detectedObstacles,
			std::vector<PlannerHNS::DetectedObject>& impObstacles);

	static void ConvertFromAutowareCloudClusterObstaclesToPlannerH(const PlannerHNS::WayPoint& currState, const PlannerHNS::CAR_BASIC_INFO& car_info,
			const autoware_msgs::CloudClusterArray& clusters,
			std::vector<PlannerHNS::DetectedObject>& impObstacles, int& nOriginalPoints, int& nContourPoints);

	static void ConvertFromPlannerObstaclesToAutoware(const PlannerHNS::WayPoint& currState, const std::vector<PlannerHNS::DetectedObject>& trackedObstacles,
			visualization_msgs::MarkerArray& detectedPolygons);

	static PlannerHNS::SHIFT_POS ConvertShiftFromAutowareToPlannerH(const PlannerXNS::AUTOWARE_SHIFT_POS& shift);
	static PlannerXNS::AUTOWARE_SHIFT_POS ConvertShiftFromPlannerHToAutoware(const PlannerHNS::SHIFT_POS& shift);
	static PlannerXNS::AutowareBehaviorState ConvertBehaviorStateFromPlannerHToAutoware(const PlannerHNS::BehaviorState& beh);
	static std::string GetBehaviorNameFromCode(const PlannerHNS::STATE_TYPE& behState);

	static void VisualizeBehaviorState(const PlannerHNS::WayPoint& currState, const PlannerHNS::BehaviorState& beh, const bool& bGreenLight,const int& avoidDirection, visualization_msgs::Marker& behaviorMarker);

	static void UpdateRoadMap(const AutowareRoadNetwork& src_map, PlannerHNS::RoadNetwork& out_map);

};

}
#endif /* ROSHELPERS_H_ */
