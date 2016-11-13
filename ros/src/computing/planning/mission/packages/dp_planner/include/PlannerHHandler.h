/*
 * PlannerHHandler.h
 *
 *  Created on: Jul 12, 2016
 *      Author: ai-driver
 */

#ifndef PLANNERHHANDLER_H_
#define PLANNERHHANDLER_H_

#include "PlannerXInterface.h"
#include "RoadNetwork.h"
#include "PlannerH.h"
#include "MappingHelpers.h"
#include "UtilityH.h"
#include "DataRW.h"
#include "CarState.h"
#include "SimpleTracker.h"

namespace PlannerXNS
{

class PlannerH_Handler : public PlannerX_Interface
{
public:
	PlannerH_Handler();
	virtual ~PlannerH_Handler();

	virtual void UpdateVehicleInfo(const double& width, const double& length, const double& wheelBase, const double& maxSteerAngle, const double& turningRadius);

	virtual void UpdatePlanningParams(const AutowarePlanningParams& planningParams);

	virtual void UpdateRoadMap(const AutowareRoadNetwork& map);

	virtual bool LoadRoadMap(const std::string& mapFilePath, const bool& bKML_Map, visualization_msgs::MarkerArray& mapToVisualize);

	virtual void UpdateOriginTransformationPoint(const geometry_msgs::Pose& originPoint);

	virtual void UpdateGlobalGoalPosition(const geometry_msgs::Pose& goalPose);

	virtual void UpdatePredefinedPath(const std::vector<int>& predefinedPath);

	virtual bool GeneratePlan(const geometry_msgs::Pose& currentPose,
			const jsk_recognition_msgs::BoundingBoxArray& detectedObstacles,
			const lidar_tracker::CloudClusterArray& clusters,
			const runtime_manager::traffic_light& trafficLight,
			const AutowareVehicleState& carState,
			AutowareBehaviorState& behaviorState,
			visualization_msgs::MarkerArray& pathToVisualize,
			waypoint_follower::LaneArray& pathToFollow,
			jsk_recognition_msgs::BoundingBoxArray& trackedObjects,
			visualization_msgs::MarkerArray& detectedPolygons,
			const bool& bEmergencyStop, const bool& bGreenLight, const bool& bOutsideControl,
			const waypoint_follower::lane& aStarPath, geometry_msgs::PoseStamped& startPoint,
			geometry_msgs::PoseStamped& goalPoint, 	bool& bExternalPlanning);

protected:
	PlannerHNS::RoadNetwork m_Map;
	SimulationNS::CarState m_State;
	//PlannerHNS::WayPoint   m_Goal;
	bool				   m_bMakeNewPlan;
	PlannerHNS::WayPoint   m_OriginPoint;
	std::vector<int> m_PredefinedPath;
	PlannerHNS::PlanningParams m_PlanningParams;
	SimulationNS::CAR_BASIC_INFO m_VehicleInfo;
	SimulationNS::ControllerParams m_ControlParams;
	PlannerHNS::PlannerH* m_pPlannerH;
	PlannerHNS::BehaviorState m_CurrentBehavior;
	struct timespec m_PlanningTimer;
	SimulationNS::SimpleTracker m_ObstacleTracking;
	std::vector<PlannerHNS::WayPoint> m_goals;
	int m_iCurrentGoal;
	bool m_bSlowDown;
	PlannerHNS::GPSPoint m_SlowDownPoint;
	PlannerHNS::GPSPoint m_GoNormalPoint;

	void InitStaticGoals(PlannerHNS::RoadNetwork& map);

	lidar_tracker::CloudCluster GenerateSimulatedObstacleCluster(const double& x_rand, const double& y_rand, const double& z_rand, const int& nPoints, const geometry_msgs::PointStamped& centerPose);
};

} /* namespace PlannerXNS */

#endif /* PLANNERHHANDLER_H_ */
