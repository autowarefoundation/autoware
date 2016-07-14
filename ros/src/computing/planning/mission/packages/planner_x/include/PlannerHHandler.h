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

namespace PlannerXNS
{

class PlannerH_Handler : public PlannerX_Interface
{
public:
	PlannerH_Handler();
	virtual ~PlannerH_Handler();

	virtual void UpdateRoadMap(const AutowareRoadNetwork& map);

	virtual void UpdateOriginTransformationPoint(const geometry_msgs::Pose& originPoint);

	virtual void UpdateGlobalGoalPosition(const geometry_msgs::Pose& goalPose);

	virtual void UpdatePredefinedPath(const std::vector<int>& predefinedPath);

	virtual bool GeneratePlan(const geometry_msgs::Pose& currentPose, const cv_tracker::obj_label& detectedObstacles,
			const runtime_manager::traffic_light& trafficLight, const AutowareVehicleState& carState,
			AutowareBehaviorState& behaviorState, visualization_msgs::MarkerArray& pathToVisualize,
			waypoint_follower::LaneArray& pathToFollow);

protected:
	PlannerHNS::RoadNetwork m_Map;
	SimulationNS::CarState m_State;
	PlannerHNS::WayPoint   m_Goal;
	PlannerHNS::WayPoint   m_OriginPoint;
	std::vector<int> m_PredefinedPath;
	PlannerHNS::PlanningInternalParams m_PlanningParams;
	PlannerHNS::PlannerH* m_pPlannerH;


	void ConvertFromPlannerHToAutowarePathFormat(const std::vector<PlannerHNS::WayPoint>& path,
			waypoint_follower::LaneArray& laneArray);

	void ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<PlannerHNS::WayPoint>& total_path,
			const std::vector<PlannerHNS::WayPoint>& curr_path, const std::vector<std::vector<PlannerHNS::WayPoint> >& paths,
				visualization_msgs::MarkerArray& markerArray);

	void ConvertFromAutowareObstaclesToPlannerH(const cv_tracker::obj_label& detectedObstacles, std::vector<PlannerHNS::Obstacle>& bstacles);
	PlannerHNS::SHIFT_POS ConvertShiftFromAutowareToPlannerH(const PlannerXNS::AUTOWARE_SHIFT_POS& shift);
	PlannerXNS::AUTOWARE_SHIFT_POS ConvertShiftFromPlannerHToAutoware(const PlannerHNS::SHIFT_POS& shift);
};

} /* namespace PlannerXNS */

#endif /* PLANNERHHANDLER_H_ */
