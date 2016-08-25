/*
 * FreePlannerHandler.h
 *
 *  Created on: Jul 14, 2016
 *      Author: ai-driver
 */

#ifndef FREEPLANNERHANDLER_H_
#define FREEPLANNERHANDLER_H_

#include <PlannerHHandler.h>

namespace PlannerXNS
{

class FreePlannerHandler: public PlannerH_Handler
{

public:


	FreePlannerHandler();
	virtual ~FreePlannerHandler();

	virtual bool GeneratePlan(const geometry_msgs::Pose& currentPose, const cv_tracker::obj_label& detectedObstacles,
			const runtime_manager::traffic_light& trafficLight, const AutowareVehicleState& carState,
			AutowareBehaviorState& behaviorState, visualization_msgs::MarkerArray& pathToVisualize,
			waypoint_follower::LaneArray& pathToFollow);
};

} /* namespace PlannerXNS */

#endif /* FREEPLANNERHANDLER_H_ */
