/*
 * SimulatedTrajectoryFollower.h
 *
 *  Created on: Jun 18, 2016
 *      Author: hatem
 */

#ifndef SimulatedTrajectoryFollower_H_
#define SimulatedTrajectoryFollower_H_
#include "op_planner/RoadNetwork.h"
#include "op_utility/UtilityH.h"
#include "op_planner/PlannerCommonDef.h"

namespace SimulationNS
{

class SimulatedTrajectoryFollower
{
public:
	SimulatedTrajectoryFollower();
	virtual ~SimulatedTrajectoryFollower();

	void PrepareNextWaypoint(const PlannerHNS::WayPoint& CurPos, const double& currVelocity, const double& currSteering);

	void UpdateCurrentPath(const std::vector<PlannerHNS::WayPoint>& path);

	int SteerControllerUpdate(const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& desiredSteerAngle);
	int VeclocityControllerUpdate(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVelocity);

	void Init(const PlannerHNS::ControllerParams& params, const PlannerHNS::CAR_BASIC_INFO& vehicleInfo);

	PlannerHNS::VehicleState DoOneStep(const double& dt, const PlannerHNS::BehaviorState& behavior,
				const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& currPose,
				const PlannerHNS::VehicleState& vehicleState, const bool& bNewTrajectory);

	//Testing Points
	PlannerHNS::WayPoint 	m_PerpendicularPoint;
	PlannerHNS::WayPoint 	m_FollowMePoint;
	double 					m_LateralError;
	double 					m_FollowingDistance;
	PlannerHNS::WayPoint 	m_CurrPos;
	int 					m_iCalculatedIndex;

private:
	PlannerHNS::ControllerParams 			m_Params;
	PlannerHNS::CAR_BASIC_INFO 				m_VehicleInfo;
	std::vector<PlannerHNS::WayPoint> 	m_Path;
	double						m_PrevDesiredSteer; // control output
	double 						m_FollowAcceleration;
	int 						m_iPrevWayPoint;
	UtilityHNS::PIDController 	m_pidSteer;
	UtilityHNS::PIDController 	m_pidVelocity;

	bool FindNextWayPoint(const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& state,
			const double& velocity, PlannerHNS::WayPoint& pursuite_point, PlannerHNS::WayPoint& prep,
			double& lateral_err, double& follow_distance);

	int SteerControllerPart(const PlannerHNS::WayPoint& state, const PlannerHNS::WayPoint& way_point,
			const double& lateral_error, double& steerd);

	int CalculateVelocityDesired(const double& dt, const double& currVel,const PlannerHNS::STATE_TYPE& CurrBehavior,
			double& desiredVel);



};

} /* namespace SimulationNS */

#endif /* SimulatedTrajectoryFollower_H_ */
