/// \file  SimuDecisionMaker.h
/// \brief Decision Maker for Simulated Vehicles
/// \author Hatem Darweesh
/// \date Jan 10, 2018


#ifndef SIMUDECISIONMAKER_H_
#define SIMUDECISIONMAKER_H_

#include "DecisionMaker.h"
#include "TrajectoryDynamicCosts.h"

namespace PlannerHNS
{


class SimuDecisionMaker: public PlannerHNS::DecisionMaker
{
public:
	SimuDecisionMaker();
	virtual ~SimuDecisionMaker();

	//For Simulation
	std::vector<PlannerHNS::GPSPoint> m_CarShapePolygon;
	PlannerHNS::WayPoint m_OdometryState;
	double m_CurrentVelocity, m_CurrentVelocityD; //meter/second
	double m_CurrentSteering, m_CurrentSteeringD; //radians
	PlannerHNS::SHIFT_POS m_CurrentShift , m_CurrentShiftD;

	double m_CurrentAccSteerAngle; //degrees steer wheel range
	double m_CurrentAccVelocity; // kilometer/hour
	double m_SimulationSteeringDelayFactor; //second , time that every degree change in the steering wheel takes
	timespec m_SteerDelayTimer;
	PlannerHNS::TrajectoryDynamicCosts m_TrajectoryCostsCalculator;

	void ReInitializePlanner(const WayPoint& start_pose);
	void InitPolygons();
	void SetSimulatedTargetOdometryReadings(const double& velocity_d, const double& steering_d, const SHIFT_POS& shift_d);
	void FirstLocalizeMe(const PlannerHNS::WayPoint& initCarPos);
	void LocalizeMe(const double& dt); // in seconds
	void UpdateState(const PlannerHNS::VehicleState& state, const bool& bUseDelay = false);
	void GenerateLocalRollOuts();

	bool SelectSafeTrajectory();
	PlannerHNS::VehicleState LocalizeStep(const double& dt, const PlannerHNS::VehicleState& desiredStatus);

	PlannerHNS::BehaviorState DoOneStep(const double& dt,
			const PlannerHNS::VehicleState& vehicleState,
			const int& goalID,
			const std::vector<TrafficLight>& trafficLight,
			const std::vector<PlannerHNS::DetectedObject>& objects,
			const bool& bEmergencyStop);
};

}

#endif /* SIMUDECISIONMAKER_H_ */
