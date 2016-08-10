/*
 * PlannerTestDraw.h
 *
 *  Created on: Jun 17, 2016
 *      Author: hatem
 */

#ifndef PLANNERTESTDRAW_H_
#define PLANNERTESTDRAW_H_
#include <iostream>
#include "DrawObjBase.h"
#include "GridMap.h"
#include "RoadNetwork.h"
#include "CarState.h"
#include "DrawingHelpers.h"
#include "TrajectoryFollower.h"
#include "SimulatedTrajectoryFollower.h"

namespace Graphics
{

class PlannerTestDraw : public DrawObjBase
{
public:
	PlannerTestDraw();
	virtual ~PlannerTestDraw();

	void DrawSimu();
	void DrawInfo(const int& centerX, const int& centerY, const int& maxX, const int& maxY);
	void OnLeftClick(const double& x, const double& y);
	void OnRightClick(const double& x, const double& y);
	void OnKeyboardPress(const SPECIAL_KEYS_TYPE& sKey, const unsigned char& key);
	void LoadMaterials();
	void Reset();
    bool IsInitState();
    void UpdatePlaneStartGoal(const double& x1,const double& y1, const double& a1, const double& x2,const double& y2, const double& a2);
    void AddSimulatedCarPos(const double& x,const double& y, const double& a);

	 static void* PlanningThreadStaticEntryPoint(void* pThis);
	 static void* ControlThreadStaticEntryPoint(void* pThis);
	 static void* SimulationThreadStaticEntryPoint(void* pThis);


	 void InitStartAndGoal(const double& x1,const double& y1, const double& a1, const double& x2,const double& y2, const double& a2);


public:
	 PlannerHNS::RoadNetwork m_RoadMap;
	PlannerHNS::GridMap* m_pMap;
	PlannerHNS::WayPoint m_goal;
	PlannerHNS::WayPoint m_start;
	bool				 m_bMakeNewPlan;

	pthread_mutex_t planning_mutex;
	pthread_mutex_t control_mutex;
	pthread_mutex_t simulation_mutex;

	pthread_t planning_thread_tid;
	pthread_t control_thread_tid;
	pthread_t simulation_thread_tid;
	bool m_bCancelThread;

	double m_PlanningCycleTime;
	double m_ControlCycleTime;

	PlannerHNS::VehicleState m_VehicleState;
	PlannerHNS::BehaviorState m_CurrentBehavior;

	SimulationNS::CarState 	m_State;
	PlannerHNS::GPSPoint 	m_FollowPoint;
	PlannerHNS::GPSPoint 	m_PerpPoint;
	std::vector<DisplayDataObj> m_DisplayList;

	GLMmodel* m_CarModel;
	std::vector<PlannerHNS::WayPoint> m_ActualPath;
	std::vector<PlannerHNS::DetectedObject> m_dummyObstacles;
	std::vector<SimulationNS::SimulatedCarState> m_SimulatedCars;
	std::vector<PlannerHNS::BehaviorState> m_SimulatedBehaviors;
	std::vector<PlannerHNS::VehicleState>  m_SimulatedVehicleState;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_SimulatedPrevTrajectory;
	std::vector<SimulationNS::SimulatedTrajectoryFollower> m_SimulatedPathFollower;

private:
	void PrepareVectorMapForDrawing();
	void DrawVectorMap();


	std::vector<int> m_LanesIds;
	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_ReadyToDrawLanes;
	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_ReadyToDrawCenterLines;



};

} /* namespace Graphics */

#endif /* PLANNERTESTDRAW_H_ */
