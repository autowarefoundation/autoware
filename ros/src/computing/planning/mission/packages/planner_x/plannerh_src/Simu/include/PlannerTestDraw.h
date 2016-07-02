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

namespace Graphics
{

class PlannerTestDraw : public DrawObjBase
{
public:
	PlannerTestDraw();
	virtual ~PlannerTestDraw();

	void DrawSimu();
	void DrawInfo();
	void OnLeftClick(const double& x, const double& y);
	void OnRightClick(const double& x, const double& y);
	void OnKeyboardPress(const SPECIAL_KEYS_TYPE& sKey, const unsigned char& key);
	void LoadMaterials();
	void Reset();

	 static void* PlanningThreadStaticEntryPoint(void* pThis);
	 static void* ControlThreadStaticEntryPoint(void* pThis);


public:
	 PlannerHNS::RoadNetwork m_RoadMap;
	PlannerHNS::GridMap* m_pMap;
	PlannerHNS::WayPoint m_goal;
	PlannerHNS::WayPoint m_start;
	bool 				 m_bNewPath;

	pthread_mutex_t planning_mutex;
	pthread_mutex_t behaviors_mutex;
	pthread_mutex_t control_mutex;

	pthread_t planning_thread_tid;
	pthread_t control_thread_tid;
	bool m_bCancelThread;

	double m_PlanningCycleTime;
	double m_ControlCycleTime;

	PlannerHNS::VehicleState m_VehicleState;
	PlannerHNS::BehaviorState m_CurrentBehavior;

	SimulationNS::CarState 	m_State;
	PlannerHNS::GPSPoint m_FollowPoint;
	PlannerHNS::GPSPoint m_PerpPoint;

	GLMmodel* m_CarModel;
	std::vector<PlannerHNS::WayPoint> m_ActualPath;

private:
	void PrepareVectorMapForDrawing();
	void DrawVectorMap();

};

} /* namespace Graphics */

#endif /* PLANNERTESTDRAW_H_ */
