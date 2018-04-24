/*
 * AlternativeVisualizer.h
 *
 *  Created on: Jun 17, 2016
 *      Author: hatem
 */

#ifndef AlternativeVisualizer_H_
#define AlternativeVisualizer_H_
#include <iostream>
#include "DrawObjBase.h"
#include "op_planner/RoadNetwork.h"
#include "CarState.h"
#include "DrawingHelpers.h"
#include "TrajectoryFollower.h"
#include "SimulatedTrajectoryFollower.h"
#include "Graph2dBase.h"

namespace Graphics
{

class AlternativeVisualizer : public DrawObjBase
{
public:
	AlternativeVisualizer();
	virtual ~AlternativeVisualizer();

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


public:
    PlannerHNS::RoadNetwork m_RoadMap;
	PlannerHNS::WayPoint m_start;
	PlannerHNS::WayPoint m_goal;
	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_ReadyToDrawLanes;
	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_ReadyToDrawCenterLines;
	std::vector<PlannerHNS::WayPoint> m_GeneratedPath;

private:
	void PrepareVectorMapForDrawing();
	void DrawVectorMap();
	void DrawGPSData();
	void TransToCarCoordinates(const PlannerHNS::WayPoint& currPose, std::vector<PlannerHNS::DetectedObject>& obj_list);
	void TransToWorldCoordinates(const PlannerHNS::WayPoint& currPose, std::vector<PlannerHNS::DetectedObject>& obj_list);
	void medianfilter(std::vector<double> signal, std::vector<double>& result, int nOrder);

};

} /* namespace Graphics */

#endif /* AlternativeVisualizer_H_ */
