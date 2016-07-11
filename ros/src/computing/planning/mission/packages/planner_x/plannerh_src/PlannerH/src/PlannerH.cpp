/*
 * HelperFunctions.cpp
 *
 *  Created on: May 14, 2016
 *      Author: hatem
 */

#include "PlannerH.h"
#include "PlanningHelpers.h"
#include "MappingHelpers.h"
#include <iostream>

using namespace std;
using namespace UtilityHNS;

namespace PlannerHNS
{
PlannerH::PlannerH(const PlanningInternalParams& params)
{
	m_Params = params;
}

 PlannerH::~PlannerH()
{
}

 double PlannerH::PlanUsingReedShepp(const WayPoint& start, const WayPoint& goal, vector<WayPoint>& generatedPath)
 {
 	RSPlanner rs_planner(m_Params.ReedSheppSmoothnessFactor);
 	int numero = 0;
 	double t=0, u=0 , v=0;
 	rs_planner.PATHDENSITY = m_Params.pathDensity;
 	double length = rs_planner.min_length_rs(start.pos.x, start.pos.y, start.pos.a, goal.pos.x, goal.pos.y, goal.pos.a, numero, t , u , v);
 	rs_planner.constRS(numero, t, u , v, start.pos.x, start.pos.y, start.pos.a, rs_planner.PATHDENSITY, generatedPath);
 	return length;
 }

 double PlannerH::PlanUsingReedSheppWithObstacleDetection(const WayPoint& start, const WayPoint& goal,GridMap& map, vector<WayPoint>& genSmoothedPath)
 {
 	RSPlanner rs_planner(m_Params.ReedSheppSmoothnessFactor);
 	int numero = 0;
 	double t=0, u=0 , v=0;
 	rs_planner.PATHDENSITY = m_Params.pathDensity;

 	genSmoothedPath.clear();
 	genSmoothedPath.clear();

 	double length = rs_planner.min_length_rs(start.pos.x, start.pos.y, UtilityHNS::UtilityH::SplitPositiveAngle(start.pos.a), goal.pos.x, goal.pos.y, UtilityHNS::UtilityH::SplitPositiveAngle(goal.pos.a), numero, t , u , v);
 	rs_planner.constRS(numero, t, u , v, start.pos.x, start.pos.y, UtilityHNS::UtilityH::SplitPositiveAngle(start.pos.a), rs_planner.PATHDENSITY, genSmoothedPath);

 	if(genSmoothedPath.size() == 0)
 		return length;

 	CELL_Info* pCellRet = 0;

 	WayPoint p = genSmoothedPath.at(0);
 	int nChanges = 0;
 	double nMinChangeDistance = length;
 	double d = 0;

 	for(unsigned int i=0; i<genSmoothedPath.size(); i++)
 	{
 		if(p.bDir != genSmoothedPath.at(i).bDir)
 		{
 			if(d < nMinChangeDistance)
 				nMinChangeDistance = d;

 			d = 0;

 			nChanges++;
 		}

 		d+= distance2points(p.pos, genSmoothedPath.at(i).pos);

 		p = genSmoothedPath.at(i);

// 		if(map.)
// 			pCellRet = map.GetCellFromPointInnerMap(p.p);
// 		else
 		pCellRet = map.GetCellFromPoint(POINT2D(p.pos.x, p.pos.y));

 		if(pCellRet)
 		{
 			if(pCellRet->nMovingPoints > 0|| pCellRet->nStaticPoints > 0 || pCellRet->heuristic == map.m_MaxHeuristics)
 			{
 				cout << "\n Obstacle Detected \n";
 				genSmoothedPath.clear();
 				return -1;
 			}
 		}
 		else
 		{
 			cout << "\n Outside the Main Grid \n";
 			genSmoothedPath.clear();
 			return -1;
 		}
 	}

 	if(nChanges > 3 || nMinChangeDistance < m_Params.turning_radius)
 	{
 		cout << "\n Too much gear changes \n";
 		genSmoothedPath.clear();
 		return -1;
 	}

 //	pthread_mutex_lock(&planning_mutex);
 //	m_CurrentPath.assign(genSmoothedPath.begin(), genSmoothedPath.end());
 //	m_CurrentSmoothPath.clear();
 //	//m_TotalPath.assign(m_CurrentPath.begin(), m_CurrentPath.end());
 //	m_CurrentSmoothPath.assign(m_CurrentPath.begin(), m_CurrentPath.end());
 //	//m_TotalSmoothPath.assign(m_CurrentSmoothPath.begin(), m_CurrentSmoothPath.end());
 //	pthread_mutex_unlock(&planning_mutex);

 	return length;

 }

 void PlannerH::GenerateRunoffTrajectory(const std::vector<WayPoint>& referencePath,const WayPoint& carPos, const bool& bEnableLaneChange, const double& speed, const double& microPlanDistance,
 				const double& maxSpeed,const double& minSpeed, const double&  carTipMargin, const double& rollInMargin,
 				const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
 				const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
 				const double& SmoothTolerance, const double& speedProfileFactor, const bool& bHeadingSmooth, std::vector<std::vector<WayPoint> >& rollOutPaths)
 {

	 if(referencePath.size()==0) return;
	 timespec t1;
	 UtilityH::GetTickCount(t1);
	 vector<WayPoint> centerTrajectory = referencePath;
	 vector<WayPoint> centerTrajectorySmoothed;



 	int s_index, e_index;
 	vector<double> e_distances;

 	 //Get position of the rear axe:
 	 PlanningHelpers::ExtractPartFromPointToDistance(centerTrajectory, carPos, microPlanDistance*2.0, pathDensity, centerTrajectorySmoothed,
 			 SmoothDataWeight, SmoothWeight, SmoothTolerance);


 	PlanningHelpers::CalculateRollInTrajectories(carPos, speed, centerTrajectorySmoothed, s_index, e_index, e_distances,
 			rollOutPaths, microPlanDistance, maxSpeed, carTipMargin, rollInMargin,
 				 rollInSpeedFactor, pathDensity, rollOutDensity,rollOutNumber,
 				 SmoothDataWeight, SmoothWeight, SmoothTolerance, bHeadingSmooth);

	for(unsigned int i=0; i< rollOutPaths.size(); i++)
	{
		PlanningHelpers::GenerateRecommendedSpeed(rollOutPaths.at(i), maxSpeed, speedProfileFactor);
		PlanningHelpers::SmoothSpeedProfiles(rollOutPaths.at(i), 0.15, 0.35, 0.1);
	}

//Debug code
// 	  if(rollOutPaths.size()>0)
// 	  {
// 		  if(m_bDebugPrint)
// 			  cout << endl << "### -> Planner Z -> Rollout Trajectories (" << RollOutPaths.size() << ") , Size of 0:" <<  RollOutPaths.at(0).size()
// 						  << ", Distance of 0: " << RollOutPaths.at(0).at(RollOutPaths.at(0).size()-1).cost << ", Processing Time: " << MathUtil::GetTimeDiffNow(t1) << endl;
//
// 		  for(unsigned int i=0; i< rollOutPaths.size(); i++)
// 		  {
// 			  ostringstream str;
// 			  str << "BehaviorsLog/Path/RollOutNumber_" << i << "_";
// 			  ConfigAndLogNS::LogMgr::WritePathToFile(str.str(), rollOutPaths.at(i));
// 		  }
// 	  }
   }

 double PlannerH::PlanUsingDP(Lane* l, const WayPoint& start,const WayPoint& goalPos,
			const WayPoint& prevWayPoint, const double& maxPlanningDistance,
			const std::vector<int>& globalPath, std::vector<WayPoint>& path)
 {
 	if(!l)
 	{
 		cout <<endl<< "Err: PlannerH -> Null Lane !!" << endl;
 		return 0;
 	}

 	int nML =0 , nMR = 0;
 	WayPoint carPos = start;
 	vector<vector<WayPoint> > tempCurrentForwardPathss;
 	vector<WayPoint*> all_cell_to_delete;
 	vector<WayPoint> mainPath;
 	WayPoint* pLaneCell = 0;

 	int closest_index = PlanningHelpers::GetClosestPointIndex(l->points, carPos);
 	WayPoint closest_p = l->points.at(closest_index);

 	if(distance2points(closest_p.pos, carPos.pos) > 8)
 	{
		cout <<endl<< "Err: PlannerH -> Distance to Lane is: " << distance2points(closest_p.pos, carPos.pos)
		<< ", Pose: " << carPos.pos.ToString() << ", LanePose:" << closest_p.pos.ToString()
		<< ", LaneID: " << l->id << " -> Check origin and vector map. " << endl;
 		return 0;
 	}

 	WayPoint* pStartWP = &l->points.at(0);
 	pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeV2(pStartWP, prevWayPoint, closest_p, goalPos, globalPath, maxPlanningDistance, nML, nMR, all_cell_to_delete);

 	if(!pLaneCell)
 	{
 		cout <<endl<< "Err PlannerH -> Null Tree Head." << endl;
 		return 0;
 	}

 	//mainPath.push_back(start);
 	//mainPath.push_back(*pLaneCell);

 	PlanningHelpers::TravesePathTreeBackwards(pLaneCell, pStartWP, globalPath, mainPath, tempCurrentForwardPathss);


 	cout << endl <<"Info: PlannerH -> Path With Size (" << (int)mainPath.size() << "), Extraction Time : " << endl;

 	if(mainPath.size()<2)
 	{
 		cout << endl << "Err: PlannerH -> Invalid Path, Car Should Stop." << endl;
 		if(pLaneCell)
 			DeleteWaypoints(all_cell_to_delete);
 		return 0 ;
 	}

 //	ostringstream str;
 //	str << "BehaviorsLog/WholePath_";
 //	ConfigAndLogNS::LogMgr::WritePathToFile(str.str(), mainPath);

 	if(pLaneCell)
 		DeleteWaypoints(all_cell_to_delete);

 	path = mainPath;

 	double totalPlanningDistance = mainPath.at(mainPath.size()-1).cost;
 	return totalPlanningDistance;
 }

 void PlannerH::DeleteWaypoints(vector<WayPoint*>& wps)
 {
 	for(unsigned int i=0; i<wps.size(); i++)
 	{
 		if(wps.at(i))
 		{
 			delete wps.at(i);
 			wps.at(i) = 0;
 		}
 	}
 	wps.clear();
 }

}
