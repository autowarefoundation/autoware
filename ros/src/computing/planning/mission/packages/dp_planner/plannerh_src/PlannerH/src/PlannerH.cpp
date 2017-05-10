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
PlannerH::PlannerH()
{
	//m_Params = params;
}

 PlannerH::~PlannerH()
{
}

 double PlannerH::PlanUsingReedShepp(const WayPoint& start, const WayPoint& goal, vector<WayPoint>& generatedPath, const double pathDensity, const double smoothFactor)
 {
 	RSPlanner rs_planner(smoothFactor);
 	int numero = 0;
 	double t=0, u=0 , v=0;
 	rs_planner.PATHDENSITY = pathDensity;
 	double length = rs_planner.min_length_rs(start.pos.x, start.pos.y, start.pos.a, goal.pos.x, goal.pos.y, goal.pos.a, numero, t , u , v);
 	rs_planner.constRS(numero, t, u , v, start.pos.x, start.pos.y, start.pos.a, rs_planner.PATHDENSITY, generatedPath);
 	return length;
 }

 double PlannerH::PlanUsingReedSheppWithObstacleDetection(const WayPoint& start, const WayPoint& goal,GridMap& map, vector<WayPoint>& genSmoothedPath,
		 const double pathDensity , const double smoothFactor )
 {
 	RSPlanner rs_planner(smoothFactor);
 	int numero = 0;
 	double t=0, u=0 , v=0;
 	rs_planner.PATHDENSITY = pathDensity;

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

 	if(nChanges > 3 || nMinChangeDistance < 3.2)
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

 void PlannerH::GenerateRunoffTrajectory(const std::vector<std::vector<WayPoint> >& referencePaths,const WayPoint& carPos, const bool& bEnableLaneChange, const double& speed, const double& microPlanDistance,
 				const double& maxSpeed,const double& minSpeed, const double&  carTipMargin, const double& rollInMargin,
 				const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
 				const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
 				const double& SmoothTolerance, const double& speedProfileFactor, const bool& bHeadingSmooth,
 				const int& iCurrGlobalPath, const int& iCurrLocalTraj,
 				std::vector<std::vector<std::vector<WayPoint> > >& rollOutsPaths,
 				std::vector<WayPoint>& sectionPath, std::vector<WayPoint>& sampledPoints)
 {

	 if(referencePaths.size()==0) return;
	 if(microPlanDistance <=0 ) return;
	 rollOutsPaths.clear();
	 vector<vector<WayPoint> > referenceSections;

	 if(iCurrGlobalPath > -1 && iCurrGlobalPath < referencePaths.size())
	 {
		 //1- extract current
		 vector<WayPoint> currExtracted;
		 vector<WayPoint> sideExtracted;

		 PlanningHelpers::ExtractPartFromPointToDistance(referencePaths.at(iCurrGlobalPath), carPos,
				 microPlanDistance, pathDensity, currExtracted,
				 SmoothDataWeight, SmoothWeight, SmoothTolerance);

		 //2- Find Left and Right lane change options Ids
		 vector<int> sideLanes = PlanningHelpers::GetUniqueLeftRightIds(currExtracted);
//		 cout << "Current Global ID: " << iCurrGlobalPath;
//		 for(int k=0; k < sideLanes.size(); k++)
//			 cout << sideLanes.at(k);
//		 cout << endl;

		 for(unsigned int i = 0; i < referencePaths.size(); i++)
		 {
			 if(i == iCurrGlobalPath)
				 referenceSections.push_back(currExtracted);
			 else
			 {
				 sideExtracted.clear();
				 PlanningHelpers::ExtractPartFromPointToDistance(referencePaths.at(i), carPos,
						 microPlanDistance, pathDensity, sideExtracted,
						 SmoothDataWeight, SmoothWeight, SmoothTolerance);

				 double d = 0;
				 double can_lane_id = 0;
				 bool bCanChangeLane = false;
				 for(unsigned int iwp = 1; iwp < sideExtracted.size(); iwp++)
				 {
					 d += hypot(sideExtracted.at(iwp).pos.y - sideExtracted.at(iwp-1).pos.y,
							 sideExtracted.at(iwp).pos.x - sideExtracted.at(iwp-1).pos.x);
					 if(d > LANE_CHANGE_SMOOTH_FACTOR_DISTANCE)
					 {
						 if(PlanningHelpers::FindInList(sideLanes, sideExtracted.at(iwp).laneId) == true)
						 {
							 can_lane_id = sideExtracted.at(iwp).laneId;
							 bCanChangeLane = true;
							 break;
						 }
					 }
				 }

				 if(bCanChangeLane)
				 {
					 referenceSections.push_back(sideExtracted);
					// cout << "Can Change To This Lane : " << can_lane_id  << ", Index: " << i << endl;
				 }
				 else
				 {
					// cout << "Skip This Lane Index : " << i << endl;
					 referenceSections.push_back(vector<WayPoint>());
				 }
			 }
		 }
	 }
	 else
	 {
		 for(unsigned int i = 0; i < referencePaths.size(); i++)
		 {
			vector<WayPoint> centerTrajectorySmoothed;
			 //Get position of the rear axe:
			 PlanningHelpers::ExtractPartFromPointToDistance(referencePaths.at(i), carPos,
					 microPlanDistance, pathDensity, centerTrajectorySmoothed,
					 SmoothDataWeight, SmoothWeight, SmoothTolerance);

			//sectionPath = centerTrajectorySmoothed; // for testing and visualization
			 referenceSections.push_back(centerTrajectorySmoothed);
		 }
	 }

	 for(unsigned int i = 0; i < referenceSections.size(); i++)
	 {
		std::vector<std::vector<WayPoint> > local_rollOutPaths;
		int s_index = 0, e_index = 0;
		vector<double> e_distances;
		if(referenceSections.at(i).size()>0)
		{
			PlanningHelpers::CalculateRollInTrajectories(carPos, speed, referenceSections.at(i), s_index, e_index, e_distances,
					local_rollOutPaths, microPlanDistance, maxSpeed, carTipMargin, rollInMargin,
						 rollInSpeedFactor, pathDensity, rollOutDensity,rollOutNumber,
						 SmoothDataWeight, SmoothWeight, SmoothTolerance, bHeadingSmooth, sampledPoints);
		}
		else
		{
			for(unsigned int j=0; j< rollOutNumber+1; j++)
			{
				local_rollOutPaths.push_back(vector<WayPoint>());
			}
		}

		rollOutsPaths.push_back(local_rollOutPaths);
	 }
 }

 double PlannerH::PlanUsingDPRandom(const WayPoint& start,
 		 const double& maxPlanningDistance,
 		 RoadNetwork& map,
 		 std::vector<std::vector<WayPoint> >& paths)
  {
 	PlannerHNS::WayPoint* pStart = PlannerHNS::MappingHelpers::GetClosestBackWaypointFromMap(start, map);

 	if(!pStart)
 	{
 		GPSPoint sp = start.pos;
 		cout << endl << "Error: PlannerH -> Can't Find Global Waypoint Nodes in the Map for Start (" <<  sp.ToString() << ")" << endl;
 		return 0;
 	}

 	if(!pStart->pLane)
 	{
 		cout << endl << "Error: PlannerH -> Null Lane, Start (" << pStart->pLane << ")" << endl;
 		return 0;
 	}

  	RelativeInfo start_info;
  	PlanningHelpers::GetRelativeInfo(pStart->pLane->points, start, start_info);

  	if(start_info.perp_distance > START_POINT_MAX_DISTANCE)
  	{
  		GPSPoint sp = start.pos;
 		cout << endl << "Error: PlannerH -> Start Distance to Lane is: " << start_info.perp_distance
 		<< ", Pose: " << sp.ToString() << ", LanePose:" << start_info.perp_point.pos.ToString()
 		<< ", LaneID: " << pStart->pLane->id << " -> Check origin and vector map. " << endl;
  		return 0;
  	}

  	vector<WayPoint*> local_cell_to_delete;
  	WayPoint* pLaneCell = 0;
 	pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeStraight(pStart, BACKUP_STRAIGHT_PLAN_DISTANCE, local_cell_to_delete);

	if(!pLaneCell)
	{
		cout << endl << "PlannerH -> Plan (B) Failed, Sorry we Don't have plan (C) This is the END." << endl;
		return 0;
	}


  	vector<WayPoint> path;
  	vector<vector<WayPoint> > tempCurrentForwardPathss;
  	const std::vector<int> globalPath;
  	PlanningHelpers::TraversePathTreeBackwards(pLaneCell, pStart, globalPath, path, tempCurrentForwardPathss);
  	paths.push_back(path);


  	cout << endl <<"Info: PlannerH -> Plan (B) Path With Size (" << (int)path.size() << "), MultiPaths No(" << paths.size() << ") Extraction Time : " << endl;


  	if(path.size()<2)
  	{
  		cout << endl << "Err: PlannerH -> Invalid Path, Car Should Stop." << endl;
  		if(pLaneCell)
  			DeleteWaypoints(local_cell_to_delete);
  		return 0 ;
  	}

  	if(pLaneCell)
  		DeleteWaypoints(local_cell_to_delete);

  	double totalPlanningDistance = path.at(path.size()-1).cost;
  	return totalPlanningDistance;
  }

double PlannerH::PlanUsingDP(const WayPoint& start,
		 const WayPoint& goalPos,
		 const double& maxPlanningDistance,
		 const bool bEnableLaneChange,
		 const std::vector<int>& globalPath,
		 RoadNetwork& map,
		 std::vector<std::vector<WayPoint> >& paths, vector<WayPoint*>* all_cell_to_delete)
 {
	PlannerHNS::WayPoint* pStart = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(start, map);
	PlannerHNS::WayPoint* pGoal = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(goalPos, map);

	if(!pStart ||  !pGoal)
	{
		GPSPoint sp = start.pos;
		GPSPoint gp = goalPos.pos;
		cout << endl << "Error: PlannerH -> Can't Find Global Waypoint Nodes in the Map for Start (" <<  sp.ToString() << ") and Goal (" << gp.ToString() << ")" << endl;
		return 0;
	}

	if(!pStart->pLane || !pGoal->pLane)
	{
		cout << endl << "Error: PlannerH -> Null Lane, Start (" << pStart->pLane << ") and Goal (" << pGoal->pLane << ")" << endl;
		return 0;
	}

 	RelativeInfo start_info, goal_info;
 	PlanningHelpers::GetRelativeInfo(pStart->pLane->points, start, start_info);
 	PlanningHelpers::GetRelativeInfo(pGoal->pLane->points, goalPos, goal_info);

 	if(start_info.perp_distance > START_POINT_MAX_DISTANCE)
 	{
 		GPSPoint sp = start.pos;
		cout << endl << "Error: PlannerH -> Start Distance to Lane is: " << start_info.perp_distance
		<< ", Pose: " << sp.ToString() << ", LanePose:" << start_info.perp_point.pos.ToString()
		<< ", LaneID: " << pStart->pLane->id << " -> Check origin and vector map. " << endl;
 		return 0;
 	}

 	if(goal_info.perp_distance > GOAL_POINT_MAX_DISTANCE)
	{
		GPSPoint gp = goalPos.pos;
		cout << endl << "Error: PlannerH -> Goal Distance to Lane is: " << goal_info.perp_distance
		<< ", Pose: " << gp.ToString() << ", LanePose:" << goal_info.perp_point.pos.ToString()
		<< ", LaneID: " << pGoal->pLane->id << " -> Check origin and vector map. " << endl;
		return 0;
	}

 	vector<WayPoint*> local_cell_to_delete;
 	WayPoint* pLaneCell = 0;
 	char bPlan = 'A';

 	if(all_cell_to_delete)
 		pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeV2(pStart, *pGoal, globalPath, maxPlanningDistance,bEnableLaneChange, *all_cell_to_delete);
 	else
 		pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeV2(pStart, *pGoal, globalPath, maxPlanningDistance,bEnableLaneChange, local_cell_to_delete);

 	if(!pLaneCell)
 	{
 		bPlan = 'B';
 		cout << endl << "PlannerH -> Plan (A) Failed, Trying Plan (B)." << endl;

 		if(all_cell_to_delete)
 		 	pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeStraight(pStart, BACKUP_STRAIGHT_PLAN_DISTANCE, *all_cell_to_delete);
		else
			pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeStraight(pStart, BACKUP_STRAIGHT_PLAN_DISTANCE, local_cell_to_delete);

 		if(!pLaneCell)
 		{
 			bPlan = 'Z';
 			cout << endl << "PlannerH -> Plan (B) Failed, Sorry we Don't have plan (C) This is the END." << endl;
 			return 0;
 		}
 	}

 	vector<WayPoint> path;
 	vector<vector<WayPoint> > tempCurrentForwardPathss;
 	PlanningHelpers::TraversePathTreeBackwards(pLaneCell, pStart, globalPath, path, tempCurrentForwardPathss);
 	if(bPlan == 'A')
 	{
 		PlanningHelpers::ExtractPlanAlernatives(path, paths);
 	}
 	else if (bPlan == 'B')
 	{
		paths.push_back(path);
 	}

 	cout << endl <<"Info: PlannerH -> Plan (" << bPlan << ") Path With Size (" << (int)path.size() << "), MultiPaths No(" << paths.size() << ") Extraction Time : " << endl;


 	if(path.size()<2)
 	{
 		cout << endl << "Err: PlannerH -> Invalid Path, Car Should Stop." << endl;
 		if(pLaneCell && !all_cell_to_delete)
 			DeleteWaypoints(local_cell_to_delete);
 		return 0 ;
 	}

 	if(pLaneCell && !all_cell_to_delete)
 		DeleteWaypoints(local_cell_to_delete);

 	double totalPlanningDistance = path.at(path.size()-1).cost;
 	return totalPlanningDistance;
 }

 double PlannerH::PredictPlanUsingDP(Lane* l, const WayPoint& start, const double& maxPlanningDistance,
			std::vector<std::vector<WayPoint> >& paths)
  {
  	if(!l)
  	{
  		cout <<endl<< "Err: PredictPlanUsingDP, PlannerH -> Null Lane !!" << endl;
  		return 0;
  	}

  	WayPoint carPos = start;
  	vector<vector<WayPoint> > tempCurrentForwardPathss;
  	vector<WayPoint*> all_cell_to_delete;
  	vector<int> globalPath;

  	RelativeInfo info;
  	PlanningHelpers::GetRelativeInfo(l->points, carPos, info);
  	WayPoint closest_p = l->points.at(info.iBack);
  	WayPoint* pStartWP = &l->points.at(info.iBack);

  	if(distance2points(closest_p.pos, carPos.pos) > 8)
  	{
 		cout <<endl<< "Err: PredictiveDP PlannerH -> Distance to Lane is: " << distance2points(closest_p.pos, carPos.pos)
 		<< ", Pose: " << carPos.pos.ToString() << ", LanePose:" << closest_p.pos.ToString()
 		<< ", LaneID: " << l->id << " -> Check origin and vector map. " << endl;
  		return 0;
  	}

  	vector<WayPoint*> pLaneCells;
  	int nPaths =  PlanningHelpers::PredictiveDP(pStartWP, maxPlanningDistance, all_cell_to_delete, pLaneCells);

  	if(nPaths==0)
  	{
  		cout <<endl<< "Err PlannerH -> Null Tree Head." << endl;
  		return 0;
  	}

  	double totalPlanDistance  = 0;
  	for(unsigned int i = 0; i< pLaneCells.size(); i++)
  	{
  		std::vector<WayPoint> path;
  		PlanningHelpers::TraversePathTreeBackwards(pLaneCells.at(i), pStartWP, globalPath, path, tempCurrentForwardPathss);
  		if(path.size()>0)
  			totalPlanDistance+= path.at(path.size()-1).cost;

  		PlanningHelpers::FixPathDensity(path, 0.5);
  		PlanningHelpers::SmoothPath(path, 0.3 , 0.3,0.1);

  		paths.push_back(path);
  	}

  	DeleteWaypoints(all_cell_to_delete);

  	return totalPlanDistance;
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
