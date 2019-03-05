/// \file PlannerH.cpp
/// \brief Main functions for path generation (global and local)
/// \author Hatem Darweesh
/// \date Dec 14, 2016

#include "op_planner/PlannerH.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MappingHelpers.h"
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

void PlannerH::GenerateRunoffTrajectory(const std::vector<std::vector<WayPoint> >& referencePaths,const WayPoint& carPos, const bool& bEnableLaneChange, const double& speed, const double& microPlanDistance,
		const double& maxSpeed,const double& minSpeed, const double&  carTipMargin, const double& rollInMargin,
		const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
		const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
		const double& SmoothTolerance, const double& speedProfileFactor, const bool& bHeadingSmooth,
		const int& iCurrGlobalPath, const int& iCurrLocalTraj,
		std::vector<std::vector<std::vector<WayPoint> > >& rollOutsPaths,
		std::vector<WayPoint>& sampledPoints_debug)
{

	if(referencePaths.size()==0) return;
	if(microPlanDistance <=0 ) return;
	rollOutsPaths.clear();

	sampledPoints_debug.clear(); //for visualization only

	for(unsigned int i = 0; i < referencePaths.size(); i++)
	{
		std::vector<std::vector<WayPoint> > local_rollOutPaths;
		int s_index = 0, e_index = 0;
		vector<double> e_distances;
		if(referencePaths.at(i).size()>0)
		{
			PlanningHelpers::CalculateRollInTrajectories(carPos, speed, referencePaths.at(i), s_index, e_index, e_distances,
					local_rollOutPaths, microPlanDistance, maxSpeed, carTipMargin, rollInMargin,
					rollInSpeedFactor, pathDensity, rollOutDensity,rollOutNumber,
					SmoothDataWeight, SmoothWeight, SmoothTolerance, bHeadingSmooth, sampledPoints_debug);
		}
		else
		{
			for(int j=0; j< rollOutNumber+1; j++)
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
	pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeStraight(pStart, maxPlanningDistance, local_cell_to_delete);

	if(!pLaneCell)
	{
		cout << endl << "PlannerH -> Plan (B) Failed, Sorry we Don't have plan (C) This is the END." << endl;
		return 0;
	}


	vector<WayPoint> path;
	vector<vector<WayPoint> > tempCurrentForwardPathss;
	const std::vector<int> globalPath;
	PlanningHelpers::TraversePathTreeBackwards(pLaneCell, pStart, globalPath, path, tempCurrentForwardPathss);
	cout << endl <<"Info: PlannerH -> Plan (B) Path With Size (" << (int)path.size() << "), MultiPaths No(" << paths.size() << ") Extraction Time : " << endl;

	//PlanningHelpers::CreateManualBranch(path, 0, FORWARD_RIGHT_DIR);
	//cout << "Right Branch Created with Size: " << path.size()  << endl;
	//PlanningHelpers::CreateManualBranch(path, 0, FORWARD_LEFT_DIR);
	paths.push_back(path);

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
	bool bEnableGoalBranching = false;

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

	vector<WayPoint> start_path, goal_path;

	if(fabs(start_info.perp_distance) > START_POINT_MAX_DISTANCE)
	{
		GPSPoint sp = start.pos;
		cout << endl << "Error: PlannerH -> Start Distance to Lane is: " << start_info.perp_distance
				<< ", Pose: " << sp.ToString() << ", LanePose:" << start_info.perp_point.pos.ToString()
				<< ", LaneID: " << pStart->pLane->id << " -> Check origin and vector map. " << endl;
		return 0;
	}

	if(fabs(goal_info.perp_distance) > GOAL_POINT_MAX_DISTANCE)
	{
		if(fabs(start_info.perp_distance) > 20)
		{
			GPSPoint gp = goalPos.pos;
			cout << endl << "Error: PlannerH -> Goal Distance to Lane is: " << goal_info.perp_distance
					<< ", Pose: " << gp.ToString() << ", LanePose:" << goal_info.perp_point.pos.ToString()
					<< ", LaneID: " << pGoal->pLane->id << " -> Check origin and vector map. " << endl;
			return 0;
		}
		else
		{
			WayPoint wp = *pGoal;
			wp.pos.x = (goalPos.pos.x+pGoal->pos.x)/2.0;
			wp.pos.y = (goalPos.pos.y+pGoal->pos.y)/2.0;
			goal_path.push_back(wp);
			goal_path.push_back(goalPos);
		}
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
	if(path.size()==0) return 0;

	paths.clear();

	if(bPlan == 'A')
	{
		PlanningHelpers::ExtractPlanAlernatives(path, paths);
	}
	else if (bPlan == 'B')
	{
		paths.push_back(path);
	}

	//attach start path to beginning of all paths, but goal path to only the path connected to the goal path.
	for(unsigned int i=0; i< paths.size(); i++ )
	{
		paths.at(i).insert(paths.at(i).begin(), start_path.begin(), start_path.end());
		if(paths.at(i).size() > 0)
		{
			//if(hypot(paths.at(i).at(paths.at(i).size()-1).pos.y-goal_info.perp_point.pos.y, paths.at(i).at(paths.at(i).size()-1).pos.x-goal_info.perp_point.pos.x) < 1.5)
			{

				if(paths.at(i).size() > 0 && goal_path.size() > 0)
				{
					goal_path.insert(goal_path.begin(), paths.at(i).end()-5, paths.at(i).end());
					PlanningHelpers::SmoothPath(goal_path, 0.25, 0.25);
					PlanningHelpers::FixPathDensity(goal_path, 0.75);
					PlanningHelpers::SmoothPath(goal_path, 0.25, 0.35);
					paths.at(i).erase(paths.at(i).end()-5, paths.at(i).end());
					paths.at(i).insert(paths.at(i).end(), goal_path.begin(), goal_path.end());
				}
			}
		}
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

double PlannerH::PredictPlanUsingDP(PlannerHNS::Lane* l, const WayPoint& start, const double& maxPlanningDistance, std::vector<std::vector<WayPoint> >& paths)
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
		PlanningHelpers::CalcAngleAndCost(path);

		paths.push_back(path);
	}

	DeleteWaypoints(all_cell_to_delete);

	return totalPlanDistance;
}

double PlannerH::PredictTrajectoriesUsingDP(const WayPoint& startPose, std::vector<WayPoint*> closestWPs, const double& maxPlanningDistance, std::vector<std::vector<WayPoint> >& paths, const bool& bFindBranches , const bool bDirectionBased, const bool pathDensity)
{
	vector<vector<WayPoint> > tempCurrentForwardPathss;
	vector<WayPoint*> all_cell_to_delete;
	vector<int> globalPath;

	vector<WayPoint*> pLaneCells;
	vector<int> unique_lanes;
	std::vector<WayPoint> path;
	for(unsigned int j = 0 ; j < closestWPs.size(); j++)
	{
		pLaneCells.clear();
		int nPaths =  PlanningHelpers::PredictiveIgnorIdsDP(closestWPs.at(j), maxPlanningDistance, all_cell_to_delete, pLaneCells, unique_lanes);
		for(unsigned int i = 0; i< pLaneCells.size(); i++)
		{
			path.clear();
			PlanningHelpers::TraversePathTreeBackwards(pLaneCells.at(i), closestWPs.at(j), globalPath, path, tempCurrentForwardPathss);

			for(unsigned int k = 0; k< path.size(); k++)
			{
				bool bFoundLaneID = false;
				for(unsigned int l_id = 0 ; l_id < unique_lanes.size(); l_id++)
				{
					if(path.at(k).laneId == unique_lanes.at(l_id))
					{
						bFoundLaneID = true;
						break;
					}
				}

				if(!bFoundLaneID)
					unique_lanes.push_back(path.at(k).laneId);
			}

			if(path.size()>0)
			{
				path.insert(path.begin(), startPose);
				if(!bDirectionBased)
					path.at(0).pos.a = path.at(1).pos.a;

				path.at(0).beh_state = path.at(1).beh_state = PlannerHNS::BEH_FORWARD_STATE;
				path.at(0).laneId = path.at(1).laneId;

				PlanningHelpers::FixPathDensity(path, pathDensity);
				PlanningHelpers::SmoothPath(path,0.4,0.3,0.1);
				PlanningHelpers::CalcAngleAndCost(path);
				paths.push_back(path);
			}
		}
	}

	if(bDirectionBased && bFindBranches)
	{
		WayPoint p1, p2;
		if(paths.size()> 0 && paths.at(0).size() > 0)
			p2 = p1 = paths.at(0).at(0);
		else
			p2 = p1 = startPose;

		double branch_length = maxPlanningDistance*0.5;

		p2.pos.y = p1.pos.y + branch_length*0.4*sin(p1.pos.a);
		p2.pos.x = p1.pos.x + branch_length*0.4*cos(p1.pos.a);

		vector<WayPoint> l_branch;
		vector<WayPoint> r_branch;

		PlanningHelpers::CreateManualBranchFromTwoPoints(p1, p2, branch_length, FORWARD_RIGHT_DIR,r_branch);
		PlanningHelpers::CreateManualBranchFromTwoPoints(p1, p2, branch_length, FORWARD_LEFT_DIR, l_branch);

		PlanningHelpers::FixPathDensity(l_branch, pathDensity);
		PlanningHelpers::SmoothPath(l_branch,0.4,0.3,0.1);
		PlanningHelpers::CalcAngleAndCost(l_branch);

		PlanningHelpers::FixPathDensity(r_branch, pathDensity);
		PlanningHelpers::SmoothPath(r_branch,0.4,0.3,0.1);
		PlanningHelpers::CalcAngleAndCost(r_branch);

		paths.push_back(l_branch);
		paths.push_back(r_branch);
	}

	DeleteWaypoints(all_cell_to_delete);

	return paths.size();
}

double PlannerH::PredictPlanUsingDP(const WayPoint& startPose, WayPoint* closestWP, const double& maxPlanningDistance, std::vector<std::vector<WayPoint> >& paths, const bool& bFindBranches)
{
	if(!closestWP || !closestWP->pLane)
	{
		cout <<endl<< "Err: PredictPlanUsingDP, PlannerH -> Null Lane !!" << endl;
		return 0;
	}

	vector<vector<WayPoint> > tempCurrentForwardPathss;
	vector<WayPoint*> all_cell_to_delete;
	vector<int> globalPath;

	vector<WayPoint*> pLaneCells;
	int nPaths =  PlanningHelpers::PredictiveDP(closestWP, maxPlanningDistance, all_cell_to_delete, pLaneCells);

	if(nPaths==0)
	{
		cout <<endl<< "Err PlannerH -> Null Tree Head." << endl;
		return 0;
	}

	double totalPlanDistance  = 0;
	for(unsigned int i = 0; i< pLaneCells.size(); i++)
	{
		std::vector<WayPoint> path;
		PlanningHelpers::TraversePathTreeBackwards(pLaneCells.at(i), closestWP, globalPath, path, tempCurrentForwardPathss);
		if(path.size()>0)
		{
			totalPlanDistance+= path.at(path.size()-1).cost;
			path.insert(path.begin(), startPose);
			//path.at(0).pos.a = path.at(1).pos.a;;
		}


		PlanningHelpers::FixPathDensity(path, 0.5);
		PlanningHelpers::SmoothPath(path, 0.3 , 0.3,0.1);
		paths.push_back(path);

		if(bFindBranches)
		{
			int max_branch_index = path.size() > 5 ? 5 : path.size();
			vector<WayPoint> l_branch;
			vector<WayPoint> r_branch;
			l_branch.insert(l_branch.begin(), path.begin(), path.begin()+5);
			r_branch.insert(r_branch.begin(), path.begin(), path.begin()+5);

			PlanningHelpers::CreateManualBranch(r_branch, 0, FORWARD_RIGHT_DIR);
			PlanningHelpers::CreateManualBranch(l_branch, 0, FORWARD_LEFT_DIR);

			paths.push_back(l_branch);
			paths.push_back(r_branch);
		}
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
