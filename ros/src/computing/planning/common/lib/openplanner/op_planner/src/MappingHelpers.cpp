
/// \file MappingHelpers.cpp
/// \brief Helper functions for mapping operation such as (load and initialize vector maps , convert map from one format to another, .. )
/// \author Hatem Darweesh
/// \date Jul 2, 2016



#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"
#include "op_planner/PlanningHelpers.h"
#include <float.h>

#include "math.h"
#include <fstream>

using namespace UtilityHNS;
using namespace std;
#define RIGHT_INITIAL_TURNS_COST 0
#define LEFT_INITIAL_TURNS_COST 0
#define DEBUG_MAP_PARSING 0
#define DEFAULT_REF_VELOCITY 60 //km/h

namespace PlannerHNS
{

int MappingHelpers::g_max_point_id = 0;
int MappingHelpers::g_max_lane_id = 0;
int MappingHelpers::g_max_stop_line_id = 0;
int MappingHelpers::g_max_traffic_light_id = 0;
double MappingHelpers::m_USING_VER_ZERO = 0;

MappingHelpers::MappingHelpers() {
}

MappingHelpers::~MappingHelpers() {
}

GPSPoint MappingHelpers::GetTransformationOrigin(const int& bToyotaCityMap)
{
//	if(bToyotaCityMap == 1)
//		return GPSPoint(-3700, 99427, -88,0); //toyota city
//	else if(bToyotaCityMap == 2)
//		return GPSPoint(14805.945, 84680.211, -39.59, 0); // for moriyama map
//	else
		return GPSPoint();
	//return GPSPoint(18221.1, 93546.1, -36.19, 0);
}

Lane* MappingHelpers::GetLaneById(const int& id,RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			if(map.roadSegments.at(rs).Lanes.at(i).id == id)
				return &map.roadSegments.at(rs).Lanes.at(i);
		}
	}

	return nullptr;
}

int MappingHelpers::GetLaneIdByWaypointId(const int& id,std::vector<Lane>& lanes)
{
	for(unsigned int in_l= 0; in_l < lanes.size(); in_l++)
	{
		for(unsigned int in_p = 0; in_p<lanes.at(in_l).points.size(); in_p++)
		{
			if(id == lanes.at(in_l).points.at(in_p).id)
			{
				return lanes.at(in_l).points.at(in_p).laneId;
			}
		}
	}

	return 0;
}

int MappingHelpers::ReplaceMyID(int& id,const std::vector<std::pair<int,int> >& rep_list)
{
	for(unsigned int i=0; i < rep_list.size(); i++)
	{
		if(rep_list.at(i).first == id)
		{
			id = rep_list.at(i).second;
			return id;
		}
	}

	return -1;
}

void MappingHelpers::ConstructRoadNetworkFromROSMessage(const std::vector<UtilityHNS::AisanLanesFileReader::AisanLane>& lanes_data,
		const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
		const std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine>& dt_data,
		const std::vector<UtilityHNS::AisanIntersectionFileReader::AisanIntersection>& intersection_data,
		const std::vector<UtilityHNS::AisanAreasFileReader::AisanArea>& area_data,
		const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
		const std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine>& stop_line_data,
		const std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal>& signal_data,
		const std::vector<UtilityHNS::AisanVectorFileReader::AisanVector>& vector_data,
		const std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb>& curb_data,
		const std::vector<UtilityHNS::AisanRoadEdgeFileReader::AisanRoadEdge>& roadedge_data,
		const std::vector<UtilityHNS::AisanWayareaFileReader::AisanWayarea>& wayarea_data,
		const std::vector<UtilityHNS::AisanCrossWalkFileReader::AisanCrossWalk>& crosswalk_data,
		const std::vector<UtilityHNS::AisanNodesFileReader::AisanNode>& nodes_data,
		const std::vector<UtilityHNS::AisanDataConnFileReader::DataConn>& conn_data,
		const GPSPoint& origin, RoadNetwork& map, const bool& bSpecialFlag,
		const bool& bFindLaneChangeLanes, const bool& bFindCurbsAndWayArea)
{
	vector<Lane> roadLanes;
	Lane lane_obj;
	int laneIDSeq = 0;
	WayPoint prevWayPoint;
	UtilityHNS::AisanLanesFileReader::AisanLane prev_lane_point;
	UtilityHNS::AisanLanesFileReader::AisanLane curr_lane_point;
	UtilityHNS::AisanLanesFileReader::AisanLane next_lane_point;
	vector<pair<int,int> > id_replace_list;

	for(unsigned int l= 0; l < lanes_data.size(); l++)
	{
		curr_lane_point = lanes_data.at(l);
		curr_lane_point.originalMapID = -1;

		if(l+1 < lanes_data.size())
		{
			next_lane_point = lanes_data.at(l+1);
			if(curr_lane_point.FLID == next_lane_point.LnID && curr_lane_point.DID == next_lane_point.DID)
			{
				next_lane_point.BLID = curr_lane_point.BLID;
				if(next_lane_point.LaneDir == 'F')
					next_lane_point.LaneDir = curr_lane_point.LaneDir;

				if(curr_lane_point.BLID2 != 0)
				{
					if(next_lane_point.BLID2 == 0)	next_lane_point.BLID2 = curr_lane_point.BLID2;
					else if(next_lane_point.BLID3 == 0)	next_lane_point.BLID3 = curr_lane_point.BLID2;
					else if(next_lane_point.BLID4 == 0)	next_lane_point.BLID4 = curr_lane_point.BLID2;
				}

				if(curr_lane_point.BLID3 != 0)
				{
					if(next_lane_point.BLID2 == 0)	next_lane_point.BLID2 = curr_lane_point.BLID3;
					else if(next_lane_point.BLID3 == 0)	next_lane_point.BLID3 = curr_lane_point.BLID3;
					else if(next_lane_point.BLID4 == 0)	next_lane_point.BLID4 = curr_lane_point.BLID3;
				}

				if(curr_lane_point.BLID3 != 0)
				{
					if(next_lane_point.BLID2 == 0)	next_lane_point.BLID2 = curr_lane_point.BLID4;
					else if(next_lane_point.BLID3 == 0)	next_lane_point.BLID3 = curr_lane_point.BLID4;
					else if(next_lane_point.BLID4 == 0)	next_lane_point.BLID4 = curr_lane_point.BLID4;
				}

				if(curr_lane_point.FLID2 != 0)
				{
					if(next_lane_point.FLID2 == 0)	next_lane_point.FLID2 = curr_lane_point.FLID2;
					else if(next_lane_point.FLID3 == 0)	next_lane_point.FLID3 = curr_lane_point.FLID2;
					else if(next_lane_point.FLID4 == 0)	next_lane_point.FLID4 = curr_lane_point.FLID2;
				}

				if(curr_lane_point.FLID3 != 0)
				{
					if(next_lane_point.FLID2 == 0)	next_lane_point.FLID2 = curr_lane_point.FLID3;
					else if(next_lane_point.FLID3 == 0)	next_lane_point.FLID3 = curr_lane_point.FLID3;
					else if(next_lane_point.FLID4 == 0)	next_lane_point.FLID4 = curr_lane_point.FLID3;
				}

				if(curr_lane_point.FLID3 != 0)
				{
					if(next_lane_point.FLID2 == 0)	next_lane_point.FLID2 = curr_lane_point.FLID4;
					else if(next_lane_point.FLID3 == 0)	next_lane_point.FLID3 = curr_lane_point.FLID4;
					else if(next_lane_point.FLID4 == 0)	next_lane_point.FLID4 = curr_lane_point.FLID4;
				}

				if(prev_lane_point.FLID == curr_lane_point.LnID)
					prev_lane_point.FLID = next_lane_point.LnID;

				id_replace_list.push_back(make_pair(curr_lane_point.LnID, next_lane_point.LnID));
				int originalMapID = curr_lane_point.LnID;
				curr_lane_point = next_lane_point;
				curr_lane_point.originalMapID = originalMapID;
				l++;
			}
		}

		if(curr_lane_point.LnID != prev_lane_point.FLID)
		{
			if(laneIDSeq != 0) //first lane
			{
				lane_obj.toIds.push_back(prev_lane_point.FLID);
				roadLanes.push_back(lane_obj);
//				if(lane_obj.points.size() <= 1)
//					prev_FLID = 0;
			}

			laneIDSeq++;
			lane_obj = Lane();
			lane_obj.speed = curr_lane_point.LimitVel;
			lane_obj.id = curr_lane_point.LnID;
			lane_obj.fromIds.push_back(curr_lane_point.BLID);
			lane_obj.roadId = laneIDSeq;
		}

		WayPoint wp;
		bool bFound = GetWayPoint(curr_lane_point.LnID, lane_obj.id, curr_lane_point.RefVel,curr_lane_point.DID,
				dt_data, points_data,origin, wp);

		wp.originalMapID = curr_lane_point.originalMapID;

		if(curr_lane_point.LaneDir == 'L')
		{
			wp.actionCost.push_back(make_pair(LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST));
			//std::cout << " Left Lane : " << curr_lane_point.LnID << std::endl ;
		}
		else  if(curr_lane_point.LaneDir == 'R')
		{
			wp.actionCost.push_back(make_pair(RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST));
			//std::cout << " Right Lane : " << curr_lane_point.LnID << std::endl ;
		}
		else
		{
			wp.actionCost.push_back(make_pair(FORWARD_ACTION, 0));
		}

		wp.fromIds.push_back(curr_lane_point.BLID);
		wp.toIds.push_back(curr_lane_point.FLID);

		//if(curr_lane_point.JCT > 0)
		if(curr_lane_point.FLID2 > 0)
		{
			lane_obj.toIds.push_back(curr_lane_point.FLID2);
			wp.toIds.push_back(curr_lane_point.FLID2);
		}
		if(curr_lane_point.FLID3 > 0)
		{
			lane_obj.toIds.push_back(curr_lane_point.FLID3);
			wp.toIds.push_back(curr_lane_point.FLID3);
		}
		if(curr_lane_point.FLID4 > 0)
		{
			lane_obj.toIds.push_back(curr_lane_point.FLID4);
			wp.toIds.push_back(curr_lane_point.FLID4);
		}

		if(curr_lane_point.BLID2 > 0)
		{
			lane_obj.fromIds.push_back(curr_lane_point.BLID2);
			wp.fromIds.push_back(curr_lane_point.BLID2);
		}
		if(curr_lane_point.BLID3 > 0)
		{
			lane_obj.fromIds.push_back(curr_lane_point.BLID3);
			wp.fromIds.push_back(curr_lane_point.BLID3);
		}
		if(curr_lane_point.BLID4 > 0)
		{
			lane_obj.fromIds.push_back(curr_lane_point.BLID4);
			wp.fromIds.push_back(curr_lane_point.BLID4);
		}

		//if(prev_lane_point.DID == curr_lane_point.DID && curr_lane_point.LnID == prev_lane_point.FLID)
//		if(prevWayPoint.pos.x == wp.pos.x && prevWayPoint.pos.y == wp.pos.y)
//		{
//			//if((prev_lane_point.FLID2 != 0 && curr_lane_point.FLID2 != 0) || (prev_lane_point.FLID3 != 0 && curr_lane_point.FLID3 != 0) || (prev_lane_point.FLID4 != 0 && curr_lane_point.FLID4 != 0))
//			{
//				cout << "Prev WP, LnID: " << prev_lane_point.LnID << ",BLID: " << prev_lane_point.BLID << ",FLID: " << prev_lane_point.FLID << ",DID: " << prev_lane_point.DID
//						<< ", Begin: " << prev_lane_point.BLID2 << "," << prev_lane_point.BLID3 << "," << prev_lane_point.BLID4
//						<< ", End: " << prev_lane_point.FLID2 << "," << prev_lane_point.FLID3 << "," << prev_lane_point.FLID4 << ": " << prev_lane_point.LaneDir <<   endl;
//				cout << "Curr WP, LnID: " << curr_lane_point.LnID << ",BLID: " << curr_lane_point.BLID << ",FLID: " << curr_lane_point.FLID << ",DID: " << curr_lane_point.DID
//						<< ", Begin: " << curr_lane_point.BLID2 <<  "," << curr_lane_point.BLID3 <<  "," << curr_lane_point.BLID4
//						<< ", End: " << curr_lane_point.FLID2 <<  "," <<curr_lane_point.FLID3 <<  "," << curr_lane_point.FLID4 <<   ": " << curr_lane_point.LaneDir << endl << endl;
//			}
//		}

		if(bFound)
		{
			lane_obj.points.push_back(wp);
			prevWayPoint = wp;
		}
		else
			cout << " Strange ! point is not in the map !! " << endl;

		prev_lane_point = curr_lane_point;
	}

	//delete first two lanes !!!!! Don't know why , you don't know why ! , these two line cost you a lot .. ! why why , works for toyota map , but not with moriyama
	if(bSpecialFlag)
	{
		if(roadLanes.size() > 0)
			roadLanes.erase(roadLanes.begin()+0);
		if(roadLanes.size() > 0)
			roadLanes.erase(roadLanes.begin()+0);
	}

	roadLanes.push_back(lane_obj);

	for(unsigned int i =0; i < roadLanes.size(); i++)
	{
		Lane* pL = &roadLanes.at(i);
		ReplaceMyID(pL->id, id_replace_list);

		for(unsigned int j = 0 ; j < pL->fromIds.size(); j++)
		{
			int id = ReplaceMyID(pL->fromIds.at(j), id_replace_list);
			if(id != -1)
				pL->fromIds.at(j) = id;
		}

		for(unsigned int j = 0 ; j < pL->toIds.size(); j++)
		{
			int id = ReplaceMyID(pL->toIds.at(j), id_replace_list);
			if(id != -1)
				pL->toIds.at(j) = id;
		}

		for(unsigned int j = 0 ; j < pL->points.size(); j++)
		{
			ReplaceMyID(pL->points.at(j).id, id_replace_list);
			ReplaceMyID(pL->points.at(j).laneId, id_replace_list);

			for(unsigned int jj = 0 ; jj < pL->points.at(j).fromIds.size(); jj++)
			{
				int id = ReplaceMyID(pL->points.at(j).fromIds.at(jj), id_replace_list);
				if(id != -1)
					pL->points.at(j).fromIds.at(jj) = id;
			}

			for(unsigned int jj = 0 ; jj < pL->points.at(j).toIds.size(); jj++)
			{
				int id = ReplaceMyID(pL->points.at(j).toIds.at(jj), id_replace_list);
				if(id != -1)
					pL->points.at(j).toIds.at(jj) = id;
			}
		}
	}

	//Link Lanes and lane's waypoints by pointers
	//For each lane, the previous code set the fromId as the id of the last waypoint of the previos lane.
	//here we fix that by finding from each fromID the corresponding point and replace the fromId by the LaneID associated with that point.
	for(unsigned int l= 0; l < roadLanes.size(); l++)
	{
		for(unsigned int fp = 0; fp< roadLanes.at(l).fromIds.size(); fp++)
		{
			roadLanes.at(l).fromIds.at(fp) = GetLaneIdByWaypointId(roadLanes.at(l).fromIds.at(fp), roadLanes);
		}

		for(unsigned int tp = 0; tp< roadLanes.at(l).toIds.size(); tp++)
		{
			roadLanes.at(l).toIds.at(tp) = GetLaneIdByWaypointId(roadLanes.at(l).toIds.at(tp), roadLanes);
		}

		double sum_a = 0;
		for(unsigned int j = 0 ; j < roadLanes.at(l).points.size(); j++)
		{
			sum_a += roadLanes.at(l).points.at(j).pos.a;
		}
		roadLanes.at(l).dir = sum_a/(double)roadLanes.at(l).points.size();
	}

	//map has one road segment
	RoadSegment roadSegment1;
	roadSegment1.id = 1;
	roadSegment1.Lanes = roadLanes;
	map.roadSegments.push_back(roadSegment1);

	//Link Lanes and lane's waypoints by pointers
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		//Link Lanes
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int j = 0 ; j < pL->fromIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->fromIds.at(j))
					{
						pL->fromLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->toIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->toIds.at(j))
					{
						pL->toLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
				pL->points.at(j).pLane  = pL;
			}
		}
	}

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
			    if(pL->points.at(j).actionCost.size() > 0)
			      {
				  if(pL->points.at(j).actionCost.at(0).first == LEFT_TURN_ACTION)
				    {
				      AssignActionCostToLane(pL, LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST);
				      break;
				    }
				  else if(pL->points.at(j).actionCost.at(0).first == RIGHT_TURN_ACTION)
				    {
				      AssignActionCostToLane(pL, RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST);
				    break;

				    }
			      }
			}
		}
	}

	if(bFindLaneChangeLanes)
	{
		cout << " >> Extract Lane Change Information... " << endl;
		FindAdjacentLanes(map);
	}

	//Extract Signals and StopLines
	// Signals
	ExtractSignalData(signal_data, vector_data, points_data, origin, map);


	//Stop Lines
	ExtractStopLinesData(stop_line_data, line_data, points_data, origin, map);


	//Link waypoints
	LinkMissingBranchingWayPoints(map);

	//Link StopLines and Traffic Lights
	LinkTrafficLightsAndStopLines(map);
	//LinkTrafficLightsAndStopLinesConData(conn_data, id_replace_list, map);

	if(bFindCurbsAndWayArea)
	{
		//Curbs
		ExtractCurbData(curb_data, line_data, points_data, origin, map);

		//Wayarea
		ExtractWayArea(area_data, wayarea_data, line_data, points_data, origin, map);
	}

	//Fix angle for lanes
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			PlannerHNS::PlanningHelpers::FixAngleOnly(pL->points);
		}
	}

	cout << "Map loaded from data with " << roadLanes.size()  << " lanes" << endl;
}

void MappingHelpers::AssignActionCostToLane(Lane* pL, ACTION_TYPE action, double cost)
{
  for(unsigned int j = 0 ; j < pL->points.size(); j++)
  {
      pL->points.at(j).actionCost.clear();
      pL->points.at(j).actionCost.push_back(make_pair(action, cost));
  }
}

WayPoint* MappingHelpers::FindWaypoint(const int& id, RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				if(map.roadSegments.at(rs).Lanes.at(i).points.at(p).id == id)
					return &map.roadSegments.at(rs).Lanes.at(i).points.at(p);
			}
		}
	}

	return nullptr;
}

WayPoint* MappingHelpers::FindWaypointV2(const int& id, const int& l_id, RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pLane = &map.roadSegments.at(rs).Lanes.at(i);
			if(pLane ->id != l_id)
			{
				for(unsigned int p= 0; p < pLane->points.size(); p++)
				{
					if(pLane->points.at(p).id == id)
						return &pLane->points.at(p);
				}
			}
		}
	}

	return nullptr;
}

void MappingHelpers::ConstructRoadNetworkFromDataFiles(const std::string vectoMapPath, RoadNetwork& map, const bool& bZeroOrigin)
{
	/**
	 * Exporting the center lines
	 */
	string laneLinesDetails = vectoMapPath + "point.csv";
	string center_lines_info = vectoMapPath + "dtlane.csv";
	string lane_info = vectoMapPath + "lane.csv";
	string node_info = vectoMapPath + "node.csv";
	string area_info = vectoMapPath + "area.csv";
	string line_info = vectoMapPath + "line.csv";
	string signal_info = vectoMapPath + "signaldata.csv";
	string stop_line_info = vectoMapPath + "stopline.csv";
	string vector_info = vectoMapPath + "vector.csv";
	string curb_info = vectoMapPath + "curb.csv";
	string roadedge_info = vectoMapPath + "roadedge.csv";
	string wayarea_info = vectoMapPath + "wayarea.csv";
	string crosswalk_info = vectoMapPath + "crosswalk.csv";
	string conn_info = vectoMapPath + "dataconnection.csv";
	string intersection_info = vectoMapPath + "intersection.csv";

	cout << " >> Loading vector map data files ... " << endl;
	AisanCenterLinesFileReader  center_lanes(center_lines_info);
	AisanLanesFileReader lanes(lane_info);
	AisanPointsFileReader points(laneLinesDetails);
	AisanNodesFileReader nodes(node_info);
	AisanLinesFileReader lines(line_info);
	AisanStopLineFileReader stop_line(stop_line_info);
	AisanSignalFileReader signal(signal_info);
	AisanVectorFileReader vec(vector_info);
	AisanCurbFileReader curb(curb_info);
	AisanRoadEdgeFileReader roadedge(roadedge_info);
	AisanDataConnFileReader conn(conn_info);
	AisanAreasFileReader areas(area_info);
	AisanWayareaFileReader way_area(wayarea_info);
	AisanCrossWalkFileReader cross_walk(crosswalk_info);


	vector<AisanIntersectionFileReader::AisanIntersection> intersection_data;
	vector<AisanNodesFileReader::AisanNode> nodes_data;
	vector<AisanLanesFileReader::AisanLane> lanes_data;
	vector<AisanPointsFileReader::AisanPoints> points_data;
	vector<AisanCenterLinesFileReader::AisanCenterLine> dt_data;
	vector<AisanLinesFileReader::AisanLine> line_data;
	vector<AisanStopLineFileReader::AisanStopLine> stop_line_data;
	vector<AisanSignalFileReader::AisanSignal> signal_data;
	vector<AisanVectorFileReader::AisanVector> vector_data;
	vector<AisanCurbFileReader::AisanCurb> curb_data;
	vector<AisanRoadEdgeFileReader::AisanRoadEdge> roadedge_data;
	vector<AisanAreasFileReader::AisanArea> area_data;
	vector<AisanWayareaFileReader::AisanWayarea> way_area_data;
	vector<AisanCrossWalkFileReader::AisanCrossWalk> crosswalk_data;
	vector<AisanDataConnFileReader::DataConn> conn_data;


	nodes.ReadAllData(nodes_data);
	lanes.ReadAllData(lanes_data);
	points.ReadAllData(points_data);
	center_lanes.ReadAllData(dt_data);
	lines.ReadAllData(line_data);
	stop_line.ReadAllData(stop_line_data);
	signal.ReadAllData(signal_data);
	vec.ReadAllData(vector_data);
	curb.ReadAllData(curb_data);
	roadedge.ReadAllData(roadedge_data);
	areas.ReadAllData(area_data);
	way_area.ReadAllData(way_area_data);
	cross_walk.ReadAllData(crosswalk_data);
	conn.ReadAllData(conn_data);

	if(points_data.size() == 0)
	{
		std::cout << std::endl << "## Alert Can't Read Points Data from vector map files in path: " << vectoMapPath << std::endl;
		return;
	}

	//Special Condtion to be able to pars old data structures
	int bSpecialMap = 0;

	// use this to transform data to origin (0,0,0)
	if(nodes_data.size() > 0 && bSpecialMap == 0)
	{
		ConstructRoadNetworkFromROSMessageV2(lanes_data, points_data, dt_data, intersection_data, area_data,
				line_data, stop_line_data, signal_data, vector_data, curb_data, roadedge_data,
				way_area_data, crosswalk_data, nodes_data, conn_data, &lanes, &points, &nodes, &lines,
				GetTransformationOrigin(0), map, false);
	}
	else
	{
		ConstructRoadNetworkFromROSMessage(lanes_data, points_data, dt_data, intersection_data, area_data,
						line_data, stop_line_data, signal_data, vector_data, curb_data, roadedge_data,
						way_area_data, crosswalk_data, nodes_data, conn_data,
						GetTransformationOrigin(0), map, bSpecialMap == 1);
	}

	WayPoint origin = GetFirstWaypoint(map);
	cout << origin.pos.ToString() ;
}

bool MappingHelpers::GetWayPoint(const int& id, const int& laneID,const double& refVel, const int& did,
		const std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine>& dtpoints,
		const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points,
		const GPSPoint& origin, WayPoint& way_point)
{

	for(unsigned int dtp =0; dtp < dtpoints.size(); dtp++)
	{
		if(dtpoints.at(dtp).DID == did)
		{
			for(unsigned int p =0; p < points.size(); p++)
			{
				if(dtpoints.at(dtp).PID == points.at(p).PID)
				{
					WayPoint wp;
					wp.id = id;
					wp.laneId = laneID;
					wp.v = refVel;
//					double integ_part = points.at(p).L;
//					double deg = trunc(points.at(p).L);
//					double min = trunc((points.at(p).L - deg) * 100.0) / 60.0;
//					double sec = modf((points.at(p).L - deg) * 100.0, &integ_part)/36.0;
//					double L =  deg + min + sec;
//
//					deg = trunc(points.at(p).B);
//					min = trunc((points.at(p).B - deg) * 100.0) / 60.0;
//					sec = modf((points.at(p).B - deg) * 100.0, &integ_part)/36.0;
//					double B = deg + min + sec;

					wp.pos = GPSPoint(points.at(p).Ly + origin.x, points.at(p).Bx + origin.y, points.at(p).H + origin.z, dtpoints.at(dtp).Dir);

					wp.pos.lat = points.at(p).L;
					wp.pos.lon = points.at(p).B;
					wp.pos.alt = points.at(p).H;
					wp.pos.dir = dtpoints.at(dtp).Dir;
					wp.iOriginalIndex = p;

					way_point = wp;
					return 1;
				}
			}
		}
	}

	return false;
}

void MappingHelpers::LoadKML(const std::string& kmlFile, RoadNetwork& map)
{
	//First, Get the main element
	TiXmlElement* pHeadElem = 0;
	TiXmlElement* pElem = 0;

	ifstream f(kmlFile.c_str());
	if(!f.good())
	{
		cout << "Can't Open KML Map File: (" << kmlFile << ")" << endl;
		return;
	}

	std::cout << " >> Loading KML Map file ... " << std::endl;

	TiXmlDocument doc(kmlFile);
	try
	{
		doc.LoadFile();
	}
	catch(exception& e)
	{
		cout << "KML Custom Reader Error, Can't Load .kml File, path is: "<<  kmlFile << endl;
		cout << e.what() << endl;
		return;
	}


	std::cout << " >> Reading Data from KML map file ... " << std::endl;

	pElem = doc.FirstChildElement();
	pHeadElem = GetHeadElement(pElem);

	std::cout << " >> Load Lanes from KML file .. " << std::endl;
	vector<Lane> laneLinksList = GetLanesList(pHeadElem);

	map.roadSegments.clear();
	map.roadSegments = GetRoadSegmentsList(pHeadElem);

	std::cout << " >> Load Traffic lights from KML file .. " << std::endl;
	vector<TrafficLight> trafficLights = GetTrafficLightsList(pHeadElem);

	std::cout << " >> Load Stop lines from KML file .. " << std::endl;
	vector<StopLine> stopLines = GetStopLinesList(pHeadElem);

	std::cout << " >> Load Signes from KML file .. " << std::endl;
	vector<TrafficSign> signs = GetTrafficSignsList(pHeadElem);

	std::cout << " >> Load Crossings from KML file .. " << std::endl;
	vector<Crossing> crossings = GetCrossingsList(pHeadElem);

	std::cout << " >> Load Markings from KML file .. " << std::endl;
	vector<Marking> markings = GetMarkingsList(pHeadElem);

	std::cout << " >> Load Road boundaries from KML file .. " << std::endl;
	vector<Boundary> boundaries = GetBoundariesList(pHeadElem);

	std::cout << " >> Load Curbs from KML file .. " << std::endl;
	vector<Curb> curbs = GetCurbsList(pHeadElem);

	map.signs.clear();
	map.signs = signs;

	map.crossings.clear();
	map.crossings = crossings;

	map.markings.clear();
	map.markings = markings;

	map.boundaries.clear();
	map.boundaries = boundaries;

	map.curbs.clear();
	map.curbs = curbs;

	//Fill the relations
	for(unsigned int i= 0; i<map.roadSegments.size(); i++ )
	{
		for(unsigned int j=0; j < laneLinksList.size(); j++)
		{
			PlanningHelpers::CalcAngleAndCost(laneLinksList.at(j).points);
			map.roadSegments.at(i).Lanes.push_back(laneLinksList.at(j));
		}
	}

	cout << " >> Link lanes and waypoints with pointers ... " << endl;
	//Link Lanes and lane's waypoints by pointers
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		//Link Lanes
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int j = 0 ; j < pL->fromIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->fromIds.at(j))
					{
						pL->fromLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->toIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->toIds.at(j))
					{
						pL->toLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
				pL->points.at(j).pLane  = pL;
			}
		}
	}

	//Link waypoints
	cout << " >> Link missing branches and waypoints... " << endl;
	LinkMissingBranchingWayPointsV2(map);

	cout << " >> Link Lane change semantics ... " << endl;
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);
				if(pWP->pLeft == 0 && pWP->LeftPointId > 0)
				{
					pWP->pLeft = FindWaypointV2(pWP->LeftPointId, pWP->laneId, map);

					if(pWP->pLeft != nullptr)
					{
						pWP->LeftLnId = pWP->pLeft->laneId;
						pWP->pLane->pLeftLane = pWP->pLeft->pLane;

						if(pWP->pLeft->RightPointId == pWP->id)
						{
							pWP->pLeft->pRight = pWP;
							pWP->pLeft->RightLnId = pWP->laneId;
							pWP->pLeft->pLane->pRightLane = pWP->pLane;
						}
					}
				}

				if(pWP->pRight == 0 && pWP->RightPointId > 0)
				{
					pWP->pRight = FindWaypointV2(pWP->RightPointId, pWP->laneId, map);

					if(pWP->pRight != nullptr)
					{
						pWP->RightLnId = pWP->pRight->laneId;
						pWP->pLane->pRightLane = pWP->pRight->pLane;

						if(pWP->pRight->LeftPointId == pWP->id)
						{
							pWP->pRight->pLeft = pWP;
							pWP->pRight->LeftLnId = pWP->laneId;
							pWP->pRight->pLane->pLeftLane = pWP->pLane;
						}
					}
				}
			}
		}
	}

	map.stopLines = stopLines;
	map.trafficLights = trafficLights;

	//Link waypoints && StopLines
	cout << " >> Link Stop lines and Traffic lights ... " << endl;
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
			{
				if(map.trafficLights.at(itl).CheckLane(map.roadSegments.at(rs).Lanes.at(i).id))
				{
					map.trafficLights.at(itl).pLanes.push_back(&map.roadSegments.at(rs).Lanes.at(i));
					map.roadSegments.at(rs).Lanes.at(i).trafficlights.push_back(map.trafficLights.at(itl));
				}
			}

			for(unsigned int isl = 0; isl < map.stopLines.size(); isl++)
			{
				if(map.stopLines.at(isl).laneId == map.roadSegments.at(rs).Lanes.at(i).id)
				{
					map.stopLines.at(isl).pLane = &map.roadSegments.at(rs).Lanes.at(i);
					map.roadSegments.at(rs).Lanes.at(i).stopLines.push_back(map.stopLines.at(isl));
					WayPoint wp((map.stopLines.at(isl).points.at(0).x+map.stopLines.at(isl).points.at(1).x)/2.0, (map.stopLines.at(isl).points.at(0).y+map.stopLines.at(isl).points.at(1).y)/2.0, (map.stopLines.at(isl).points.at(0).z+map.stopLines.at(isl).points.at(1).z)/2.0, (map.stopLines.at(isl).points.at(0).a+map.stopLines.at(isl).points.at(1).a)/2.0);
					map.roadSegments.at(rs).Lanes.at(i).points.at(PlanningHelpers::GetClosestNextPointIndexFast(map.roadSegments.at(rs).Lanes.at(i).points, wp)).stopLineID = map.stopLines.at(isl).id;
				}
			}
		}
	}

	cout << " >> Find Max IDs ... " << endl;
	GetMapMaxIds(map);

	cout << "Map loaded from kml file with (" << laneLinksList.size()  << ") lanes, First Point ( " << GetFirstWaypoint(map).pos.ToString() << ")"<< endl;

}

TiXmlElement* MappingHelpers::GetHeadElement(TiXmlElement* pMainElem)
{
	TiXmlElement* pElem = pMainElem;
	if(pElem)
		pElem = pElem->FirstChildElement("Folder");
	if(pElem && pElem->FirstChildElement("Folder"))
		pElem = pElem->FirstChildElement("Folder");
	if(pElem && pElem->FirstChildElement("Document"))
		pElem = pElem->FirstChildElement("Document");

	if(!pElem)
	{
		return nullptr;
	}
	return pElem;
}

TiXmlElement* MappingHelpers::GetDataFolder(const string& folderName, TiXmlElement* pMainElem)
{
	if(!pMainElem) return nullptr;

	TiXmlElement* pElem = pMainElem->FirstChildElement("Folder");

	string folderID="";
	for(; pElem; pElem=pElem->NextSiblingElement())
	{
		folderID="";
		if(pElem->FirstChildElement("name")->GetText()) //Map Name
			folderID = pElem->FirstChildElement("name")->GetText();
		if(folderID.compare(folderName)==0)
			return pElem;
	}
	return nullptr;
}

WayPoint* MappingHelpers::GetClosestWaypointFromMap(const WayPoint& pos, RoadNetwork& map, const bool bDirectionBased)
{
	double distance_to_nearest_lane = 1;
	Lane* pLane = 0;
	while(distance_to_nearest_lane < 100 && pLane == 0)
	{
		pLane = GetClosestLaneFromMap(pos, map, distance_to_nearest_lane, bDirectionBased);
		distance_to_nearest_lane += 1;
	}

	if(!pLane) return nullptr;

	int closest_index = PlanningHelpers::GetClosestNextPointIndexFast(pLane->points, pos);

	return &pLane->points.at(closest_index);
}

vector<WayPoint*> MappingHelpers::GetClosestWaypointsListFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance, const bool bDirectionBased)
{
	vector<Lane*> pLanes = GetClosestLanesListFromMap(pos, map, distance, bDirectionBased);

	vector<WayPoint*> waypoints_list;

	if(pLanes.size() == 0) return waypoints_list;

	for(unsigned int i = 0; i < pLanes.size(); i++)
	{
		if(pLanes.at(i))
		{
			int closest_index = PlanningHelpers::GetClosestNextPointIndexFast(pLanes.at(i)->points, pos);
			waypoints_list.push_back(&pLanes.at(i)->points.at(closest_index));
		}
	}

	return waypoints_list;
}

WayPoint* MappingHelpers::GetClosestBackWaypointFromMap(const WayPoint& pos, RoadNetwork& map)
{
	double distance_to_nearest_lane = 1;
	Lane* pLane = 0;
	while(distance_to_nearest_lane < 100 && pLane == 0)
	{
		pLane = GetClosestLaneFromMap(pos, map, distance_to_nearest_lane);
		distance_to_nearest_lane += 1;
	}

	if(!pLane) return nullptr;

	int closest_index = PlanningHelpers::GetClosestNextPointIndexFast(pLane->points, pos);

	if(closest_index>2)
		return &pLane->points.at(closest_index-3);
	else if(closest_index>1)
		return &pLane->points.at(closest_index-2);
	else if(closest_index>0)
		return &pLane->points.at(closest_index-1);
	else
		return &pLane->points.at(closest_index);
}

std::vector<Lane*> MappingHelpers::GetClosestLanesFast(const WayPoint& center, RoadNetwork& map, const double& distance)
{
	vector<Lane*> lanesList;
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			Lane* pL = &map.roadSegments.at(j).Lanes.at(k);
			int index = PlanningHelpers::GetClosestNextPointIndexFast(pL->points, center);

			if(index < 0 || index >= pL->points.size()) continue;

			double d = hypot(pL->points.at(index).pos.y - center.pos.y, pL->points.at(index).pos.x - center.pos.x);
			if(d <= distance)
				lanesList.push_back(pL);
		}
	}

	return lanesList;
}

Lane* MappingHelpers::GetClosestLaneFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance, const bool bDirectionBased)
{
	vector<pair<double, Lane*> > laneLinksList;
	double d = 0;
	double min_d = DBL_MAX;
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			//Lane* pLane = &pEdge->lanes.at(k);
			 d = 0;
			min_d = DBL_MAX;
			for(unsigned int pindex=0; pindex< map.roadSegments.at(j).Lanes.at(k).points.size(); pindex ++)
			{

				d = distance2points(map.roadSegments.at(j).Lanes.at(k).points.at(pindex).pos, pos.pos);
				if(d < min_d)
					min_d = d;
			}

			if(min_d < distance)
				laneLinksList.push_back(make_pair(min_d, &map.roadSegments.at(j).Lanes.at(k)));
		}
	}

	if(laneLinksList.size() == 0) return nullptr;

	min_d = DBL_MAX;
	Lane* closest_lane = 0;
	for(unsigned int i = 0; i < laneLinksList.size(); i++)
	{
		RelativeInfo info;
		PlanningHelpers::GetRelativeInfo(laneLinksList.at(i).second->points, pos, info);

		if(info.perp_distance == 0 && laneLinksList.at(i).first != 0)
			continue;

		if(bDirectionBased && fabs(info.perp_distance) < min_d && fabs(info.angle_diff) < 45)
		{
			min_d = fabs(info.perp_distance);
			closest_lane = laneLinksList.at(i).second;
		}
		else if(!bDirectionBased && fabs(info.perp_distance) < min_d)
		{
			min_d = fabs(info.perp_distance);
			closest_lane = laneLinksList.at(i).second;
		}
	}

	return closest_lane;
}

vector<Lane*> MappingHelpers::GetClosestLanesListFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance, const bool bDirectionBased)
{
	vector<pair<double, Lane*> > laneLinksList;
	double d = 0;
	double min_d = DBL_MAX;
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			//Lane* pLane = &pEdge->lanes.at(k);
			 d = 0;
			min_d = DBL_MAX;
			for(unsigned int pindex=0; pindex< map.roadSegments.at(j).Lanes.at(k).points.size(); pindex ++)
			{

				d = distance2points(map.roadSegments.at(j).Lanes.at(k).points.at(pindex).pos, pos.pos);
				if(d < min_d)
					min_d = d;
			}

			if(min_d < distance)
				laneLinksList.push_back(make_pair(min_d, &map.roadSegments.at(j).Lanes.at(k)));
		}
	}

	vector<Lane*> closest_lanes;
	if(laneLinksList.size() == 0) return closest_lanes;


	for(unsigned int i = 0; i < laneLinksList.size(); i++)
	{
		RelativeInfo info;
		PlanningHelpers::GetRelativeInfo(laneLinksList.at(i).second->points, pos, info);

		if(info.perp_distance == 0 && laneLinksList.at(i).first != 0)
			continue;

		if(bDirectionBased && fabs(info.perp_distance) < distance && fabs(info.angle_diff) < 30)
		{
			closest_lanes.push_back(laneLinksList.at(i).second);
		}
		else if(!bDirectionBased && fabs(info.perp_distance) < distance)
		{
			closest_lanes.push_back(laneLinksList.at(i).second);
		}
	}

	return closest_lanes;
}

Lane* MappingHelpers::GetClosestLaneFromMapDirectionBased(const WayPoint& pos, RoadNetwork& map, const double& distance)
{
	vector<pair<double, WayPoint*> > laneLinksList;
	double d = 0;
	double min_d = DBL_MAX;
	int min_i = 0;
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			//Lane* pLane = &pEdge->lanes.at(k);
			d = 0;
			min_d = DBL_MAX;
			for(unsigned int pindex=0; pindex< map.roadSegments.at(j).Lanes.at(k).points.size(); pindex ++)
			{

				d = distance2points(map.roadSegments.at(j).Lanes.at(k).points.at(pindex).pos, pos.pos);
				if(d < min_d)
				{
					min_d = d;
					min_i = pindex;
				}
			}

			if(min_d < distance)
				laneLinksList.push_back(make_pair(min_d, &map.roadSegments.at(j).Lanes.at(k).points.at(min_i)));
		}
	}

	if(laneLinksList.size() == 0) return nullptr;

	min_d = DBL_MAX;
	Lane* closest_lane = 0;
	double a_diff = 0;
	for(unsigned int i = 0; i < laneLinksList.size(); i++)
	{
		RelativeInfo info;
		PlanningHelpers::GetRelativeInfo(laneLinksList.at(i).second->pLane->points, pos, info);
		if(info.perp_distance == 0 && laneLinksList.at(i).first != 0)
			continue;

		a_diff = UtilityH::AngleBetweenTwoAnglesPositive(laneLinksList.at(i).second->pos.a, pos.pos.a);

		if(fabs(info.perp_distance)<min_d && a_diff <= M_PI_4)
		{
			min_d = fabs(info.perp_distance);
			closest_lane = laneLinksList.at(i).second->pLane;
		}
	}

	return closest_lane;
}


 std::vector<Lane*> MappingHelpers::GetClosestMultipleLanesFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance)
{
	vector<Lane*> lanesList;
	double d = 0;
	double a_diff = 0;
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			for(unsigned int pindex=0; pindex< map.roadSegments.at(j).Lanes.at(k).points.size(); pindex ++)
			{
				d = distance2points(map.roadSegments.at(j).Lanes.at(k).points.at(pindex).pos, pos.pos);
				a_diff = UtilityH::AngleBetweenTwoAnglesPositive(map.roadSegments.at(j).Lanes.at(k).points.at(pindex).pos.a, pos.pos.a);

				if(d <= distance && a_diff <= M_PI_4)
				{
					bool bLaneExist = false;
					for(unsigned int il = 0; il < lanesList.size(); il++)
					{
						if(lanesList.at(il)->id == map.roadSegments.at(j).Lanes.at(k).id)
						{
							bLaneExist = true;
							break;
						}
					}

					if(!bLaneExist)
						lanesList.push_back(&map.roadSegments.at(j).Lanes.at(k));

					break;
				}
			}
		}
	}

	return lanesList;
}

WayPoint MappingHelpers::GetFirstWaypoint(RoadNetwork& map)
{
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			for(unsigned int pindex=0; pindex< map.roadSegments.at(j).Lanes.at(k).points.size(); pindex ++)
			{
				WayPoint fp =  map.roadSegments.at(j).Lanes.at(k).points.at(pindex);
				return fp;
			}
		}
	}

	return WayPoint();
}

WayPoint* MappingHelpers::GetLastWaypoint(RoadNetwork& map)
{
	if(map.roadSegments.size() > 0 && map.roadSegments.at(map.roadSegments.size()-1).Lanes.size() > 0)
	{
		std::vector<Lane>* lanes = &map.roadSegments.at(map.roadSegments.size()-1).Lanes;
		if(lanes->at(lanes->size()-1).points.size() > 0)
			return &lanes->at(lanes->size()-1).points.at(lanes->at(lanes->size()-1).points.size()-1);
	}

	return nullptr;
}

void MappingHelpers::GetUniqueNextLanes(const Lane* l,  const vector<Lane*>& traversed_lanes, vector<Lane*>& lanes_list)
{
	if(!l) return;

	for(unsigned int i=0; i< l->toLanes.size(); i++)
	{
		bool bFound = false;
		for(unsigned int j = 0; j < traversed_lanes.size(); j++)
		if(l->toLanes.at(i)->id == traversed_lanes.at(j)->id)
		{
			bFound = true;
			break;
		}

		if(!bFound)
			lanes_list.push_back(l->toLanes.at(i));
	}
}

Lane* MappingHelpers::GetLaneFromPath(const WayPoint& currPos, const std::vector<WayPoint>& currPath)
{
	if(currPath.size() < 1) return nullptr;

	int closest_index = PlanningHelpers::GetClosestNextPointIndexFast(currPath, currPos);

	return currPath.at(closest_index).pLane;
}

std::vector<Curb> MappingHelpers::GetCurbsList(TiXmlElement* pElem)
{
	vector<Curb> cList;
	TiXmlElement* pCurbs = GetDataFolder("CurbsLines", pElem);
	if(!pCurbs)
		return cList;

	TiXmlElement* pE = pCurbs->FirstChildElement("Placemark");
	for( ; pE; pE=pE->NextSiblingElement())
	{
		string tfID;
		TiXmlElement* pNameXml = pE->FirstChildElement("name");

		if(pNameXml)
		{
			tfID = pNameXml->GetText();
			Curb c;
			c.id = GetIDsFromPrefix(tfID, "BID", "LnID").at(0);
			c.laneId = GetIDsFromPrefix(tfID, "LnID", "RdID").at(0);
			c.roadId = GetIDsFromPrefix(tfID, "RdID", "").at(0);

			TiXmlElement* pPoints = pE->FirstChildElement("LineString")->FirstChildElement("coordinates");
			c.points = GetPointsData(pPoints);

			cList.push_back(c);
		}
	}

	return cList;
}

std::vector<Boundary> MappingHelpers::GetBoundariesList(TiXmlElement* pElem)
{
	vector<Boundary> bList;
	TiXmlElement* pBoundaries = GetDataFolder("Boundaries", pElem);
	if(!pBoundaries)
		return bList;

	TiXmlElement* pE = pBoundaries->FirstChildElement("Placemark");
	for( ; pE; pE=pE->NextSiblingElement())
	{
		string tfID;
		TiXmlElement* pNameXml = pE->FirstChildElement("name");

		if(pNameXml)
		{
			tfID = pNameXml->GetText();
			Boundary b;
			b.id = GetIDsFromPrefix(tfID, "BID", "RdID").at(0);
			b.roadId = GetIDsFromPrefix(tfID, "RdID", "").at(0);

			TiXmlElement* pPoints = pE->FirstChildElement("LineString")->FirstChildElement("coordinates");
			b.points = GetPointsData(pPoints);

			bList.push_back(b);
		}
	}

	return bList;
}

std::vector<Marking> MappingHelpers::GetMarkingsList(TiXmlElement* pElem)
{
	vector<Marking> mList;
	TiXmlElement* pMarkings= GetDataFolder("Markings", pElem);
	if(!pMarkings)
		return mList;

	TiXmlElement* pE = pMarkings->FirstChildElement("Placemark");
	for( ; pE; pE=pE->NextSiblingElement())
	{
		string tfID;
		TiXmlElement* pNameXml =pE->FirstChildElement("name");

		if(pNameXml)
		{
			tfID = pNameXml->GetText();
			Marking m;
			m.id = GetIDsFromPrefix(tfID, "MID", "LnID").at(0);
			m.laneId = GetIDsFromPrefix(tfID, "LnID", "RdID").at(0);
			m.roadId = GetIDsFromPrefix(tfID, "RdID", "").at(0);

			TiXmlElement* pPoints = pE->FirstChildElement("LineString")->FirstChildElement("coordinates");

			m.points = GetPointsData(pPoints);

			if(m.points.size() > 0)
			{
				//first item is the center of the marking
				m.center = m.points.at(0);
				m.points.erase(m.points.begin()+0);
			}

			mList.push_back(m);
		}
	}

	return mList;
}

std::vector<Crossing> MappingHelpers::GetCrossingsList(TiXmlElement* pElem)
{
	vector<Crossing> crossList;
	TiXmlElement* pCrossings= GetDataFolder("Crossings", pElem);

	if(!pCrossings)
		return crossList;

	TiXmlElement* pE = pCrossings->FirstChildElement("Placemark");
	for( ; pE; pE=pE->NextSiblingElement())
	{
		string tfID;
		TiXmlElement* pNameXml =pE->FirstChildElement("name");

		if(pNameXml)
		{
			tfID = pNameXml->GetText();
			Crossing cross;
			cross.id = GetIDsFromPrefix(tfID, "CRID", "RdID").at(0);
			cross.roadId = GetIDsFromPrefix(tfID, "RdID", "").at(0);

			TiXmlElement* pPoints = pE->FirstChildElement("LineString")->FirstChildElement("coordinates");
			cross.points = GetPointsData(pPoints);

			crossList.push_back(cross);
		}
	}

	return crossList;
}

std::vector<TrafficSign> MappingHelpers::GetTrafficSignsList(TiXmlElement* pElem)
{
	vector<TrafficSign> tsList;
	TiXmlElement* pSigns = GetDataFolder("TrafficSigns", pElem);

	if(!pSigns)
		return tsList;

	TiXmlElement* pE = pSigns->FirstChildElement("Placemark");
	for( ; pE; pE=pE->NextSiblingElement())
	{
		string tfID;
		TiXmlElement* pNameXml =pE->FirstChildElement("name");

		if(pNameXml)
		{
		  tfID = pNameXml->GetText();

		  	TrafficSign ts;
			ts.id = GetIDsFromPrefix(tfID, "TSID", "LnID").at(0);
			ts.laneId = GetIDsFromPrefix(tfID, "LnID", "RdID").at(0);
			ts.roadId = GetIDsFromPrefix(tfID, "RdID", "Type").at(0);
			int iType = GetIDsFromPrefix(tfID, "Type", "").at(0);
			switch(iType)
			{
			case 0:
				ts.signType = UNKNOWN_SIGN;
				break;
			case 1:
				ts.signType = STOP_SIGN;
				break;
			case 2:
				ts.signType = MAX_SPEED_SIGN;
				break;
			case 3:
				ts.signType = MIN_SPEED_SIGN;
				break;
			default:
				ts.signType = STOP_SIGN;
				break;
			}


			TiXmlElement* pPoints = pE->FirstChildElement("LineString")->FirstChildElement("coordinates");
			ts.pos = GetPointsData(pPoints).at(0);

			tsList.push_back(ts);
		}
	}

	return tsList;
}

std::vector<StopLine> MappingHelpers::GetStopLinesList(TiXmlElement* pElem)
{
	vector<StopLine> slList;
	TiXmlElement* pStopLines = GetDataFolder("StopLines", pElem);

	if(!pStopLines)
		return slList;

	TiXmlElement* pE = pStopLines->FirstChildElement("Placemark");
	for( ; pE; pE=pE->NextSiblingElement())
	{
		string tfID;
		TiXmlElement* pNameXml =pE->FirstChildElement("name");

		if(pNameXml)
		{
		  tfID = pNameXml->GetText();

			StopLine sl;
			sl.id = GetIDsFromPrefix(tfID, "SLID", "LnID").at(0);
			sl.laneId = GetIDsFromPrefix(tfID, "LnID", "TSID").at(0);
			sl.stopSignID = GetIDsFromPrefix(tfID, "TSID", "TLID").at(0);
			sl.trafficLightID = GetIDsFromPrefix(tfID, "TLID", "").at(0);


			TiXmlElement* pPoints = pE->FirstChildElement("LineString")->FirstChildElement("coordinates");
			sl.points = GetPointsData(pPoints);

			slList.push_back(sl);
		}
	}

	return slList;
}

std::vector<TrafficLight> MappingHelpers::GetTrafficLightsList(TiXmlElement* pElem)
{
	vector<TrafficLight> tlList;
	TiXmlElement* pLightsLines = GetDataFolder("TrafficLights", pElem);

	if(!pLightsLines)
		return tlList;

	TiXmlElement* pE = pLightsLines->FirstChildElement("Placemark");
	for( ; pE; pE=pE->NextSiblingElement())
	{
		string tfID;
		TiXmlElement* pNameXml =pE->FirstChildElement("name");

		if(pNameXml)
		{
		  tfID = pNameXml->GetText();

			TrafficLight tl;
			tl.id = GetIDsFromPrefix(tfID, "TLID", "LnID").at(0);
			tl.laneIds = GetIDsFromPrefix(tfID, "LnID", "");

			TiXmlElement* pPoints = pE->FirstChildElement("Point")->FirstChildElement("coordinates");
			tl.pos = GetPointsData(pPoints).at(0);

			tlList.push_back(tl);
		}
	}

	return tlList;
}

vector<Lane> MappingHelpers::GetLanesList(TiXmlElement* pElem)
{
	vector<Lane> llList;
	TiXmlElement* pLaneLinks = GetDataFolder("Lanes", pElem);

	if(!pLaneLinks)
		return llList;

	TiXmlElement* pE = pLaneLinks->FirstChildElement("Folder");
	for( ; pE; pE=pE->NextSiblingElement())
	{
		string tfID;
		TiXmlElement* pNameXml =pE->FirstChildElement("description");
		if(pNameXml)
		{
		  tfID = pNameXml->GetText();

			Lane ll;
			ll.id = GetIDsFromPrefix(tfID, "LID", "RSID").at(0);
			ll.roadId = GetIDsFromPrefix(tfID, "RSID", "NUM").at(0);
			ll.num = GetIDsFromPrefix(tfID, "NUM", "From").at(0);
			ll.fromIds = GetIDsFromPrefix(tfID, "From", "To");
			ll.toIds = GetIDsFromPrefix(tfID, "To", "Vel");
			ll.speed = GetIDsFromPrefix(tfID, "Vel", "").at(0);
	if(m_USING_VER_ZERO == 1)
			ll.points = GetCenterLaneDataVer0(pE, ll.id);
	else
			ll.points = GetCenterLaneData(pE, ll.id);

			llList.push_back(ll);
		}
	}

	return llList;
}

vector<RoadSegment> MappingHelpers::GetRoadSegmentsList(TiXmlElement* pElem)
{
	vector<RoadSegment> rlList;
	TiXmlElement* pRoadLinks = GetDataFolder("RoadSegments", pElem);

	if(!pRoadLinks)
		return rlList;

	TiXmlElement* pE = pRoadLinks->FirstChildElement("Placemark");
	for( ; pE; pE=pE->NextSiblingElement())
	{
		string tfID;
		TiXmlElement* pNameXml =pE->FirstChildElement("description");
		if(pNameXml)
		  tfID = pNameXml->GetText();

		RoadSegment rl;
		rl.id = GetIDsFromPrefix(tfID, "RSID", "").at(0);
		rlList.push_back(rl);
	}

	return rlList;
}

std::vector<GPSPoint> MappingHelpers::GetPointsData(TiXmlElement* pElem)
{
	std::vector<GPSPoint> points;
	if(pElem)
	{
		string coordinate_list;
		if(!pElem->NoChildren())
			coordinate_list = pElem->GetText();

		istringstream str_stream(coordinate_list);
		string token, temp;

		while(getline(str_stream, token, ' '))
		{
			string lat, lon, alt;
			double numLat=0, numLon=0, numAlt=0;

			istringstream ss(token);

			getline(ss, lat, ',');
			getline(ss, lon, ',');
			getline(ss, alt, ',');

			numLat = atof(lat.c_str());
			numLon = atof(lon.c_str());
			numAlt = atof(alt.c_str());

			GPSPoint p;

			p.x = p.lat = numLat;
			p.y = p.lon = numLon;
			p.z = p.alt = numAlt;
			points.push_back(p);
		}
	}

	return points;
}

vector<WayPoint> MappingHelpers::GetCenterLaneData(TiXmlElement* pElem, const int& currLaneID)
{
	vector<WayPoint> gps_points;

	TiXmlElement* pV = pElem->FirstChildElement("Placemark");

	if(pV)
	 pV = pV->FirstChildElement("LineString");

	if(pV)
		pV = pV->FirstChildElement("coordinates");

	if(pV)
	{
		string coordinate_list;
		if(!pV->NoChildren())
			coordinate_list = pV->GetText();

		istringstream str_stream(coordinate_list);
		string token, temp;


		while(getline(str_stream, token, ' '))
		{
			string lat, lon, alt;
			double numLat=0, numLon=0, numAlt=0;

			istringstream ss(token);

			getline(ss, lat, ',');
			getline(ss, lon, ',');
			getline(ss, alt, ',');

			numLat = atof(lat.c_str());
			numLon = atof(lon.c_str());
			numAlt = atof(alt.c_str());

			WayPoint wp;

			wp.pos.x = wp.pos.lat = numLat;
			wp.pos.y = wp.pos.lon = numLon;
			wp.pos.z = wp.pos.alt = numAlt;

			wp.laneId = currLaneID;
			gps_points.push_back(wp);
		}

		TiXmlElement* pInfoEl =pElem->FirstChildElement("Folder")->FirstChildElement("description");
		string additional_info;
		if(pInfoEl)
			additional_info = pInfoEl->GetText();
		additional_info.insert(additional_info.begin(), ',');
		additional_info.erase(additional_info.end()-1);
		vector<string> add_info_list = SplitString(additional_info, ",");
		if(gps_points.size() == add_info_list.size())
		{
			for(unsigned int i=0; i< gps_points.size(); i++)
			{
				gps_points.at(i).id =  GetIDsFromPrefix(add_info_list.at(i), "WPID", "AC").at(0);
				gps_points.at(i).actionCost.push_back(GetActionPairFromPrefix(add_info_list.at(i), "AC", "From"));
				gps_points.at(i).fromIds =  GetIDsFromPrefix(add_info_list.at(i), "From", "To");
				gps_points.at(i).toIds =  GetIDsFromPrefix(add_info_list.at(i), "To", "Lid");

				vector<int> ids = GetIDsFromPrefix(add_info_list.at(i), "Lid", "Rid");
				if(ids.size() > 0)
					gps_points.at(i).LeftPointId =  ids.at(0);

				ids = GetIDsFromPrefix(add_info_list.at(i), "Rid", "Vel");
				if(ids.size() > 0)
					gps_points.at(i).RightPointId =  ids.at(0);

				vector<double> dnums = GetDoubleFromPrefix(add_info_list.at(i), "Vel", "Dir");
				if(dnums.size() > 0)
					gps_points.at(i).v =  dnums.at(0);

				dnums = GetDoubleFromPrefix(add_info_list.at(i), "Dir", "");
				if(dnums.size() > 0)
					gps_points.at(i).pos.a = gps_points.at(i).pos.dir =  dnums.at(0);
			}
		}
	}

	return gps_points;
}

vector<WayPoint> MappingHelpers::GetCenterLaneDataVer0(TiXmlElement* pElem, const int& currLaneID)
{
	vector<WayPoint> gps_points;

	TiXmlElement* pV = pElem->FirstChildElement("Placemark");

	if(pV)
	 pV = pV->FirstChildElement("LineString");

	if(pV)
		pV = pV->FirstChildElement("coordinates");

	if(pV)
	{
		string coordinate_list;
		if(!pV->NoChildren())
			coordinate_list = pV->GetText();

		istringstream str_stream(coordinate_list);
		string token, temp;


		while(getline(str_stream, token, ' '))
		{
			string lat, lon, alt;
			double numLat=0, numLon=0, numAlt=0;

			istringstream ss(token);

			getline(ss, lat, ',');
			getline(ss, lon, ',');
			getline(ss, alt, ',');

			numLat = atof(lat.c_str());
			numLon = atof(lon.c_str());
			numAlt = atof(alt.c_str());

			WayPoint wp;

			wp.pos.x = wp.pos.lat = numLat;
			wp.pos.y = wp.pos.lon = numLon;
			wp.pos.z = wp.pos.alt = numAlt;

			wp.laneId = currLaneID;
			gps_points.push_back(wp);
		}

		TiXmlElement* pInfoEl =pElem->FirstChildElement("Folder")->FirstChildElement("description");
		string additional_info;
		if(pInfoEl)
			additional_info = pInfoEl->GetText();
		additional_info.insert(additional_info.begin(), ',');
		additional_info.erase(additional_info.end()-1);
		vector<string> add_info_list = SplitString(additional_info, ",");
		if(gps_points.size() == add_info_list.size())
		{
			for(unsigned int i=0; i< gps_points.size(); i++)
			{
				gps_points.at(i).id =  GetIDsFromPrefix(add_info_list.at(i), "WPID", "C").at(0);
				gps_points.at(i).fromIds =  GetIDsFromPrefix(add_info_list.at(i), "From", "To");
				gps_points.at(i).toIds =  GetIDsFromPrefix(add_info_list.at(i), "To", "Vel");
				gps_points.at(i).v =  GetDoubleFromPrefix(add_info_list.at(i), "Vel", "Dir").at(0);
				gps_points.at(i).pos.a = gps_points.at(i).pos.dir =  GetDoubleFromPrefix(add_info_list.at(i), "Dir", "").at(0);
//				if(currLaneID == 11115)
//				{
//
//					pair<ACTION_TYPE, double> act_cost;
//						act_cost.first = FORWARD_ACTION;
//						act_cost.second = 100;
//					gps_points.at(i).actionCost.push_back(act_cost);
//				}
			}
		}
	}

	return gps_points;
}

vector<int> MappingHelpers::GetIDsFromPrefix(const string& str, const string& prefix, const string& postfix)
{
	int index1 = str.find(prefix)+prefix.size();
	int index2 = str.find(postfix, index1);
	if(index2 < 0  || postfix.size() ==0)
		index2 = str.size();

	string str_ids = str.substr(index1, index2-index1);

	vector<int> ids;
	vector<string> idstr = SplitString(str_ids, "_");

	for(unsigned  int i=0; i< idstr.size(); i++ )
	{
		if(idstr.at(i).size()>0)
		{
			int num = atoi(idstr.at(i).c_str());
			//if(num>-1)
				ids.push_back(num);
		}
	}

	return ids;
}

vector<double> MappingHelpers::GetDoubleFromPrefix(const string& str, const string& prefix, const string& postfix)
{
	int index1 = str.find(prefix)+prefix.size();
	int index2 = str.find(postfix, index1);
	if(index2 < 0  || postfix.size() ==0)
		index2 = str.size();

	string str_ids = str.substr(index1, index2-index1);

	vector<double> ids;
	vector<string> idstr = SplitString(str_ids, "_");

	for(unsigned  int i=0; i< idstr.size(); i++ )
	{
		if(idstr.at(i).size()>0)
		{
			double num = atof(idstr.at(i).c_str());
			//if(num>-1)
				ids.push_back(num);
		}
	}

	return ids;
}

pair<ACTION_TYPE, double> MappingHelpers::GetActionPairFromPrefix(const string& str, const string& prefix, const string& postfix)
{
	int index1 = str.find(prefix)+prefix.size();
	int index2 = str.find(postfix, index1);
	if(index2<0  || postfix.size() ==0)
		index2 = str.size();

	string str_ids = str.substr(index1, index2-index1);

	pair<ACTION_TYPE, double> act_cost;
	act_cost.first = FORWARD_ACTION;
	act_cost.second = 0;

	vector<string> idstr = SplitString(str_ids, "_");

	if(idstr.size() >= 2)
	{
		if(idstr.at(0).size() > 0 && idstr.at(0).at(0) == 'L')
			act_cost.first = LEFT_TURN_ACTION;
		else if(idstr.at(0).size() > 0 && idstr.at(0).at(0) == 'R')
			act_cost.first = RIGHT_TURN_ACTION;
		if(idstr.at(1).size() > 0)
			act_cost.second = atof(idstr.at(1).c_str());
	}

	return act_cost;
}

vector<string> MappingHelpers::SplitString(const string& str, const string& token)
{
	vector<string> str_parts;
	int iFirstPart = str.find(token);

	while(iFirstPart >= 0)
	{
		iFirstPart++;
		int iSecondPart = str.find(token, iFirstPart);
		if(iSecondPart>0)
		{
			str_parts.push_back(str.substr(iFirstPart,iSecondPart - iFirstPart));
		}
		else
		{
			str_parts.push_back(str.substr(iFirstPart,str.size() - iFirstPart));
		}

		iFirstPart = iSecondPart;
	}

	return str_parts;
}

void MappingHelpers::FindAdjacentLanes(RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		//Link Lanes
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			//Link left and right lanes
			for(unsigned int rs_2 = 0; rs_2 < map.roadSegments.size(); rs_2++)
			{
				for(unsigned int i2 =0; i2 < map.roadSegments.at(rs_2).Lanes.size(); i2++)
				{
					int iCenter1 = pL->points.size()/2;
					WayPoint wp_1 = pL->points.at(iCenter1);
					int iCenter2 = PlanningHelpers::GetClosestNextPointIndexFast(map.roadSegments.at(rs_2).Lanes.at(i2).points, wp_1 );
					WayPoint closest_p = map.roadSegments.at(rs_2).Lanes.at(i2).points.at(iCenter2);
					double mid_a1 = wp_1.pos.a;
					double mid_a2 = closest_p.pos.a;
					double angle_diff = UtilityH::AngleBetweenTwoAnglesPositive(mid_a1, mid_a2);
					double distance = distance2points(wp_1.pos, closest_p.pos);

					if(pL->id != map.roadSegments.at(rs_2).Lanes.at(i2).id && angle_diff < 0.05 && distance < 3.5 && distance > 2.5)
					{
						double perp_distance = DBL_MAX;
						if(pL->points.size() > 2 && map.roadSegments.at(rs_2).Lanes.at(i2).points.size()>2)
						{
							RelativeInfo info;
							PlanningHelpers::GetRelativeInfo(pL->points, closest_p, info);
							perp_distance = info.perp_distance;
							//perp_distance = PlanningHelpers::GetPerpDistanceToVectorSimple(pL->points.at(iCenter1-1), pL->points.at(iCenter1+1), closest_p);
						}

						if(perp_distance > 1.0 && perp_distance < 10.0)
						{
							pL->pRightLane = &map.roadSegments.at(rs_2).Lanes.at(i2);
							for(unsigned int i_internal = 0; i_internal< pL->points.size(); i_internal++)
							{
								if(i_internal<map.roadSegments.at(rs_2).Lanes.at(i2).points.size())
								{
									pL->points.at(i_internal).RightPointId = map.roadSegments.at(rs_2).Lanes.at(i2).id;
									pL->points.at(i_internal).pRight = &map.roadSegments.at(rs_2).Lanes.at(i2).points.at(i_internal);
//									map.roadSegments.at(rs_2).Lanes.at(i2).points.at(i_internal).pLeft = &pL->points.at(i_internal);
								}
							}
						}
						else if(perp_distance < -1.0 && perp_distance > -10.0)
						{
							pL->pLeftLane = &map.roadSegments.at(rs_2).Lanes.at(i2);
							for(unsigned int i_internal = 0; i_internal< pL->points.size(); i_internal++)
							{
								if(i_internal<map.roadSegments.at(rs_2).Lanes.at(i2).points.size())
								{
									pL->points.at(i_internal).LeftPointId = map.roadSegments.at(rs_2).Lanes.at(i2).id;
									pL->points.at(i_internal).pLeft = &map.roadSegments.at(rs_2).Lanes.at(i2).points.at(i_internal);
//									map.roadSegments.at(rs_2).Lanes.at(i2).points.at(i_internal).pRight = &pL->points.at(i_internal);
								}
							}
						}
					}
				}
			}
		}
	}
}

void MappingHelpers::ExtractSignalData(const std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal>& signal_data,
			const std::vector<UtilityHNS::AisanVectorFileReader::AisanVector>& vector_data,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
			const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int is=0; is< signal_data.size(); is++)
	{
		if(signal_data.at(is).Type == 2)
		{
			TrafficLight tl;
			tl.id = signal_data.at(is).ID;
			tl.linkID = signal_data.at(is).LinkID;
			tl.stoppingDistance = 0;

			for(unsigned int iv = 0; iv < vector_data.size(); iv++)
			{
				if(signal_data.at(is).VID == vector_data.at(iv).VID)
				{
					for(unsigned int ip = 0; ip < points_data.size(); ip++)
					{
						if(vector_data.at(iv).PID == points_data.at(ip).PID)
						{
							tl.pos = GPSPoint(points_data.at(ip).Ly + origin.x, points_data.at(ip).Bx + origin.y, points_data.at(ip).H + origin.z, vector_data.at(iv).Hang*DEG2RAD);
							break;
						}
					}
				}
			}
			map.trafficLights.push_back(tl);
			if(tl.id > g_max_traffic_light_id)
				g_max_traffic_light_id = tl.id;
		}
	}
}

void MappingHelpers::ExtractStopLinesData(const std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine>& stop_line_data,
			const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
			const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int ist=0; ist < stop_line_data.size(); ist++)
		{
		StopLine sl;
		sl.linkID = stop_line_data.at(ist).LinkID;
		sl.id = stop_line_data.at(ist).ID;
		if(stop_line_data.at(ist).TLID>0)
			sl.trafficLightID = stop_line_data.at(ist).TLID;
		else
			sl.stopSignID = 100+ist;

		for(unsigned int il=0; il < line_data.size(); il++)
		{
			if(stop_line_data.at(ist).LID == line_data.at(il).LID)
			{
				int s_id = line_data.at(il).BPID;
				int e_id = line_data.at(il).FPID;
				for(unsigned int ip = 0; ip < points_data.size(); ip++)
				{
					if(points_data.at(ip).PID == s_id || points_data.at(ip).PID == e_id)
					{
						sl.points.push_back(GPSPoint(points_data.at(ip).Ly + origin.x, points_data.at(ip).Bx + origin.y, points_data.at(ip).H + origin.z, 0));
					}
				}
			}
		}
		map.stopLines.push_back(sl);
		if(sl.id > g_max_stop_line_id)
			g_max_stop_line_id = sl.id;
	}
}

void MappingHelpers::ExtractCurbData(const std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb>& curb_data,
				const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
				const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
				const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int ic=0; ic < curb_data.size(); ic++)
		{
			Curb c;
			c.id = curb_data.at(ic).ID;

			for(unsigned int il=0; il < line_data.size(); il++)
			{
				if(curb_data.at(ic).LID == line_data.at(il).LID)
				{
					int s_id = line_data.at(il).BPID;
					//int e_id = line_data.at(il).FPID;
					for(unsigned int ip = 0; ip < points_data.size(); ip++)
					{
						if(points_data.at(ip).PID == s_id)
						{
							c.points.push_back(GPSPoint(points_data.at(ip).Ly + origin.x, points_data.at(ip).Bx + origin.y, points_data.at(ip).H + origin.z, 0));
							WayPoint wp;
							wp.pos = c.points.at(0);
							Lane* pLane = GetClosestLaneFromMap(wp, map, 5);
							if(pLane)
							{
								c.laneId = pLane->id;
								c.pLane = pLane;
							}
						}
					}
				}
			}
			map.curbs.push_back(c);
		}
}

void MappingHelpers::ExtractWayArea(const std::vector<UtilityHNS::AisanAreasFileReader::AisanArea>& area_data,
		const std::vector<UtilityHNS::AisanWayareaFileReader::AisanWayarea>& wayarea_data,
			const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
			const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int iw=0; iw < wayarea_data.size(); iw ++)
	{
		Boundary bound;
		bound.id = wayarea_data.at(iw).ID;

		for(unsigned int ia=0; ia < area_data.size(); ia ++)
		{
			if(wayarea_data.at(iw).AID == area_data.at(ia).AID)
			{
				int s_id = area_data.at(ia).SLID;
				int e_id = area_data.at(ia).ELID;

				for(unsigned int il=0; il< line_data.size(); il++)
				{
					if(line_data.at(il).LID >= s_id && line_data.at(il).LID <= e_id)
					{
						for(unsigned int ip=0; ip < points_data.size(); ip++)
						{
							if(points_data.at(ip).PID == line_data.at(il).BPID)
							{
								GPSPoint p(points_data.at(ip).Ly + origin.x, points_data.at(ip).Bx + origin.y, points_data.at(ip).H + origin.z, 0);
								bound.points.push_back(p);
							}
						}
					}
				}
			}
		}

		map.boundaries.push_back(bound);
	}
}

void MappingHelpers::LinkMissingBranchingWayPoints(RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);
				for(unsigned int j = 0 ; j < pWP->toIds.size(); j++)
				{
					pWP->pFronts.push_back(FindWaypoint(pWP->toIds.at(j), map));
				}
			}
		}
	}
}

void MappingHelpers::LinkMissingBranchingWayPointsV2(RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pLane = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int p= 0; p < pLane->points.size(); p++)
			{
				WayPoint* pWP = &pLane->points.at(p);

				if(p+1 == pLane->points.size()) // Last Point in Lane
				{
					for(unsigned int j = 0 ; j < pLane->toLanes.size(); j++)
					{
						//cout << "Link, Next Lane: " << pWP->laneId << ", WP: " << pWP->id << " To WP: " << pWP->toIds.at(j) << endl;
						pWP->pFronts.push_back(&pLane->toLanes.at(j)->points.at(0));
					}
				}
				else
				{
					if(pWP->toIds.size() > 1)
					{
						cout << "Error Error Erro ! Lane: " << pWP->laneId << ", Point: " << pWP->originalMapID << endl;
					}
					else
					{
					//	cout << "Link, Same Lane: " << pWP->laneId << ", WP: " << pWP->id << " To WP: " << map.roadSegments.at(rs).Lanes.at(i).points.at(p+1).id << endl;
						pWP->pFronts.push_back(&pLane->points.at(p+1));
					}
				}
			}
		}
	}
}

void MappingHelpers::LinkTrafficLightsAndStopLines(RoadNetwork& map)
{

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				for(unsigned int isl = 0; isl < map.stopLines.size(); isl++)
				{
					WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);

					if(map.stopLines.at(isl).linkID == pWP->id)
					{
						map.stopLines.at(isl).laneId = pWP->laneId;
						map.stopLines.at(isl).pLane = pWP->pLane;
						map.roadSegments.at(rs).Lanes.at(i).stopLines.push_back(map.stopLines.at(isl));

						pWP->stopLineID = map.stopLines.at(isl).id;

						for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
						{
							if(map.trafficLights.at(itl).id == map.stopLines.at(isl).trafficLightID)
							{
								map.trafficLights.at(itl).laneIds.push_back(pWP->laneId);
								map.trafficLights.at(itl).pLanes.push_back(pWP->pLane);
							}
						}
						break;
					}
				}
			}
		}
	}

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
			{
				for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
				{
					WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);
					if(map.trafficLights.at(itl).linkID == pWP->id)
					{
						map.roadSegments.at(rs).Lanes.at(i).trafficlights.push_back(map.trafficLights.at(itl));
						break;
					}
				}
			}
		}
	}
}

void MappingHelpers::LinkTrafficLightsAndStopLinesConData(const std::vector<UtilityHNS::AisanDataConnFileReader::DataConn>& conn_data,
		const std::vector<std::pair<int,int> >& id_replace_list, RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{

			for(unsigned int ic = 0; ic < conn_data.size(); ic++)
			{
				UtilityHNS::AisanDataConnFileReader::DataConn data_conn = conn_data.at(ic);
				ReplaceMyID(data_conn.LID , id_replace_list);

				if(map.roadSegments.at(rs).Lanes.at(i).id == data_conn.LID)
				{
					for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
					{
						if(map.trafficLights.at(itl).id == data_conn.SID)
						{
							map.trafficLights.at(itl).laneIds.push_back(map.roadSegments.at(rs).Lanes.at(i).id);
							map.trafficLights.at(itl).pLanes.push_back(&map.roadSegments.at(rs).Lanes.at(i));
							map.roadSegments.at(rs).Lanes.at(i).trafficlights.push_back(map.trafficLights.at(itl));
						}
					}

					for(unsigned int isl = 0; isl < map.stopLines.size(); isl++)
					{
						if(map.stopLines.at(isl).id == data_conn.SLID)
						{
							map.stopLines.at(isl).laneId = map.roadSegments.at(rs).Lanes.at(i).id;
							map.stopLines.at(isl).pLane = &map.roadSegments.at(rs).Lanes.at(i);
							map.stopLines.at(isl).trafficLightID = data_conn.SID;
							map.stopLines.at(isl).stopSignID = data_conn.SSID;
							map.roadSegments.at(rs).Lanes.at(i).stopLines.push_back(map.stopLines.at(isl));
							WayPoint wp((map.stopLines.at(isl).points.at(0).x+map.stopLines.at(isl).points.at(1).x)/2.0, (map.stopLines.at(isl).points.at(0).y+map.stopLines.at(isl).points.at(1).y)/2.0, (map.stopLines.at(isl).points.at(0).z+map.stopLines.at(isl).points.at(1).z)/2.0, (map.stopLines.at(isl).points.at(0).a+map.stopLines.at(isl).points.at(1).a)/2.0);
							map.roadSegments.at(rs).Lanes.at(i).points.at(PlanningHelpers::GetClosestNextPointIndexFast(map.roadSegments.at(rs).Lanes.at(i).points, wp)).stopLineID = map.stopLines.at(isl).id;
						}
					}
				}
			}

		}
	}
}

void MappingHelpers::UpdateMapWithOccupancyGrid(OccupancyToGridMap& map_info, const std::vector<int>& data, RoadNetwork& map, std::vector<WayPoint*>& updated_list)
{
	PlannerHNS::Mat3 rotationMat(- map_info.center.pos.a);
	PlannerHNS::Mat3 translationMat(-map_info.center.pos.x, -map_info.center.pos.y);
	updated_list.clear();

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);

				GPSPoint relative_point = pWP->pos;
				relative_point = translationMat * relative_point;
				relative_point = rotationMat *relative_point;

				int cell_value = 0;
				if(map_info.GetCellIndexFromPoint(relative_point, data, cell_value) == true)
				{
					if(cell_value == 0)
					{
						bool bFound = false;
						for(unsigned int i_action=0; i_action < pWP->actionCost.size(); i_action++)
						{
							if(pWP->actionCost.at(i_action).first == FORWARD_ACTION)
							{
								pWP->actionCost.at(i_action).second = 100;
								bFound = true;
							}
						}

						if(!bFound)
							pWP->actionCost.push_back(make_pair(FORWARD_ACTION, 100));

						updated_list.push_back(pWP);
					}
				}
			}
		}
	}
}

void MappingHelpers::ConstructRoadNetworkFromROSMessageV2(const std::vector<UtilityHNS::AisanLanesFileReader::AisanLane>& lanes_data,
		const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
		const std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine>& dt_data,
		const std::vector<UtilityHNS::AisanIntersectionFileReader::AisanIntersection>& intersection_data,
		const std::vector<UtilityHNS::AisanAreasFileReader::AisanArea>& area_data,
		const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
		const std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine>& stop_line_data,
		const std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal>& signal_data,
		const std::vector<UtilityHNS::AisanVectorFileReader::AisanVector>& vector_data,
		const std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb>& curb_data,
		const std::vector<UtilityHNS::AisanRoadEdgeFileReader::AisanRoadEdge>& roadedge_data,
		const std::vector<UtilityHNS::AisanWayareaFileReader::AisanWayarea>& wayarea_data,
		const std::vector<UtilityHNS::AisanCrossWalkFileReader::AisanCrossWalk>& crosswalk_data,
		const std::vector<UtilityHNS::AisanNodesFileReader::AisanNode>& nodes_data,
		const std::vector<UtilityHNS::AisanDataConnFileReader::DataConn>& conn_data,
		UtilityHNS::AisanLanesFileReader* pLaneData,
		UtilityHNS::AisanPointsFileReader* pPointsData,
		UtilityHNS::AisanNodesFileReader* pNodesData,
		UtilityHNS::AisanLinesFileReader* pLinedata,
		const GPSPoint& origin, RoadNetwork& map, const bool& bSpecialFlag,
		const bool& bFindLaneChangeLanes, const bool& bFindCurbsAndWayArea)
{
	vector<Lane> roadLanes;

	for(unsigned int i=0; i< pLaneData->m_data_list.size(); i++)
	{
		if(pLaneData->m_data_list.at(i).LnID > g_max_lane_id)
			g_max_lane_id = pLaneData->m_data_list.at(i).LnID;
	}

	cout << " >> Extracting Lanes ... " << endl;
	CreateLanes(pLaneData, pPointsData, pNodesData, roadLanes);

	cout << " >> Fix Waypoints errors ... " << endl;
	FixTwoPointsLanes(roadLanes);
	FixRedundantPointsLanes(roadLanes);

	ConnectLanes(pLaneData, roadLanes);

	cout << " >> Create Missing lane connections ... " << endl;
	FixUnconnectedLanes(roadLanes);
	////FixTwoPointsLanes(roadLanes);

	//map has one road segment
	RoadSegment roadSegment1;
	roadSegment1.id = 1;
	roadSegment1.Lanes = roadLanes;
	map.roadSegments.push_back(roadSegment1);

	//Fix angle for lanes
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			PlannerHNS::PlanningHelpers::FixAngleOnly(pL->points);
		}
	}

	//Link Lanes and lane's waypoints by pointers
	cout << " >> Link lanes and waypoints with pointers ... " << endl;
	LinkLanesPointers(map);

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
			    if(pL->points.at(j).actionCost.size() > 0)
			  {
				  if(pL->points.at(j).actionCost.at(0).first == LEFT_TURN_ACTION)
					{
					  AssignActionCostToLane(pL, LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST);
					  break;
					}
				  else if(pL->points.at(j).actionCost.at(0).first == RIGHT_TURN_ACTION)
					{
					  AssignActionCostToLane(pL, RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST);
					break;

					}
				  }
			}
		}
	}

	if(bFindLaneChangeLanes)
	{
		cout << " >> Extract Lane Change Information... " << endl;
		FindAdjacentLanesV2(map);
	}

	//Extract Signals and StopLines
	cout << " >> Extract Signal data ... " << endl;
	ExtractSignalDataV2(signal_data, vector_data, pPointsData, origin, map);

	//Stop Lines
	cout << " >> Extract Stop lines data ... " << endl;
	ExtractStopLinesDataV2(stop_line_data, pLinedata, pPointsData, origin, map);

	if(bFindCurbsAndWayArea)
	{
		//Curbs
		cout << " >> Extract curbs data ... " << endl;
		ExtractCurbDataV2(curb_data, pLinedata, pPointsData, origin, map);

		//Wayarea
		cout << " >> Extract wayarea data ... " << endl;
		ExtractWayArea(area_data, wayarea_data, line_data, points_data, origin, map);
	}

	//Link waypoints
	cout << " >> Link missing branches and waypoints... " << endl;
	LinkMissingBranchingWayPointsV2(map);

	//Link StopLines and Traffic Lights
	cout << " >> Link StopLines and Traffic Lights ... " << endl;
	LinkTrafficLightsAndStopLinesV2(map);
//	//LinkTrafficLightsAndStopLinesConData(conn_data, id_replace_list, map);

	cout << " >> Map loaded from data with " << roadLanes.size()  << " lanes" << endl;
}

bool MappingHelpers::GetPointFromDataList(UtilityHNS::AisanPointsFileReader* pPointsData,const int& pid, WayPoint& out_wp)
{
	if(pPointsData == nullptr) return false;

	AisanPointsFileReader::AisanPoints* pP =  pPointsData->GetDataRowById(pid);

	if(pP!=nullptr)
	{
		out_wp.id = pP->PID;
		out_wp.pos.x = pP->Ly;
		out_wp.pos.y = pP->Bx;
		out_wp.pos.z = pP->H;
		return true;
	}

	return false;
}

int MappingHelpers::GetBeginPointIdFromLaneNo(UtilityHNS::AisanLanesFileReader* pLaneData,
		UtilityHNS::AisanPointsFileReader* pPointsData,
		UtilityHNS::AisanNodesFileReader* pNodesData,const int& LnID)
{
	if(pLaneData == nullptr) return false;
	UtilityHNS::AisanLanesFileReader::AisanLane* pL = nullptr;
	UtilityHNS::AisanPointsFileReader::AisanPoints* pP = nullptr;
	UtilityHNS::AisanNodesFileReader::AisanNode* pN = nullptr;

	pL = pLaneData->GetDataRowById(LnID);
	if(pL!=nullptr)
	{
		return pL->FNID;
	}

	return 0;
}

int MappingHelpers::GetEndPointIdFromLaneNo(UtilityHNS::AisanLanesFileReader* pLaneData,
		UtilityHNS::AisanPointsFileReader* pPointsData,
		UtilityHNS::AisanNodesFileReader* pNodesData,const int& LnID)
{
	UtilityHNS::AisanLanesFileReader::AisanLane* pL = nullptr;
	UtilityHNS::AisanPointsFileReader::AisanPoints* pP = nullptr;
	UtilityHNS::AisanNodesFileReader::AisanNode* pN = nullptr;

	pL = pLaneData->GetDataRowById(LnID);
	if(pL!=nullptr)
	{
		return pL->BNID;
	}

	return 0;
}

bool MappingHelpers::IsStartLanePoint(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanLanesFileReader::AisanLane* pL)
{
	if(pL->JCT > 0 || pL->BLID == 0)
		return true;

	if(pL->BLID2 != 0 || pL->BLID3 != 0 || pL->BLID4 != 0)
		return true;

	UtilityHNS::AisanLanesFileReader::AisanLane* pPrevL = pLaneData->GetDataRowById(pL->BLID);
	if(pPrevL == nullptr || pPrevL->FLID2 > 0 || pPrevL->FLID3 > 0 || pPrevL->FLID4 > 0 || pPrevL->JCT > 0)
		return true;

	return false;
}

bool MappingHelpers::IsEndLanePoint(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanLanesFileReader::AisanLane* pL)
{
	if(pL->FLID2 > 0 || pL->FLID3 > 0 || pL->FLID4 > 0)
		return true;

	UtilityHNS::AisanLanesFileReader::AisanLane* pNextL = pLaneData->GetDataRowById(pL->FLID);

	return IsStartLanePoint(pLaneData, pNextL);
}

void MappingHelpers::GetLanesStartPoints(UtilityHNS::AisanLanesFileReader* pLaneData,
				std::vector<int>& m_LanesStartIds)
{
	m_LanesStartIds.clear();
	UtilityHNS::AisanLanesFileReader::AisanLane* pL = nullptr;
	UtilityHNS::AisanLanesFileReader::AisanLane* pPrevL = nullptr;
	for(unsigned int il=0; il < pLaneData->m_data_list.size(); il++)
	{
		pL = &pLaneData->m_data_list.at(il);

		if(IsStartLanePoint(pLaneData, pL) == true)
		{
			m_LanesStartIds.push_back(pL->LnID);
		}

		if(DEBUG_MAP_PARSING)
		{
			if(IsStartLanePoint(pLaneData, pL) && IsEndLanePoint(pLaneData, pL))
				cout << " :( :( :( Start And End in the same time !! " << pL->LnID << endl;
		}
	}
}

void MappingHelpers::ConnectLanes(UtilityHNS::AisanLanesFileReader* pLaneData, std::vector<PlannerHNS::Lane>& lanes)
{
	for(unsigned int il = 0; il < lanes.size(); il++)
	{
		WayPoint fp = lanes.at(il).points.at(0);
		UtilityHNS::AisanLanesFileReader::AisanLane* pFirstL = pLaneData->GetDataRowById(fp.originalMapID);
		if(pFirstL!=nullptr)
		{
			if(pFirstL->BLID > 0)
				lanes.at(il).fromIds.push_back(pFirstL->BLID);
			if(pFirstL->BLID2 > 0)
				lanes.at(il).fromIds.push_back(pFirstL->BLID2);
			if(pFirstL->BLID3 > 0)
				lanes.at(il).fromIds.push_back(pFirstL->BLID3);
			if(pFirstL->BLID4 > 0)
				lanes.at(il).fromIds.push_back(pFirstL->BLID4);
		}

		WayPoint ep = lanes.at(il).points.at(lanes.at(il).points.size()-1);
		UtilityHNS::AisanLanesFileReader::AisanLane* pEndL = pLaneData->GetDataRowById(ep.originalMapID);
		if(pEndL!=nullptr)
		{
			if(pEndL->FLID > 0)
				lanes.at(il).toIds.push_back(pEndL->FLID);
			if(pEndL->FLID2 > 0)
				lanes.at(il).toIds.push_back(pEndL->FLID2);
			if(pEndL->FLID3 > 0)
				lanes.at(il).toIds.push_back(pEndL->FLID3);
			if(pEndL->FLID4 > 0)
				lanes.at(il).toIds.push_back(pEndL->FLID4);
		}
	}
}

void MappingHelpers::CreateLanes(UtilityHNS::AisanLanesFileReader* pLaneData,
				UtilityHNS::AisanPointsFileReader* pPointsData,
				UtilityHNS::AisanNodesFileReader* pNodesData,
				std::vector<PlannerHNS::Lane>& out_lanes)
{

	out_lanes.clear();
	std::vector<int> start_lines;
	GetLanesStartPoints(pLaneData, start_lines);
	for(unsigned int l =0; l < start_lines.size(); l++)
	{
		Lane _lane;
		GetLanePoints(pLaneData, pPointsData, pNodesData, start_lines.at(l), _lane);
		out_lanes.push_back(_lane);
	}
}

void MappingHelpers::GetLanePoints(UtilityHNS::AisanLanesFileReader* pLaneData,
			UtilityHNS::AisanPointsFileReader* pPointsData,
			UtilityHNS::AisanNodesFileReader* pNodesData, int lnID,
			PlannerHNS::Lane& out_lane)
{
	int next_lnid = lnID;
	bool bStart = false;
	bool bLast = false;
	int _rID = 0;
	out_lane.points.clear();
	UtilityHNS::AisanLanesFileReader::AisanLane* pL = nullptr;
	out_lane.id = lnID;
	out_lane.speed = 0;

	while(!bStart)
	{
		pL = pLaneData->GetDataRowById(next_lnid);
		if(pL == nullptr)
		{
			cout << "## Error, Can't find lane from ID: " << next_lnid <<endl;
			break;
		}

		next_lnid = pL->FLID;
		if(next_lnid == 0)
			bStart = true;
		else
			bStart = IsStartLanePoint(pLaneData, pLaneData->GetDataRowById(next_lnid));

//		if(_lnid == 1267 ) //|| _lnid == 1268 || _lnid == 1269 || _lnid == 958)
//			out_lane.id = lnID;

		if(out_lane.points.size() == 0)
		{
			WayPoint wp1, wp2;
			GetPointFromDataList(pPointsData, pNodesData->GetDataRowById(pL->BNID)->PID, wp1);
			wp1.v = pL->RefVel == 0 ? DEFAULT_REF_VELOCITY : pL->RefVel;

			wp1.id = pL->BNID;

			if(pL->BLID != 0)
			{
				_rID = GetBeginPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->BLID);
				if(_rID > 0)
					wp1.fromIds.push_back(_rID);
			}
			if(pL->BLID2 != 0)
			{
				_rID = GetBeginPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->BLID2);
				if(_rID > 0)
					wp1.fromIds.push_back(_rID);
			}
			if(pL->BLID3 != 0)
			{
				_rID = GetBeginPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->BLID3);
				if(_rID > 0)
					wp1.fromIds.push_back(_rID);
			}
			if(pL->BLID4 != 0)
			{
				_rID = GetBeginPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->BLID4);
				if(_rID > 0)
					wp1.fromIds.push_back(_rID);
			}

			GetPointFromDataList(pPointsData, pNodesData->GetDataRowById(pL->FNID)->PID, wp2);
			wp2.v = pL->RefVel == 0 ? DEFAULT_REF_VELOCITY : pL->RefVel;
			wp2.id = pL->FNID;

			if(bStart && pL->FLID != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID);
				if(_rID > 0)
					wp2.toIds.push_back(_rID);
			}
			if(pL->FLID2 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID2);
				if(_rID > 0)
					wp2.toIds.push_back(_rID);
			}
			if(pL->FLID3 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID3);
				if(_rID > 0)
					wp2.toIds.push_back(_rID);
			}
			if(pL->FLID4 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID4);
				if(_rID > 0)
					wp2.toIds.push_back(_rID);
			}

			if(pL->LaneDir == 'L')
			{
				wp1.actionCost.push_back(make_pair(LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST));
				wp2.actionCost.push_back(make_pair(LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST));
			}
			else  if(pL->LaneDir == 'R')
			{
				wp1.actionCost.push_back(make_pair(RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST));
				wp2.actionCost.push_back(make_pair(RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST));
			}
			else
			{
				wp1.actionCost.push_back(make_pair(FORWARD_ACTION, 0));
				wp2.actionCost.push_back(make_pair(FORWARD_ACTION, 0));
			}

			wp1.originalMapID = pL->LnID;
			wp2.originalMapID = pL->LnID;

			wp1.laneId = lnID;
			wp2.laneId = lnID;

			wp1.toIds.push_back(wp2.id);
			wp2.fromIds.push_back(wp1.id);
			out_lane.points.push_back(wp1);
			out_lane.points.push_back(wp2);
			out_lane.speed = pL->RefVel == 0 ? DEFAULT_REF_VELOCITY : pL->RefVel;
		}
		else
		{
			WayPoint wp;
			GetPointFromDataList(pPointsData, pNodesData->GetDataRowById(pL->FNID)->PID, wp);
			wp.v = pL->RefVel == 0 ? DEFAULT_REF_VELOCITY : pL->RefVel;
			wp.id = pL->FNID;

			out_lane.points.at(out_lane.points.size()-1).toIds.push_back(wp.id);
			wp.fromIds.push_back(out_lane.points.at(out_lane.points.size()-1).id);

			if(bStart && pL->FLID != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID);
				if(_rID > 0)
					wp.toIds.push_back(_rID);
			}
			if(pL->FLID2 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID2);
				if(_rID > 0)
					wp.toIds.push_back(_rID);
			}
			if(pL->FLID3 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID3);
				if(_rID > 0)
					wp.toIds.push_back(_rID);
			}
			if(pL->FLID4 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID4);
				if(_rID > 0)
					wp.toIds.push_back(_rID);
			}

			if(pL->LaneDir == 'L')
			{
				wp.actionCost.push_back(make_pair(LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST));
			}
			else  if(pL->LaneDir == 'R')
			{
				wp.actionCost.push_back(make_pair(RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST));
			}
			else
			{
				wp.actionCost.push_back(make_pair(FORWARD_ACTION, 0));
			}

			wp.originalMapID = pL->LnID;
			wp.laneId = lnID;

			out_lane.points.push_back(wp);
		}

//		if(next_lnid == 0)
//			break;
	}
}

void MappingHelpers::FixRedundantPointsLanes(std::vector<Lane>& lanes)
{
	//Fix Redundant point for two points in a row
	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		for(int ip = 1; ip < lanes.at(il).points.size(); ip++)
		{
			WayPoint* p1 = &lanes.at(il).points.at(ip-1);
			WayPoint* p2 = &lanes.at(il).points.at(ip);
			WayPoint* p3 = nullptr;
			if(ip+1 < lanes.at(il).points.size())
				p3 = &lanes.at(il).points.at(ip+1);

			double d = hypot(p2->pos.y-p1->pos.y, p2->pos.x-p1->pos.x);
			if(d == 0)
			{
				p1->toIds = p2->toIds;
				p1->originalMapID = p2->originalMapID;
				if(p3 != nullptr)
					p3->fromIds = p2->fromIds;

				lanes.at(il).points.erase(lanes.at(il).points.begin()+ip);
				ip--;

				if(DEBUG_MAP_PARSING)
					cout << "Fixed Redundant Points for Lane:" << lanes.at(il).id << ", Current: " << ip << ", Size: " << lanes.at(il).points.size() << endl;
			}
		}
	}
}

void MappingHelpers::FixTwoPointsLane(Lane& l)
{
	if(l.points.size() == 2)
	{
		g_max_point_id++;
		WayPoint wp = l.points.at(0);
		wp.id = g_max_point_id;
		wp.fromIds.clear();
		wp.fromIds.push_back(l.points.at(0).id);
		wp.toIds.clear();
		wp.toIds.push_back(l.points.at(1).id);

		l.points.at(0).toIds.clear();
		l.points.at(0).toIds.push_back(wp.id);

		l.points.at(1).fromIds.clear();
		l.points.at(1).fromIds.push_back(wp.id);

		wp.pos.x = (l.points.at(0).pos.x + l.points.at(1).pos.x)/2.0;
		wp.pos.y = (l.points.at(0).pos.y + l.points.at(1).pos.y)/2.0;
		wp.pos.z = (l.points.at(0).pos.z + l.points.at(1).pos.z)/2.0;

		l.points.insert(l.points.begin()+1, wp);
	}
	else if(l.points.size() < 2)
	{
		cout << "## WOW Lane " <<  l.id << " With Size (" << l.points.size() << ") " << endl;
	}
}

void MappingHelpers::FixTwoPointsLanes(std::vector<Lane>& lanes)
{
	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		for(unsigned int ip = 0; ip < lanes.at(il).points.size(); ip++)
		{
			if(lanes.at(il).points.at(ip).id > g_max_point_id)
			{
				g_max_point_id = lanes.at(il).points.at(ip).id;
			}
		}
	}

	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		FixTwoPointsLane(lanes.at(il));
		PlannerHNS::PlanningHelpers::CalcAngleAndCost(lanes.at(il).points);
	}
}

void MappingHelpers::InsertWayPointToBackOfLane(const WayPoint& wp, Lane& lane, int& global_id)
{
	WayPoint* pFirst = &lane.points.at(0);
	pFirst->pos = wp.pos;

//	WayPoint f_wp = *pFirst;
//	f_wp.pos = wp.pos;
//
//	//Give old first new ID
//	global_id++;
//	pFirst->id = global_id;
//
//	//link ids
//	f_wp.toIds.clear(); //only see front
//	f_wp.toIds.push_back(pFirst->id);
//
//	pFirst->fromIds.clear();
//	pFirst->fromIds.push_back(f_wp.id);
//
//	if(lane.points.size() > 1)
//	{
//		lane.points.at(1).fromIds.clear();
//		lane.points.at(1).fromIds.push_back(pFirst->id);
//	}
//
//	lane.points.insert(lane.points.begin(), f_wp);
}

void MappingHelpers::InsertWayPointToFrontOfLane(const WayPoint& wp, Lane& lane, int& global_id)
{
	WayPoint* pLast = &lane.points.at(lane.points.size()-1);
	pLast->pos = wp.pos;

//	WayPoint l_wp = *pLast;
//	l_wp.pos = wp.pos;
//
//	//Give old first new ID
//	global_id++;
//	pLast->id = global_id;
//
//	//link ids
//	l_wp.fromIds.clear(); //only see front
//	l_wp.fromIds.push_back(pLast->id);
//
//	pLast->toIds.clear();
//	pLast->toIds.push_back(l_wp.id);
//
//	if(lane.points.size() > 1)
//	{
//		lane.points.at(lane.points.size()-2).toIds.clear();
//		lane.points.at(lane.points.size()-2).toIds.push_back(pLast->id);
//	}
//
//	lane.points.push_back(l_wp);
}

void MappingHelpers::FixUnconnectedLanes(std::vector<Lane>& lanes)
{
	std::vector<Lane> sp_lanes = lanes;
	bool bAtleastOneChange = false;
	//Find before lanes
	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		if(lanes.at(il).fromIds.size() == 0)
		{
			double closest_d = DBL_MAX;
			Lane* pL = nullptr;
			Lane* pFL = nullptr;
			for(int l=0; l < sp_lanes.size(); l ++)
			{
				if(lanes.at(il).id == sp_lanes.at(l).id)
				{
					pFL = &sp_lanes.at(l);
					break;
				}
			}

			PlannerHNS::RelativeInfo closest_info;
			int closest_index = -1;
			for(int l=0; l < sp_lanes.size(); l ++)
			{
				if(pFL->id != sp_lanes.at(l).id )
				{
					PlannerHNS::RelativeInfo info;
					WayPoint lastofother = sp_lanes.at(l).points.at(sp_lanes.at(l).points.size()-1);
					PlanningHelpers::GetRelativeInfoLimited(sp_lanes.at(l).points, pFL->points.at(0), info, 0);
					double back_distance = hypot(lastofother.pos.y - pFL->points.at(0).pos.y, lastofother.pos.x - pFL->points.at(0).pos.x);
					bool bCloseFromBack = false;
					if((info.bAfter == true && back_distance < 15.0) || info.bAfter == false)
						bCloseFromBack = true;


					if(fabs(info.perp_distance) < 2 && fabs(info.perp_distance) < closest_d && info.bBefore == false && bCloseFromBack)
					{
						closest_d = fabs(info.perp_distance);
						pL = &sp_lanes.at(l);
						closest_info = info;
						closest_info.angle_diff = back_distance;
						closest_index = l;
					}
				}
			}

			if(pL != nullptr && pFL != nullptr)
			{
				if(closest_info.iFront == pL->points.size()-1)
				{
					pL->toIds.push_back(pFL->id);
					pL->points.at(closest_info.iFront).toIds.push_back(pFL->points.at(0).id);

					pFL->points.at(0).fromIds.push_back(pL->points.at(closest_info.iFront).id);
					pFL->fromIds.push_back(pL->id);
					bAtleastOneChange = true;
					if(DEBUG_MAP_PARSING)
					{
						cout << "Closest Next Lane For: " << pFL->id << " , Is:" << pL->id << ", Distance=" << fabs(closest_info.perp_distance) << ", Size: " << pL->points.size() << ", back_index: " << closest_info.iBack <<", front_index: " << closest_info.iFront << ", Direct: " << closest_info.angle_diff << endl;
						cout << "Don't Split , Perfect !" << endl;
					}
				}
				else
				{
					 // split from previous point
					if(DEBUG_MAP_PARSING)
						cout << "Closest Next Lane For: " << pFL->id << " , Is:" << pL->id << ", Distance=" << fabs(closest_info.perp_distance) << ", Size: " << pL->points.size() << ", back_index: " << closest_info.iBack <<", front_index: " << closest_info.iFront << ", Direct: " << closest_info.angle_diff << endl;

					Lane front_half, back_half;
					front_half.points.insert(front_half.points.begin(), pL->points.begin()+closest_info.iFront, pL->points.end());
					front_half.toIds = pL->toIds;
					front_half.fromIds.push_back(pL->id);
					front_half.id = front_half.points.at(0).originalMapID;
					front_half.areaId = pL->areaId;
					front_half.dir = pL->dir;
					front_half.num = pL->num;
					front_half.roadId = pL->roadId;
					front_half.speed = pL->speed;
					front_half.type = pL->type;
					front_half.width = pL->width;

					back_half.points.insert(back_half.points.begin(), pL->points.begin(), pL->points.begin()+closest_info.iFront);
					back_half.toIds.clear();
					back_half.toIds.push_back(front_half.id);
					back_half.toIds.push_back(pFL->id);
					back_half.fromIds = pL->fromIds;
					back_half.id = pL->id;
					back_half.areaId = pL->areaId;
					back_half.dir = pL->dir;
					back_half.num = pL->num;
					back_half.roadId = pL->roadId;
					back_half.speed = pL->speed;
					back_half.type = pL->type;
					back_half.width = pL->width;

					WayPoint* last_from_back =  &back_half.points.at(back_half.points.size()-1);
					WayPoint* first_from_front =  &pFL->points.at(0);

					last_from_back->toIds.push_back(first_from_front->id);
					first_from_front->fromIds.push_back(last_from_back->id);

					if(front_half.points.size() > 1 && back_half.points.size() > 1)
					{
						if(DEBUG_MAP_PARSING)
							cout << "Split this one Nicely! first_half_size: " << front_half.points.size() << ", second_hald_size: " << back_half.points.size() << endl;

						pFL->fromIds.push_back(back_half.id);

						if(closest_index >= 0)
							sp_lanes.erase(sp_lanes.begin()+closest_index);
						else
							cout << "## Alert Alert Alert !!!! " << endl;

						// add perp point to lane points
						InsertWayPointToBackOfLane(closest_info.perp_point, front_half, g_max_point_id);
						InsertWayPointToFrontOfLane(closest_info.perp_point, back_half, g_max_point_id);

						sp_lanes.push_back(front_half);
						sp_lanes.push_back(back_half);
						bAtleastOneChange = true;
					}
					else
					{
						if(DEBUG_MAP_PARSING)
							cout << "=> Can't Split this one :( !" << endl;
					}
				}
			}
			else
			{
				if(DEBUG_MAP_PARSING)
					cout << "=> Can't find Before Lanes For:  " << lanes.at(il).id  << endl;
			}
		}
	}

	lanes = sp_lanes;

	//Find to lanes

	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		if(lanes.at(il).toIds.size() == 0)
		{
			double closest_d = DBL_MAX;
			Lane* pL = nullptr;
			Lane* pBL = nullptr;
			for(int l=0; l < sp_lanes.size(); l ++)
			{
				if(lanes.at(il).id == sp_lanes.at(l).id)
				{
					pBL = &sp_lanes.at(l);
					break;
				}
			}

			PlannerHNS::RelativeInfo closest_info;
			int closest_index = -1;
			for(int l=0; l < sp_lanes.size(); l ++)
			{
				if(pBL->id != sp_lanes.at(l).id )
				{
					PlannerHNS::RelativeInfo info;
					WayPoint firstofother = sp_lanes.at(l).points.at(0);
					WayPoint last_point = pBL->points.at(pBL->points.size()-1);
					PlanningHelpers::GetRelativeInfoLimited(sp_lanes.at(l).points, last_point, info, 0);
					double front_distance = hypot(firstofother.pos.y - last_point.pos.y, firstofother.pos.x - last_point.pos.x);
					bool bCloseFromFront = false;
					if((info.bBefore == true && front_distance < 15.0) || info.bBefore == false)
						bCloseFromFront = true;

					if(fabs(info.perp_distance) < 2 && fabs(info.perp_distance) < closest_d && info.bAfter == false && bCloseFromFront)
					{
						closest_d = fabs(info.perp_distance);
						pL = &sp_lanes.at(l);
						closest_info = info;
						closest_info.angle_diff = front_distance;
						closest_index = l;
					}
				}
			}

			if(pL != nullptr && pBL != nullptr)
			{
				if(closest_info.iFront == 0)
				{
					pL->fromIds.push_back(pBL->id);
					pL->points.at(closest_info.iFront).fromIds.push_back(pBL->points.at(pBL->points.size()-1).id);

					pBL->points.at(pBL->points.size()-1).toIds.push_back(pL->points.at(closest_info.iFront).id);
					pBL->toIds.push_back(pL->id);

					bAtleastOneChange = true;
					if(DEBUG_MAP_PARSING)
					{
						cout << "Closest Back Lane For: " << pBL->id << " , Is:" << pL->id << ", Distance=" << fabs(closest_info.perp_distance) << ", Size: " << pL->points.size() << ", back_index: " << closest_info.iBack <<", front_index: " << closest_info.iFront << ", Direct: " << closest_info.angle_diff << endl;
						cout << "Don't Split , Perfect !" << endl;
					}
				}
				else
				{
					 // split from previous point
					if(DEBUG_MAP_PARSING)
						cout << "Closest Back Lane For: " << pBL->id << " , Is:" << pL->id << ", Distance=" << fabs(closest_info.perp_distance) << ", Size: " << pL->points.size() << ", back_index: " << closest_info.iBack <<", front_index: " << closest_info.iFront << ", Direct: " << closest_info.angle_diff << endl;

					Lane front_half, back_half;
					front_half.points.insert(front_half.points.begin(), pL->points.begin()+closest_info.iFront, pL->points.end());
					front_half.toIds = pL->toIds;
					front_half.fromIds.push_back(pL->id);
					front_half.fromIds.push_back(pBL->id);
					front_half.id = front_half.points.at(0).originalMapID;
					front_half.areaId = pL->areaId;
					front_half.dir = pL->dir;
					front_half.num = pL->num;
					front_half.roadId = pL->roadId;
					front_half.speed = pL->speed;
					front_half.type = pL->type;
					front_half.width = pL->width;

					back_half.points.insert(back_half.points.begin(), pL->points.begin(), pL->points.begin()+closest_info.iFront);
					back_half.toIds.push_back(front_half.id);
					back_half.fromIds = pL->fromIds;
					back_half.id = pL->id;
					back_half.areaId = pL->areaId;
					back_half.dir = pL->dir;
					back_half.num = pL->num;
					back_half.roadId = pL->roadId;
					back_half.speed = pL->speed;
					back_half.type = pL->type;
					back_half.width = pL->width;

					WayPoint* first_from_next =  &front_half.points.at(0);
					WayPoint* last_from_front =  &pBL->points.at(pBL->points.size()-1);

					first_from_next->fromIds.push_back(last_from_front->id);
					last_from_front->toIds.push_back(first_from_next->id);

					if(front_half.points.size() > 1 && back_half.points.size() > 1)
					{
						if(DEBUG_MAP_PARSING)
							cout << "Split this one Nicely! first_half_size: " << front_half.points.size() << ", second_hald_size: " << back_half.points.size() << endl;

						pBL->toIds.push_back(front_half.id);

						if(closest_index >= 0)
							sp_lanes.erase(sp_lanes.begin()+closest_index);
						else
							cout << "## Alert Alert Alert !!!! " << endl;

						// add perp point to lane points
						InsertWayPointToBackOfLane(closest_info.perp_point, front_half, g_max_point_id);
						InsertWayPointToFrontOfLane(closest_info.perp_point, back_half, g_max_point_id);

						sp_lanes.push_back(front_half);
						sp_lanes.push_back(back_half);
						bAtleastOneChange = true;
					}
					else
					{
						if(DEBUG_MAP_PARSING)
							cout << "=> Can't Split this one :( !" << endl;
					}
				}
			}
			else
			{
				if(DEBUG_MAP_PARSING)
					cout << "=> Can't find After Lanes For:  " << lanes.at(il).id  << endl;
			}
		}
	}

	lanes = sp_lanes;
}

void MappingHelpers::LinkLanesPointers(PlannerHNS::RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		//Link Lanes
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int j = 0 ; j < pL->fromIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->fromIds.at(j))
					{
						pL->fromLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->toIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->toIds.at(j))
					{
						pL->toLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
				pL->points.at(j).pLane  = pL;
			}
		}
	}
}

void MappingHelpers::ExtractCurbDataV2(const std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb>& curb_data,
				UtilityHNS::AisanLinesFileReader* pLinedata,
				UtilityHNS::AisanPointsFileReader* pPointsData,
				const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int ic=0; ic < curb_data.size(); ic++)
	{
		Curb c;
		c.id = curb_data.at(ic).ID;

		for(unsigned int il=0; il < pLinedata->m_data_list.size() ; il++)
		{
			if(curb_data.at(ic).LID == pLinedata->m_data_list.at(il).LID)
			{
				int s_id = pLinedata->m_data_list.at(il).BPID;
				if(s_id == 0)
					s_id = pLinedata->m_data_list.at(il).FPID;

				AisanPointsFileReader::AisanPoints* pP = pPointsData->GetDataRowById(s_id);
				if(pP != nullptr)
				{
					c.points.push_back(GPSPoint(pP->Ly + origin.x, pP->Bx + origin.y, pP->H + origin.z, 0));
					WayPoint wp;
					wp.pos = c.points.at(0);
					Lane* pLane = GetClosestLaneFromMap(wp, map, 5);
					if(pLane != nullptr)
					{
						c.laneId = pLane->id;
						c.pLane = pLane;
					}
				}
			}
		}
		map.curbs.push_back(c);
	}
}

void MappingHelpers::ExtractSignalDataV2(const std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal>& signal_data,
			const std::vector<UtilityHNS::AisanVectorFileReader::AisanVector>& vector_data,
			UtilityHNS::AisanPointsFileReader* pPointData,
			const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int is=0; is< signal_data.size(); is++)
	{
		//if(signal_data.at(is).Type == 2)
		{
			TrafficLight tl;
			tl.id = signal_data.at(is).ID;
			tl.linkID = signal_data.at(is).LinkID;
			tl.stoppingDistance = 0;

			for(unsigned int iv = 0; iv < vector_data.size(); iv++)
			{
				if(signal_data.at(is).VID == vector_data.at(iv).VID)
				{
					AisanPointsFileReader::AisanPoints* pP =  pPointData->GetDataRowById(vector_data.at(iv).PID);
					if(pP != nullptr)
					{
						tl.pos = GPSPoint(pP->Ly + origin.x, pP->Bx + origin.y, pP->H + origin.z, vector_data.at(iv).Hang*DEG2RAD);
					}
				}
			}
			map.trafficLights.push_back(tl);
			if(tl.id > g_max_traffic_light_id)
				g_max_traffic_light_id = tl.id;
		}
	}
}

void MappingHelpers::ExtractStopLinesDataV2(const std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine>& stop_line_data,
			UtilityHNS::AisanLinesFileReader* pLineData,
			UtilityHNS::AisanPointsFileReader* pPointData,
			const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int ist=0; ist < stop_line_data.size(); ist++)
	{
		StopLine sl;
		sl.linkID = stop_line_data.at(ist).LinkID;
		sl.id = stop_line_data.at(ist).ID;
		if(stop_line_data.at(ist).TLID>0)
			sl.trafficLightID = stop_line_data.at(ist).TLID;
		else
			sl.stopSignID = 100+ist;

		AisanLinesFileReader::AisanLine* pLine = pLineData->GetDataRowById(stop_line_data.at(ist).LID);
		if(pLine != nullptr)
		{
			UtilityHNS::AisanPointsFileReader::AisanPoints* pStart = pPointData->GetDataRowById(pLine->BPID);
			UtilityHNS::AisanPointsFileReader::AisanPoints* pEnd = pPointData->GetDataRowById(pLine->FPID);
			if(pStart != nullptr)
				sl.points.push_back(GPSPoint(pStart->Ly + origin.x, pStart->Bx + origin.y, pStart->H + origin.z, 0));

			if(pEnd != nullptr)
				sl.points.push_back(GPSPoint(pEnd->Ly + origin.x, pEnd->Bx + origin.y, pEnd->H + origin.z, 0));
		}

		map.stopLines.push_back(sl);
		if(sl.id > g_max_stop_line_id)
			g_max_stop_line_id = sl.id;
	}
}

void MappingHelpers::LinkTrafficLightsAndStopLinesV2(RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);

				for(unsigned int isl = 0; isl < map.stopLines.size(); isl++)
				{
					if(map.stopLines.at(isl).linkID == pWP->originalMapID)
					{
						map.stopLines.at(isl).laneId = pWP->laneId;
						map.stopLines.at(isl).pLane = pWP->pLane;
						map.roadSegments.at(rs).Lanes.at(i).stopLines.push_back(map.stopLines.at(isl));

						pWP->stopLineID = map.stopLines.at(isl).id;

						for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
						{
							if(map.trafficLights.at(itl).id == map.stopLines.at(isl).trafficLightID)
							{
								map.trafficLights.at(itl).laneIds.push_back(pWP->laneId);
								map.trafficLights.at(itl).pLanes.push_back(pWP->pLane);
							}
						}
						break;
					}
				}
			}
		}
	}

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
			{
				for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
				{
					WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);
					if(map.trafficLights.at(itl).linkID == pWP->originalMapID)
					{
						map.roadSegments.at(rs).Lanes.at(i).trafficlights.push_back(map.trafficLights.at(itl));
						break;
					}
				}
			}
		}
	}
}

void MappingHelpers::FindAdjacentLanesV2(RoadNetwork& map)
{
	bool bTestDebug = false;
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int i2 =0; i2 < map.roadSegments.at(rs).Lanes.size(); i2++)
			{
				Lane* pL2 = &map.roadSegments.at(rs).Lanes.at(i2);

				if(pL->id == pL2->id) continue;


				if(pL->id == 1683)
					bTestDebug = true;

				for(unsigned int p=0; p < pL->points.size(); p++)
				{
					WayPoint* pWP = &pL->points.at(p);
					RelativeInfo info;
					PlanningHelpers::GetRelativeInfoLimited(pL2->points, *pWP, info);

					if(!info.bAfter && !info.bBefore && fabs(info.perp_distance) > 1.2 && fabs(info.perp_distance) < 3.5 && UtilityH::AngleBetweenTwoAnglesPositive(info.perp_point.pos.a, pWP->pos.a) < 0.06)
					{
						WayPoint* pWP2 = &pL2->points.at(info.iFront);
						if(info.perp_distance < 0)
						{
							if(pWP->pRight == 0)
							{
								pWP->pRight = pWP2;
								pWP->RightPointId = pWP2->id;
								pWP->RightLnId = pL2->id;
								pL->pRightLane = pL2;

							}

							if(pWP2->pLeft == 0)
							{
								pWP2->pLeft = pWP;
								pWP2->LeftPointId = pWP->id;
								pWP2->LeftLnId = pL->id;
								pL2->pLeftLane = pL;
							}
						}
						else
						{
							if(pWP->pLeft == 0)
							{
								pWP->pLeft = pWP2;
								pWP->LeftPointId = pWP2->id;
								pWP->LeftLnId = pL2->id;
								pL->pLeftLane = pL2;
							}

							if(pWP2->pRight == 0)
							{
								pWP2->pRight = pWP->pLeft;
								pWP2->RightPointId = pWP->id;
								pWP2->RightLnId = pL->id;
								pL2->pRightLane = pL;
							}
						}
					}
				}
			}
		}
	}
}

void MappingHelpers::GetMapMaxIds(PlannerHNS::RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			if(pL->id > g_max_lane_id)
				g_max_lane_id = pL->id;

			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
				if(pL->points.at(j).id > g_max_point_id)
				{
					g_max_point_id = pL->points.at(j).id;
				}
			}
		}
	}

	for(unsigned int i=0; i < map.stopLines.size(); i++)
	{
		if(map.stopLines.at(i).id > g_max_stop_line_id)
			g_max_stop_line_id = map.stopLines.at(i).id;
	}

	for(unsigned int i=0; i < map.trafficLights.size(); i++)
	{
		if(map.trafficLights.at(i).id > g_max_traffic_light_id)
			g_max_traffic_light_id = map.trafficLights.at(i).id;
	}
}


} /* namespace PlannerHNS */
