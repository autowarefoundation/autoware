/*
 * MappingHelpers.cpp
 *
 *  Created on: Jul 2, 2016
 *      Author: Hatem
 */


#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"
#include "op_planner/PlanningHelpers.h"

#include "math.h"
#include <fstream>

using namespace UtilityHNS;
using namespace std;
#define _FIND_LEFT_RIGHT_LANES
#define RIGHT_INITIAL_TURNS_COST 0
#define LEFT_INITIAL_TURNS_COST 0


namespace PlannerHNS {

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

	return 0;
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

void MappingHelpers::ConstructRoadNetworkFromRosMessage(const std::vector<UtilityHNS::AisanLanesFileReader::AisanLane>& lanes_data,
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
		const std::vector<UtilityHNS::AisanDataConnFileReader::DataConn>& conn_data,
		const GPSPoint& origin, RoadNetwork& map, const bool& bSpecialFlag)
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
				curr_lane_point = next_lane_point;
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

#ifdef FIND_LEFT_RIGHT_LANES
	FindAdjacentLanes(map);
#endif

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

	//Curbs
	ExtractCurbData(curb_data, line_data, points_data, origin, map);


	cout << "Map loaded from data with " << roadLanes.size()  << " lanes" << endl;
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

	return 0;
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

	string conn_info = vectoMapPath + "dataconnection.csv";


	string intersection_info = vectoMapPath + "intersection.csv";

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

	//AisanAreasFileReader areas(node_info);
	//AisanIntersectionFileReader intersections(node_info);



	vector<AisanNodesFileReader::AisanNode> nodes_data;
	nodes.ReadAllData(nodes_data);

	vector<AisanLanesFileReader::AisanLane> lanes_data;
	lanes.ReadAllData(lanes_data);

	vector<AisanPointsFileReader::AisanPoints> points_data;
	points.ReadAllData(points_data);

	vector<AisanCenterLinesFileReader::AisanCenterLine> dt_data;
	center_lanes.ReadAllData(dt_data);

	vector<AisanAreasFileReader::AisanArea> area_data;
	vector<AisanIntersectionFileReader::AisanIntersection> intersection_data;
	vector<AisanLinesFileReader::AisanLine> line_data;
	lines.ReadAllData(line_data);
	vector<AisanStopLineFileReader::AisanStopLine> stop_line_data;
	stop_line.ReadAllData(stop_line_data);
	vector<AisanSignalFileReader::AisanSignal> signal_data;
	signal.ReadAllData(signal_data);
	vector<AisanVectorFileReader::AisanVector> vector_data;
	vec.ReadAllData(vector_data);

	vector<AisanCurbFileReader::AisanCurb> curb_data;
	curb.ReadAllData(curb_data);

	vector<AisanRoadEdgeFileReader::AisanRoadEdge> roadedge_data;
	roadedge.ReadAllData(roadedge_data);

	vector<AisanDataConnFileReader::DataConn> conn_data;
	conn.ReadAllData(conn_data);

	//Traffic Light Type from the file
	// 4 , 5 -> pedestrian crossing light
	// 1 Red , 2 Green, 3 Yellow -> traffic light that is important for cars (normal traffic lights )


	int bToyotaCityMap = 0;
	if((vectoMapPath.find("toyota") >= 0 || vectoMapPath.find("Toyota") >= 0) && !bZeroOrigin)
		bToyotaCityMap = 1;
	else if((vectoMapPath.find("moriyama") >= 0 || vectoMapPath.find("Moriyama") >= 0) && ! bZeroOrigin)
		bToyotaCityMap = 2;

	// use this to transform data to origin (0,0,0)
	ConstructRoadNetworkFromRosMessage(lanes_data, points_data, dt_data, intersection_data, area_data,
			line_data, stop_line_data, signal_data, vector_data, curb_data, roadedge_data, conn_data,
			GetTransformationOrigin(bToyotaCityMap), map, bToyotaCityMap == 1);

	//use this when using the same coordinates as the map
//	ConstructRoadNetworkFromRosMessage(lanes_data, points_data, dt_data, intersection_data, area_data, GPSPoint(), map);



	WayPoint origin = GetFirstWaypoint(map);
//	WayPoint origin2 = GetFirstWaypoint(map);
//	WayPoint lastPoint2 = lastPoint;
//
//	llaToxyz(origin.pos, GPSPoint());
//	llaToxyz(lastPoint.pos, GPSPoint());
//
//	double distance = distance2points(origin.pos, lastPoint.pos);
//
	cout << origin.pos.ToString() ;
}

void MappingHelpers::GetWayPoint(const int& pid, const vector<AisanPointsFileReader::AisanPoints>& points, std::vector<WayPoint>& path)
{

	for(unsigned int p =0; p < points.size(); p++)
	{
		if(pid == points.at(p).PID)
		{
			WayPoint wp;
			wp.laneId = points.at(p).PID;
			wp.pos = GPSPoint(points.at(p).L, points.at(p).B, points.at(p).H, 0);
			path.push_back(wp);
			break;
		}
	}
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

	return 0;
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


	pElem = doc.FirstChildElement();
	pHeadElem = GetHeadElement(pElem);

	vector<Lane> laneLinksList = GetLanesList(pHeadElem);
	vector<RoadSegment> roadLinksList = GetRoadSegmentsList(pHeadElem);
	vector<StopLine> stopLines = GetStopLinesList(pHeadElem);
	vector<TrafficLight> trafficLights = GetTrafficLightsList(pHeadElem);


	//Fill the relations
	for(unsigned int i= 0; i<roadLinksList.size(); i++ )
	{
		for(unsigned int j=0; j < laneLinksList.size(); j++)
		{
			//if(laneLinksList.at(j).roadId == roadLinksList.at(i).id)
			{
				PlanningHelpers::CalcAngleAndCost(laneLinksList.at(j).points);
				roadLinksList.at(i).Lanes.push_back(laneLinksList.at(j));
			}
		}
	}

	map.roadSegments.clear();
	map.roadSegments = roadLinksList;

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
					if(pWP->LeftLaneId > 0)
					{
						pWP->pLeft = FindWaypoint(pWP->LeftLaneId, map);
						if(pWP->pLeft)
						{
							pWP->LeftLaneId = pWP->pLeft->laneId;
							pWP->pLane->pLeftLane = pWP->pLeft->pLane;
						}
					}

					if(pWP->RightLaneId > 0)
					{
						pWP->pRight = FindWaypoint(pWP->RightLaneId, map);
						if(pWP->pRight)
						{
							pWP->RightLaneId = pWP->pRight->laneId;
							pWP->pLane->pRightLane = pWP->pRight->pLane;
						}
					}
				}
			}
		}
	}

	map.stopLines = stopLines;
	map.trafficLights = trafficLights;

	//Link waypoints && StopLines
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
		return 0;
	return pElem;
}

TiXmlElement* MappingHelpers::GetDataFolder(const string& folderName, TiXmlElement* pMainElem)
{
	if(!pMainElem) return 0;

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
	return 0;
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

	if(!pLane) return 0;

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

	if(!pLane) return 0;

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

Lane* MappingHelpers::GetClosestLaneFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance, const bool bDirectionBased)
{
	vector<pair<double, Lane*> > laneLinksList;
	double d = 0;
	double min_d = 9999999999;
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			//Lane* pLane = &pEdge->lanes.at(k);
			 d = 0;
			min_d = 9999999999;
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

	if(laneLinksList.size() == 0) return 0;

	min_d = 999999999;
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
	double min_d = 9999999999;
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			//Lane* pLane = &pEdge->lanes.at(k);
			 d = 0;
			min_d = 9999999999;
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

		if(bDirectionBased && fabs(info.perp_distance) < distance && fabs(info.angle_diff) < 45)
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
	double min_d = 9999999999;
	int min_i = 0;
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			//Lane* pLane = &pEdge->lanes.at(k);
			d = 0;
			min_d = 9999999999;
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

	if(laneLinksList.size() == 0) return 0;

	min_d = 999999999;
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

	return 0;
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
	if(currPath.size() < 1) return 0;

	int closest_index = PlanningHelpers::GetClosestNextPointIndexFast(currPath, currPos);

	return currPath.at(closest_index).pLane;
}


std::vector<StopLine> MappingHelpers::GetStopLinesList(TiXmlElement* pElem)
{
	vector<StopLine> slList;
	TiXmlElement* pStopLines = GetDataFolder("StopLines", pElem);

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
			sl.stopSignID = GetIDsFromPrefix(tfID, "TSID", "TLTID").at(0);
			sl.trafficLightID = GetIDsFromPrefix(tfID, "TLTID", "").at(0);


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

	TiXmlElement* pE = pLightsLines->FirstChildElement("Placemark");
	for( ; pE; pE=pE->NextSiblingElement())
	{
		string tfID;
		TiXmlElement* pNameXml =pE->FirstChildElement("name");

		if(pNameXml)
		{
		  tfID = pNameXml->GetText();

			TrafficLight tl;
			tl.id = GetIDsFromPrefix(tfID, "SLID", "LnID").at(0);
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
					gps_points.at(i).LeftLaneId =  ids.at(0);

				ids = GetIDsFromPrefix(add_info_list.at(i), "Rid", "Vel");
				if(ids.size() > 0)
					gps_points.at(i).RightLaneId =  ids.at(0);

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
						double perp_distance = 99999;
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
									pL->points.at(i_internal).RightLaneId = map.roadSegments.at(rs_2).Lanes.at(i2).id;
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
									pL->points.at(i_internal).LeftLaneId = map.roadSegments.at(rs_2).Lanes.at(i2).id;
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
					int e_id = line_data.at(il).FPID;
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

void MappingHelpers::LinkTrafficLightsAndStopLines(RoadNetwork& map)
{
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
						map.trafficLights.at(itl).laneIds.push_back(pWP->laneId);
						map.trafficLights.at(itl).pLanes.push_back(pWP->pLane);
						map.roadSegments.at(rs).Lanes.at(i).trafficlights.push_back(map.trafficLights.at(itl));
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
				for(unsigned int isl = 0; isl < map.stopLines.size(); isl++)
				{
					for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
					{
						WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);
						if(map.stopLines.at(isl).linkID == pWP->id)
						{
							map.stopLines.at(isl).laneId = pWP->laneId;
							map.stopLines.at(isl).pLane = pWP->pLane;
							if(pWP->pLane->trafficlights.size() > 0)
								map.stopLines.at(isl).trafficLightID = pWP->pLane->trafficlights.at(0).id;
							map.stopLines.at(isl).stopSignID = 100+isl;
							map.roadSegments.at(rs).Lanes.at(i).stopLines.push_back(map.stopLines.at(isl));
							WayPoint wp((map.stopLines.at(isl).points.at(0).x+map.stopLines.at(isl).points.at(1).x)/2.0, (map.stopLines.at(isl).points.at(0).y+map.stopLines.at(isl).points.at(1).y)/2.0, (map.stopLines.at(isl).points.at(0).z+map.stopLines.at(isl).points.at(1).z)/2.0, (map.stopLines.at(isl).points.at(0).a+map.stopLines.at(isl).points.at(1).a)/2.0);
							map.roadSegments.at(rs).Lanes.at(i).points.at(PlanningHelpers::GetClosestNextPointIndexFast(map.roadSegments.at(rs).Lanes.at(i).points, wp)).stopLineID = map.stopLines.at(isl).id;
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
} /* namespace PlannerHNS */
