/*
 * MappingHelpers.cpp
 *
 *  Created on: Jul 2, 2016
 *      Author: Hatem
 */

#include "MappingHelpers.h"
#include "MatrixOperations.h"
#include "PlanningHelpers.h"
#include "proj_api.h"
#include "geo_pos_conv.hh"
#include "math.h"


using namespace UtilityHNS;
using namespace std;
#define FIND_LEFT_RIGHT_LANES


namespace PlannerHNS {



MappingHelpers::MappingHelpers() {
	// TODO Auto-generated constructor stub

}

MappingHelpers::~MappingHelpers() {
	// TODO Auto-generated destructor stub
}

GPSPoint MappingHelpers::GetTransformationOrigin()
{
	//return GPSPoint(-3700, 99427, -88,0);
	return GPSPoint(18221.1, 93546.1, -36.19, 0);
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

void MappingHelpers::ConstructRoadNetworkFromRosMessage(const std::vector<UtilityHNS::AisanLanesFileReader::AisanLane>& lanes_data,
		const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
		const std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine>& dt_data,
		const GPSPoint& origin, RoadNetwork& map)
{
	vector<Lane> roadLanes;
	Lane lane_obj;
	int prev_FLID = -1;
	int laneIDSeq = 0;
	//WayPoint prevWayPoint;
	for(unsigned int l= 0; l < lanes_data.size(); l++)
	{
		if(lanes_data.at(l).LnID != prev_FLID)
		{
			if(laneIDSeq != 0) //first lane
			{
				lane_obj.toIds.push_back(prev_FLID);
				roadLanes.push_back(lane_obj);
//				if(lane_obj.points.size() <= 1)
//					prev_FLID = 0;
			}

			laneIDSeq++;
			lane_obj = Lane();
			lane_obj.speed = lanes_data.at(l).LimitVel;
			lane_obj.id = lanes_data.at(l).LnID;
			lane_obj.fromIds.push_back(lanes_data.at(l).BLID);
			lane_obj.roadId = laneIDSeq;
		}

//		if(lanes_data.at(l).LnID == 41)
//			prev_FLID = 0;

		WayPoint wp;
		bool bFound = GetWayPoint(lanes_data.at(l).LnID, lane_obj.id, lanes_data.at(l).RefVel,lanes_data.at(l).DID,
				dt_data, points_data,origin, wp);

		if(lanes_data.at(l).LaneDir == 'L')
		{
			wp.actionCost.push_back(make_pair(LEFT_TURN_ACTION, -5));
			//std::cout << " Left Lane : " << lanes_data.at(l).LnID << std::endl ;
		}
		else  if(lanes_data.at(l).LaneDir == 'R')
		{
			wp.actionCost.push_back(make_pair(RIGHT_TURN_ACTION, -5));
			//std::cout << " Right Lane : " << lanes_data.at(l).LnID << std::endl ;
		}
		else
		{
			wp.actionCost.push_back(make_pair(FORWARD_ACTION, 0));
		}

		wp.fromIds.push_back(lanes_data.at(l).BLID);
		wp.toIds.push_back(lanes_data.at(l).FLID);

		//if(lanes_data.at(l).JCT > 0)
		if(lanes_data.at(l).FLID2 > 0)
		{
			lane_obj.toIds.push_back(lanes_data.at(l).FLID2);
			wp.toIds.push_back(lanes_data.at(l).FLID2);
		}
		if(lanes_data.at(l).FLID3 > 0)
		{
			lane_obj.toIds.push_back(lanes_data.at(l).FLID3);
			wp.toIds.push_back(lanes_data.at(l).FLID3);
		}
		if(lanes_data.at(l).FLID4 > 0)
		{
			lane_obj.toIds.push_back(lanes_data.at(l).FLID4);
			wp.toIds.push_back(lanes_data.at(l).FLID4);
		}

		if(lanes_data.at(l).BLID2 > 0)
		{
			lane_obj.fromIds.push_back(lanes_data.at(l).BLID2);
			wp.fromIds.push_back(lanes_data.at(l).BLID2);
		}
		if(lanes_data.at(l).BLID3 > 0)
		{
			lane_obj.fromIds.push_back(lanes_data.at(l).BLID3);
			wp.fromIds.push_back(lanes_data.at(l).BLID3);
		}
		if(lanes_data.at(l).BLID4 > 0)
		{
			lane_obj.fromIds.push_back(lanes_data.at(l).BLID4);
			wp.fromIds.push_back(lanes_data.at(l).BLID4);
		}



		if(bFound)
			lane_obj.points.push_back(wp);

		prev_FLID = lanes_data.at(l).FLID;

		//std::cout << "Map WP Dir : " <<  wp.pos.a*RAD2DEG << ", Actual WP Dir : " << UtilityH::FixNegativeAngle(angle2points(prevWayPoint.pos, wp.pos))*RAD2DEG << std::endl;
		//prevWayPoint = wp;


//			cout << " ID = " << lanes_data.at(l).LnID << ", BLID: " << lanes_data.at(l).BLID << ", FLID:" << lanes_data.at(l).FLID<<
//					", JCT:" << lanes_data.at(l).JCT << ", LCnt:" << lanes_data.at(l).LCnt << ", LNo:" << lanes_data.at(l).Lno <<
//					", SecID:" << lanes_data.at(l).RoadSecID <<
//					", BLID2: " << lanes_data.at(l).BLID2 << ", FLID2: " << lanes_data.at(l).FLID2 <<
//					", BLID3: " << lanes_data.at(l).BLID3 << ", FLID3: " << lanes_data.at(l).FLID3 <<
//					", BLID4: " << lanes_data.at(l).BLID4 << ", FLID4: " << lanes_data.at(l).FLID4 << endl;
	}

//	//delete first two lanes !!!!! Don't know why
	if(roadLanes.size() > 0)
		roadLanes.erase(roadLanes.begin()+0);
	if(roadLanes.size() > 0)
		roadLanes.erase(roadLanes.begin()+0);

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

	roadLanes.push_back(lane_obj);
	cout << "Lanes No = " << roadLanes.size() << endl;

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

#ifdef FIND_LEFT_RIGHT_LANES
			//Link left and right lanes
			for(unsigned int rs_2 = 0; rs_2 < map.roadSegments.size(); rs_2++)
			{
				for(unsigned int i2 =0; i2 < map.roadSegments.at(rs_2).Lanes.size(); i2++)
				{
					int iCenter1 = pL->points.size()/2;
					WayPoint wp_1 = pL->points.at(iCenter1);
					int iCenter2 = PlanningHelpers::GetClosestPointIndex(map.roadSegments.at(rs_2).Lanes.at(i2).points, wp_1 );
					WayPoint closest_p = map.roadSegments.at(rs_2).Lanes.at(i2).points.at(iCenter2);
					double mid_a1 = wp_1.pos.a;
					double mid_a2 = closest_p.pos.a;
					double angle_diff = UtilityH::AngleBetweenTwoAnglesPositive(mid_a1, mid_a2);
					double distance = distance2points(wp_1.pos, closest_p.pos);

					if(pL->id != map.roadSegments.at(rs_2).Lanes.at(i2).id && angle_diff < 0.05 && distance < 3.5 && distance > 2.5)
					{
						double perp_distance = 99999;
						if(pL->points.size() > 2 && map.roadSegments.at(rs_2).Lanes.at(i2).points.size()>2)
							perp_distance = PlanningHelpers::GetPerpDistanceToVectorSimple(pL->points.at(iCenter1-1), pL->points.at(iCenter1+1), closest_p);

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
#endif
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
				}
			}
		}
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

	return 0;
}

void MappingHelpers::ConstructRoadNetworkFromDataFiles(const std::string vectoMapPath, RoadNetwork& map)
{
	/**
	 * Exporting the center lines
	 */
	string laneLinesDetails = vectoMapPath + "point.csv";
	string center_lines_info = vectoMapPath + "dtlane.csv";
	string lane_info = vectoMapPath + "lane.csv";
	string node_info = vectoMapPath + "node.csv";

	AisanCenterLinesFileReader  center_lanes(center_lines_info);
	AisanLanesFileReader lanes(lane_info);
	AisanPointsFileReader points(laneLinesDetails);
	AisanNodesFileReader nodes(node_info);


	vector<AisanNodesFileReader::AisanNode> nodes_data;
	nodes.ReadAllData(nodes_data);

	vector<AisanLanesFileReader::AisanLane> lanes_data;
	lanes.ReadAllData(lanes_data);

	vector<AisanPointsFileReader::AisanPoints> points_data;
	points.ReadAllData(points_data);

	vector<AisanCenterLinesFileReader::AisanCenterLine> dt_data;
	center_lanes.ReadAllData(dt_data);


	ConstructRoadNetworkFromRosMessage(lanes_data, points_data, dt_data,GetTransformationOrigin(), map);


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
//
//
//	geo_pos_conv geo;
//	geo.set_plane(6);
//	geo.llh_to_xyz(origin2.pos.lon, origin2.pos.lat, 0);
//
//	origin2.pos.x = geo.x();
//	origin2.pos.y = geo.y();
//
//	geo.llh_to_xyz(lastPoint2.pos.lon, lastPoint2.pos.lat, 0);
//
//	lastPoint2.pos.x = geo.x();
//	lastPoint2.pos.y = geo.y();
//
//	double distance2 = distance2points(origin2.pos, lastPoint2.pos);
//	cout << origin2.pos.ToString() ;

}

void MappingHelpers::SaveTrajectoryLonLatToKMLFile(const string& fileName, const vector<vector<WayPoint> >& trajectory)
  {
  	vector<vector<string> > m_str_kml;

  	for(unsigned l = 0 ; l < trajectory.size(); l++)
  	{
  		vector<string>  lane_str_kml;
		for(unsigned k = 0 ; k < trajectory.at(l).size(); k++)
		{
			ostringstream gps_str;
			gps_str.precision(12);
			GPSPoint gps_p = trajectory.at(l).at(k).pos;
			gps_str << gps_p.x << "," << gps_p.y << "," << gps_p.z;
			lane_str_kml.push_back(gps_str.str());
		}
		m_str_kml.push_back(lane_str_kml);
  	}

  	if(m_str_kml.size() > 0)
  		DataRW::WriteKMLFile(fileName, m_str_kml);
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
					double integ_part = points.at(p).L;
					double deg = trunc(points.at(p).L);
					double min = trunc((points.at(p).L - deg) * 100.0) / 60.0;
					double sec = modf((points.at(p).L - deg) * 100.0, &integ_part)/36.0;
					double L =  deg + min + sec;

					deg = trunc(points.at(p).B);
					min = trunc((points.at(p).B - deg) * 100.0) / 60.0;
					sec = modf((points.at(p).B - deg) * 100.0, &integ_part)/36.0;
					double B = deg + min + sec;

					wp.pos = GPSPoint(points.at(p).Ly + origin.x, points.at(p).Bx + origin.y, points.at(p).H + origin.z, dtpoints.at(dtp).Dir);

					wp.pos.lat = points.at(p).L;
					wp.pos.lon = points.at(p).B;
					wp.pos.alt = points.at(p).H;
					wp.pos.dir = dtpoints.at(dtp).Dir;

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

	//Fill the relations
	for(unsigned int i= 0; i<roadLinksList.size(); i++ )
	{
		for(unsigned int j=0; j < laneLinksList.size(); j++)
		{
			//if(laneLinksList.at(j).roadId == roadLinksList.at(i).id)
			{
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
				}
			}
		}
	}

}

void MappingHelpers::WriteKML(const string& kmlFile, const string& kmlTemplat, RoadNetwork& map)
{
	//First, Get the main element
	TiXmlElement* pHeadElem = 0;
	TiXmlElement* pElem = 0;

	TiXmlDocument doc(kmlTemplat);
	if(!doc.LoadFile())
	{
		cout << doc.ErrorDesc() << endl;
	}
	else
		pElem = doc.FirstChildElement();

	pHeadElem = GetHeadElement(pElem);

	vector<Lane> allLanes;
	vector<RoadSegment> roadSegments;

	for(unsigned int j=0; j< map.roadSegments.size(); j++)
	{
		allLanes.insert(allLanes.end(), map.roadSegments.at(j).Lanes.begin(),
				map.roadSegments.at(j).Lanes.end());
		roadSegments.push_back(map.roadSegments.at(j));
	}

	SetLaneLinksList(pHeadElem, allLanes);
	SetRoadLinksList(pHeadElem, roadSegments);

	doc.SaveFile(kmlFile.c_str());
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

void MappingHelpers::SetLaneLinksList(TiXmlElement* pElem, vector<Lane>& lanes)
{
	TiXmlElement* pLaneLinks = GetDataFolder("Lanes", pElem);
	TiXmlNode* pE = pLaneLinks->FirstChild("Folder");
	TiXmlNode* pN = pE;
	TiXmlText * pText = 0;
	TiXmlElement* pElement = 0;
	int roadId = -1;
	int roadsCount = 0;

	if(lanes.size() ==0)
		pE->Clear();

	for(unsigned int i=0; i< lanes.size(); i++)
	{
		Lane* pLane = &lanes.at(i);

		if(pLane->roadId != roadId)
		{
			roadId = pLane->roadId;
			roadsCount++;
		}

		if(i>0)
			pN = pE->Clone();

		ostringstream name, desc;
		name << "LID_" << pLane->id;
		pElement = pN->FirstChild("name")->ToElement();
		pText = new TiXmlText(name.str());
		pElement->Clear();
		pElement->LinkEndChild(pText);

		desc << "LID_" << pLane->id << "_RSID_" << pLane->roadId << "_NUM_" << pLane->num << "_From_";
		for(unsigned int j=0; j< pLane->fromIds.size(); j++)
			desc << pLane->fromIds.at(j) << "_";

		desc << "To";
		for(unsigned int j=0; j< pLane->toIds.size(); j++)
			desc << "_" << pLane->toIds.at(j);

		desc << "_Vel_" << pLane->speed;

		pElement = pN->FirstChild("description")->ToElement();
		pText = new TiXmlText(desc.str());
		pElement->Clear();
		pElement->LinkEndChild(pText);

		ostringstream style;
		pElement = pN->FirstChild("Placemark")->FirstChild("styleUrl")->ToElement();
		style << "#mms_route_line_" << roadsCount%4;
		pText = new TiXmlText(style.str());
		pElement->Clear();
		pElement->LinkEndChild(pText);


		pElement = pN->FirstChild("Placemark")->FirstChild("LineString")->FirstChild("coordinates")->ToElement();

		ostringstream val;
		val.precision(18);

		for(unsigned int j =0; j < pLane->points.size() ; j++)
		{
			GPSPoint p = pLane->points.at(j).pos;
			val << p.x << "," << p.y << "," << p.z <<  " ";
		}

		pText = new TiXmlText(val.str());
		pElement->Clear();
		pElement->LinkEndChild(pText);

		TiXmlNode* pWPNode = pN->FirstChild("Folder");
		TiXmlNode* pWPNodeCopy =  pWPNode;

		ostringstream valInfo;
		for(unsigned int j =0; j < pLane->points.size() ; j++)
		{
			char action = 'F';

//			if(pLane->points.at(j).actionCost. == LEFT_TURN_ACTION)
//				action = 'L';
//			else if(pLane->points.at(j).actionCost == RIGHT_TURN_ACTION)
//				action = 'R';

			valInfo << "WPID_" << pLane->points.at(j).id
					<< "_C_" << action << "_From_";

			for(unsigned int k=0; k< pLane->points.at(j).fromIds.size(); k++)
				valInfo << pLane->points.at(j).fromIds.at(k) << "_";

			valInfo << "To";
			for(unsigned int k=0; k< pLane->points.at(j).toIds.size(); k++)
				valInfo << "_" << pLane->points.at(j).toIds.at(k);

			valInfo << "_Vel_" << pLane->points.at(j).v;
			valInfo << "_Dir_" << pLane->points.at(j).pos.a;

			valInfo << ",";
		}

		TiXmlElement* pWPElem = pWPNodeCopy->FirstChild("description")->ToElement();
		pText = new TiXmlText(valInfo.str());
		pWPElem->Clear();
		pWPElem->LinkEndChild(pText);

		if(i>0)
			pLaneLinks->InsertEndChild(*pN);
	}
}

void MappingHelpers::SetRoadLinksList(TiXmlElement* pElem, vector<RoadSegment>& roadSegments)
{
	TiXmlElement* pRoadLinks = GetDataFolder("RoadSegments", pElem);
	TiXmlNode* pE = pRoadLinks->FirstChild("Placemark");
	TiXmlNode* pN = pE;
	TiXmlText * pText = 0;
	TiXmlElement* pElement = 0;

	if(roadSegments.size() ==0)
		pE->Clear();

	for(unsigned int i=0; i< roadSegments.size(); i++)
	{
		RoadSegment* pRS = &roadSegments.at(i);

		if(i>0)
			pN = pE->Clone();

		ostringstream name, desc;
		name << "RSID_" << pRS->id;
		pElement = pN->FirstChild("name")->ToElement();
		pElement->Clear();
		pText = new TiXmlText(name.str());
		pElement->LinkEndChild(pText);

		desc << "RSID_" << pRS->id;
		pElement = pN->FirstChild("description")->ToElement();
		pElement->Clear();
		pText = new TiXmlText(desc.str());
		pElement->LinkEndChild(pText);
		if(i>0)
			pRoadLinks->InsertEndChild(*pN);
	}
}

WayPoint* MappingHelpers::GetClosestWaypointFromMap(const WayPoint& pos, RoadNetwork& map)
{
	double distance_to_nearest_lane = 1;
	Lane* pLane = 0;
	while(distance_to_nearest_lane < 100 && pLane == 0)
	{
		pLane = GetClosestLaneFromMap(pos, map, distance_to_nearest_lane);
		distance_to_nearest_lane += 2;
	}

	if(!pLane) return 0;

	int closest_index = PlanningHelpers::GetClosestPointIndex(pLane->points, pos);

	return &pLane->points.at(closest_index);
}

Lane* MappingHelpers::GetClosestLaneFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance)
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

	d = 0, min_d = 999999999;
	Lane* closest_lane = 0;
	for(unsigned int i = 0; i < laneLinksList.size(); i++)
	{
		d = PlanningHelpers::GetPerpDistanceToTrajectorySimple(laneLinksList.at(i).second->points, pos);
		if(d == 0 && laneLinksList.at(i).first != 0)
			continue;

		if(abs(d)<min_d)
		{
			min_d = abs(d);
			closest_lane = laneLinksList.at(i).second;
		}
	}

	return closest_lane;
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

	d = 0, min_d = 999999999;
	Lane* closest_lane = 0;
	double a_diff = 0;
	for(unsigned int i = 0; i < laneLinksList.size(); i++)
	{
		d = PlanningHelpers::GetPerpDistanceToTrajectorySimple(laneLinksList.at(i).second->pLane->points, pos);
		if(d == 0 && laneLinksList.at(i).first != 0)
			continue;

		a_diff = UtilityH::AngleBetweenTwoAnglesPositive(laneLinksList.at(i).second->pos.a, pos.pos.a);

		if(abs(d)<min_d && a_diff <= M_PI_4)
		{
			min_d = abs(d);
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

void MappingHelpers::llaToxyz(GPSPoint& lla_p, const GPSPoint& origin)
{
	projPJ pj_latlong, pj_utm;
	pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84");
	pj_utm = pj_init_plus("+proj=utm +zone=53 +ellps=WGS84 +datum=WGS84");

	GPSPoint p = lla_p;

	p.x = lla_p.lon * DEG2RAD;
	p.y = lla_p.lat * DEG2RAD;

	if(pj_latlong != 0 && pj_utm !=0 )
	{
		pj_transform(pj_latlong, pj_utm, 1, 1, &p.y, &p.x, &p.z);
		p.x -= origin.x;
		p.y -= origin.y;
		p.z -= origin.z;
	}

	lla_p = p;

}

void MappingHelpers::xyzTolla(GPSPoint& xyz_p, const GPSPoint& origin)
{
	projPJ pj_latlong, pj_utm;
	pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84");
	pj_utm = pj_init_plus("+proj=utm +ellps=WGS84 +datum=WGS84 +units=m");

	GPSPoint p = xyz_p;

	if(pj_latlong != 0 && pj_utm !=0 )
	{
		pj_transform(pj_utm,pj_latlong, 1, 1, &p.lon, &p.lat, &p.alt);
		p.lon = p.lon * RAD2DEG;
		p.lat = p.lat * RAD2DEG;

		p.lon -= origin.lon;
		p.lat -= origin.lat;
		p.alt -= origin.alt;
	}

	xyz_p = p;

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

	int closest_index = PlanningHelpers::GetClosestNextPointIndex(currPath, currPos);

	return currPath.at(closest_index).pLane;
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
		  tfID = pNameXml->GetText();

		Lane ll;
		ll.id = GetIDsFromPrefix(tfID, "LID", "RSID").at(0);
		ll.roadId = GetIDsFromPrefix(tfID, "RSID", "NUM").at(0);
		ll.num = GetIDsFromPrefix(tfID, "NUM", "From").at(0);
		ll.fromIds = GetIDsFromPrefix(tfID, "From", "To");
		ll.toIds = GetIDsFromPrefix(tfID, "To", "Vel");
		ll.speed = GetIDsFromPrefix(tfID, "Vel", "").at(0);
		ll.points = GetCenterLaneData(pE, ll.id);



		llList.push_back(ll);
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
				gps_points.at(i).id =  GetIDsFromPrefix(add_info_list.at(i), "WPID", "C").at(0);
				gps_points.at(i).fromIds =  GetIDsFromPrefix(add_info_list.at(i), "From", "To");
				gps_points.at(i).toIds =  GetIDsFromPrefix(add_info_list.at(i), "To", "Vel");
				gps_points.at(i).v =  GetDoubleFromPrefix(add_info_list.at(i), "Vel", "Dir").at(0);
				gps_points.at(i).pos.a = gps_points.at(i).pos.dir =  GetDoubleFromPrefix(add_info_list.at(i), "Dir", "").at(0);
			}
		}
	}

	return gps_points;
}

vector<int> MappingHelpers::GetIDsFromPrefix(const string& str, const string& prefix, const string& postfix)
{
	int index1 = str.find(prefix)+prefix.size();
	int index2 = str.find(postfix, index1);
	if(index2<0  || postfix.size() ==0)
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
	if(index2<0  || postfix.size() ==0)
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

void MappingHelpers::InsertTrafficLightToMap(const TrafficLight& trafficLightPose, const double& stopingDistance, RoadNetwork& map)
{
	WayPoint wp;
	wp.pos = trafficLightPose.pos;
	WayPoint* pWP =  GetClosestWaypointFromMap(wp, map);
	if(pWP)
	{
		pWP->pLane->trafficlight = trafficLightPose;
		pWP->pLane->trafficlight.pLane = pWP->pLane;
	}
}

void MappingHelpers::CreateKmlFromLocalizationPathFile(const std::string& pathFileName,
		const std::vector<TrafficLight>& trafficLights,
		const std::vector<GPSPoint> stopLines)
{

	//Read Data From csv file
	LocalizationPathReader pathReader(pathFileName);
	vector<LocalizationPathReader::LocalizationWayPoint> path_data;
	pathReader.ReadAllData(path_data);
	PlannerHNS::RoadSegment segment;
	segment.id = 1;

	double d_accum = 0;
	double laneMaxLength = 40;

	std::vector<WayPoint> wayPointsList;

	PlannerHNS::Lane lane;
	lane.id = 1;
	lane.num = 0;
	lane.roadId = 1;
	WayPoint p_prev;

	for(unsigned int i=0; i< path_data.size() ; i++)
	{
		WayPoint p(path_data.at(i).x,path_data.at(i).y,path_data.at(i).z,path_data.at(i).a );
		p.pos.lat = p.pos.x;
		p.pos.lon = p.pos.y;
		p.pos.alt = p.pos.z;
		p.pos.dir = p.pos.a;
		p.v = path_data.at(i).v;
		p.laneId = lane.id;
		p.id = i+1;
		if(i>0)
		{
			p.fromIds.push_back(i);
			d_accum += hypot(p.pos.y-p_prev.pos.y, p.pos.x-p_prev.pos.x);
		}

		if(i<path_data.size()-1)
			p.toIds.push_back(i+2);

		wayPointsList.push_back(p);

		p_prev = p;

		if(d_accum > laneMaxLength)
		{
			if(segment.Lanes.size()>0)
				lane.fromIds.push_back(segment.Lanes.at(segment.Lanes.size()-1).id);
			lane.toIds.push_back(lane.id+1);
			lane.points = wayPointsList;
			segment.Lanes.push_back(lane);

			d_accum = 0;
			PlannerHNS::Lane n_lane;
			n_lane.id = lane.id+1;
			n_lane.num = 0;
			n_lane.roadId = 1;
			lane = n_lane;
			wayPointsList.clear();
		}
	}

	if(segment.Lanes.size()>0)
		segment.Lanes.at(segment.Lanes.size()-1).toIds.clear();

	PlannerHNS::RoadNetwork roadMap;
	roadMap.roadSegments.push_back(segment);

	ostringstream fileName;
	fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName;
	fileName << UtilityHNS:: UtilityH::GetFilePrefixHourMinuteSeconds();
	fileName << "_RoadNetwork.kml";
	string kml_templateFilePath = UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::KmlMapsFolderName+"PlannerX_MapTemplate.kml";

	PlannerHNS::MappingHelpers::WriteKML(fileName.str(),kml_templateFilePath , roadMap);
}

} /* namespace PlannerHNS */
