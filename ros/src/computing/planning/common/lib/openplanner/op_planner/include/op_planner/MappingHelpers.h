/*
 * MappingHelpers.h
 *
 *  Created on: Jul 2, 2016
 *      Author: Hatem
 */

#ifndef MAPPINGHELPERS_H_
#define MAPPINGHELPERS_H_

#include <math.h>
#include "RoadNetwork.h"
#include "op_utility/UtilityH.h"
#include "op_utility/DataRW.h"
#include "tinyxml.h"


namespace PlannerHNS {


class MappingHelpers {
public:
	MappingHelpers();
	virtual ~MappingHelpers();

	static void ConstructRoadNetworkFromRosMessage(const std::vector<UtilityHNS::AisanLanesFileReader::AisanLane>& lanes_data,
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
			const GPSPoint& origin, RoadNetwork& map, const bool& bSpecialFlag = false);

	static void ConstructRoadNetworkFromDataFiles(const std::string vectoMapPath, RoadNetwork& map, const bool& bZeroOrigin = false);

	//static void SaveTrajectoryLonLatToKMLFile(const std::string& fileName, const std::vector<std::vector<WayPoint> >& trajectory);

	static void GetWayPoint(const int& pid, const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points, std::vector<WayPoint>& path);
	static bool GetWayPoint(const int& id, const int& laneID,const double& refVel, const int& did,
			const std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine>& dtpoints,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points,
			const GPSPoint& origin, WayPoint& way_point);

	//static void WriteKML(const std::string& kmlFile, const std::string& kmlTemplat, RoadNetwork& ap);
	static void LoadKML(const std::string& kmlMap, RoadNetwork& map);

	static TiXmlElement* GetHeadElement(TiXmlElement* pMainElem);
	static TiXmlElement* GetDataFolder(const std::string& folderName, TiXmlElement* pMainElem);


	static Lane* GetClosestLaneFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance = 5.0, const bool bDirectionBased = true);
	static std::vector<Lane*> GetClosestLanesListFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance = 2.0, const bool bDirectionBased = true);
	static Lane* GetClosestLaneFromMapDirectionBased(const WayPoint& pos, RoadNetwork& map, const double& distance = 5.0);
	static std::vector<Lane*> GetClosestMultipleLanesFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance = 5.0);
	static WayPoint* GetClosestWaypointFromMap(const WayPoint& pos, RoadNetwork& map, const bool bDirectionBased = true);

	static std::vector<WayPoint*> GetClosestWaypointsListFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance = 2.0, const bool bDirectionBased = true);

	static WayPoint* GetClosestBackWaypointFromMap(const WayPoint& pos, RoadNetwork& map);
	static WayPoint GetFirstWaypoint(RoadNetwork& map);
	static WayPoint* GetLastWaypoint(RoadNetwork& map);
	static void FindAdjacentLanes(RoadNetwork& map);
	static void ExtractSignalData(const std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal>& signal_data,
			const std::vector<UtilityHNS::AisanVectorFileReader::AisanVector>& vector_data,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
			const GPSPoint& origin, RoadNetwork& map);

	static void ExtractStopLinesData(const std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine>& stop_line_data,
			const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
			const GPSPoint& origin, RoadNetwork& map);

	static void ExtractCurbData(const std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb>& curb_data,
				const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
				const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
				const GPSPoint& origin, RoadNetwork& map);

	static void LinkMissingBranchingWayPoints(RoadNetwork& map);
	static void LinkTrafficLightsAndStopLinesConData(const std::vector<UtilityHNS::AisanDataConnFileReader::DataConn>& conn_data,
			const std::vector<std::pair<int,int> >& id_replace_list, RoadNetwork& map);

	static void LinkTrafficLightsAndStopLines(RoadNetwork& map);

	static void GetUniqueNextLanes(const Lane* l,  const std::vector<Lane*>& traversed_lanes, std::vector<Lane*>& lanes_list);

	static GPSPoint GetTransformationOrigin(const int& bToyotaCityMap = 0);

	static Lane* GetLaneFromPath(const WayPoint& currPos, const std::vector<WayPoint>& currPath);
	static Lane* GetLaneById(const int& id,RoadNetwork& map);
	static int GetLaneIdByWaypointId(const int& id,std::vector<Lane>& lanes);

	static WayPoint* FindWaypoint(const int& id, RoadNetwork& map);


	static std::vector<TrafficLight> GetTrafficLightsList(TiXmlElement* pElem);
	static std::vector<StopLine> GetStopLinesList(TiXmlElement* pElem);
	static std::vector<Lane> GetLanesList(TiXmlElement* pElem);
	static std::vector<RoadSegment> GetRoadSegmentsList(TiXmlElement* pElem);
	static std::vector<int> GetIDsFromPrefix(const std::string& str, const std::string& prefix, const std::string& postfix);
	static std::vector<double> GetDoubleFromPrefix(const std::string& str, const std::string& prefix, const std::string& postfix);
	static std::pair<ACTION_TYPE, double> GetActionPairFromPrefix(const std::string& str, const std::string& prefix, const std::string& postfix);
	static std::vector<WayPoint> GetCenterLaneData(TiXmlElement* pElem, const int& currLaneID);
	static std::vector<WayPoint> GetCenterLaneDataVer0(TiXmlElement* pElem, const int& currLaneID);
	static std::vector<GPSPoint> GetPointsData(TiXmlElement* pElem);
	static std::vector<std::string> SplitString(const std::string& str, const std::string& token);

	//static void CreateKmlFromLocalizationPathFile(const std::string& pathFileName,const double& maxLaneDistance, const double& density,const std::vector<TrafficLight>& trafficLights, const std::vector<GPSPoint>& stopLines);

	static int ReplaceMyID(int& id, const std::vector<std::pair<int,int> >& rep_list);

	static double m_USING_VER_ZERO;

};

} /* namespace PlannerHNS */

#endif /* MAPPINGHELPERS_H_ */
