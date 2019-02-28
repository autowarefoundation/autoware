
/// \file MappingHelpers.h
/// \brief Helper functions for mapping operation such as (load and initialize vector maps , convert map from one format to another, .. )
/// \author Hatem Darweesh
/// \date Jul 2, 2016



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

	static void ConstructRoadNetworkFromROSMessage(const std::vector<UtilityHNS::AisanLanesFileReader::AisanLane>& lanes_data,
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
			const GPSPoint& origin, RoadNetwork& map, const bool& bSpecialFlag = false,
			const bool& bFindLaneChangeLanes = false,
			const bool& bFindCurbsAndWayArea = false);

	static void ConstructRoadNetworkFromROSMessageV2(const std::vector<UtilityHNS::AisanLanesFileReader::AisanLane>& lanes_data,
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
				const GPSPoint& origin, RoadNetwork& map, const bool& bSpecialFlag = false,
				const bool& bFindLaneChangeLanes = false,
				const bool& bFindCurbsAndWayArea = false);

	static void ConstructRoadNetworkFromDataFiles(const std::string vectoMapPath, RoadNetwork& map, const bool& bZeroOrigin = false);

	static void UpdateMapWithOccupancyGrid(OccupancyToGridMap& map_info, const std::vector<int>& data, RoadNetwork& map, std::vector<WayPoint*>& updated_list);

	static bool GetWayPoint(const int& id, const int& laneID,const double& refVel, const int& did,
			const std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine>& dtpoints,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points,
			const GPSPoint& origin, WayPoint& way_point);

	static void LoadKML(const std::string& kmlMap, RoadNetwork& map);

	static TiXmlElement* GetHeadElement(TiXmlElement* pMainElem);
	static TiXmlElement* GetDataFolder(const std::string& folderName, TiXmlElement* pMainElem);


	static Lane* GetClosestLaneFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance = 5.0, const bool bDirectionBased = true);
	static std::vector<Lane*> GetClosestLanesListFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance = 2.0, const bool bDirectionBased = true);
	static Lane* GetClosestLaneFromMapDirectionBased(const WayPoint& pos, RoadNetwork& map, const double& distance = 5.0);
	static std::vector<Lane*> GetClosestMultipleLanesFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance = 5.0);
	static WayPoint* GetClosestWaypointFromMap(const WayPoint& pos, RoadNetwork& map, const bool bDirectionBased = true);
	static std::vector<Lane*> GetClosestLanesFast(const WayPoint& pos, RoadNetwork& map, const double& distance = 10.0);

	static std::vector<WayPoint*> GetClosestWaypointsListFromMap(const WayPoint& center, RoadNetwork& map, const double& distance = 2.0, const bool bDirectionBased = true);

	static WayPoint* GetClosestBackWaypointFromMap(const WayPoint& pos, RoadNetwork& map);
	static WayPoint GetFirstWaypoint(RoadNetwork& map);
	static WayPoint* GetLastWaypoint(RoadNetwork& map);
	static void FindAdjacentLanes(RoadNetwork& map);
	static void FindAdjacentLanesV2(RoadNetwork& map);
	static void ExtractSignalData(const std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal>& signal_data,
			const std::vector<UtilityHNS::AisanVectorFileReader::AisanVector>& vector_data,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
			const GPSPoint& origin, RoadNetwork& map);

	static void ExtractSignalDataV2(const std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal>& signal_data,
				const std::vector<UtilityHNS::AisanVectorFileReader::AisanVector>& vector_data,
				UtilityHNS::AisanPointsFileReader* pPointsData,
				const GPSPoint& origin, RoadNetwork& map);

	static void ExtractStopLinesData(const std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine>& stop_line_data,
			const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
			const GPSPoint& origin, RoadNetwork& map);

	static void ExtractStopLinesDataV2(const std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine>& stop_line_data,
				UtilityHNS::AisanLinesFileReader* pLineData,
				UtilityHNS::AisanPointsFileReader* pPointData,
				const GPSPoint& origin, RoadNetwork& map);

	static void ExtractCurbData(const std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb>& curb_data,
				const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
				const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
				const GPSPoint& origin, RoadNetwork& map);

	static void ExtractCurbDataV2(const std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb>& curb_data,
					UtilityHNS::AisanLinesFileReader* pLinedata,
					UtilityHNS::AisanPointsFileReader* pPointsData,
					const GPSPoint& origin, RoadNetwork& map);

	static void ExtractWayArea(const std::vector<UtilityHNS::AisanAreasFileReader::AisanArea>& area_data,
			const std::vector<UtilityHNS::AisanWayareaFileReader::AisanWayarea>& wayarea_data,
			const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
			const GPSPoint& origin, RoadNetwork& map);

	static void LinkMissingBranchingWayPoints(RoadNetwork& map);
	static void LinkMissingBranchingWayPointsV2(RoadNetwork& map);
	static void LinkTrafficLightsAndStopLinesConData(const std::vector<UtilityHNS::AisanDataConnFileReader::DataConn>& conn_data,
			const std::vector<std::pair<int,int> >& id_replace_list, RoadNetwork& map);

	static void LinkTrafficLightsAndStopLines(RoadNetwork& map);

	static void LinkTrafficLightsAndStopLinesV2(RoadNetwork& map);

	static void GetUniqueNextLanes(const Lane* l,  const std::vector<Lane*>& traversed_lanes, std::vector<Lane*>& lanes_list);

	static GPSPoint GetTransformationOrigin(const int& bToyotaCityMap = 0);

	static Lane* GetLaneFromPath(const WayPoint& currPos, const std::vector<WayPoint>& currPath);
	static Lane* GetLaneById(const int& id,RoadNetwork& map);
	static int GetLaneIdByWaypointId(const int& id,std::vector<Lane>& lanes);

	static WayPoint* FindWaypoint(const int& id, RoadNetwork& map);
	static WayPoint* FindWaypointV2(const int& id, const int& l_id, RoadNetwork& map);

	static std::vector<Curb> GetCurbsList(TiXmlElement* pElem);
	static std::vector<Boundary> GetBoundariesList(TiXmlElement* pElem);
	static std::vector<Marking> GetMarkingsList(TiXmlElement* pElem);
	static std::vector<Crossing> GetCrossingsList(TiXmlElement* pElem);
	static std::vector<TrafficSign> GetTrafficSignsList(TiXmlElement* pElem);
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

	static void AssignActionCostToLane(Lane* pL, ACTION_TYPE action, double cost);

	static int ReplaceMyID(int& id, const std::vector<std::pair<int,int> >& rep_list);

	static void GetLanesStartPoints(UtilityHNS::AisanLanesFileReader* pLaneData,
				std::vector<int>& m_LanesStartIds);

	static void GetLanePoints(UtilityHNS::AisanLanesFileReader* pLaneData,
				UtilityHNS::AisanPointsFileReader* pPointsData,
				UtilityHNS::AisanNodesFileReader* pNodesData, int lnID,
				PlannerHNS::Lane& out_lane);

	static void CreateLanes(UtilityHNS::AisanLanesFileReader* pLaneData,
			UtilityHNS::AisanPointsFileReader* pPointsData,
			UtilityHNS::AisanNodesFileReader* pNodesData,
			std::vector<PlannerHNS::Lane>& out_lanes);

	static void ConnectLanes(UtilityHNS::AisanLanesFileReader* pLaneData,
			std::vector<PlannerHNS::Lane>& lanes);

	static bool GetPointFromDataList(UtilityHNS::AisanPointsFileReader* pPointsData,const int& pid, WayPoint& out_wp);

	static int GetBeginPointIdFromLaneNo(UtilityHNS::AisanLanesFileReader* pLaneData,
			UtilityHNS::AisanPointsFileReader* pPointsData,
			UtilityHNS::AisanNodesFileReader* pNodesData, const int& LnID);
	static int GetEndPointIdFromLaneNo(UtilityHNS::AisanLanesFileReader* pLaneData,
			UtilityHNS::AisanPointsFileReader* pPointsData,
			UtilityHNS::AisanNodesFileReader* pNodesData,const int& LnID);

	static bool IsStartLanePoint(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanLanesFileReader::AisanLane* pL);
	static bool IsEndLanePoint(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanLanesFileReader::AisanLane* pL);

	static void FixRedundantPointsLanes(std::vector<Lane>& lanes);
	static void FixTwoPointsLanes(std::vector<Lane>& lanes);
	static void FixTwoPointsLane(Lane& lanes);
	static void FixUnconnectedLanes(std::vector<Lane>& lanes);
	static void InsertWayPointToBackOfLane(const WayPoint& wp, Lane& lane, int& global_id);
	static void InsertWayPointToFrontOfLane(const WayPoint& wp, Lane& lane, int& global_id);

	static void LinkLanesPointers(PlannerHNS::RoadNetwork& map);

	static void GetMapMaxIds(PlannerHNS::RoadNetwork& map);

	static double m_USING_VER_ZERO;

	static int g_max_point_id;
	static int g_max_lane_id;
	static int g_max_stop_line_id;
	static int g_max_traffic_light_id ;

};

} /* namespace PlannerHNS */

#endif /* MAPPINGHELPERS_H_ */
