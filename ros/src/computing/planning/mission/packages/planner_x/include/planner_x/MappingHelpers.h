/*
 * MappingHelpers.h
 *
 *  Created on: Jul 2, 2016
 *      Author: ai-driver
 */

#ifndef MAPPINGHELPERS_H_
#define MAPPINGHELPERS_H_

#include <math.h>
#include "RoadNetwork.h"
#include "UtilityH.h"
#include "DataRW.h"
#include "tinyxml.h"

namespace PlannerHNS {

class MappingHelpers {
public:
	MappingHelpers();
	virtual ~MappingHelpers();

	static void ConstructRoadNetworkFromDataFiles(const std::string vectoMapPath, RoadNetwork& map);
	static void SaveTrajectoryLonLatToKMLFile(const std::string& fileName, const std::vector<std::vector<WayPoint> >& trajectory);

	static void GetWayPoint(const int& pid, const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points, std::vector<WayPoint>& path);
	static void GetWayPoint(const int& id, const int& laneID,const double& refVel, const int& did,
			const std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine>& dtpoints,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points,std::vector<WayPoint>& path);

	static void WriteKML(const std::string& kmlFile, const std::string& kmlTemplat, RoadNetwork& ap);

	static void SetLaneLinesList(TiXmlElement* pElem, std::vector<Lane>& stopLines);
	static TiXmlElement* GetHeadElement(TiXmlElement* pMainElem);
	static TiXmlElement* GetDataFolder(const std::string& folderName, TiXmlElement* pMainElem);
	static void SetLaneLinksList(TiXmlElement* pElem, std::vector<Lane>& lanes);

	static Lane* GetClosestLaneFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance = 5.0);
	static WayPoint GetFirstWaypoint(RoadNetwork& map);
};

} /* namespace PlannerHNS */

#endif /* MAPPINGHELPERS_H_ */
