/*
 * PlanningHelpers.h
 *
 *  Created on: Jun 16, 2016
 *      Author: hatem
 */

#ifndef PLANNINGHELPERS_H_
#define PLANNINGHELPERS_H_

#include <math.h>
#include "RoadNetwork.h"
#include "UtilityH.h"
#include "DataRW.h"
#include "tinyxml.h"


namespace PlannerHNS {

#define distance2points(from, to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2pointsSqr(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x*v.x + v.y*v.y)
#define angle2points(from, to) atan2(to.y - from.y, to.x - from.x )
#define LANE_CHANGE_SPEED_FACTOR 0.5

class PlanningHelpers {
public:
	PlanningHelpers();
	virtual ~PlanningHelpers();


	static int GetClosestNextPointIndex(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0);
	static int GetClosestPointIndex(const std::vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex = 0 );
	static WayPoint GetPerpendicularOnTrajectory(const std::vector<WayPoint>& trajectory, const WayPoint& p, double& distance, const int& prevIndex = 0);
	static double GetPerpDistanceToTrajectorySimple(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0);
	static WayPoint GetNextPointOnTrajectory(const std::vector<WayPoint>& trajectory, const double& distance, const int& currIndex = 0);

	static void FixPathDensity(std::vector<WayPoint>& path, const double& distanceDensity);
	static void SmoothPath(std::vector<WayPoint>& path, double weight_data =0.25,double weight_smooth = 0.25,double tolerance = 0.01);
	static double CalcCircle(const GPSPoint& pt1, const GPSPoint& pt2, const GPSPoint& pt3, GPSPoint& center);
	static double CalcAngleAndCost(std::vector<WayPoint>& path, const double& lastCost = 0, const bool& bSmooth = true );
	static double CalcAngleAndCostAndCurvatureAnd2D(std::vector<WayPoint>& path, const double& lastCost = 0, const bool& bSmooth = true );
	static double GetDistanceOnTrajectory(std::vector<WayPoint>& path, const int& start_index, const WayPoint& p);

	static void ExtractPartFromPointToDistance(const std::vector<WayPoint>& originalPath, const WayPoint& pos, const double& minDistance,
			const double& pathDensity, std::vector<WayPoint>& extractedPath, const double& SmoothDataWeight, const double& SmoothWeight, const double& SmoothTolerance);

	static void CalculateRollInTrajectories(const WayPoint& carPos, const double& speed, const std::vector<WayPoint>& originalCenter, int& start_index,
			int& end_index, std::vector<double>& end_laterals ,
			std::vector<std::vector<WayPoint> >& rollInPaths, const double& max_roll_distance,
			const double& maxSpeed, const double&  carTipMargin, const double& rollInMargin,
			const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
			const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
			const double& SmoothTolerance, const bool& bHeadingSmooth);


	static void SmoothSpeedProfiles(std::vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance	= 0.1);
	static void SmoothCurvatureProfiles(std::vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance = 0.1);
	static void SmoothWayPointsDirections(std::vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance	= 0.1);

	static void GenerateRecommendedSpeed(std::vector<WayPoint>& path, const double& max_speed, const double& speedProfileFactor);
//	static WayPoint* BuildPlanningSearchTree(Lane* l, const WayPoint& prevWayPointIndex,
//			const WayPoint& startPos, const WayPoint& goalPos,
//			const std::vector<int>& globalPath, const double& DistanceLimit,
//			int& nMaxLeftBranches, int& nMaxRightBranches,
//			std::vector<WayPoint*>& all_cells_to_delete );

	static WayPoint* BuildPlanningSearchTreeV2(WayPoint* pStart, const WayPoint& prevWayPointIndex,
				const WayPoint& startPos, const WayPoint& goalPos,
				const std::vector<int>& globalPath, const double& DistanceLimit,
				int& nMaxLeftBranches, int& nMaxRightBranches,
				std::vector<WayPoint*>& all_cells_to_delete );

	static bool CheckLaneIdExits(const std::vector<int>& lanes, const Lane* pL);
	static WayPoint* CheckLaneExits(const std::vector<std::pair<Lane*, WayPoint*> >& lanes, const Lane* pL);
	static WayPoint* CheckNodeExits(const std::vector<WayPoint*>& nodes, const WayPoint* pL);

	static WayPoint* CreateLaneHeadCell(Lane* pLane, WayPoint* pLeft, WayPoint* pRight,
			WayPoint* pBack);
	static double GetLanePoints(Lane* l, const WayPoint& prevWayPointIndex,
			const double& minDistance , const double& prevCost, std::vector<WayPoint>& points);

	static WayPoint* GetMinCostCell(const std::vector<WayPoint*>& cells, const std::vector<int>& globalPathIds);

	static void TravesePathTreeBackwards(WayPoint* pHead, WayPoint* pStartWP, const std::vector<int>& globalPathIds,
			std::vector<WayPoint>& localPath, std::vector<std::vector<WayPoint> >& localPaths);

	static ACTION_TYPE GetBranchingDirection(WayPoint& currWP, WayPoint& nextWP);

	static void CalcContourPointsForDetectedObjects(const WayPoint& currPose, std::vector<DetectedObject>& obj_list, const double& filterDistance = 100);

	static double GetVelocityAhead(const std::vector<WayPoint>& path, const WayPoint& pose, const double& distance);
	static bool CompareTrajectories(const std::vector<WayPoint>& path1, const std::vector<WayPoint>& path2);

	static void WritePathToFile(const std::string& fileName, const std::vector<WayPoint>& path);

};

} /* namespace PlannerHNS */

#endif /* PLANNINGHELPERS_H_ */
