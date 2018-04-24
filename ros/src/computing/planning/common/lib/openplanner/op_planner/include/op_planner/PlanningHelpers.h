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
#include "op_utility/UtilityH.h"
#include "op_utility/DataRW.h"
#include "tinyxml.h"


namespace PlannerHNS {

#define distance2points(from , to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2pointsSqr(from , to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x*v.x + v.y*v.y)
#define angle2points(from , to) atan2(to.y - from.y, to.x - from.x )
#define LANE_CHANGE_SPEED_FACTOR 0.5
#define LANE_CHANGE_COST 3.0 // meters
#define BACKUP_STRAIGHT_PLAN_DISTANCE 75 //meters

class PlanningHelpers
{

public:
	static std::vector<std::pair<GPSPoint, GPSPoint> > m_TestingClosestPoint;

public:
	PlanningHelpers();
	virtual ~PlanningHelpers();

	/**
	 * @brief Find all relative information from the point p to the trajectory such as (perpendicular distance , closest next point , closest back point, distance from perpendicular intersection point to next point, distance from perpendicular intersection point to previous point)
	 * @param trajectory list of waypoints
	 * @param p query point
	 * @param info collection of calculated information
	 * @param prevIndex initial search index
	 * @return true if success without errors, false otherwise
	 */
	static bool GetRelativeInfo(const std::vector<WayPoint>& trajectory, const WayPoint& p, RelativeInfo& info, const int& prevIndex = 0);


	static bool GetRelativeInfoRange(const std::vector<std::vector<WayPoint> >& trajectories, const WayPoint& p, const double& searchDistance, RelativeInfo& info);

	/**
	 * @brief Find point on the trajectory after initial relative point with specific distance
	 * @param trajectory list of waypoints
	 * @param init_p initial relative point on trajectory
	 * @param distance distance from initial relative point to follow point
	 * @return point on trajectory
	 */
	static WayPoint GetFollowPointOnTrajectory(const std::vector<WayPoint>& trajectory, const RelativeInfo& init_p, const double& distance, unsigned int& point_index);

	/**
	 * @brief Calculate the precise distance from projection of point p2 (relative) to projection of point 1 (relative)
	 * @param trajectory list of waypoints
	 * @param p1 first relative point
	 * @param p2 second relative point
	 * @return distance on trajectory
	 */
	static double GetExactDistanceOnTrajectory(const std::vector<WayPoint>& trajectory, const RelativeInfo& p1,const RelativeInfo& p2);

	/**
	 * @brief Find the closest next point on the trajectory index
	 * @param trajectory list of waypoints
	 * @param p query point
	 * @param prevIndex initial search index
	 * @return index of the closest next point from trajectory
	 */
	static int GetClosestNextPointIndex_obsolete(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0);

	static int GetClosestNextPointIndexFast(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0);

	static int GetClosestNextPointIndexFastV2(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0);

	static int GetClosestNextPointIndexDirectionFast(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0);

	static int GetClosestNextPointIndexDirectionFastV2(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0);

	static int GetClosestPointIndex_obsolete(const std::vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex = 0 );
	static WayPoint GetPerpendicularOnTrajectory_obsolete(const std::vector<WayPoint>& trajectory, const WayPoint& p, double& distance, const int& prevIndex = 0);
	static double GetPerpDistanceToTrajectorySimple_obsolete(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0);
	static double GetPerpDistanceToVectorSimple_obsolete(const WayPoint& p1, const WayPoint& p2, const WayPoint& pose);
	static WayPoint GetNextPointOnTrajectory_obsolete(const std::vector<WayPoint>& trajectory, const double& distance, const int& currIndex = 0);
	static double GetDistanceOnTrajectory_obsolete(const std::vector<WayPoint>& path, const int& start_index, const WayPoint& p);


	static void CreateManualBranch(std::vector<WayPoint>& path, const int& degree, const DIRECTION_TYPE& direction);
	static void CreateManualBranchFromTwoPoints(WayPoint& p1,WayPoint& p2 , const double& distance, const DIRECTION_TYPE& direction, std::vector<WayPoint>& path);

	static void FixPathDensity(std::vector<WayPoint>& path, const double& distanceDensity);
	static void SmoothPath(std::vector<WayPoint>& path, double weight_data =0.25,double weight_smooth = 0.25,double tolerance = 0.01);
	static double CalcCircle(const GPSPoint& pt1, const GPSPoint& pt2, const GPSPoint& pt3, GPSPoint& center);
	static double CalcAngleAndCost(std::vector<WayPoint>& path, const double& lastCost = 0, const bool& bSmooth = true );
	//static double CalcAngleAndCostSimple(std::vector<WayPoint>& path, const double& lastCost = 0);
	static double CalcAngleAndCostAndCurvatureAnd2D(std::vector<WayPoint>& path, const double& lastCost = 0);

	static double GetAccurateDistanceOnTrajectory(std::vector<WayPoint>& path, const int& start_index, const WayPoint& p);

	static void ExtractPartFromPointToDistance(const std::vector<WayPoint>& originalPath, const WayPoint& pos, const double& minDistance,
			const double& pathDensity, std::vector<WayPoint>& extractedPath, const double& SmoothDataWeight, const double& SmoothWeight, const double& SmoothTolerance);

	static void ExtractPartFromPointToDistanceFast(const std::vector<WayPoint>& originalPath, const WayPoint& pos, const double& minDistance,
				const double& pathDensity, std::vector<WayPoint>& extractedPath, const double& SmoothDataWeight, const double& SmoothWeight, const double& SmoothTolerance);

	static void CalculateRollInTrajectories(const WayPoint& carPos, const double& speed, const std::vector<WayPoint>& originalCenter, int& start_index,
			int& end_index, std::vector<double>& end_laterals ,
			std::vector<std::vector<WayPoint> >& rollInPaths, const double& max_roll_distance,
			const double& maxSpeed, const double&  carTipMargin, const double& rollInMargin,
			const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
			const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
			const double& SmoothTolerance, const bool& bHeadingSmooth,
			std::vector<WayPoint>& sampledPoints);

	static void SmoothSpeedProfiles(std::vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance	= 0.1);
	static void SmoothCurvatureProfiles(std::vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance = 0.1);
	static void SmoothWayPointsDirections(std::vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance	= 0.1);

	static void SmoothGlobalPathSpeed(std::vector<WayPoint>& path);

	static void GenerateRecommendedSpeed(std::vector<WayPoint>& path, const double& max_speed, const double& speedProfileFactor);
//	static WayPoint* BuildPlanningSearchTree(Lane* l, const WayPoint& prevWayPointIndex,
//			const WayPoint& startPos, const WayPoint& goalPos,
//			const std::vector<int>& globalPath, const double& DistanceLimit,
//			int& nMaxLeftBranches, int& nMaxRightBranches,
//			std::vector<WayPoint*>& all_cells_to_delete );

	static WayPoint* BuildPlanningSearchTreeV2(WayPoint* pStart,
			const WayPoint& goalPos,
			const std::vector<int>& globalPath, const double& DistanceLimit,
			const bool& bEnableLaneChange,
			std::vector<WayPoint*>& all_cells_to_delete );

	static WayPoint* BuildPlanningSearchTreeStraight(WayPoint* pStart,
			const double& DistanceLimit,
			std::vector<WayPoint*>& all_cells_to_delete );

	static int PredictiveDP(WayPoint* pStart, const double& DistanceLimit,
			std::vector<WayPoint*>& all_cells_to_delete, std::vector<WayPoint*>& end_waypoints);

	static int PredictiveIgnorIdsDP(WayPoint* pStart, const double& DistanceLimit,
				std::vector<WayPoint*>& all_cells_to_delete, std::vector<WayPoint*>& end_waypoints, std::vector<int>& lanes_ids);

	static bool CheckLaneIdExits(const std::vector<int>& lanes, const Lane* pL);
	static WayPoint* CheckLaneExits(const std::vector<WayPoint*>& nodes, const Lane* pL);
	static WayPoint* CheckNodeExits(const std::vector<WayPoint*>& nodes, const WayPoint* pL);

	static WayPoint* CreateLaneHeadCell(Lane* pLane, WayPoint* pLeft, WayPoint* pRight,
			WayPoint* pBack);
	static double GetLanePoints(Lane* l, const WayPoint& prevWayPointIndex,
			const double& minDistance , const double& prevCost, std::vector<WayPoint>& points);

	static WayPoint* GetMinCostCell(const std::vector<WayPoint*>& cells, const std::vector<int>& globalPathIds);

	static void TraversePathTreeBackwards(WayPoint* pHead, WayPoint* pStartWP, const std::vector<int>& globalPathIds,
			std::vector<WayPoint>& localPath, std::vector<std::vector<WayPoint> >& localPaths);

	static void ExtractPlanAlernatives(const std::vector<WayPoint>& singlePath, std::vector<std::vector<WayPoint> >& allPaths);

	static std::vector<int> GetUniqueLeftRightIds(const std::vector<WayPoint>& path);

	static bool FindInList(const std::vector<int>& list,const int& x);
	static void RemoveWithValue(std::vector<int>& list,const int& x);

	static ACTION_TYPE GetBranchingDirection(WayPoint& currWP, WayPoint& nextWP);

	static void CalcContourPointsForDetectedObjects(const WayPoint& currPose, std::vector<DetectedObject>& obj_list, const double& filterDistance = 100);

	static double GetVelocityAhead(const std::vector<WayPoint>& path, const RelativeInfo& info,int& prev_index, const double& reasonable_brake_distance);
	static bool CompareTrajectories(const std::vector<WayPoint>& path1, const std::vector<WayPoint>& path2);

	static double GetDistanceToClosestStopLineAndCheck(const std::vector<WayPoint>& path, const WayPoint& p, int& stopLineID,int& stopSignID, int& trafficLightID, const int& prevIndex = 0);

	static bool GetThreePointsInfo(const WayPoint& p0, const WayPoint& p1, const WayPoint& p2, WayPoint& perp_p, double& long_d, double lat_d);

	static void WritePathToFile(const std::string& fileName, const std::vector<WayPoint>& path);

	static void TestQuadraticSpline(const std::vector<WayPoint>& center_line, std::vector<WayPoint>& path);
	static double frunge ( double x );
	static double fprunge ( double x );
	static double fpprunge ( double x );

};

} /* namespace PlannerHNS */

#endif /* PLANNINGHELPERS_H_ */
