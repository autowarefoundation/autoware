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
};

} /* namespace PlannerHNS */

#endif /* PLANNINGHELPERS_H_ */
