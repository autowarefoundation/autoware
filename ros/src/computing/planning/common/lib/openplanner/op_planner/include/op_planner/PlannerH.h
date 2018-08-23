
/// \file PlannerH.h
/// \brief Main functions for path generation (global and local)
/// \author Hatem Darweesh
/// \date Dec 14, 2016


#define START_POINT_MAX_DISTANCE 8 // meters
#define GOAL_POINT_MAX_DISTANCE 8 // meters
#define LANE_CHANGE_SMOOTH_FACTOR_DISTANCE 8 // meters

#include "RoadNetwork.h"

namespace PlannerHNS
{

enum PLANDIRECTION {MOVE_FORWARD_ONLY, MOVE_BACKWARD_ONLY, 	MOVE_FREE};
enum HeuristicConstrains {EUCLIDEAN, NEIGBORHOOD,DIRECTION };

class PlannerH
{
public:
	PlannerH();
	virtual ~PlannerH();

	void GenerateRunoffTrajectory(const std::vector<std::vector<WayPoint> >& referencePaths, const WayPoint& carPos, const bool& bEnableLaneChange, const double& speed, const double& microPlanDistance,
				const double& maxSpeed,const double& minSpeed, const double&  carTipMargin, const double& rollInMargin,
				const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
				const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
				const double& SmoothTolerance, const double& speedProfileFactor, const bool& bHeadingSmooth,
				const int& iCurrGlobalPath, const int& iCurrLocalTraj,
				std::vector<std::vector<std::vector<WayPoint> > >& rollOutsPaths,
				std::vector<WayPoint>& sampledPoints);

	double PlanUsingDP(const WayPoint& carPos,const WayPoint& goalPos,
			const double& maxPlanningDistance, const bool bEnableLaneChange, const std::vector<int>& globalPath,
			RoadNetwork& map, std::vector<std::vector<WayPoint> >& paths, std::vector<WayPoint*>* all_cell_to_delete = 0);

	 double PlanUsingDPRandom(const WayPoint& start,
	 		 const double& maxPlanningDistance,
	 		 RoadNetwork& map,
	 		 std::vector<std::vector<WayPoint> >& paths);

	double PredictPlanUsingDP(Lane* lane, const WayPoint& carPos, const double& maxPlanningDistance,
			std::vector<std::vector<WayPoint> >& paths);

	double PredictPlanUsingDP(const WayPoint& startPose, WayPoint* closestWP, const double& maxPlanningDistance, std::vector<std::vector<WayPoint> >& paths, const bool& bFindBranches = true);

	double PredictTrajectoriesUsingDP(const WayPoint& startPose, std::vector<WayPoint*> closestWPs, const double& maxPlanningDistance, std::vector<std::vector<WayPoint> >& paths, const bool& bFindBranches = true, const bool bDirectionBased = false, const bool pathDensity = 1.0);

	void DeleteWaypoints(std::vector<WayPoint*>& wps);
};

}


