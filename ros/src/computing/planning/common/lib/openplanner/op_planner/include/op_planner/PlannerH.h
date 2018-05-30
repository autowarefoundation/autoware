/*
 * PlannerH.h
 *
 *  Created on: May 14, 2016
 *      Author: hatem
 */

#include "RSPlanner.h"

#define START_POINT_MAX_DISTANCE 8 // meters
#define GOAL_POINT_MAX_DISTANCE 8 // meters
#define LANE_CHANGE_SMOOTH_FACTOR_DISTANCE 8 // meters

namespace PlannerHNS
{

enum PLANDIRECTION {MOVE_FORWARD_ONLY, MOVE_BACKWARD_ONLY, 	MOVE_FREE};
enum HeuristicConstrains {EUCLIDEAN, NEIGBORHOOD,DIRECTION };

class PlannerH
{
public:
	PlannerH();
	virtual ~PlannerH(); 

	/**
	 * @brief Generates Trajectory using Reeds Shepp, this method will not try to avoid obstacles , but if there an obstacle on the trajectory function will fail. , also this function does not guaranteed to generate trajectories
	 * @param start: Start position for the trajectory (x,y,theta)
	 * @param goal:  Goal position (Destination point) (x,y,theta)
	 * @param generatedPath: pointer to return the smooth trajectory. - Better Not to use this
	 * @return path length
	 */
	double PlanUsingReedShepp(const WayPoint& start, const WayPoint& goal, std::vector<WayPoint>& generatedPath,
			const double pathDensity = 0.25, const double smoothFactor = 12.0);


	/**
	 * @brief Generate Roll outs for global generated path
	 * @param referencePath center lint reference path
	 * @param carPos current car position
	 * @param bEnableLaneChange is lane change is available
	 * @param speed current car speed
	 * @param microPlanDistance distance limit for roll outs
	 * @param maxSpeed maximum forward speed
	 * @param minSpeed minimum forward speed
	 * @param carTipMargin 1st roll out smoothing parameter
	 * @param rollInMargin 2nd roll out smoothing parameter
	 * @param rollInSpeedFactor roll out 3rd smoothing parameter
	 * @param pathDensity distance between every two waypoints in the generated trajectory
	 * @param rollOutDensity distance between the center line and adjacent trajectories
	 * @param rollOutNumber number of sampled trajectories
	 * @param SmoothDataWeight general smoothing parameter , how smoother keep close to original data 0 - 0.5
	 * @param SmoothWeight general smoothing parameter, how smoother is trying to pull away to form the shortest and strait line possible
	 * @param SmoothTolerance performance measure , conjugate gradient conversion factor should be 0.1 - 0.01
	 * @param speedProfileFactor how car should slow for corners
	 * @param bHeadingSmooth follow car heading direction or center line path heading for sampling direction
	 */
	void GenerateRunoffTrajectory(const std::vector<std::vector<WayPoint> >& referencePaths, const WayPoint& carPos, const bool& bEnableLaneChange, const double& speed, const double& microPlanDistance,
				const double& maxSpeed,const double& minSpeed, const double&  carTipMargin, const double& rollInMargin,
				const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
				const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
				const double& SmoothTolerance, const double& speedProfileFactor, const bool& bHeadingSmooth,
				const int& iCurrGlobalPath, const int& iCurrLocalTraj,
				std::vector<std::vector<std::vector<WayPoint> > >& rollOutsPaths,
				std::vector<WayPoint>& sampledPoints);

	/**
	 * @brief Path planning for structured environment using dynamic programming
	 * @param lane
	 * @param carPos
	 * @param goalPos
	 * @param prevWayPoint
	 * @param maxPlanningDistance
	 * @param globalPath
	 * @param path
	 * @return generated path length
	 */
	double PlanUsingDP(const WayPoint& carPos,const WayPoint& goalPos,
			const double& maxPlanningDistance, const bool bEnableLaneChange, const std::vector<int>& globalPath,
			RoadNetwork& map, std::vector<std::vector<WayPoint> >& paths, std::vector<WayPoint*>* all_cell_to_delete = 0);

	 double PlanUsingDPRandom(const WayPoint& start,
	 		 const double& maxPlanningDistance,
	 		 RoadNetwork& map,
	 		 std::vector<std::vector<WayPoint> >& paths);


	/**
	 * @brief Return all possible trajectories from current position to max planning distance in all directions
	 * @param lane
	 * @param carPos
	 * @param maxPlanningDistance
	 * @param paths
	 * @return
	 */
	double PredictPlanUsingDP(Lane* lane, const WayPoint& carPos, const double& maxPlanningDistance,
			std::vector<std::vector<WayPoint> >& paths);

	double PredictPlanUsingDP(const WayPoint& startPose, WayPoint* closestWP, const double& maxPlanningDistance, std::vector<std::vector<WayPoint> >& paths, const bool& bFindBranches = true);

	double PredictTrajectoriesUsingDP(const WayPoint& startPose, std::vector<WayPoint*> closestWPs, const double& maxPlanningDistance, std::vector<std::vector<WayPoint> >& paths, const bool& bFindBranches = true, const bool bDirectionBased = false);

	void DeleteWaypoints(std::vector<WayPoint*>& wps);

	//PlanningInternalParams m_Params;
};

}


