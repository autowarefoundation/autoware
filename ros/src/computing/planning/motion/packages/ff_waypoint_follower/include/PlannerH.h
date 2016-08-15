/*
 * PlannerH.h
 *
 *  Created on: May 14, 2016
 *      Author: hatem
 */

#include "RSPlanner.h"
#include "GridMap.h"

namespace PlannerHNS
{

enum PLANDIRECTION {MOVE_FORWARD_ONLY, MOVE_BACKWARD_ONLY, 	MOVE_FREE};
enum HeuristicConstrains {EUCLIDEAN, NEIGBORHOOD,DIRECTION };

class PlanningInternalParams
{
public:
	double 	pathDensity;
	double 	dataWeight;
	double 	smoothWeight;
	double 	tolerance;
	bool 	bOuterPath;

	double 	MoveCycleTime;
	double 	SpeedCurvatureCoefficient;
	int 	PlanSimulationResolution;

	double 	turning_radius;
	double 	wheel_base;
	double 	max_speed;
	double 	min_speed;
	double 	max_curvature;
	double 	max_steer;

	double car_width;
	double car_length;

	double ReedSheppSmoothnessFactor;

	PLANDIRECTION dir;
	HeuristicConstrains heuristics;

	PlanningInternalParams()
	{


		pathDensity = 0.25;
		dataWeight = 0.3;
		smoothWeight = 0.35;
		tolerance = 0.1;
		bOuterPath = 0;

		MoveCycleTime = 0.01;
		SpeedCurvatureCoefficient = 20;
		PlanSimulationResolution = 3;

		turning_radius = 3.2;
		wheel_base = 1.53;
		max_speed = 5;
		min_speed = -1;
		max_steer = 0.42;
		max_curvature  = 250;

		car_width = 1.2;
		car_length = 3.0;

		ReedSheppSmoothnessFactor = 12.0; //From 1 - 20

		dir = MOVE_FORWARD_ONLY;
		heuristics = EUCLIDEAN;
	}
};

class PlannerH
{
public:
	PlannerH(const PlanningInternalParams& params);
	virtual ~PlannerH(); 


	/**
	 * @brief Generates Trajectory using Reeds Shepp, this method will not try to avoid obstacles
	 * @param start: Start position for the trajectory (x,y,theta)
	 * @param goal:  Goal position (Destination point) (x,y,theta)
	 * @param map:  2d grid map, ( cost map or occupancy grid.
	 * @param generatedPath: pointer to return the smooth trajectory. - Better Not to use this
	 * @return path length
	 */
	double PlanUsingReedSheppWithObstacleDetection(const WayPoint& start, const WayPoint& goal, GridMap& map, std::vector<WayPoint>& genSmoothedPath);

	/**
	 * @brief Generates Trajectory using Reeds Shepp, this method will not try to avoid obstacles , but if there an obstacle on the trajectory function will fail. , also this function does not guaranteed to generate trajectories
	 * @param start: Start position for the trajectory (x,y,theta)
	 * @param goal:  Goal position (Destination point) (x,y,theta)
	 * @param generatedPath: pointer to return the smooth trajectory. - Better Not to use this
	 * @return path length
	 */
	double PlanUsingReedShepp(const WayPoint& start, const WayPoint& goal, std::vector<WayPoint>& generatedPath);


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
	void GenerateRunoffTrajectory(const std::vector<WayPoint>& referencePath, const WayPoint& carPos, const bool& bEnableLaneChange, const double& speed, const double& microPlanDistance,
				const double& maxSpeed,const double& minSpeed, const double&  carTipMargin, const double& rollInMargin,
				const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
				const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
				const double& SmoothTolerance, const double& speedProfileFactor, const bool& bHeadingSmooth, std::vector<std::vector<WayPoint> >& rollOutPaths);

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
	double PlanUsingDP(Lane* lane, const WayPoint& carPos,const WayPoint& goalPos,
			const WayPoint& prevWayPoint, const double& maxPlanningDistance,
			const std::vector<int>& globalPath, std::vector<WayPoint>& path);

	void DeleteWaypoints(std::vector<WayPoint*>& wps);

	PlanningInternalParams m_Params;
};

}


