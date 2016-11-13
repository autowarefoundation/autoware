/*
 * PlannerXInterface.h
 *
 *  Created on: Jul 11, 2016
 *      Author: ai-driver
 */

#ifndef PLANNERXINTERFACE_H_
#define PLANNERXINTERFACE_H_

#include <ros/ros.h>
#include <cv_tracker/obj_label.h>
#include <runtime_manager/traffic_light.h>

#include <map_file/PointClassArray.h>
#include <map_file/LaneArray.h>
#include <map_file/NodeArray.h>
#include <map_file/StopLineArray.h>
#include <map_file/DTLaneArray.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <lidar_tracker/centroids.h>
#include <lidar_tracker/CloudCluster.h>
#include <lidar_tracker/CloudClusterArray.h>

#include "waypoint_follower/libwaypoint_follower.h"
#include "waypoint_follower/LaneArray.h"
#include <visualization_msgs/MarkerArray.h>
#include "geo_pos_conv.hh"

#include "RosHelpers.h"


namespace PlannerXNS
{

class PlannerX_Interface
{
public:
	PlannerX_Interface();
	virtual ~PlannerX_Interface();
	static PlannerX_Interface* CreatePlannerInstance(const std::string& plannerName);

	virtual void UpdateVehicleInfo(const double& width, const double& length, const double& wheelBase, const double& maxSteerAngle, const double& turningRadius) = 0;

	virtual void UpdatePlanningParams(const AutowarePlanningParams& planningParams) = 0;

	virtual void UpdateRoadMap(const AutowareRoadNetwork& map) = 0;

	virtual bool LoadRoadMap(const std::string& mapFilePath, const bool& bKML_Map, visualization_msgs::MarkerArray& mapToVisualize) = 0;

	virtual void UpdateOriginTransformationPoint(const geometry_msgs::Pose& originPoint) = 0;

	virtual void UpdateGlobalGoalPosition(const geometry_msgs::Pose& goalPose) = 0;

	virtual void UpdatePredefinedPath(const std::vector<int>& predefinedPath) = 0;

	virtual bool GeneratePlan(const geometry_msgs::Pose& currentPose, const jsk_recognition_msgs::BoundingBoxArray& detectedObstacles,
			const lidar_tracker::CloudClusterArray& clusters,
			const runtime_manager::traffic_light& trafficLight, const AutowareVehicleState& carState,
			AutowareBehaviorState& behaviorState, visualization_msgs::MarkerArray& pathToVisualize,
			waypoint_follower::LaneArray& pathToFollow,
			jsk_recognition_msgs::BoundingBoxArray& trackedObjects,
			visualization_msgs::MarkerArray& detectedPolygons,
			const bool& bEmergencyStop, const bool& bGreenLight, const bool& bOutsideControl,
			const waypoint_follower::lane& aStarPath, geometry_msgs::PoseStamped& startPoint,
			geometry_msgs::PoseStamped& goalPoint, bool& bExternalPlanning) = 0;

protected:
	bool bMap;
	bool bGoal;
	bool bOriginPoint;
	bool bPredefinedPath;
};

} /* namespace PlannerHNS */

#endif /* PLANNERXINTERFACE_H_ */
