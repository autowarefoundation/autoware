/*
// *  Copyright (c) 2016, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef KF_CONTOUR_TRACKER_CORE
#define KF_CONTOUR_TRACKER_CORE

// ROS includes
#include <ros/ros.h>
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/MatrixOperations.h"
#include "SimpleTracker.h"
#include "PolygonGenerator.h"

#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

namespace ContourTrackerNS
{

class PerceptionParams
{
public:

	double 	VehicleWidth;
	double 	VehicleLength;
	double 	DetectionRadius;
	double 	MinObjSize;
	double 	MaxObjSize;
	double  nQuarters;
	double 	PolygonRes;
	TRACKING_TYPE	trackingType; // 0 association only , 1 simple tracking, 2 contour based tracking
	bool    bEnableSimulation;
	bool 	bEnableStepByStep;
	bool 	bEnableLogging;
	bool bEnableTTC;

	PerceptionParams()
	{
		VehicleWidth =0;
		VehicleLength =0;
		DetectionRadius =0;
		MinObjSize =0;
		MaxObjSize =0;
		nQuarters = 0;
		PolygonRes = 0;
		trackingType = SIMPLE_TRACKER;
		bEnableStepByStep = false;
		bEnableSimulation = false;
		bEnableLogging = false;
		bEnableTTC = false;
	}
};

class ContourTracker
{
protected:
	std::vector<PlannerHNS::DetectedObject> m_OriginalClusters;
	autoware_msgs::DetectedObjectArray m_OutPutResults;
	bool bNewClusters;
	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;
	PerceptionParams m_Params;
	SimpleTracker m_ObstacleTracking;

	//Visualization Section
	int m_nDummyObjPerRep;
	int m_nDetectedObjRepresentations;
	std::vector<visualization_msgs::MarkerArray> m_DetectedPolygonsDummy;
	std::vector<visualization_msgs::MarkerArray> m_DetectedPolygonsActual;
	visualization_msgs::MarkerArray m_DetectedPolygonsAllMarkers;
	visualization_msgs::MarkerArray m_DetectionCircles;

	std::vector<visualization_msgs::MarkerArray> m_MatchingInfoDummy;
	std::vector<visualization_msgs::MarkerArray> m_MatchingInfoActual;


	visualization_msgs::MarkerArray m_TTC_Path;
	visualization_msgs::Marker m_TTC_Info;

	std::vector<std::string>    m_LogData;
	PlannerHNS::MAP_SOURCE_TYPE m_MapType;
	std::string m_MapPath;
	PlannerHNS::RoadNetwork m_Map;
	bool bMap;
	bool bVectorMapCheck;
	double m_MapFilterDistance;

	std::vector<PlannerHNS::Lane*> m_ClosestLanesList;

	int m_nOriginalPoints;
	int m_nContourPoints;
	double m_FilteringTime;
	double m_PolyEstimationTime;
	double m_tracking_time;
	double m_dt;
	struct timespec  m_loop_timer;

	//ROS subscribers
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_AllTrackedObjects;

	ros::Publisher pub_DetectedPolygonsRviz;
	ros::Publisher pub_TrackedObstaclesRviz;
	ros::Publisher pub_TTC_PathRviz;

	// define subscribers.
	ros::Subscriber sub_cloud_clusters;
	ros::Subscriber sub_current_pose ;


	// Callback function for subscriber.
	void callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr &msg);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);

	//Helper Functions
	void VisualizeLocalTracking();
	void ImportCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr& msg, std::vector<PlannerHNS::DetectedObject>& originalClusters);
	bool IsCar(const PlannerHNS::DetectedObject& obj, const PlannerHNS::WayPoint& currState, PlannerHNS::RoadNetwork& map);
	void CalculateTTC(const std::vector<PlannerHNS::DetectedObject>& objs, const PlannerHNS::WayPoint& currState, PlannerHNS::RoadNetwork& map);
	void GetFrontTrajectories(std::vector<PlannerHNS::Lane*>& lanes, const PlannerHNS::WayPoint& currState, const double& max_distance, std::vector<std::vector<PlannerHNS::WayPoint> >& trajectories);
	void ReadNodeParams();
	void ReadCommonParams();
	void LogAndSend();

public:
  ContourTracker();
  ~ContourTracker();
  void MainLoop();
};

}

#endif  // KF_CONTOUR_TRACKER_CORE
