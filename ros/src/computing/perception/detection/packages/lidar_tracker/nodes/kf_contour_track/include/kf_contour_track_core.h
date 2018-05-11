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
#include "op_simu/SimpleTracker.h"

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
	SimulationNS::TRACKING_TYPE	trackingType; // 0 association only , 1 simple tracking, 2 contour based tracking

	PerceptionParams()
	{
		VehicleWidth =0;
		VehicleLength =0;
		DetectionRadius =0;
		MinObjSize =0;
		MaxObjSize =0;
		nQuarters = 0;
		PolygonRes = 0;
		trackingType = SimulationNS::SIMPLE_TRACKER;
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

	SimulationNS::SimpleTracker m_ObstacleTracking;

	//Visualization Section
	int m_nDummyObjPerRep;
	int m_nDetectedObjRepresentations;
	std::vector<visualization_msgs::MarkerArray> m_DetectedPolygonsDummy;
	std::vector<visualization_msgs::MarkerArray> m_DetectedPolygonsActual;
	visualization_msgs::MarkerArray m_DetectedPolygonsAllMarkers;

	visualization_msgs::MarkerArray m_DetectionCircles;

protected: //ROS messages (topics)
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_AllTrackedObjects;

	ros::Publisher pub_DetectedPolygonsRviz;
	ros::Publisher pub_TrackedObstaclesRviz;

	// define subscribers.
	ros::Subscriber sub_cloud_clusters;
	ros::Subscriber sub_current_pose ;


protected: // Callback function for subscriber.
	void callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr &msg);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);

protected: //Helper Functions
	void VisualizeLocalTracking();

public:
  ContourTracker();
  ~ContourTracker();
  void MainLoop();
};

}

#endif  // KF_CONTOUR_TRACKER_CORE
