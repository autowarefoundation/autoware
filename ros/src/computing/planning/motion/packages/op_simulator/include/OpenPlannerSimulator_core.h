/*
// *  Copyright (c) 2017, Nagoya University
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

#ifndef OpenPlannerSimulator_CORE_H
#define OpenPlannerSimulator_CORE_H

#define _ENABLE_ZMP_LIBRARY_LINK

// ROS includes
#include <ros/ros.h>
#include "autoware_msgs/obj_label.h"

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include "op_simu/TrajectoryFollower.h"
#include "op_planner/LocalPlannerH.h"
#include "op_planner/PlannerH.h"
#include "op_planner/MappingHelpers.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "op_simu/SimpleTracker.h"

namespace OpenPlannerSimulatorNS
{

enum MAP_SOURCE_TYPE{MAP_AUTOWARE, MAP_FOLDER, MAP_KML_FILE};

class SimuCommandParams
{
public:
	int id;
	std::string 	KmlMapPath;
	std::string 	strID;
	std::string 	meshPath;
	std::string 	logPath;
	MAP_SOURCE_TYPE	mapSource;
	bool			bRandomStart;
	bool 			bLooper;
	PlannerHNS::WayPoint startPose;
	std_msgs::ColorRGBA modelColor;

	SimuCommandParams()
	{
		id = 0;
		bRandomStart = false;
		bLooper = false;
		mapSource = MAP_KML_FILE;
		modelColor.a = 1;
		modelColor.b = 1;
		modelColor.r = 1;
		modelColor.g = 1;
	}
};

class OpenPlannerSimulator
{
protected:
	timespec m_PlanningTimer;
	geometry_msgs::Pose m_OriginPos;

	bool 						m_bMap;
	PlannerHNS::RoadNetwork		m_Map;
	PlannerHNS::PlannerH		m_GlobalPlanner;
	PlannerHNS::LocalPlannerH 	m_LocalPlanner;
	SimulationNS::TrajectoryFollower m_PredControl;
	SimulationNS::SimpleTracker m_ObstacleTracking;
	std::vector<PlannerHNS::DetectedObject> m_OriginalClusters;
	std::vector<PlannerHNS::DetectedObject> m_TrackedClusters;

	bool bInitPos;
	bool bNewClusters;


	SimuCommandParams m_SimParams;
	PlannerHNS::CAR_BASIC_INFO m_CarInfo;
	PlannerHNS::ControllerParams m_ControlParams;
	PlannerHNS::PlanningParams m_PlanningParams;

	ros::Publisher pub_SafetyBorderRviz;
	ros::Publisher pub_SimuBoxPose;
	//ros::Publisher pub_SimuVelocity;
	ros::Publisher pub_CurrPoseRviz;
	ros::Publisher pub_LocalTrajectoriesRviz;
	ros::Publisher pub_BehaviorStateRviz;

	// define subscribers.
	ros::Subscriber sub_initialpose;
	ros::Subscriber sub_cloudClusters;

	// Callback function for subscriber.
	void callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
	void callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr& msg);

public:
	OpenPlannerSimulator();

	virtual ~OpenPlannerSimulator();

	void PlannerMainLoop();

  void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);

  void ReadParamFromLaunchFile(PlannerHNS::CAR_BASIC_INFO& m_CarInfo,
		  PlannerHNS::ControllerParams& m_ControlParams);

  void displayFollowingInfo(const std::vector<PlannerHNS::GPSPoint>& safety_rect, PlannerHNS::WayPoint& curr_pose);
  void visualizePath(const std::vector<PlannerHNS::WayPoint>& path);

  void ConvertFromAutowareCloudClusterObstaclesToPlannerH(const PlannerHNS::WayPoint& currState, const PlannerHNS::CAR_BASIC_INFO& car_info,
  		const autoware_msgs::CloudClusterArray& clusters, std::vector<PlannerHNS::DetectedObject>& obstacles_list,
  		int& nOriginalPoints, int& nContourPoints);

  PlannerHNS::WayPoint GetRealCenter(const PlannerHNS::WayPoint& currState);

  void visualizeBehaviors();

  void SaveSimulationData();
  bool LoadSimulationData(PlannerHNS::WayPoint& start_p);
  void InitializeSimuCar(PlannerHNS::WayPoint start_pose);
};

}

#endif  // OpenPlannerSimulator_H
