/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OP_PERCEPTION_SIMULATOR
#define OP_PERCEPTION_SIMULATOR

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>

#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include <geometry_msgs/PoseArray.h>

#define OBJECT_KEEP_TIME 1
#define POINT_CLOUD_ADDTIONAL_ERR_NUM 25
#define CONTOUR_DISTANCE_ERROR 0.5

namespace PerceptionSimulatorNS
{

class DetectionCommandParams
{
public:
	int 	nSimuObjs;
	double 	errFactor;
	double  nPointsPerObj;

	DetectionCommandParams()
	{
		nSimuObjs = 3;
		errFactor = 0;
		nPointsPerObj = 50;
	}
};

class OpenPlannerSimulatorPerception
{
protected:
	ros::NodeHandle nh;
	timespec m_Timer;
	DetectionCommandParams m_DecParams;

	autoware_msgs::CloudCluster m_SimulatedCluter;

	autoware_msgs::CloudClusterArray m_ObjClustersArray;
	autoware_msgs::CloudClusterArray m_AllObjClustersArray;
	bool m_bSetSimulatedObj;
	std::vector<std::pair<int, double> > m_keepTime;

	ros::Publisher pub_DetectedObjects;

	// define subscribers.
	std::vector<ros::Subscriber> sub_objs;
	ros::Subscriber sub_simulated_obstacle_pose_rviz;


	// Callback function for subscriber.
	void callbackGetSimuData(const geometry_msgs::PoseArray &msg);
	void callbackGetRvizPoint(const geometry_msgs::PointStampedConstPtr& msg);

public:
	OpenPlannerSimulatorPerception();
	virtual ~OpenPlannerSimulatorPerception();
	autoware_msgs::CloudCluster GenerateSimulatedObstacleCluster(const double& x_rand, const double& y_rand, const double& z_rand, const int& nPoints, const geometry_msgs::Pose& centerPose);

	void MainLoop();
};

}

#endif  // OP_PERCEPTION_SIMULATOR
