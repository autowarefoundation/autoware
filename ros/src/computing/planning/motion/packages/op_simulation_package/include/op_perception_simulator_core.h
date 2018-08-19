/*
// *  Copyright (c) 2018, Nagoya University
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
