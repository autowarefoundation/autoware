/*
// *  Copyright (c) 2015, Nagoya University
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

#ifndef PLANNER_X_CORE_H
#define PLANNER_X_CORE_H

// ROS includes
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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>



class PlannerX
{
public:
	//bool m_bReadyToPlan;
	timespec m_Timer;
	int m_counter;
	int m_frequency;
	geometry_msgs::Pose m_InitPos;
	geometry_msgs::Pose m_GoalPos;


	map_file::PointClassArray points;
	std::vector<map_file::Lane> lanes;
	std::vector<map_file::Node> nodes;
	std::vector<map_file::StopLine> stoplines;
	std::vector<map_file::DTLane> dtlanes;

	ros::Publisher m_PositionPublisher;
	ros::Publisher m_PathPublisherRviz;
	ros::Publisher m_PathPublisher;

	ros::NodeHandle* pNodeHandle;


  // Constructor.
  PlannerX(ros::NodeHandle* pnh);

  // Destructor.
  ~PlannerX();

  // Callback function for subscriber.
  void callbackSimuGoalPose(const geometry_msgs::PoseStamped &msg);
  void callbackSimuInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input);
  void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
  void callbackFromLightColor(const runtime_manager::traffic_light& msg);

  void callbackFromObjCar(const cv_tracker::obj_label& msg);

  void callbackGetVMPoints(const map_file::PointClassArray& msg);
  void callbackGetVMLanes(const map_file::LaneArray& msg);
  void callbackGetVMNodes(const map_file::NodeArray& msg);
  void callbackGetVMStopLines(const map_file::StopLineArray& msg);
  void callbackGetVMCenterLines(const map_file::DTLaneArray& msg);

};

#endif  // PLANNER_X_H
