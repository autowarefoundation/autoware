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

#ifndef OP_TESTING_CORE
#define OP_TESTING_CORE

#include <iostream>
#include "op_planner/RoadNetwork.h"
#include "DrawObjBase.h"
#include "DrawingHelpers.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <rosbag/bag.h>
#include "BagTopicPlayer.h"

namespace OP_TESTING_NS
{

class BagReaderParams
{
public:
	std::string fileName;
	std::string lidarTopic;
	std::string poseTopic;
	std::string imageTopic;

	std::string lidarTopic_pub;
	std::string poseTopic_pub;
	std::string imageTopic_pub;
};

enum TESTING_MODE {SIMULATION_MODE, ROSBAG_MODE, LIVE_MODE};
enum ROSBAG_PLAY_MODE {PLAY_FORWARD, PLAY_BACKWARD, PLAY_PAUSE, PLAY_STEP_FORWARD, PLAY_STEP_BACKWARD };

class TestingUI : public DrawObjBase
{
public:
	TestingUI();
	virtual ~TestingUI();
	void InitNode(const BagReaderParams& params, const int& mode);

	void DrawSimu();
	void OnKeyboardPress(const int& sKey, const unsigned char& key);
    void SimulationModeMainLoop();

    double m_VehicleTargetStateAccelerator;
    double m_VehicleTargetStateBrake;

    bool m_bStepForward;
    bool m_bPredStepForward;
    bool m_bGenerateSignal;

    ros::Publisher pub_SimuStepSignal;
    ros::Publisher pub_SimuGenSignal;
    ros::Publisher pub_PredStepSignal;
	TESTING_MODE m_TestMode;

	//ROSbag reader
private:
	BagReaderParams m_BagParams;
	rosbag::Bag m_bag;
	std::string m_ReadingInfo;

	bool m_bBagOpen;
	ROSBAG_PLAY_MODE m_PlayMode;
	bool m_bStepDone;

	geometry_msgs::PoseStampedPtr m_pLatestPose;
	sensor_msgs::PointCloud2Ptr m_pLatestCloud;
	sensor_msgs::ImagePtr m_pLatestImage;

	ros::Publisher pub_Point_Raw;
	ros::Publisher pub_Image_Raw;
	ros::Publisher pub_NDT_pose;

	UtilityHNS::BagTopicPlayer<sensor_msgs::PointCloud2> m_CloudReader;
	UtilityHNS::BagTopicPlayer<sensor_msgs::Image> m_ImageReader;
	UtilityHNS::BagTopicPlayer<geometry_msgs::PoseStamped> m_PoseReader;


	bool OpenROSBag();
	void BagReaderModeMainLoop();
	bool ReadNextFrame();
	bool ReadPrevFrame();
};

} /* namespace Graphics */

#endif /* AlternativeVisualizer_H_ */
