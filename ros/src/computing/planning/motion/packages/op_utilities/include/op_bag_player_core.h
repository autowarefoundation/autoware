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

	//Rosbag reader
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


	bool OpenRosBag();
	void BagReaderModeMainLoop();
	bool ReadNextFrame();
	bool ReadPrevFrame();
};

} /* namespace Graphics */

#endif /* AlternativeVisualizer_H_ */
