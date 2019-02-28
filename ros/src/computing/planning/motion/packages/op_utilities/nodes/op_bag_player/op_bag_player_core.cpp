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
#include "op_bag_player_core.h"
#include <sstream>
#include <algorithm>
#include "op_utility/UtilityH.h"
#include "op_utility/DataRW.h"
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <sys/termios.h>
#include <boost/foreach.hpp>

#ifndef foreach
#define foreach BOOST_FOREACH
#endif

namespace OP_TESTING_NS
{

using namespace std;

TestingUI::TestingUI()
{
	m_bStepDone = false;
	m_PlayMode = PLAY_PAUSE;
	m_TestMode = SIMULATION_MODE;
	m_bBagOpen = false;
	m_bStepForward = false;
	m_bPredStepForward = false;
	m_bGenerateSignal = false;

	m_VehicleTargetStateAccelerator = 0;
	m_VehicleTargetStateBrake = 0;
}

TestingUI::~TestingUI()
{
}

void TestingUI::DrawSimu()
{
	glPushMatrix();
//	std::ostringstream str_out ;
//	str_out <<  "Frame Number:" << 1;
//	DrawingHelpers::DrawString(-2, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)m_ReadingInfo.c_str());
//	DrawingHelpers::DrawString(-2, -2, GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
	glPopMatrix();

	ros::Rate loop_rate(100);

	if(ros::ok())
	{
		if(m_TestMode == SIMULATION_MODE)
		{
			SimulationModeMainLoop();
		}
		else if (m_TestMode == ROSBAG_MODE)
		{
			BagReaderModeMainLoop();
			std::ostringstream str_out_pose,  str_out_image, str_out_cloud;
			int _sec=0;
			int _nsec=0;
			int iFrame = 0;
			int nFrames = 0;

			m_PoseReader.GetReadingInfo(_sec, _nsec, iFrame, nFrames);
			str_out_pose <<"* Pose : " << _sec << " (" << iFrame << "/" << nFrames << ")";

			m_CloudReader.GetReadingInfo(_sec, _nsec, iFrame, nFrames);
			str_out_cloud <<"* Cloud: " << _sec << " (" << iFrame << "/" << nFrames << ")";

			m_ImageReader.GetReadingInfo(_sec, _nsec, iFrame, nFrames);
			str_out_image <<"* Image: " << _sec << " (" << iFrame << "/" << nFrames << ")";

			glPushMatrix();
			DrawingHelpers::DrawString(-3, 2, GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out_pose.str().c_str());
			DrawingHelpers::DrawString(-3, 1, GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out_cloud.str().c_str());
			DrawingHelpers::DrawString(-3, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out_image.str().c_str());
			glPopMatrix();
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void TestingUI::SimulationModeMainLoop()
{
	if(m_bPredStepForward)
	{
		geometry_msgs::TwistStamped s_signal;
		s_signal.header.frame_id = "velodyne";
		s_signal.header.stamp = ros::Time();
		s_signal.twist.linear.x = 1;
		pub_PredStepSignal.publish(s_signal);
		m_bPredStepForward = false;
	}

	if(m_bStepForward)
	{
		geometry_msgs::TwistStamped s_signal;
		s_signal.header.frame_id = "velodyne";
		s_signal.header.stamp = ros::Time();
		s_signal.twist.linear.x = 1;
		pub_SimuStepSignal.publish(s_signal);
		m_bStepForward = false;
		m_bPredStepForward = true;
	}

	if(m_bGenerateSignal)
	{
		geometry_msgs::TwistStamped g_signal;
		g_signal.header.frame_id = "velodyne";
		g_signal.header.stamp = ros::Time();
		g_signal.twist.linear.x = 1;
		pub_SimuGenSignal.publish(g_signal);
		m_bGenerateSignal = false;
	}
}

bool TestingUI::OpenROSBag()
{
    try
    {
    	cout << "Openning ROSbag File: " << m_BagParams.fileName << endl;
    	m_bag.open(m_BagParams.fileName, rosbag::bagmode::Read);

    	m_CloudReader.InitPlayer(m_bag, m_BagParams.lidarTopic);
    	m_ImageReader.InitPlayer(m_bag, m_BagParams.imageTopic);
    	m_PoseReader.InitPlayer(m_bag, m_BagParams.poseTopic);

		return true;
    }
    catch (rosbag::BagIOException& e)
    {
    	std::cout << "Can't Open ROSbaf with path: " << m_BagParams.fileName << std::endl;
        ROS_ERROR_STREAM(e.what());
        return false;
    }
}

void TestingUI::BagReaderModeMainLoop()
{
	if(m_bBagOpen)
	{
		bool bFreshMsg = false;

		if(m_PlayMode == PLAY_FORWARD)
		{
			bFreshMsg = ReadNextFrame();
		}
		else if(m_PlayMode == PLAY_STEP_FORWARD)
		{
			if(!m_bStepDone)
			{
				bFreshMsg = ReadNextFrame();
				m_bStepDone = true;
			}
		}
		else if(m_PlayMode == PLAY_STEP_BACKWARD)
		{
			if(!m_bStepDone)
			{
				bFreshMsg = ReadPrevFrame();
				m_bStepDone = true;
			}
		}

		if(bFreshMsg && m_pLatestCloud != NULL)
		{
			sensor_msgs::PointCloud2 cloud = *m_pLatestCloud;
			cloud.header.stamp = ros::Time::now();
			pub_Point_Raw.publish(cloud);
		}

		if(bFreshMsg && m_pLatestImage != NULL)
		{
			sensor_msgs::Image img = *m_pLatestImage;
			img.header.stamp = ros::Time().now();
			pub_Image_Raw.publish(img);
		}

		if(bFreshMsg && m_pLatestPose != NULL)
		{
			geometry_msgs::PoseStamped pos = *m_pLatestPose;
			pos.header.stamp = ros::Time::now();
			pub_NDT_pose.publish(pos);
		}
	}

}

bool TestingUI::ReadNextFrame()
{
	geometry_msgs::PoseStampedPtr pPose;

	bool bPose = false, bCloud = false, bImage = false;

	 bPose = m_PoseReader.ReadNext(pPose, NULL);
	if(pPose != NULL)
		m_pLatestPose = pPose;

	if(m_pLatestPose != NULL)
	{
		bCloud = m_CloudReader.ReadNext(m_pLatestCloud, &m_pLatestPose->header.stamp);
		while(m_CloudReader.ReadNext(m_pLatestCloud, &m_pLatestPose->header.stamp))
		{
		}

		bImage = m_ImageReader.ReadNext(m_pLatestImage, &m_pLatestPose->header.stamp);
		while(!m_ImageReader.ReadNext(m_pLatestImage, &m_pLatestPose->header.stamp))
		{

		}
	}

	return bPose || bCloud || bImage;
}

bool TestingUI::ReadPrevFrame()
{
	geometry_msgs::PoseStampedPtr pPose;

	bool bPose = false, bCloud = false, bImage = false;

	 bPose = m_PoseReader.ReadPrev(pPose, NULL);
	if(pPose != NULL)
		m_pLatestPose = pPose;

	if(m_pLatestPose != NULL)
	{
		bCloud = m_CloudReader.ReadPrev(m_pLatestCloud, &m_pLatestPose->header.stamp);
		bImage = m_ImageReader.ReadPrev(m_pLatestImage, &m_pLatestPose->header.stamp);
	}

	return bPose || bCloud || bImage;
}

void TestingUI::InitNode(const BagReaderParams& params, const int& mode)
{
	m_BagParams = params;
	switch(mode)
	{
	case 0:
		m_TestMode = SIMULATION_MODE;
		break;
	case 1:
		m_TestMode = ROSBAG_MODE;
		break;
	case 2:
		m_TestMode = LIVE_MODE;
		break;
	default:
		break;
	}

	ros::NodeHandle nh;
	if(m_TestMode == ROSBAG_MODE)
	{
		m_bBagOpen = OpenROSBag();

		pub_Point_Raw		= nh.advertise<sensor_msgs::PointCloud2>(m_BagParams.lidarTopic_pub, 10);
		pub_Image_Raw		= nh.advertise<sensor_msgs::Image>(m_BagParams.imageTopic_pub, 10);
		pub_NDT_pose		= nh.advertise<geometry_msgs::PoseStamped>(m_BagParams.poseTopic_pub, 10);
	}
	else if(m_TestMode == SIMULATION_MODE)
	{
		pub_SimuStepSignal 		= nh.advertise<geometry_msgs::TwistStamped>("simu_step_signal", 1);
		pub_PredStepSignal 		= nh.advertise<geometry_msgs::TwistStamped>("pred_step_signal", 1);
		pub_SimuGenSignal		= nh.advertise<geometry_msgs::TwistStamped>("simu_generate_signal", 1);
	}
}

void TestingUI::OnKeyboardPress(const int& sKey, const unsigned char& key)
{
	//std::cout << "key" << std::endl;

	if(m_TestMode == SIMULATION_MODE)
	{
		switch(key)
		{
		case 's':
		{
			m_bStepForward = true;
		}
		break;
		case 'g':
		{
			m_bGenerateSignal = true;
		}
		break;
		default:
			break;

		}
	}
	else if(m_TestMode == ROSBAG_MODE)
	{
		switch(key)
		{
		case 32:
		{
			if(m_PlayMode == PLAY_PAUSE || m_PlayMode == PLAY_STEP_FORWARD || m_PlayMode == PLAY_STEP_BACKWARD)
				m_PlayMode = PLAY_FORWARD;
			else if(m_PlayMode == PLAY_FORWARD)
				m_PlayMode = PLAY_PAUSE;
		}
		break;
		default:
			break;
		}

		switch (sKey)
		{
		case 101: // Up
		{
			m_PlayMode = PLAY_STEP_FORWARD;
			m_bStepDone = false;
		}
		break;
		case 103: //Down
		{
			m_PlayMode = PLAY_STEP_BACKWARD;
			m_bStepDone = false;
		}
		break;
		case 102: //Right
		{
		}
		break;
		case 100: //Left
		{
		}
		break;
		default:
			break;
		}
	}
}

} /* namespace Graphics */
