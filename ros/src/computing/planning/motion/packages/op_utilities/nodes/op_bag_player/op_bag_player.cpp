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

#include <ros/ros.h>
#include "op_bag_player_core.h"
#include "MainWindowWrapper.h"

using namespace  OP_TESTING_NS;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "op_bag_player");

	WindowParams pms;
	DisplayParams dpms;
	dpms.centerRotX = 0;
	dpms.centerRotY = 0;
	dpms.translateX = 0;
	dpms.translateY = 0;
	ros::NodeHandle nh;

	nh.getParam("/op_bag_player/width", pms.w);
	nh.getParam("/op_bag_player/height", pms.h);
	nh.getParam("/op_bag_player/info_ratio", pms.info_ratio);

	BagReaderParams bag_params;

	nh.getParam("/op_bag_player/rosbag_file", bag_params.fileName);

	nh.getParam("/op_bag_player/lidar_topic_bag", bag_params.lidarTopic);
	nh.getParam("/op_bag_player/pose_topic_bag", bag_params.poseTopic);
	nh.getParam("/op_bag_player/image_topic_bag", bag_params.imageTopic);

	nh.getParam("/op_bag_player/lidar_topic_pub", bag_params.lidarTopic_pub);
	nh.getParam("/op_bag_player/pose_topic_pub", bag_params.poseTopic_pub);
	nh.getParam("/op_bag_player/image_topic_pub", bag_params.imageTopic_pub);

	int test_mode = 0;
	nh.getParam("/op_bag_player/testing_mode", test_mode);

	DrawObjBase* pSimulator =  0;
	pSimulator = new TestingUI();
	((TestingUI*)pSimulator)->InitNode(bag_params, test_mode);

	MainWindowWrapper wrapper(pSimulator);
	wrapper.UpdateParams(pms, dpms);
	wrapper.InitOpenGLWindow(argc, argv);

	return 0;
}
