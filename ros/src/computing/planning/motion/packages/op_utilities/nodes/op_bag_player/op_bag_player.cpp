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
