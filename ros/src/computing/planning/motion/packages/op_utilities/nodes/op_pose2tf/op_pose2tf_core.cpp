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

#include "op_pose2tf_core.h"
#include "math.h"
#include <std_msgs/Empty.h>


namespace PoseTFNS
{

PoseToTF::PoseToTF()
{
	ros::NodeHandle _nh("~");

	std::string pose_topic_name;
	_nh.getParam("pose_topic" , pose_topic_name);

	sub_ndt_pose = nh.subscribe(pose_topic_name, 10, &PoseToTF::callbackGetPose, this);
	pub_reset_time = nh.advertise<std_msgs::Empty>("/reset_time", 1);

	std::cout << "PoseToTF initialized successfully " << std::endl;
}

PoseToTF::~PoseToTF()
{
}

void PoseToTF::callbackGetPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	tf::Quaternion bt_q;
	tf::quaternionMsgToTF(msg->pose.orientation, bt_q);

	static tf::TransformBroadcaster map_base_link_broadcaster;
	geometry_msgs::TransformStamped base_link_trans;
	base_link_trans.header.stamp = ros::Time::now();
	base_link_trans.header.frame_id = "/map";
	base_link_trans.child_frame_id = "/base_link";
	base_link_trans.transform.translation.x = msg->pose.position.x;
	base_link_trans.transform.translation.y = msg->pose.position.y;
	base_link_trans.transform.translation.z = msg->pose.position.z;
	base_link_trans.transform.rotation = msg->pose.orientation;

	// send the transform
	map_base_link_broadcaster.sendTransform(base_link_trans);

	//std::cout << "Publish TF !! "  << std::endl;
}

void PoseToTF::MainLoop()
{

	std_msgs::Empty msg;
	pub_reset_time.publish(msg);
	ros::spin();
}

}
