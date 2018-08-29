/*
 *  Copyright (c) 2018, Nagoya University
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
