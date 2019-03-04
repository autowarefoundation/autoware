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

#ifndef OP_POSETF
#define OP_POSETF

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

namespace PoseTFNS
{

class PoseToTF
{

protected:

	ros::Subscriber sub_ndt_pose;
	ros::Publisher pub_reset_time;
	ros::NodeHandle nh;

	void callbackGetPose(const geometry_msgs::PoseStampedConstPtr &msg);

public:
	PoseToTF();
	virtual ~PoseToTF();
	void MainLoop();
};

}

#endif  // OP_POSETF
