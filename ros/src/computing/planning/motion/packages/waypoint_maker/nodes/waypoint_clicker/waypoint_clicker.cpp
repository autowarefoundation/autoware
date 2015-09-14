/*
 *  Copyright (c) 2015, Nagoya University
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

#include <fstream>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

static tf::StampedTransform transform;
static bool output_first;

static int sub_point_queue_size;

static double velocity;		// km/h
static std::string output_file;

static void write_clicked_point(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	if (output_first) {
		std::ofstream ofs(output_file.c_str());
		ofs << std::fixed << msg->pose.pose.position.x + transform.getOrigin().x() << ","
		    << std::fixed << msg->pose.pose.position.y + transform.getOrigin().y() << ","
		    << std::fixed << msg->pose.pose.position.z + transform.getOrigin().z() << std::endl;
	} else {
		std::ofstream ofs(output_file.c_str(), std::ios_base::app);
		ofs << std::fixed << msg->pose.pose.position.x + transform.getOrigin().x() << ","
		    << std::fixed << msg->pose.pose.position.y + transform.getOrigin().y() << ","
		    << std::fixed << msg->pose.pose.position.z + transform.getOrigin().z() << ","
		    << std::fixed << velocity << std::endl;
	}

	if (output_first)
		output_first = false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_clicker");

	ros::NodeHandle n;
	n.param<int>("/waypoint_clicker/sub_point_queue_size", sub_point_queue_size, 1);
	n.param<double>("/waypoint_clicker/velocity", velocity, 40);
	n.param<std::string>("/waypoint_clicker/output_file", output_file, "/tmp/lane_waypoint.csv");

	ros::Subscriber sub_point = n.subscribe("/initialpose", sub_point_queue_size, write_clicked_point);

	tf::TransformListener listener;
	try {
		ros::Time zero = ros::Time(0);
		listener.waitForTransform("map", "world", zero, ros::Duration(10));
		listener.lookupTransform("map", "world", zero, transform);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
	}
	output_first = true;

	ros::spin();

	return 0;
}
