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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include "points2image/PointsImage.h"
#include "calibration_camera_lidar/projection_matrix.h"
//#include "points2image/CameraExtrinsic.h"

#include <points_image.hpp>

#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

static cv::Mat cameraExtrinsicMat;
static cv::Mat cameraMat;
static cv::Mat distCoeff;
static cv::Size imageSize;

static ros::Publisher pub;

static void projection_callback(const calibration_camera_lidar::projection_matrix& msg)
{
	cameraExtrinsicMat = cv::Mat(4,4,CV_64F);
	for (int row=0; row<4; row++) {
		for (int col=0; col<4; col++) {
			cameraExtrinsicMat.at<double>(row, col) = msg.projection_matrix[row * 4 + col];
		}
	}
}

static void intrinsic_callback(const sensor_msgs::CameraInfo& msg)
{
	imageSize.height = msg.height;
	imageSize.width = msg.width;

	cameraMat = cv::Mat(3,3, CV_64F);
	for (int row=0; row<3; row++) {
		for (int col=0; col<3; col++) {
			cameraMat.at<double>(row, col) = msg.K[row * 3 + col];
		}
	}

	distCoeff = cv::Mat(1,5,CV_64F);
	for (int col=0; col<5; col++) {
		distCoeff.at<double>(col) = msg.D[col];
	}
}

static void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	if (cameraExtrinsicMat.empty() || cameraMat.empty() || distCoeff.empty() || imageSize.height == 0 || imageSize.width == 0)
	{
		ROS_INFO("Looks like /camera/camera_info or /projection_matrix are not being published.. Please check that both are running..");
		return;
	}

	points2image::PointsImage pub_msg
		= pointcloud2_to_image(msg, cameraExtrinsicMat, cameraMat,
				       distCoeff, imageSize);
	pub.publish(pub_msg);

	/*points2image::CameraExtrinsic cpub_msg
		= pointcloud2_to_3d_calibration(msg, cameraExtrinsicMat);
	cpub.publish(cpub_msg);*/
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "points2image");
	ros::NodeHandle n;

	/*if(argc < 2){
		std::cout<<"Need calibration filename as the first parameter.";
		return 0;
	}*/

	/*cv::FileStorage fs(argv[1], cv::FileStorage::READ);
	if(!fs.isOpened()){
		std::cout<<"Invalid calibration filename.";
		return 0;
	}*/

	/*fs[CAMERAEXTRINSICMAT] >> cameraExtrinsicMat;
	fs[CAMERAMAT] >> cameraMat;
	fs[DISTCOEFF] >> distCoeff;
	fs[IMAGESIZE] >> imageSize;*/
	//imageSize.width = IMAGE_WIDTH;
	//imageSize.height = IMAGE_HEIGHT;

	pub = n.advertise<points2image::PointsImage>("points_image", 10);
	//cpub = n.advertise<points2image::CameraExtrinsic>("threeD_calibration", 1);
	ros::NodeHandle private_nh("~");

	std::string points_topic;
	if (private_nh.getParam("points_node", points_topic))
	{
		ROS_INFO("Setting points node to %s", points_topic.c_str());
	}
	else
	{
		ROS_INFO("No points node received, defaulting to points_raw, you can use _points_node:=YOUR_TOPIC");
		points_topic = "points_raw";
	}

	ros::Subscriber sub = n.subscribe(points_topic, 1, callback);
	ros::Subscriber projection = n.subscribe("projection_matrix", 1, projection_callback);
	ros::Subscriber intrinsic = n.subscribe("camera/camera_info", 1, intrinsic_callback);

	ros::spin();
	return 0;
}
