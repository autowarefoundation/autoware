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
#include "points2image/PointsImage.h"
#include "points2image/CameraExtrinsic.h"

#include <points_image.hpp>

#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

#define IMAGE_WIDTH 800
#define IMAGE_HEIGHT 640

static cv::Mat cameraExtrinsicMat;
static cv::Mat cameraMat;
static cv::Mat distCoeff;
static cv::Size imageSize;

static ros::Publisher pub;
static ros::Publisher cpub;

static void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	points2image::PointsImage pub_msg
		= pointcloud2_to_image(msg, cameraExtrinsicMat, cameraMat,
				       distCoeff, imageSize);
	pub.publish(pub_msg);

	points2image::CameraExtrinsic cpub_msg
		= pointcloud2_to_3d_calibration(msg, cameraExtrinsicMat);
	cpub.publish(cpub_msg);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "points2image");
	ros::NodeHandle n;

	if(argc < 2){
		std::cout<<"Need calibration filename as the first parameter.";
		return 0;
	}

	cv::FileStorage fs(argv[1], cv::FileStorage::READ);
	if(!fs.isOpened()){
		std::cout<<"Invalid calibration filename.";
		return 0;
	}

	fs[CAMERAEXTRINSICMAT] >> cameraExtrinsicMat;
	fs[CAMERAMAT] >> cameraMat;
	fs[DISTCOEFF] >> distCoeff;
	fs[IMAGESIZE] >> imageSize;
	//imageSize.width = IMAGE_WIDTH;
	//imageSize.height = IMAGE_HEIGHT;

	pub = n.advertise<points2image::PointsImage>("points_image", 10);
	cpub = n.advertise<points2image::CameraExtrinsic>("threeD_calibration", 1);
	ros::Subscriber sub = n.subscribe("velodyne_points", 1, callback);

	ros::spin();
	return 0;
}
