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

/*
  This program requires ROS and Flycapture SDK installed
  Author: Abraham Monrroy (amonrroy@ertl.jp)
  Initial version 		2014-11-14
  Added signal handler 		2015-05-01
  Added CameraInfo msg 		2015-05-01
*/

#include <iostream>

#include <FlyCapture2.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <signal.h>

static volatile int running = 1;

static void signalHandler(int)
{
	running = 0;
	ros::shutdown();
}

void parseCameraInfo(const cv::Mat  &camMat,
                       const cv::Mat  &disCoeff,
                       const cv::Size &imgSize,
                       sensor_msgs::CameraInfo &msg)
{
	msg.header.frame_id = "camera";
	//  msg.header.stamp    = ros::Time::now();

	msg.height = imgSize.height;
	msg.width  = imgSize.width;

	for (int row=0; row<3; row++)
	{
		for (int col=0; col<3; col++)
		{
			msg.K[row * 3 + col] = camMat.at<double>(row, col);
		}
	}

	for (int row=0; row<3; row++)
	{
		for (int col=0; col<4; col++)
		{
			if (col == 3)
			{
				msg.P[row * 4 + col] = 0.0f;
			} else
			{
				msg.P[row * 4 + col] = camMat.at<double>(row, col);
			}
		}
	}

	for (int row=0; row<disCoeff.rows; row++)
	{
		for (int col=0; col<disCoeff.cols; col++)
		{
			msg.D.push_back(disCoeff.at<double>(row, col));
		}
	}
}


static void print_camera_info(FlyCapture2::CameraInfo* info)
{
	std::cout << "\n*** CAMERA INFORMATION ***\n"
		  << "\tSerial number       - " << info->serialNumber << "\n"
		  << "\tCamera model        - " << info->modelName << "\n"
		  << "\tCamera vendor       - " << info->vendorName << "\n"
		  << "\tSendor              - " << info->sensorInfo << "\n"
		  << "\tResolution          - " << info->sensorResolution << "\n"
		  << "\tFirmware version    - " << info->firmwareVersion << "\n"
		  << "\tFirmware build time - " << info->firmwareBuildTime
		  << std::endl;
}

static std::vector<FlyCapture2::Camera*>
initializeCameras(FlyCapture2::BusManager *bus_manger, int camera_num)
{
	// Connect to all detected cameras and attempt to set them to
	// a common video mode and frame rate

	std::vector<FlyCapture2::Camera*> cameras;
	for (int i = 0; i < camera_num; i++)
	{
		FlyCapture2::Camera *camera = new FlyCapture2::Camera();

		FlyCapture2::PGRGuid guid;
		FlyCapture2::Error error = bus_manger->GetCameraFromIndex(i, &guid);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			error.PrintErrorTrace();
			std::exit(-1);
		}

		error = camera->Connect( &guid );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			error.PrintErrorTrace();
			std::exit(-1);
		}

		FlyCapture2::EmbeddedImageInfo image_info;
		error = camera->GetEmbeddedImageInfo(&image_info);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			error.PrintErrorTrace();
			std::exit(-1);
		}

		image_info.timestamp.onOff = true;
		error = camera->SetEmbeddedImageInfo(&image_info);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			error.PrintErrorTrace();
			std::exit(-1);
		}

		// Get the camera information
		FlyCapture2::CameraInfo camera_info;
		error = camera->GetCameraInfo(&camera_info);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			error.PrintErrorTrace();
			std::exit(-1);
		}

		print_camera_info(&camera_info);
		cameras.push_back(camera);
	}

	return cameras;
}

static int getNumCameras(FlyCapture2::BusManager *bus_manager)
{
	unsigned int cameras;
	FlyCapture2::Error error = bus_manager->GetNumOfCameras(&cameras);
	if (error != FlyCapture2::PGRERROR_OK)
	{
		error.PrintErrorTrace();
		std::exit(-1);
	}

	std::cout << "Number of cameras detected: " << cameras << std::endl;

	if (cameras < 1)
	{
		std::cerr << "Error: This program requires at least 1 camera." << std::endl;
		std::exit(-1);
	}

	return static_cast<int>(cameras);
}

static void startCapture(std::vector<FlyCapture2::Camera*>& cameras)
{
	for (auto *camera : cameras)
	{
		FlyCapture2::Error error = camera->StartCapture();
		if (error != FlyCapture2::PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return;
		}
	}

	return;
}

void getMatricesFromFile(ros::NodeHandle nh, sensor_msgs::CameraInfo &camerainfo_msg)
{
	//////////////////CAMERA INFO/////////////////////////////////////////
	cv::Mat  cameraExtrinsicMat;
	cv::Mat  cameraMat;
	cv::Mat  distCoeff;
	cv::Size imageSize;
	std::string filename;

	if (nh.getParam("calibrationfile", filename) && filename!="")
	{
		ROS_INFO("Trying to parse calibrationfile :");
		ROS_INFO("> %s", filename.c_str());
	}
	else
	{
		ROS_INFO("No calibrationfile param was received");
		return;
	}

	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		ROS_INFO("Cannot open %s", filename.c_str());;
		return;
	}
	else
	{
		fs["CameraMat"] >> cameraMat;
		fs["DistCoeff"] >> distCoeff;
		fs["ImageSize"] >> imageSize;
	}
	parseCameraInfo(cameraMat, distCoeff, imageSize, camerainfo_msg);
}

int main(int argc, char **argv)
{
	////////////////POINT GREY CAMERA /////////////////////////////
	FlyCapture2::BusManager busMgr;

	int camera_num = getNumCameras(&busMgr);
	std::vector<FlyCapture2::Camera*> cameras = initializeCameras(&busMgr, camera_num);

	////ROS STUFF////
	ros::init(argc, argv, "grasshopper3");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	signal(SIGTERM, signalHandler);//detect closing

	double fps;
	if (private_nh.getParam("fps", fps))
	{
		ROS_INFO("fps set to %.2f", fps);
	} else {
		fps = 15.0;
		ROS_INFO("No param received, defaulting to %.2f", fps);
	}

	///////calibration data
	sensor_msgs::CameraInfo camerainfo_msg;
	getMatricesFromFile(private_nh, camerainfo_msg);

	ros::Publisher pub[camera_num];
	ros::Publisher camera_info_pub;

	camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1, true);

	for (int i = 0; i < camera_num; i++) {
	  std::string topic(std::string("image_raw"));

	  if (camera_num > 1) {
		topic = "camera" + std::to_string(i) + "/" + topic;
	  } 
		pub[i] = n.advertise<sensor_msgs::Image>(topic, 100);
		ROS_INFO("Publishing.. %s", topic.c_str());
	}

	startCapture(cameras);

	std::cout << "Capturing by " << camera_num << " cameras..." << std::endl;

	int count = 0;
	ros::Rate loop_rate(fps); // Hz
	while (running && ros::ok())
	{
		int i = 0;
		for (auto *camera : cameras)
		{
			FlyCapture2::Image image;
			FlyCapture2::Error error = camera->RetrieveBuffer(&image);
			if (error != FlyCapture2::PGRERROR_OK)
			{
				error.PrintErrorTrace();
				std::exit(-1);
			}

			// check encoding pattern
			std::string encoding_pattern;
			switch (image.GetBayerTileFormat()) {
			case FlyCapture2::RGGB: 
			  encoding_pattern = "bayer_rggb8";
			  break;
			case FlyCapture2::GRBG:
			  encoding_pattern = "bayer_grbg8";
			  break;
			case FlyCapture2::GBRG:
			  encoding_pattern = "bayer_gbrg8";
			  break;
			case FlyCapture2::BGGR:
			  encoding_pattern = "bayer_bggr8";
			  break;
			default:
			  encoding_pattern = "rgb8";
			}

			sensor_msgs::Image msg;
			//publish*******************

			msg.header.seq = count;
			msg.header.frame_id = "camera";
			msg.header.stamp.sec = ros::Time::now().sec;
			msg.header.stamp.nsec = ros::Time::now().nsec;
			msg.height = image.GetRows();
			msg.width  = image.GetCols();
			msg.encoding = encoding_pattern;
			msg.step = image.GetStride();

			size_t image_size = image.GetDataSize();
			msg.data.resize(image_size);
			memcpy(msg.data.data(), image.GetData(), image_size);



			pub[i].publish(msg);
			i++;
		}

		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	//close cameras
	for (auto *camera : cameras)
	{
		camera->StopCapture();
		camera->Disconnect();
		delete camera;
	}

	ROS_INFO("Camera node closed correctly");
	return 0;
}
