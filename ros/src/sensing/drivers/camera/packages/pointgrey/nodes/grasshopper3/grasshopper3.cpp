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
  Initial version 2014-11-14
*/

#include <iostream>

#include <FlyCapture2.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

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

static void initializeCameras(FlyCapture2::BusManager* pbusMgr, int numCameras,
			      FlyCapture2::Camera*** pCams)
{
	FlyCapture2::Error error;
	FlyCapture2::Camera** ppCameras = *pCams;//*unwrap pointers

	// Connect to all detected cameras and attempt to set them to
	// a common video mode and frame rate
	for (int i = 0; i < numCameras; i++) {
		ppCameras[i] = new FlyCapture2::Camera();

		FlyCapture2::PGRGuid guid;
		error = pbusMgr->GetCameraFromIndex(i, &guid);
		if (error != FlyCapture2::PGRERROR_OK) {
			error.PrintErrorTrace();
			std::exit(-1);
		}

		// Connect to a camera
		error = ppCameras[i]->Connect( &guid );
		if (error != FlyCapture2::PGRERROR_OK) {
			error.PrintErrorTrace();
			std::exit(-1);
		}

		FlyCapture2::EmbeddedImageInfo image_info;
		error = ppCameras[i]->GetEmbeddedImageInfo(&image_info);
		if (error != FlyCapture2::PGRERROR_OK) {
			error.PrintErrorTrace();
			std::exit(-1);
		}

		image_info.timestamp.onOff = true;
		error = ppCameras[i]->SetEmbeddedImageInfo(&image_info);
		if (error != FlyCapture2::PGRERROR_OK) {
			error.PrintErrorTrace();
			std::exit(-1);
		}

		// Get the camera information
		FlyCapture2::CameraInfo camera_info;
		error = ppCameras[i]->GetCameraInfo(&camera_info);
		if (error != FlyCapture2::PGRERROR_OK) {
			error.PrintErrorTrace();
			std::exit(-1);
		}

		print_camera_info(&camera_info);
	}
}

static void captureImage(FlyCapture2::Camera **ppCameras,
			 int numCameras, FlyCapture2::Image images[])
{
	// Display the time stamps for all cameras to show that the image
	// capture is synchronized for each image

	//retrieve images continuously
	for (int i = 0; i< numCameras; i++) {
		FlyCapture2::Error error = ppCameras[i]->RetrieveBuffer(&(images[i]));
		if (error != FlyCapture2::PGRERROR_OK) {
			error.PrintErrorTrace();
			std::exit(-1);
		}
	}
}

static int getNumCameras(FlyCapture2::BusManager* bus_manager)
{
	unsigned int cameras;
	FlyCapture2::Error error = bus_manager->GetNumOfCameras(&cameras);
	if (error != FlyCapture2::PGRERROR_OK) {
		error.PrintErrorTrace();
		std::exit(-1);
	}

	std::cout << "Number of cameras detected: " << cameras << std::endl;

	if (cameras < 1) {
		std::cerr << "Error: This program requires at least 1 camera." << std::endl;
		std::exit(-1);
	}

	return static_cast<int>(cameras);
}

static void startCapture(int numCameras, FlyCapture2::Camera** ppCameras)
{
	for (int i = 0; i < numCameras; i++) {
		FlyCapture2::Error error = ppCameras[i]->StartCapture();
		if (error != FlyCapture2::PGRERROR_OK) {
			error.PrintErrorTrace();
			return;
		}
	}
	return;
}

int main(int argc, char **argv)
{
	FlyCapture2::BusManager busMgr;

	int camera_num = getNumCameras(&busMgr);
	FlyCapture2::Camera** ppCameras = new FlyCapture2::Camera*[camera_num];
	initializeCameras(&busMgr, camera_num, &ppCameras);

	ros::init(argc, argv, "grasshopper3");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	double fps;
	if (private_nh.getParam("fps", fps)) {
		ROS_INFO("fps set to %.2f", fps);
	} else {
		fps = 15.0;
		ROS_INFO("No param received, defaulting to %.2f", fps);
	}

	ros::Publisher pub[camera_num];
	for (int i = 0; i < camera_num; i++) {
		std::string topic(std::string("image_raw") + std::to_string(i));
		pub[i] = n.advertise<sensor_msgs::Image>(topic, 100);
	}

	FlyCapture2::Image images[camera_num];
	startCapture(camera_num, ppCameras);

	std::cout << "Capturing by " << camera_num << " cameras..." << std::endl;

	int count = 0;
	ros::Rate loop_rate(fps); // Hz
	while (ros::ok()) {
		sensor_msgs::Image imagemsg[camera_num];
		for (int i = 0; i < camera_num; i++) {
			captureImage(ppCameras, camera_num, images);//Get image from camera

			//fill ROS Message structure
			imagemsg[i].header.seq=count;
			imagemsg[i].header.frame_id=count;
			imagemsg[i].header.stamp.sec=ros::Time::now().toSec();
			imagemsg[i].header.stamp.nsec=ros::Time::now().toNSec();
			imagemsg[i].height= images[i].GetRows();
			imagemsg[i].width= images[i].GetCols();
			imagemsg[i].encoding = "rgb8";
			imagemsg[i].step = images[i].GetStride();
			imagemsg[i].data.resize(images[i].GetDataSize());
			memcpy(imagemsg[i].data.data(),images[i].GetData(), images[i].GetDataSize());

			pub[i].publish(imagemsg[i]);//publish
		}
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	//close cameras
	for (int i = 0; i < camera_num; i++) {
		ppCameras[i]->StopCapture();
		ppCameras[i]->Disconnect();
		delete ppCameras[i];
	}

	delete [] ppCameras;

	std::cout << "Done!" << std::endl;
	return 0;
}
