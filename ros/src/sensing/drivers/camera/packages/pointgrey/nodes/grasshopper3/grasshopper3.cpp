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
#include <stdlib.h>
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

static void signal_handler(int)
{
	running = 0;
	ros::shutdown();
}

void parse_camera_info(const cv::Mat& camMat,
                       const cv::Mat& disCoeff,
                       const cv::Size& imgSize,
                       sensor_msgs::CameraInfo& msg)
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


void print_camera_info(FlyCapture2::CameraInfo* info)
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

void print_image_settings(
		const FlyCapture2::Format7ImageSettings& image_settings,
		const unsigned int& packet_size,
		const float& percentage
	)
{
	std::cout << "Image settings: " << std::endl;
	std::cout << "\tMode: " << image_settings.mode << std::endl;
	std::cout << "\tPixel Format: 0x" << std::hex << image_settings.pixelFormat << std::dec << std::endl;
	std::cout << "\tOffset X: " << image_settings.offsetX << std::endl;
	std::cout << "\tOffset Y: " << image_settings.offsetY << std::endl;
	std::cout << "\tWidth: " << image_settings.width << std::endl;
	std::cout << "\tHeight: " << image_settings.height << std::endl;
	std::cout << "Packet size: " << packet_size << " (" << percentage << "%)" << std::endl;
}

void print_format7_info(const FlyCapture2::Format7Info& info, bool supported)
{
	std::cout << "supported: " << supported << std::endl;
	std::cout << "mode: " << info.mode << std::endl;
	std::cout << "maxWidth: " << info.maxWidth << std::endl;
	std::cout << "maxHeight: " << info.maxHeight << std::endl;
	std::cout << "packetSize: " << info.packetSize << std::endl;
	std::cout << "percentage: " << info.percentage << std::endl;
	std::cout << "pixelFormatBitField: " << info.pixelFormatBitField << std::endl;
}

void initialize_cameras(std::vector<FlyCapture2::Camera *> &cameras,
                        FlyCapture2::BusManager *bus_manger,
                        int camera_num,
                        FlyCapture2::Mode desired_mode,
                        FlyCapture2::PixelFormat desired_pixel_format,
                        int timeout_ms)
{
	// Connect to all detected cameras and attempt to set them to
	// a common video mode and frame rate
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

		image_info.timestamp.onOff = false;
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

		//obtain working settings
		FlyCapture2::VideoMode default_video_mode;
		FlyCapture2::FrameRate default_frame_rate;

		error = camera->GetVideoModeAndFrameRate(&default_video_mode, &default_frame_rate);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			error.PrintErrorTrace();
			std::exit(-1);
		}

		//try to set Format7, according to the desired mode and pixel format.
		FlyCapture2::Format7ImageSettings image_settings;
		bool supported = false;
		unsigned int packet_size;
		float percentage;

		FlyCapture2::Format7Info format7_info;
		format7_info.mode = desired_mode;

		error = camera->GetFormat7Info(&format7_info, &supported);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			error.PrintErrorTrace();
			std::exit(-1);
		}

		print_format7_info(format7_info, supported);

		if (supported)
		{
			error = camera->GetFormat7Configuration(&image_settings, &packet_size, &percentage);
			if (error != FlyCapture2::PGRERROR_OK)
			{
				error.PrintErrorTrace();
				std::exit(-1);
			}

			image_settings.mode = desired_mode;
			image_settings.pixelFormat = desired_pixel_format;
			image_settings.offsetX = 0;
			image_settings.offsetY = 0;
			image_settings.width = format7_info.maxWidth;
			image_settings.height = format7_info.maxHeight;

			FlyCapture2::Format7PacketInfo packet_info;
			bool valid_settings = false;
			error = camera->ValidateFormat7Settings(&image_settings, &valid_settings, &packet_info);
			if (error != FlyCapture2::PGRERROR_OK)
			{
				error.PrintErrorTrace();
				std::exit(-1);
			}
			packet_size = packet_info.recommendedBytesPerPacket;
			error = camera->SetFormat7Configuration(&image_settings, packet_size);
			if (error != FlyCapture2::PGRERROR_OK)
			{
				error.PrintErrorTrace();
				std::exit(-1);
			}

			error = camera->GetFormat7Configuration(&image_settings, &packet_size, &percentage);
			if (error != FlyCapture2::PGRERROR_OK)
			{
				error.PrintErrorTrace();
				std::exit(-1);
			}

			print_image_settings(image_settings, packet_size, percentage);
		}
		else
		{
			ROS_ERROR("Selected Mode not supported, using last working mode.");
		}

		FlyCapture2::FC2Config camera_config;
		error = camera->GetConfiguration(&camera_config);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			error.PrintErrorTrace();
			ROS_INFO("Could not read configuration from Camera");
		}
		else
		{
			if (timeout_ms > 0)
				camera_config.grabTimeout = timeout_ms;

			error = camera->SetConfiguration(&camera_config);
			if (error != FlyCapture2::PGRERROR_OK)
			{
				error.PrintErrorTrace();
				ROS_INFO("Could not set configuration on Camera");
			}
		}

		print_camera_info(&camera_info);
		cameras.push_back(camera);
	}
}

/*!
 * Get the number of cameras connected to the system
 * @param bus_manager Valid pointer to the BusManager
 * @return The number of detected cameras
 */
unsigned int get_num_cameras(FlyCapture2::BusManager* bus_manager)
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
	return cameras;
}

/*!
 * Initialize the capture on all the cameras
 * @param cameras An array of valid pointers to the camera objects
 */
void start_capture(std::vector<FlyCapture2::Camera *>& cameras)
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
}

/*!
 * Reads and parses the Autoware calibration file format
 * @param nh ros node handle
 * @param camerainfo_msg CameraInfo message to fill
 */
void getMatricesFromFile(const ros::NodeHandle& nh, sensor_msgs::CameraInfo &camerainfo_msg)
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
	parse_camera_info(cameraMat, distCoeff, imageSize, camerainfo_msg);
}

/*!
 * Reads the params from the console
 * @param private_nh[in] Private Ros node handle
 * @param fps[out] Read value from the console double
 * @param mode[out] Read value from the console integer
 * @param format[out] Read value from the console raw or rgb
 * @param timeout[out] Read value from the console timeout in ms
 */
void ros_get_params(const ros::NodeHandle& private_nh, int& fps, int& mode, std::string& format, int& timeout)
{
	if (private_nh.getParam("fps", fps))
	{
		ROS_INFO("fps set to %d", fps);
	} else {
		fps = 20;
		ROS_INFO("No param received, defaulting fps to %d", fps);
	}

	if (private_nh.getParam("mode", mode))
	{
		ROS_INFO("mode set to %d", mode);
	} else {
		mode = 0;
		ROS_INFO("No param received, defaulting mode to %d", mode);
	}

	if (private_nh.getParam("format", format))
	{
		ROS_INFO("format set to %s", format.c_str());
	} else {
		format = "raw";
		ROS_INFO("No param received, defaulting format to %s", format.c_str());
	}

	if (private_nh.getParam("timeout", timeout))
	{
		ROS_INFO("timeout set to %d ms", timeout);
	} else {
		timeout = 1000;
		ROS_INFO("No param received, defaulting timeout to %d ms", timeout);
	}

}

int main(int argc, char **argv)
{
	////////////////POINT GREY CAMERA /////////////////////////////
	FlyCapture2::BusManager busMgr;

	////ROS STUFF////
	ros::init(argc, argv, "grasshopper3");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	signal(SIGTERM, signal_handler);//detect closing

	int fps, camera_mode, timeout;
	std::string pixel_format;

	ros_get_params(private_nh, fps, camera_mode, pixel_format, timeout);

	//
	FlyCapture2::Mode desired_mode;
	FlyCapture2::PixelFormat desired_pixel_format;

	desired_mode = (FlyCapture2::Mode)camera_mode;

	if(pixel_format == "rgb")
	{
		desired_pixel_format = FlyCapture2::PIXEL_FORMAT_RGB8;
	}
	else
	{
		desired_pixel_format = FlyCapture2::PIXEL_FORMAT_RAW8;
	}

	//init cameras
	int camera_num = get_num_cameras(&busMgr);
	std::vector<FlyCapture2::Camera*> cameras;
	initialize_cameras(cameras, &busMgr, camera_num, desired_mode, desired_pixel_format, timeout);

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

	start_capture(cameras);

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
				continue;
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
			std::string frame = "camera" + std::to_string(i);
			msg.header.frame_id = frame;
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
