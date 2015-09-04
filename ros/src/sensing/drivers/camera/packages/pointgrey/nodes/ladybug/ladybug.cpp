#include <iostream>
#include <string>
#include <sstream>
#include "ladybug.h"
#include "ladybugstream.h"
#include <stdexcept>
#include <unistd.h>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include "ladybug.h"

using namespace std;

static volatile int running_ = 1;


LadybugContext m_context;
LadybugDataFormat m_dataFormat;
//camera config settings
float m_frameRate;
bool m_isFrameRateAuto;
unsigned int m_jpegQualityPercentage;

ros::Publisher pub[LADYBUG_NUM_CAMERAS];

static void signalHandler(int)
{
	running_ = 0;
	ros::shutdown();
}

void parseCameraInfo(const cv::Mat  &camMat,
						const cv::Mat  &disCoeff,
						const cv::Size &imgSize,
						sensor_msgs::CameraInfo &msg)
{
	msg.header.frame_id = "camera";

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

void GetMatricesFromFile(ros::NodeHandle nh, sensor_msgs::CameraInfo &camerainfo_msg)
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

void publishImage(cv::Mat& image, ros::Publisher& image_pub, long int& count)
{
	sensor_msgs::Image msg;
	//publish*******************
	msg.header.seq = count;
	msg.header.frame_id = "camera";
	msg.header.stamp.sec = ros::Time::now().sec; msg.header.stamp.nsec = ros::Time::now().nsec;
	msg.height = image.size().height; msg.width  = image.size().width;
	msg.encoding = "rgb8";
	msg.step = image.cols * image.elemSize();
	size_t image_size = image.rows * image.cols * image.elemSize();

	msg.data.resize(image_size);
	memcpy(msg.data.data(), image.data, image_size);

	image_pub.publish(msg);
}

LadybugError init_camera()
{
	LadybugError error;
	error = ladybugCreateContext(&m_context);
	if (error != LADYBUG_OK)
	{
		throw std::runtime_error("Unable to create Ladybug context.");
	}

	LadybugCameraInfo enumeratedCameras[16];
	unsigned int numCameras = 16;

	error = ladybugBusEnumerateCameras(m_context, enumeratedCameras, &numCameras);
	if (error != LADYBUG_OK)
	{
		return error;
	}

	cout << "Cameras detected: " << numCameras << endl << endl;

	if (numCameras == 0)
	{
		ROS_INFO("Insufficient number of cameras detected. ");
		return LADYBUG_FAILED;
	}

	error = ladybugInitializeFromIndex(m_context, 0);
	if (error != LADYBUG_OK)
	{
		return error;
	}

	LadybugCameraInfo camInfo;
	error = ladybugGetCameraInfo(m_context, &camInfo);
	if (error != LADYBUG_OK)
	{
		return error;
	}

	ROS_INFO("Camera information: ");

	ROS_INFO("Base s/n: %d", camInfo.serialBase );
	ROS_INFO("Head s/n: %d", camInfo.serialHead );
	ROS_INFO("Model: %s", camInfo.pszModelName );
	ROS_INFO("Sensor: %s", camInfo.pszSensorInfo);
	ROS_INFO("Vendor: %s", camInfo.pszVendorName);
	ROS_INFO("Bus / Node: %d ,%d" , camInfo.iBusNum , camInfo.iNodeNum );

	switch (camInfo.deviceType)
	{
		case LADYBUG_DEVICE_LADYBUG3:
		{
			m_dataFormat = LADYBUG_DATAFORMAT_RAW8;
			m_frameRate = 16.0f;
			m_isFrameRateAuto = true;
			m_jpegQualityPercentage = 80;
		}
		break;

		case LADYBUG_DEVICE_LADYBUG5:
		{
			m_dataFormat = LADYBUG_DATAFORMAT_RAW8;
			m_frameRate = 10.0f;
			m_isFrameRateAuto = true;
			m_jpegQualityPercentage = 80;
		}
		break;

		default: assert(false); break;
	}

	return error;
}

LadybugError start_camera()
{
	LadybugError error;
	error = ladybugStartLockNext(m_context, m_dataFormat);
	if (error != LADYBUG_OK)
	{
		return error;
	}

	error = ladybugSetAbsPropertyEx(m_context, LADYBUG_FRAME_RATE, false, true, m_isFrameRateAuto, m_frameRate);
	if (error != LADYBUG_OK)
	{
		return error;
	}

	error = ladybugSetJPEGQuality(m_context, m_jpegQualityPercentage);
	if (error != LADYBUG_OK)
	{
		return error;
	}

	// Perform a quick test to make sure images can be successfully acquired
	for (int i=0; i < 10; i++)
	{
		LadybugImage tempImage;
		error = ladybugLockNext(m_context, &tempImage);
	}

	error = ladybugUnlockAll(m_context);
	if (error != LADYBUG_OK)
	{
		return error;
	}

	return error;
}

LadybugError stop_camera()
{
    const LadybugError cameraError = ladybugStop(m_context);
    if (cameraError != LADYBUG_OK)
    {
        ROS_INFO("Error: Unable to stop camera (%s)", ladybugErrorToString(cameraError) );
    }
    return cameraError;
}

LadybugError acquire_image( LadybugImage& image )
{
    return ladybugLockNext(m_context, &image);
}

LadybugError unlock_image( unsigned int bufferIndex )
{
    return ladybugUnlock(m_context, bufferIndex);
}

int main (int argc, char **argv)
{
	////ROS STUFF
	ros::init(argc, argv, "lady_bug");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	signal(SIGTERM, signalHandler);//detect closing

	/////////////////////////////
	//Config camera
	m_dataFormat = LADYBUG_DATAFORMAT_RAW8;
	m_frameRate = 10;
	m_isFrameRateAuto = true;
	m_jpegQualityPercentage = 80;

	// Initialize ladybug camera
	const LadybugError grabberInitError = init_camera();
	if (LADYBUG_OK != init_camera())
	{
		ROS_INFO("Error: Failed to initialize camera (%s). Terminating...", ladybugErrorToString(grabberInitError) );
		return -1;
	}

	LadybugCameraInfo camInfo;
	if (LADYBUG_OK != ladybugGetCameraInfo(m_context, &camInfo))
	{
		ROS_INFO("Error: Failed to get camera information. Terminating...");
		return -1;
	}

	const LadybugError startError = start_camera();
	if (startError != LADYBUG_OK)
	{
		ROS_INFO("Error: Failed to start camera (%s). Terminating...", ladybugErrorToString(startError) );
		return -1;
	}
	/////////////////////
	//ROS
	// Get the camera information
	///////calibration data
	sensor_msgs::CameraInfo camerainfo_msg;
	GetMatricesFromFile(private_nh, camerainfo_msg);
	int image_scale = 100;
	if (private_nh.getParam("scale", image_scale) && image_scale>0 && image_scale<100)
	{
		ROS_INFO("Ladybug ImageScale > %i%%", image_scale);
	}
	else
	{
		ROS_INFO("Ladybug ImageScale scale must be (0,100]. Defaulting to 100 ");
		image_scale=100;
	}

	ros::Publisher camera_info_pub;

	camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1, true);
	ROS_INFO("Successfully started ladybug camera and stream");
	for (int i = 0; i < LADYBUG_NUM_CAMERAS + 1; i++) {
		std::string topic(std::string("image_raw"));

			topic = "camera" + std::to_string(i) + "/" + topic;

		pub[i] = n.advertise<sensor_msgs::Image>(topic, 100);
		ROS_INFO("Publishing.. %s", topic.c_str());
	}
	//////////////////

	//start camera
	ros::Rate loop_rate(10); // Hz Ladybug works at 10fps
	long int count = 0;
	while (running_ && ros::ok())
	{
		LadybugImage currentImage;

		const LadybugError acquisitionError = acquire_image(currentImage);
		if (acquisitionError != LADYBUG_OK)
		{
			ROS_INFO("Failed to acquire image. Error (%s). Trying to continue..", ladybugErrorToString(acquisitionError) );
			continue;
		}

		// convert to OpenCV Mat
		//receive Bayer Image, convert to Color 3 channels
		cv::Size size(currentImage.uiFullCols, currentImage.uiFullRows);

		cv::Mat full_size;
		for(size_t i =0;i<LADYBUG_NUM_CAMERAS; i++)
		{
			std::ostringstream out;
			out << "image" << i;
			cv::Mat rawImage(size, CV_8UC1, currentImage.pData + (i * size.width*size.height));
			//cv::flip(mat, mat, -1);
			cv::Mat image(size, CV_8UC3);
			cv::cvtColor(rawImage, image, cv::COLOR_BayerBG2RGB);
			cv::resize(image,image,cv::Size(size.width*image_scale/100, size.height*image_scale/100));
			cv::transpose(image, image);
			if (i==0)
				image.copyTo(full_size);
			else
				cv::hconcat(image, full_size, full_size);

			unlock_image(currentImage.uiBufferIndex);

			publishImage(image, pub[LADYBUG_NUM_CAMERAS - i], count);

		}
		//publish stitched one
		publishImage(full_size, pub[0], count);

		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	cout << "Stopping ladybug..." << endl;

	// Shutdown
	stop_camera();

	ROS_INFO("ladybug stopped");

	return 0;
}
