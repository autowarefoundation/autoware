#include "bgapi.hpp"
#include "bgapi_init.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "baumer_vlg22");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	ros::Publisher camera_info_pub;

	int system_count = 0;
	int currSystem = 0;
	int i = 0;
	int camera_num = 0;

	vector<BGAPI::System*> ppSystem;
	vector<BGAPI::System*>::iterator systemIter;

	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	std::vector<BGAPI::Camera*> cameraPointers;
	std::vector<BGAPI::Image*> imagePointers;

	double fps;
	if (private_nh.getParam("fps", fps))
	{
		ROS_INFO("fps set to %.2f", fps);
	} else {
		fps = 15.0;
		ROS_INFO("No param received, defaulting to %.2f", fps);
	}

	//try to initialize baumer system
	res = init_systems( &system_count, &ppSystem );
	if( res != BGAPI_RESULT_OK )
	{
		printf("init_systems Errorcode: %d\n", res);
	}
	ROS_INFO("Baumer Camera init_systems OK. %d systems found\n", system_count);

	//init all cameras in all systems
	res = init_cameras( system_count, &ppSystem, &currSystem, camera_num, cameraPointers );
	if( res != BGAPI_RESULT_OK )
	{
		ROS_INFO("init_camera Errorcode: %d\n", res);
	}

	if (camera_num > 0)
	{
		ROS_INFO("init_cameras OK. Found %d Baumer cameras\n", camera_num);
	}
	else
	{
		ROS_INFO("No cameras detected, finalizing...");
		return -1;
	}

	//set all cameras to BayerRGB8 to save ethernet bandwidth
	if(!setup_cameras(cameraPointers,  "BayerRG8"))
	{
		ROS_INFO("Could not setup cameras for capture. Finalizing...");
		return -1;
	}

	BGAPI::Image* pImage = NULL;
	//create an image
	res = BGAPI::createImage( &pImage );
	if( res != BGAPI_RESULT_OK )
	{
		ROS_INFO( "Error %d while creating an image.\n", res );
		return false;
	}
	res = cameraPointers[i]->setImage( pImage );
	if( res != BGAPI_RESULT_OK )
	{
		ROS_INFO( "Error %d while setting an image to the camera.\n", res );
		return false;
	}
	if (!start_cameras(cameraPointers))
	{
		ROS_INFO("Could not create images for capture. Finalizing...");
		return -1;
	}

	//create ros camera publishers
	ros::Publisher pub[camera_num];

	for (int i = 0; i < camera_num; i++)
	{
		std::ostringstream topic;
		topic << "image_raw";
		if (camera_num > 1)
		{
			topic << i;
		}
		pub[i] = n.advertise<sensor_msgs::Image>(topic.str(), 100);
	}

	int receiveTimeout = 1000;
	int count = 0;
	ros::Rate loop_rate(fps); // Hz
	while (ros::ok())
	{
		unsigned char* imagebuffer = NULL;
		int swc = 0;
		int hwc = 0;
		int width = 0;
		int height = 0;

		for (unsigned int i = 0; i < cameraPointers.size(); i++)
		{

			res = cameraPointers[i]->getImage( &pImage, receiveTimeout );
			if( res != BGAPI_RESULT_OK )
			{
				ROS_INFO("BGAPI_Camera_getImage returned with errorcode %d\n", res );
			}
			else
			{
			        free(imagebuffer);
				(pImage)->get( &imagebuffer );
				(pImage)->getNumber( &swc, &hwc );

				res = (pImage)->getSize(&width, &height);
				if( res != BGAPI_RESULT_OK )
				{
					ROS_INFO("BGAPI::Image::getSize Errorcode: %d", res);
				}

				//receive Bayer Image, convert to Color 3 channels
				cv::Mat mat(cv::Size(width, height), CV_8UC1, imagebuffer);
				//cv::flip(mat, mat, -1);
				cv::Mat dest(cv::Size(width, height), CV_8UC3);
				cv::cvtColor(mat, dest, CV_BayerBG2RGB);
				//to fix aspect ratio and to avoid stretching we crop the image
				//after conversion of the bayer grid
				//to mantain ratio of 1.33 we need to remove 2*297 from the wide
				//image, so we use a roi to crop the 2048x1084 image
				cv::Rect roi(297,0,1445,1084);//for [4:3] aspect ratio. 297 half of the width to remove
				//cv::Rect roi(56,0,1927,1084);//for [16:9] aspect ratio. 56 half of the width to remove
				cv::Mat tmp = dest(roi);
				cv::Mat cropped = tmp.clone();
				int w = 800;//fixed
				int h = 600;//fixed 
				cv::resize(cropped, cropped, cv::Size(w, h));

				//cv::imshow("window", dest);
				//cv::waitKey(2);
				//ROS publish*******************
				sensor_msgs::Image msg;

				msg.header.seq = count;
				msg.header.frame_id = "camera";
				msg.header.stamp.sec = ros::Time::now().sec;
				msg.header.stamp.nsec = ros::Time::now().nsec;
				msg.height = cropped.size().height;
				msg.width = cropped.size().width;
				msg.encoding = "rgb8";
				msg.step = cropped.cols * cropped.elemSize();

				size_t image_size = cropped.rows * cropped.cols * cropped.elemSize();

				msg.data.resize(image_size);
				memcpy(msg.data.data(), cropped.data, image_size);

				pub[i].publish(msg);
				res = cameraPointers[i]->setImage( pImage );
				if( res != BGAPI_RESULT_OK )
				{
					ROS_INFO( "setImage failed with %d\n", res );
				}
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	//stop acquisition
	stop_cameras(cameraPointers);

	//release the system
	res = release_systems( &ppSystem );
	if( res != BGAPI_RESULT_OK )
	{
		ROS_INFO("release_systems Errorcode: %d\n", res);
	}
	//release image memory
	release_images(&imagePointers);
	return 0;
}
