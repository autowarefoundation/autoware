
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
	//create images for each camera to be used while capturing
	//

	if (!create_images_start(imagePointers, cameraPointers))
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
			BGAPI::Image* pImage = NULL;
			//create an image for each camera
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
			res = cameraPointers[i]->getImage( &pImage, receiveTimeout );
			if( res != BGAPI_RESULT_OK )
			{
				ROS_INFO("BGAPI_Camera_getImage returned with errorcode %d\n", res );
			}
			else
			{
				(pImage)->get( &imagebuffer );				
				(pImage)->getNumber( &swc, &hwc );

				res = (pImage)->getSize(&width, &height);
				if( res != BGAPI_RESULT_OK )
				{
					ROS_INFO("BGAPI::Image::getSize Errorcode: %d", res);
				}
				//ROS_INFO("Get Image OK %d, %d\n", width, height);
				//receive Bayer Image, convert to Color 3 channels
				cv::Mat mat(cv::Size(width, height), CV_8UC1, imagebuffer);
				//cv::flip(mat, mat, -1);
				cv::Mat dest(cv::Size(width, height), CV_8UC3);
				cv::cvtColor(mat, dest, CV_BayerBG2RGB);

				//cv::imshow("window", dest);
				//cv::waitKey(2);
				//ROS publish*******************
				sensor_msgs::Image msg;

				msg.header.seq = count;
				msg.header.frame_id = count;
				msg.header.stamp.sec = ros::Time::now().toSec();
				msg.header.stamp.nsec = ros::Time::now().toNSec();
				msg.height = height;
				msg.width  = width;
				msg.encoding = "rgb8";
				msg.step = dest.rows * dest.cols * dest.elemSize1();

				size_t image_size = dest.rows * dest.cols * dest.elemSize();

				msg.data.resize(image_size);
				memcpy(msg.data.data(), dest.data, image_size);

				pub[i].publish(msg);
				i++;

				//after you are ready with this image, return it to the camera for the next image
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
