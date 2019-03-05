#include <bgapidef.hpp>
#include "bgapi.hpp"
#include "bgapi_init.h"

/*
 * Initialize cameras
 * @return Returns True if the camera systems and at least one camera was correctly initialized
 */
bool baumer_startup(std::vector<BGAPI::System*>& ppSystem,
                    std::vector<BGAPI::System*>::iterator& systemIter,
                    BGAPI_RESULT& res,
                    std::vector<BGAPI::Camera*>& cameraPointers,
                    std::vector<BGAPI::Image*>& imagePointers,
                    int& camera_num,
                    int& system_count,
                    BGAPI::Image* pImage)
{
	int currSystem = 0;
	int i = 0;
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
		return false;
	}

	if (camera_num > 0)
	{
		ROS_INFO("init_cameras OK. Found %d Baumer cameras\n", camera_num);
	}
	else
	{
		ROS_INFO("No cameras detected, finalizing...");
		return false;
	}

	//set all cameras to BayerRGB8 to save ethernet bandwidth
	if(!setup_cameras(cameraPointers,  "BayerRG8"))
	{
		ROS_INFO("Could not setup cameras for capture. Finalizing...");
		return false;
	}

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
		return false;
	}
	return true;
}

/*!
 * brightness in this program is defined as the ratio between the pixels with higher values over the lowest ones.
 * Ratio is calculated splitting the histogram in half, and dividing the number of pixels in the seconf (higher) part
 * of the histogram (brightness higher than 128) over the total number of pixels.
 * @param in_image
 * @return
 */
float get_image_brightness_percent(cv::Mat& in_image)
{
	cv::Mat histogram;
	int histSize = 16;
	float range[] = { 0, 16 } ;
	const float* histRange = { range };
	bool uniform = true; bool accumulate = false;

	cv::calcHist( &in_image, 1, 0, cv::Mat(), histogram, 1, &histSize, &histRange, uniform, accumulate );

	float high_half_samples = 0;
	float total_samples = 0;

	for (int k =0; k <= histSize; k++)
	{
		if (k >= histSize/2)
			high_half_samples+=histogram.at<float>(k);

		total_samples+=histogram.at<float>(k);
	}

	float brightness;
	if(high_half_samples == 0)
		brightness = 0.0;
	else
		brightness = high_half_samples/total_samples;

#ifdef VLG22_DEBUG
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound( (double) hist_w/histSize );
	cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	/// Normalize the result to [ 0, histImage.rows ]
	cv::normalize(histogram, histogram, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	for( int j = 1; j < histSize; j++ )
	{
		total_samples+=histogram.at<float>(j);
		cv::line( histImage, cv::Point( bin_w*(j-1), hist_h - cvRound(histogram.at<float>(j-1)) ),
		          cv::Point( bin_w*(j), hist_h - cvRound(histogram.at<float>(j)) ),
		          cv::Scalar( 255, 255, 255), 2, 8, 0  );
	}

	std::cout << high_half_samples << "/" << total_samples << "==>" << brightness << std::endl;
	/// Display
	cv::imshow("Histogram", histImage );
	cv::waitKey(1);
#endif

	return brightness;

}


/*!
 * Changes, if possible, the camera time exposure.
 * @param camera_pointer Valid pointer of the camera to change.
 * @param exposure New exposure
 */
void baumer_change_exposure(BGAPI::Camera* camera_pointer, int exposure)
{
	if (camera_pointer == NULL)
		return;

	BGAPI_FeatureState state;
	state.cbSize = sizeof( BGAPI_FeatureState );

	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	BGAPIX_TypeRangeINT rangedint;

	rangedint.cbSize = sizeof( BGAPIX_TypeRangeINT );
	res = camera_pointer->getExposure( &state, &rangedint );
	if( res != BGAPI_RESULT_OK )
	{
		ROS_INFO("BGAPI::Camera::getExposure Errorcode: %d", res);
	}

	if (exposure > rangedint.maximum)
		exposure = rangedint.maximum;

	if (exposure < rangedint.minimum)
		exposure = rangedint.minimum;

	res = camera_pointer->setExposure( exposure );
	if( res != BGAPI_RESULT_OK )
	{
		ROS_INFO("BGAPI::Camera::setExposure Errorcode: %d", res);
	}

	/*res = camera_pointer->setGain( 2.0 );
	if( res != BGAPI_RESULT_OK )
	{
		printf("BGAPI::Camera::setGain Errorcode: %d\n", res);
	}*/
}

void get_ros_params(ros::NodeHandle& node_handle, int& fps, double& scale, double& brightness)
{
	if (node_handle.getParam("fps", fps))
	{
		ROS_INFO("fps set to %d", fps);
	}
	else
	{
		fps = 20;
		ROS_INFO("No param received, defaulting fps to %df", fps);
	}

	if (node_handle.getParam("scale", scale))
	{
		if (scale > 1.0 || scale < 0.01)
		{
			scale = 1.0;
		}
		ROS_INFO("scale set to %.2f", scale);
	}
	else
	{
		scale = 0.5;
		ROS_INFO("No param received, defaulting scale to %.2f", scale);
	}

	if (node_handle.getParam("brightness", brightness))
	{
		if (brightness > 1.0 || brightness < 0.01)
		{
			brightness = 0.7;
		}
		ROS_INFO("brightness set to %.2f", brightness);
	}
	else
	{
		brightness = 0.7;
		ROS_INFO("No param received, defaulting brightness to %.2f", brightness);
	}
}

void ros_init_publishers(ros::NodeHandle node_handle, ros::Publisher publishers[], int camera_num)
{
	for (int i = 0; i < camera_num; i++) {
		std::string topic(std::string("image_raw"));

		if (camera_num > 1) {
			topic = "camera" + std::to_string(i) + "/" + topic;
		}
		publishers[i] = node_handle.advertise<sensor_msgs::Image>(topic, 100);
		ROS_INFO("Publishing.. %s", topic.c_str());
	}
}

sensor_msgs::Image ros_prepare_image(int count, cv::Mat image, size_t camera_id)
{
	//ROS publish*******************
	sensor_msgs::Image msg;

	msg.header.seq = count;
	std::string frame = "camera" + std::to_string(camera_id);
	msg.header.frame_id = frame;
	msg.header.stamp.sec = ros::Time::now().sec;
	msg.header.stamp.nsec = ros::Time::now().nsec;
	msg.height = image.size().height;
	msg.width = image.size().width;
	msg.encoding = "rgb8";
	msg.step = image.cols * image.elemSize();

	size_t image_size = image.rows * image.cols * image.elemSize();

	msg.data.resize(image_size);
	memcpy(msg.data.data(), image.data, image_size);

	return msg;
}

void adjust_exposure(BGAPI::Camera* camera_pointer,
                     cv::Mat& bayer_image,
                     float& current_brightness,
                     float threshold_brightness,
                     int& prev_exposure,
                     int& exposure,
                     int exposure_delta)
{
	current_brightness = get_image_brightness_percent(bayer_image);

	if (current_brightness < threshold_brightness)
	{
		if (exposure <= VLG22_MAX_EXPOSURE)
		{
			exposure = prev_exposure + exposure_delta;
			ROS_INFO("Increasing exposure");
			baumer_change_exposure(camera_pointer, exposure);
		}
	}

	if (current_brightness == 1.0)
	{
		if (exposure >= VLG22_MIN_EXPOSURE)
		{
			exposure = prev_exposure - exposure_delta;
			ROS_INFO("Reducing exposure");
			baumer_change_exposure(camera_pointer, exposure);
		}
	}
	prev_exposure = exposure;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "baumer_vlg22");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	ros::Publisher camera_info_pub;

	int system_count = 0;
	int camera_num = 0;

	int fps;
	double brightness;
	double scale;

	get_ros_params(private_nh, fps, scale, brightness);

	std::vector<BGAPI::System*> ppSystem;
	std::vector<BGAPI::System*>::iterator systemIter;

	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	std::vector<BGAPI::Camera*> cameraPointers;
	std::vector<BGAPI::Image*> imagePointers;
	BGAPI::Image* pImage = NULL;

	if(!baumer_startup(ppSystem, systemIter, res, cameraPointers, imagePointers, camera_num, system_count, pImage))
	{
		return -1;
	}

	//create ros camera publishers
	ros::Publisher pub[camera_num];
	ros_init_publishers(n, pub, camera_num);

	int receiveTimeout = 1000;
	int count = 0;
	ros::Rate loop_rate(fps); // Hz
	float current_brightness = 0.0;
	int prev_exposure = VLG22_DEFAULT_EXPOSURE;
	const int exposure_delta = 1000;
	while (ros::ok())
	{
		unsigned char* imagebuffer = NULL;
		int swc = 0;
		int hwc = 0;
		int width = 0;
		int height = 0;

		int exposure = VLG22_DEFAULT_EXPOSURE;
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

				cv::Mat bayer_image(cv::Size(width, height), CV_8UC1, imagebuffer);
				cv::Mat color_image(cv::Size(width, height), CV_8UC3);
				cv::cvtColor(bayer_image, color_image, CV_BayerBG2RGB);
				cv::Mat resized_color_image;
				cv::resize(color_image, resized_color_image, cv::Size(), scale, scale);

				adjust_exposure(cameraPointers[i],
				                bayer_image,
				                current_brightness,
				                brightness,
				                prev_exposure,
				                exposure,
				                exposure_delta);

				sensor_msgs::Image msg = ros_prepare_image(count, resized_color_image, i);

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
