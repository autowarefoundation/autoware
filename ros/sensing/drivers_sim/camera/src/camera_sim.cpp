#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_sim");
	ros::NodeHandle n;

	if(argc < 2){
		fprintf(stderr, "image file name ?\n");
		exit(1);
	}
	const char* fn = argv[1];
	fprintf(stderr, "fn='%s'\n", fn);

	IplImage* img = cvLoadImage(fn, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
	if(img == NULL){
	  fprintf(stderr, "Can't load '%s'\n", fn);
	  exit(1);
	}

	ros::Publisher pub = n.advertise<sensor_msgs::Image>("image_raw", 1000);

	sensor_msgs::Image msg;
	msg.width = img->width;
	msg.height = img->height;
	msg.is_bigendian = 0;
	msg.step = img->widthStep;

	uint8_t* dataPtr = (uint8_t*)img->imageData;
	std::vector<uint8_t> data(dataPtr, dataPtr + img->imageSize);
	msg.data = data;

	msg.encoding = sensor_msgs::image_encodings::RGB8; // TODO

	ros::Rate loop_rate(10); // Hz

	while(ros::ok()){
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
