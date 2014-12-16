#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include "points_to_image/PointsImage.h"

bool existImage = false;
bool existPoints = false;
sensor_msgs::Image image_msg;
points_to_image::PointsImageConstPtr points_msg;

void show(void)
{
	if(!existImage || !existPoints){
		return;
	}
	const auto& encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, encoding);
	IplImage frame = cv_image->image;

	int w = points_msg->width;
	int h = points_msg->height;

	int i, n = w * h;
	int min_d = 1<<16, max_d = -1;
	int min_i = 1<<8, max_i = -1;
	for(i=0; i<n; i++){
		int di = points_msg->distance[i];
		max_d = di > max_d ? di : max_d;
		min_d = di < min_d ? di : min_d;
		int it = points_msg->intensity[i];
		max_i = it > max_i ? it : max_i;
		min_i = it < min_i ? it : min_i;
	}
	int wid_d = max_d - min_d;
	int wid_i = max_i - min_i;

	int x, y;
	for(y=0; y<h; y++){
		for(x=0; x<w; x++){
			int i = y * w + x;
			int distance = points_msg->distance[i];
			int intensity = points_msg->intensity[i];
			if(distance == 0){
				continue;
			}
			int g = wid_i ? ( (intensity - min_i) * 255 / wid_i ) : 128;
			int b = wid_d ? ( (distance - min_d) * 255 / wid_d ) : 128;
			int r = 255 - b;
			cvRectangle(&frame, cvPoint(x, y), cvPoint(x+1, y+1), CV_RGB(r, g, b));
		}
	}
	cvShowImage("Image Viewer", &frame);
	cvWaitKey(2);
}

void image_cb(const sensor_msgs::Image& msg)
{
	image_msg = msg;
	existImage = true;
	show();
}

void points_cb(const points_to_image::PointsImageConstPtr& msg)
{
	points_msg = msg;
	existPoints = true;
	show();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "points_image_viewer");
	ros::NodeHandle n;
	ros::Subscriber sub_image = n.subscribe("image_raw", 1, image_cb);
	ros::Subscriber sub_points = n.subscribe("points_image", 1, points_cb);
	ros::spin();
	return 0;
}
