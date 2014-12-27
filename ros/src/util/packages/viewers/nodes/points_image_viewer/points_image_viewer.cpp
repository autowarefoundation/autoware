#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include "points_to_image/PointsImage.h"

#define IMAGE_WIDTH 800
#define IMAGE_HEIGHT 640

bool existImage = false;
bool existPoints = false;
sensor_msgs::Image image_msg;
points_to_image::PointsImageConstPtr points_msg;
cv::Mat colormap;

void show(void)
{
	if(!existImage || !existPoints){
		return;
	}
	const auto& encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, encoding);
	IplImage frame = cv_image->image;

	cv::Mat matImage(&frame, false);	
	cv::cvtColor(matImage, matImage, CV_BGR2RGB);

	int w = IMAGE_WIDTH;
	int h = IMAGE_HEIGHT;
	int i, n = w * h;

	float min_d, max_d;
	min_d = max_d = points_msg->distance[0];
	for(i=1; i<n; i++){
		float di = points_msg->distance[i];
		max_d = di > max_d ? di : max_d;
		min_d = di < min_d ? di : min_d;
	}
	float wid_d = max_d - min_d;

	int x, y;
	for(y=0; y<h; y++){
		for(x=0; x<w; x++){
			int i = y * w + x;
			float distance = points_msg->distance[i];
			if(distance == 0){
				continue;
			}
			int colorid= wid_d ? ( (distance - min_d) * 255 / wid_d ) : 128;
			cv::Vec3b color=colormap.at<cv::Vec3b>(colorid);
			int g = color[1];
			int b = color[2];
			int r = color[0];
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
	
	cv::Mat grayscale(256,1,CV_8UC1);
	int i;
	for(i=0;i<256;i++)
	{
		grayscale.at<uchar>(i)=i;
	}
	cv::applyColorMap(grayscale,colormap,cv::COLORMAP_JET);

	ros::spin();
	return 0;
}
