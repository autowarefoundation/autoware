#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include "points_to_image/PointsImage.h"

#include "dpm/ImageObjects.h"

#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

bool existImage = false;
bool existPoints = false;
sensor_msgs::Image image_msg;
points_to_image::PointsImageConstPtr points_msg;
cv::Mat colormap;

vector<Rect> cars;

#define	IMAGE_WIDTH	640
#define	IMAGE_HEIGHT	480

void rect_updater_callback(dpm::ImageObjects image_objects_msg)
{
	int num = image_objects_msg.car_num;
	vector<int> points = image_objects_msg.corner_point; 
	//points are X,Y,W,H and repeat for each instance
    	cars.clear();
	
	for (int i=0; i<num;i++)
	{
		Rect tmp;
		tmp.x = points[i*4 + 0];
		tmp.y = points[i*4 + 1];
		tmp.width = points[i*4 + 2];
		tmp.height = points[i*4 + 3];
		cars.push_back(tmp);
	}
	
}

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

	/*DRAW RECTANGLES of detected objects*/
	for(std::size_t i=0; i<cars.size();i++)
	{
		
		cvRectangle( &frame, 
				cvPoint(cars[i].x, cars[i].y),
				cvPoint(cars[i].x+cars[i].width, cars[i].y+cars[i].height),
				Scalar( 0, 255, 0,0 ), 1, 8,0 );
	}

	int w = IMAGE_WIDTH;
	int h = IMAGE_HEIGHT;

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
			double distance = points_msg->distance[i];
			double intensity = points_msg->intensity[i];
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

	ros::Subscriber scriber_car = n.subscribe("/car_pos_xy", 1,
					      rect_updater_callback); 
	
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
