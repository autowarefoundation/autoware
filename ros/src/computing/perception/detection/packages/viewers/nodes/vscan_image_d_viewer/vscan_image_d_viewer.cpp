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

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <points2image/PointsImage.h>

#include <cv_tracker/image_obj_ranged.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <float.h>

#define NO_DATA 0
static char window_name[] = "vscan_image_d_viewer";

static bool existImage = false;
static bool existPoints = false;
static sensor_msgs::Image image_msg;
static points2image::PointsImageConstPtr points_msg;
static cv::Mat colormap;

#if 0
static std::vector<cv::Rect> cars;
static std::vector<cv::Rect> peds;
#else
static cv_tracker::image_obj_ranged car_fused_objects;
static cv_tracker::image_obj_ranged pedestrian_fused_objects;
#endif

/* check whether floating value x is nearly 0 or not */
static inline bool isNearlyNODATA(float x)
{
	float abs_x  = (float)fabs(x);
	const int rangeScale = 100;
	return(abs_x < FLT_MIN*rangeScale);
}

static std::vector<cv::Scalar> _colors;

#define	IMAGE_WIDTH		800
#define	IMAGE_HEIGHT 	600

#define POINTS_THRESHOLD 0.1

static const int OBJ_RECT_THICKNESS = 3;

static void drawRects(cv::Mat image,
                    std::vector<cv_tracker::image_rect_ranged> objects,
					CvScalar color,
					int threshold_height,
					std::string objectClass)
{
	int object_num = objects.size();
	char distance_string[32];
	int fontFace = cv::FONT_HERSHEY_SIMPLEX; double fontScale = 0.55; int fontThick = 2;
	std::vector<int> pointsInBoundingBox;
	for(int i = 0; i < object_num; i++)
	{
		//corner_point[0]=>X1		corner_point[1]=>Y1
		//corner_point[2]=>width	corner_point[3]=>height
		cv::Rect detection = cv::Rect(objects.at(i).rect.x, objects.at(i).rect.y, objects.at(i).rect.width, objects.at(i).rect.height);

		rectangle(image, detection, color, OBJ_RECT_THICKNESS);//draw bounding box
		putText(image, objectClass, cv::Point(detection.x + 4, detection.y + 10), fontFace, fontScale, color, fontThick);//draw label text

		sprintf(distance_string, "D:%.2f m H:%.1f,%.1f", objects.at(i).range / 100, objects.at(i).min_height, objects.at(i).max_height);
		//Size textSize = getTextSize(string(distance_string), fontFace, fontScale, fontThick, 0);
		//rectangle(image, cv::Rect( detection.x, detection.y, textSize.width + 4, textSize.height + 10), Scalar::all(0), CV_FILLED);//draw fill distance rectangle
		putText(image, std::string(distance_string), cv::Point(detection.x + 4, detection.y - 10), fontFace, fontScale, color, fontThick);//draw distance text
	}
}

static void drawVScanPoints(cv::Mat image)
{
	/* DRAW POINTS of lidar scanning */
    int w = image.size().width;
	int h = image.size().height;

	int i, n = w * h;
	float min_d = 1<<16, max_d = -1;
	//	int min_i = 1<<8, max_i = -1;
	for(i=0; i<n; i++){
		int di = points_msg->distance[i];
		max_d = di > max_d ? di : max_d;
		min_d = di < min_d ? di : min_d;
		// int it = points_msg->intensity[i];
		// max_i = it > max_i ? it : max_i;
		// min_i = it < min_i ? it : min_i;
	}
	float wid_d = max_d - min_d;

	int x, y;
	for(y=0; y<h; y++){
		for(x=0; x<w; x++){
			int i = y * w + x;
			double distance = points_msg->distance[i];

			if(distance == 0){
				continue;
			}
			int colorid= wid_d ? ( (distance - min_d) * 255 / wid_d ) : 128;
			cv::Vec3b color=colormap.at<cv::Vec3b>(colorid);
			int g = color[1];
			int b = color[2];
			int r = color[0];
			rectangle(image, cv::Rect(x, y,1, 1), cv::Scalar(r, g, b), OBJ_RECT_THICKNESS);
		}
	}
}

static void show(void)
{
	if(!existImage || !existPoints){
		return;
	}
	const auto& encoding = sensor_msgs::image_encodings::BGR8;
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, encoding);
	IplImage frame = cv_image->image;

	cv::Mat matImage(&frame, false);

	//Draw VScan Points
	drawVScanPoints(matImage);

	/* DRAW RECTANGLES of detected objects */
	drawRects(matImage,
		  car_fused_objects.obj,
		  cv::Scalar(255.0, 255.0, 0,0),
		  matImage.rows*.25,
		  car_fused_objects.type);

	drawRects(matImage,
		  pedestrian_fused_objects.obj,
		  cv::Scalar(0.0, 255.0, 0,0),
		  matImage.rows*.25,
		  pedestrian_fused_objects.type);

	if (cvGetWindowHandle(window_name) != NULL) // Guard not to write destroyed window by using close button on the window
	{
		cvShowImage(window_name, &frame);
		cvWaitKey(2);
	}
}
static void car_updater_callback(const cv_tracker::image_obj_ranged& fused_car_msg)
{
	car_fused_objects = fused_car_msg;
	//  show();
}

static void ped_updater_callback(const cv_tracker::image_obj_ranged& fused_pds_msg)
{
  pedestrian_fused_objects = fused_pds_msg;
  //  show();
}

static void image_cb(const sensor_msgs::Image& msg)
{
	image_msg = msg;
	existImage = true;
	show();
}

static void points_cb(const points2image::PointsImageConstPtr& msg)
{
	points_msg = msg;
	existPoints = true;
	show();
}

int main(int argc, char **argv)
{
	/* create resizable window */
	cvNamedWindow(window_name, CV_WINDOW_NORMAL);
	cvStartWindowThread();

	ros::init(argc, argv, "vscan_image_d_viewer");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	std::string image_node;
	std::string car_node;
	std::string pedestrian_node;
	std::string points_node;

	if (private_nh.getParam("image_node", image_node))
	{
		ROS_INFO("Setting image node to %s", image_node.c_str());
	}
	else
	{
		ROS_INFO("No image node received, defaulting to image_raw, you can use _image_node:=YOUR_NODE");
		image_node = "/image_raw";
	}

	if (private_nh.getParam("car_node", car_node))
	{
		ROS_INFO("Setting car positions node to %s", car_node.c_str());
	}
	else
	{
		ROS_INFO("No car positions node received, defaulting to car_pixel_xyz, you can use _car_node:=YOUR_TOPIC");
		car_node = "/obj_car/image_obj_ranged";
	}

	if (private_nh.getParam("pedestrian_node", pedestrian_node))
	{
		ROS_INFO("Setting pedestrian positions node to %s", pedestrian_node.c_str());
	}
	else
	{
		ROS_INFO("No pedestrian positions node received, defaulting to pedestrian_pixel_xyz, you can use _pedestrian_node:=YOUR_TOPIC");
		pedestrian_node = "/obj_person/image_obj_ranged";
	}

	if (private_nh.getParam("points_node", points_node))
	{
		ROS_INFO("Setting pedestrian positions node to %s", points_node.c_str());
	}
	else
	{
		ROS_INFO("No points node received, defaulting to points_image, you can use _points_node:=YOUR_TOPIC");
		points_node = "/vscan_image";
	}

	cv::generateColors(_colors, 25);

	ros::Subscriber scriber = n.subscribe(image_node, 1,
					    image_cb);
	ros::Subscriber scriber_car = n.subscribe(car_node, 1,
						car_updater_callback);
	ros::Subscriber scriber_ped = n.subscribe(pedestrian_node, 1,
						ped_updater_callback);
	ros::Subscriber scriber_points = n.subscribe(points_node, 1,
						points_cb);

	cv::Mat grayscale(256,1,CV_8UC1);
	for(int i=0;i<256;i++)
	{
		grayscale.at<uchar>(i)=i;
	}
	cv::applyColorMap(grayscale,colormap,cv::COLORMAP_JET);

	ros::spin();

	cvDestroyWindow(window_name);

	return 0;
}
