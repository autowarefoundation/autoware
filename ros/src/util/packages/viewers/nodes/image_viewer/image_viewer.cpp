/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *	list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *	this list of conditions and the following disclaimer in the documentation
 *	and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *	contributors may be used to endorse or promote products derived from
 *	this software without specific prior written permission.
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

#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/contrib/contrib.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <dpm/ImageObjects.h>
#include <kf/KFObjects.h>

using namespace std;
using namespace cv;

/*		CARS 	*/
//DPM related
static vector<Rect> cars;					//objects detected
static vector<float> cars_score;			//score of each object
//KF related
static vector<Rect> cars_tracked;			//objects tracked for current frame
static vector<Rect> cars_tracked_predicted;	//objects tracked for next frame
static vector<int> cars_tracked_lifespan;	//remaining lifespan
static vector<int> cars_tracked_id;			//objects' id

/*		PEDS 	*/
//DPM related
static vector<Rect> peds;
static vector<float> peds_score;
//KF related
static vector<Rect> peds_tracked;
static vector<Rect> peds_tracked_predicted;
static vector<int> peds_tracked_lifespan;
static vector<int> peds_tracked_id;

static vector<Scalar> _colors;

static const int OBJ_RECT_THICKNESS = 3;

static bool _drawing = false;
static bool car_track_ready = false;
static bool car_dpm_ready = false;
static bool ped_track_ready = false;
static bool ped_dpm_ready = false;

void drawDetections(vector<Rect> dets, vector<float> scores, std::string objectLabel, IplImage frame)
{
	/* variables for object label */
	CvFont	  font;
	const float hscale	  = 0.5f;
	const float vscale	  = 0.5f;
	const float italicScale = 0.0f;
	const int   thickness   = 1;
	CvSize	  text_size;
	int		 baseline	= 0;


	cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, hscale, vscale, italicScale, thickness, CV_AA);

	//UNTRACKED
	for(std::size_t i=0; i<dets.size();i++)
	{
		//if(dets[i].y > frame.height*.3)//temporal way to avoid drawing detections in the sky
		{
			ostringstream label;
			label << objectLabel << ":" << std::setprecision(2) << scores[i];
			string text = label.str();

			//get text size
			cvGetTextSize(text.data(),
						&font,
						&text_size,
						&baseline);

			cvRectangle( &frame,
					cvPoint(dets[i].x, dets[i].y),
					cvPoint(dets[i].x+dets[i].width, dets[i].y+dets[i].height),
					CV_RGB(0, 0, 255), OBJ_RECT_THICKNESS, CV_AA, 0);

			/* draw object label */
			CvPoint textOrg = cvPoint(dets[i].x - OBJ_RECT_THICKNESS, dets[i].y - baseline - OBJ_RECT_THICKNESS);

			cvRectangle(&frame,
					cvPoint(textOrg.x + 0 , textOrg.y + baseline),
					cvPoint(textOrg.x + text_size.width, textOrg.y - text_size.height),
					CV_RGB(0, 0, 0), // text background is black
					-1, 8, 0
				);
			cvPutText(&frame,
					text.data(),
					textOrg,
					&font,
					CV_RGB(255, 255, 255) // text color is white
				);
		}
	}

	////
}

void drawTracked(vector<Rect> dets, vector<int> lifespan, vector<int> obj_id, std::string objectLabel, Mat imageTrack)
{
	for(std::size_t i=0; i<dets.size();i++)
	{
		//if(dets[i].y > imageTrack.rows*.3)//temporal way to avoid drawing detections in the sky
		{
			ostringstream label;
			label << objectLabel << "_" << obj_id[i] << ":" << std::setprecision(2) << lifespan[i];
			string text = label.str();
			rectangle(imageTrack, dets[i], _colors[obj_id[i]], 3);
			putText(imageTrack, text.c_str(), Point(dets[i].x + 4, dets[i].y + 15), FONT_HERSHEY_SIMPLEX, 0.55, _colors[obj_id[i]], 2);
		}
	}
}

static void image_viewer_callback(const sensor_msgs::Image& image_source)
{
	_drawing = true;

	const auto& encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source,
								 encoding);
	IplImage frame = cv_image->image;

	Mat matImage(cv_image->image);
	cvtColor(matImage, matImage, CV_BGR2RGB);
	Mat imageTrack = matImage.clone();

	//UNTRACKED
	putText(matImage, "PIXEL_XY", Point(10,10), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(0, 0, 255), 2);
	if (car_dpm_ready)
		drawDetections(cars, cars_score, "car", frame);
	if(ped_dpm_ready)
		drawDetections(peds, peds_score, "pedestrian", frame);

	//TRACKED
	putText(imageTrack, "PIXEL_XY_TRACKED", Point(10,10), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(0, 255, 0), 2);
	if(car_track_ready)
		drawTracked(cars_tracked, cars_tracked_lifespan, cars_tracked_id, "car", imageTrack);
	if(ped_track_ready)
		drawTracked(peds_tracked, peds_tracked_lifespan, peds_tracked_id, "pedestrian", imageTrack);
	////////////////////

	Mat merged;

	hconcat(matImage, imageTrack, merged);
	imshow("Image Viewer", merged);
	//cvShowImage("Image Viewer", &frame);
	cvWaitKey(2);

	_drawing = false;
}

static void car_updater_callback(const dpm::ImageObjects& image_objects_msg)
{
	if(_drawing)
		return;
	int num = image_objects_msg.car_num;
	vector<int> points = image_objects_msg.corner_point;
	cars_score = image_objects_msg.score;
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

	car_dpm_ready = true;
}

static void ped_updater_callback(const dpm::ImageObjects& image_objects_msg)
{
	if(_drawing)
			return;
	int num = image_objects_msg.car_num;
	vector<int> points = image_objects_msg.corner_point;
	peds_score.clear();
	peds_score = image_objects_msg.score;
	//points are X,Y,W,H and repeat for each instance
	peds.clear();
	for (int i=0; i<num;i++)
	{
		Rect tmp;
		tmp.x = points[i*4 + 0];
		tmp.y = points[i*4 + 1];
		tmp.width = points[i*4 + 2];
		tmp.height = points[i*4 + 3];
		peds.push_back(tmp);
	}
	ped_track_ready = true;
}

static void car_updater_callback_tracked(const kf::KFObjects& image_objects_msg)
{
	if(_drawing)
			return;
	int num = image_objects_msg.total_num;
	vector<int> points = image_objects_msg.corner_point;
	vector<int> pred_points = image_objects_msg.next_corner_point;

	cars_tracked_lifespan = image_objects_msg.lifespan;
	cars_tracked_id = image_objects_msg.obj_id;

	//points are X,Y,W,H and repeat for each instance
	cars_tracked.clear();

	for (int i=0; i<num;i++)
	{
		Rect tmp;
		tmp.x = points[i*4 + 0];
		tmp.y = points[i*4 + 1];
		tmp.width = points[i*4 + 2];
		tmp.height = points[i*4 + 3];
		cars_tracked.push_back(tmp);

		tmp.x = pred_points[i*4 + 0];
		tmp.y = pred_points[i*4 + 1];
		tmp.width = pred_points[i*4 + 2];
		tmp.height = pred_points[i*4 + 3];
		cars_tracked_predicted.push_back(tmp);
	}
	car_track_ready = true;
}

static void ped_updater_callback_tracked(const kf::KFObjects& image_objects_msg)
{
	if(_drawing)
			return;
	int num = image_objects_msg.total_num;
	vector<int> points = image_objects_msg.corner_point;
	vector<int> pred_points = image_objects_msg.next_corner_point;

	peds_tracked_lifespan = image_objects_msg.lifespan;
	peds_tracked_id = image_objects_msg.obj_id;

	//points are X,Y,W,H and repeat for each instance
	peds_tracked.clear();

	for (int i=0; i<num;i++)
	{
		Rect tmp;
		tmp.x = points[i*4 + 0];
		tmp.y = points[i*4 + 1];
		tmp.width = points[i*4 + 2];
		tmp.height = points[i*4 + 3];
		peds_tracked.push_back(tmp);

		tmp.x = pred_points[i*4 + 0];
		tmp.y = pred_points[i*4 + 1];
		tmp.width = pred_points[i*4 + 2];
		tmp.height = pred_points[i*4 + 3];
		peds_tracked_predicted.push_back(tmp);
	}
	ped_track_ready = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_viewer");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	string image_node;

	if (private_nh.getParam("image_node", image_node))
	{
		ROS_INFO("Setting image node to %s", image_node.c_str());
	}
	else
	{
		ROS_INFO("No image node received, defaulting to image_raw, you can use _image_node:=YOUR_NODE");
		image_node = "/image_raw";
	}

	cv::generateColors(_colors, 25);

	ros::Subscriber scriber = n.subscribe(image_node, 1,
						image_viewer_callback);

	ros::Subscriber scriber_car = n.subscribe("/car_pixel_xy", 1,
						car_updater_callback);
	ros::Subscriber scriber_ped = n.subscribe("/pedestrian_pixel_xy", 1,
						ped_updater_callback);

	ros::Subscriber scriber_ped_tracked = n.subscribe("/pedestrian_pixel_xy_tracked", 1,
						ped_updater_callback_tracked);
	ros::Subscriber scriber_car_tracked = n.subscribe("/car_pixel_xy_tracked", 1,
						car_updater_callback_tracked);

	ros::spin();
	return 0;
}
