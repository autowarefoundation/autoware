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

#include <string>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/contrib/contrib.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_tracker/image_obj_tracked.h>
#include <cv_tracker/image_obj.h>

//DPM related
static std::vector<cv::Rect> cars;		//objects detected
static std::vector<float> cars_score;		//score of each object
//KF related
static std::vector<cv::Rect> cars_tracked;	//objects tracked for current frame
static std::vector<int> cars_tracked_lifespan;	//remaining lifespan
static std::vector<int> cars_tracked_id;	//objects' id
static std::vector<int> cars_tracked_real_data;	//states if the data contained in the index is real or prediction

//DPM related
static std::vector<cv::Rect> peds;
static std::vector<float> peds_score;
//KF related
static std::vector<cv::Rect> peds_tracked;
static std::vector<int> peds_tracked_lifespan;
static std::vector<int> peds_tracked_id;
static std::vector<int> peds_tracked_real_data;

static std::vector<cv::Scalar> _colors;

static const int OBJ_RECT_THICKNESS = 3;

static bool _drawing = false;
static bool car_track_ready = false;
static bool car_dpm_ready = false;
static bool ped_track_ready = false;
static bool ped_dpm_ready = false;

static bool car_image_obj_ready = false;
static bool pedestrian_image_obj_ready = false;

static const std::string window_name = "Image Viewer";

/*static void dashed_rectangle(cv::Mat& img, const cv::Rect& r, const cv::Scalar& color,
			     int thickness = 2, int dash_length = 10)
{
	//draw horizontal dashed lines
	for (int i = 0; i < r.width; i+=dash_length) {
		cv::line(img, cv::Point(r.x+i, r.y),  cv::Point(r.x+i+(dash_length/2), r.y), color, thickness);
		cv::line(img, cv::Point(r.x+i, r.y + r.height), cv::Point(r.x+i+(dash_length/2), r.y + r.height), color, thickness);
	}

	//draw vertical dashes lines
	for (int i = 0; i < r.height; i+=dash_length) {
		cv::line(img, cv::Point(r.x, r.y+i), cv::Point(r.x, r.y+i+(dash_length/2)), color, thickness);
		cv::line(img, cv::Point(r.x +r.width, r.y+i), cv::Point(r.x+ r.width, r.y+i+(dash_length/2)), color, thickness);
	}
}*/

static void drawDetections(std::vector<cv::Rect> dets, std::vector<float> scores, std::string objectLabel, IplImage frame)
{
	/* variables for object label */
	CvFont font;
	const float hscale = 0.5f;
	const float vscale = 0.5f;
	const float italicScale = 0.0f;
	const int thickness = 1;
	CvSize text_size;
	int baseline = 0;

	cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, hscale, vscale, italicScale, thickness, CV_AA);

	//UNTRACKED
	for(std::size_t i = 0; i < dets.size(); ++i) {
#ifdef TEMP_DISABLED
		//temporal way to avoid drawing detections in the sky
		if (dets[i].y <= frame.height * 0.3)
			continue;
#endif
		std::ostringstream label;
		label << objectLabel << ":" << std::setprecision(2) << scores[i];
		std::string text = label.str();

		//get text size
		cvGetTextSize(text.data(),
			&font,
			&text_size,
			&baseline);

		//cvRectangle( &frame,
			//cvPoint(dets[i].x, dets[i].y),
			//cvPoint(dets[i].x+dets[i].width, dets[i].y+dets[i].height),
			//CV_RGB(0, 0, 255), OBJ_RECT_THICKNESS, CV_AA, 0);
		cvCircle(&frame, cvPoint(dets[i].x+dets[i].width/2, dets[i].y+dets[i].height/2), 30, cvScalar(0,255,0),3);		/* draw object label */
		CvPoint textOrg = cvPoint(dets[i].x - OBJ_RECT_THICKNESS, dets[i].y - baseline - OBJ_RECT_THICKNESS);

		cvRectangle(&frame,
			cvPoint(textOrg.x + 0 , textOrg.y + baseline),
			cvPoint(textOrg.x + text_size.width, textOrg.y - text_size.height),
			CV_RGB(0, 0, 0), // text background is black
			-1, 8, 0);
		cvPutText(&frame,
			text.data(),
			textOrg,
			&font,
			CV_RGB(255, 255, 255) // text color is white
			);
	}
}

static void drawTracked(std::vector<cv::Rect> dets, std::vector<int> lifespan, std::vector<int> obj_id,
			std::vector<int> real_data, std::string objectLabel, cv::Mat imageTrack)
{
	for(std::size_t i=0; i<dets.size();i++) {
#ifdef TEMP_DISABLED
		//temporal way to avoid drawing detections in the sky
		if (dets[i].y <= imageTrack.rows * 0.3)
			continue;
#endif

		std::ostringstream label;
		label << objectLabel << "_" << obj_id[i] << ":" << std::setprecision(2) << lifespan[i];
		std::string text = label.str();

		//if (real_data[i])
			//rectangle(imageTrack, dets[i], _colors[obj_id[i]], 3);
	//	else
			//dashed_rectangle(imageTrack, dets[i], _colors[obj_id[i]], 3, 10);

		putText(imageTrack, text.c_str(), cv::Point(dets[i].x + 4, dets[i].y + 15),
			cv::FONT_HERSHEY_SIMPLEX, 0.55, _colors[obj_id[i]], 2);
		cv::circle(imageTrack, cv::Point(dets[i].x+dets[i].width/2, dets[i].y+dets[i].height/2), 30, _colors[obj_id[i]],3);
	}
}

static void image_viewer_callback(const sensor_msgs::Image& image_source)
{
	_drawing = true;

	const auto& encoding = sensor_msgs::image_encodings::BGR8;
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source,
							     encoding);
	IplImage frame = cv_image->image;

	cv::Mat matImage(cv_image->image);
	cv::Mat imageTrack = matImage.clone();

	//UNTRACKED
	putText(matImage, "PIXEL_XY", cv::Point(10,10), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 255), 2);
	if (car_dpm_ready)
		drawDetections(cars, cars_score, "car", frame);
	if (ped_dpm_ready)
		drawDetections(peds, peds_score, "pedestrian", frame);

	if (car_image_obj_ready)
		drawDetections(cars, cars_score, "car", frame);
	if (pedestrian_image_obj_ready)
		drawDetections(peds, peds_score, "pedestrian", frame);

	//TRACKED
	putText(imageTrack, "PIXEL_XY_TRACKED", cv::Point(10,10), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2);
	if(car_track_ready)
		drawTracked(cars_tracked, cars_tracked_lifespan, cars_tracked_id, cars_tracked_real_data, "car", imageTrack);
	if(ped_track_ready)
		drawTracked(peds_tracked, peds_tracked_lifespan, peds_tracked_id, peds_tracked_real_data, "pedestrian", imageTrack);

	cv::Mat merged;
	hconcat(matImage, imageTrack, merged);

	if (cvGetWindowHandle(window_name.c_str()) != NULL) // Guard not to write destroyed window by using close button on the window
		{
			imshow(window_name, merged);
			cvWaitKey(2);
		}

	_drawing = false;
}

static void image_obj_update_cb(const cv_tracker::image_obj& image_objs)
{
	if(_drawing)
		return;

	bool is_car = (image_objs.type == "car");
	std::vector<cv::Rect>& objs = is_car ? cars : peds;
	std::vector<float>& scores = is_car ? cars_score : peds_score;
	
	objs.clear();
	scores.clear();

	for (const auto& obj : image_objs.obj) {
		cv::Rect tmp;
		tmp.x = obj.x;
		tmp.y = obj.y;
		tmp.width = obj.width;
		tmp.height = obj.height;

		objs.push_back(tmp);
		scores.push_back(obj.score);
	}

	if (is_car) {
		car_image_obj_ready = true;
	} else {
		pedestrian_image_obj_ready = true;
	}
}

static void image_obj_updater_cb_tracked(const cv_tracker::image_obj_tracked& image_objs_tracked_msg)
{
	if(_drawing)
		return;
	bool is_car = (image_objs_tracked_msg.type == "car");
	std::vector<cv::Rect>& objs_tracked = is_car ? cars_tracked : peds_tracked;
	std::vector<int>& objs_tracked_lifespan = is_car ? cars_tracked_lifespan : peds_tracked_lifespan;
	std::vector<int>& objs_tracked_id = is_car ? cars_tracked_id : peds_tracked_id;
	std::vector<int>& objs_tracked_real_data = is_car ? cars_tracked_real_data : peds_tracked_real_data;

	objs_tracked_lifespan = image_objs_tracked_msg.lifespan;
	objs_tracked_id = image_objs_tracked_msg.obj_id;
	objs_tracked_real_data = image_objs_tracked_msg.real_data;

	objs_tracked.clear();
	for (const auto& rect_ranged : image_objs_tracked_msg.rect_ranged)
		{
			cv::Rect tmp;
			tmp.x = rect_ranged.rect.x;
			tmp.y = rect_ranged.rect.y;
			tmp.width = rect_ranged.rect.width;
			tmp.height = rect_ranged.rect.height;

			objs_tracked.push_back(tmp);
		}

	if(is_car) {
		car_track_ready = true;
	} else {
		ped_track_ready = true;
	}
}

int main(int argc, char **argv)
{

	/* create resizable window */
	cv::namedWindow(window_name, cv::WINDOW_NORMAL);
	cv::startWindowThread();

	ros::init(argc, argv, "image_viewer");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	std::string image_node;

	if (private_nh.getParam("image_node", image_node)) {
		ROS_INFO("Setting image node to %s", image_node.c_str());
	} else {
		ROS_INFO("No image node received, defaulting to image_raw, you can use _image_node:=YOUR_NODE");
		image_node = "/image_raw";
	}

	cv::generateColors(_colors, 25);

	ros::Subscriber scriber = n.subscribe(image_node, 1, image_viewer_callback);

	ros::Subscriber scriber_car = n.subscribe("/obj_car/image_obj", 1,
						image_obj_update_cb);
	ros::Subscriber scriber_ped = n.subscribe("/obj_person/image_obj", 1,
						image_obj_update_cb);

	ros::Subscriber scriber_ped_tracked = n.subscribe("/obj_person/image_obj_tracked", 1,
						image_obj_updater_cb_tracked);
	ros::Subscriber scriber_car_tracked = n.subscribe("/obj_car/image_obj_tracked", 1,
						image_obj_updater_cb_tracked);

	ros::spin();

	/* destroy window */
	cv::destroyWindow(window_name);

	return 0;
}
