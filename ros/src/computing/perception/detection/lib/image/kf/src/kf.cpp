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

//ROS STUFF
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <runtime_manager/ConfigCarKf.h>
#include <cv_tracker/image_obj_ranged.h>

#include <cv_tracker/image_obj_tracked.h>
#include <std_msgs/Header.h>

//TRACKING STUFF
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <stdio.h>

#include <sstream>
#include <algorithm>
#include <iterator>

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

ros::Publisher image_objects;//ROS

static int 			DEFAULT_LIFESPAN; //LIFESPAN of objects before stop being tracked, in frames
static int	 		INITIAL_LIFESPAN; //LIFESPAN of objects before stop being tracked, in frames
static int			ORB_NUM_FEATURES;
static unsigned int	ORB_MIN_MATCHES;
static float		ORB_KNN_RATIO;
static float 		NOISE_COV;
static float 		MEAS_NOISE_COV;
static float 		ERROR_ESTIMATE_COV;
static float 		OVERLAPPING_PERC;
static bool 		SHOW_PREDICTIONS;
static bool 		USE_ORB;

static bool 		track_ready_;
static bool 		detect_ready_;
static cv_tracker::image_obj_tracked kf_objects_msg_;

struct kstate
{
	cv::KalmanFilter	KF;//KalmanFilter for this object
	cv::Rect		pos;//position of the object centerx, centery, width, height
	float			score;//DPM score
	bool			active;//if too old (lifespan) don't use
	unsigned int		id;//id of this tracked object
	cv::Mat			image;//image containing the detected and tracked object
	int			lifespan;//remaining lifespan before deprecate
	cv::LatentSvmDetector::ObjectDetection obj;//currently not used
	cv::Scalar	color;
	int		real_data;
	//std::vector<KeyPoint> orbKeypoints;
	//cv::Mat				orbDescriptors;
	float range;//range to this object gotten by range_fusion
	float min_height;//minimum height detected by range_fusion
	float max_height;//maximum height detected by range_fusion
};

//tracking required code
std::vector<kstate> 	_kstates;
std::vector<bool> 	_active;
std::vector<cv::Scalar>	_colors;
std::vector<cv::LatentSvmDetector::ObjectDetection> _dpm_detections;

std::string object_type;
std::vector<float> _ranges;
std::vector<float> _min_heights;
std::vector<float> _max_heights;

std_msgs::Header    image_objects_header;

bool _ready =false;

long int _counter = 0;
//

void getRectFromPoints(std::vector< cv::Point2f > corners, cv::Rect& outBoundingBox)
{
	int min_x=0, min_y=0, max_x=0, max_y=0;
	for (unsigned int i=0; i<corners.size(); i++)
	{
		if (corners[i].x > 0)
		{
			if (corners[i].x < min_x)
				min_x = corners[i].x;
			if (corners[i].x>max_x)
				max_x = corners[i].x;
		}
		if (corners[i].y > 0)
		{
			if (corners[i].y < min_y)
				min_y = corners[i].y;
			if (corners[i].y > max_y)
				max_y = corners[i].y;
		}
	}
	outBoundingBox.x 		= min_x;
	outBoundingBox.y 		= min_y;
	outBoundingBox.width 	= max_x - min_x;
	outBoundingBox.height 	= max_y - min_y;

}

bool orbMatch(cv::Mat& inImageScene, cv::Mat& inImageObj, cv::Rect& outBoundingBox, unsigned int inMinMatches=2, float inKnnRatio=0.7)
{
	//vector of keypoints
	std::vector< cv::KeyPoint > keypointsO;
	std::vector< cv::KeyPoint > keypointsS;

	cv::Mat descriptors_object, descriptors_scene;

	cv::Mat outImg;
	inImageScene.copyTo(outImg);

	//-- Step 1: Extract keypoints
	cv::OrbFeatureDetector orb(ORB_NUM_FEATURES);
	orb.detect(inImageScene, keypointsS);
	if (keypointsS.size() < ORB_MIN_MATCHES)
	{
		//cout << "Not enough keypoints S, object not found>" << keypointsS.size() << endl;
		return false;
	}
	orb.detect(inImageObj, keypointsO);
	if (keypointsO.size() < ORB_MIN_MATCHES)
	{
		//cout << "Not enough keypoints O, object not found>" << keypointsO.size() << endl;
		return false;
	}

	//Calculate descriptors (feature vectors)
	cv::OrbDescriptorExtractor extractor;
	extractor.compute(inImageScene, keypointsS, descriptors_scene);
	extractor.compute(inImageObj, keypointsO, descriptors_object);

	//Matching descriptor vectors using FLANN matcher
	cv::BFMatcher matcher;
	//descriptors_scene.size(), keypointsO.size(), keypointsS.size();
	std::vector< std::vector< cv::DMatch >  > matches;
	matcher.knnMatch(descriptors_object, descriptors_scene, matches, 2);
	std::vector< cv::DMatch > good_matches;
	good_matches.reserve(matches.size());

	for (size_t i = 0; i < matches.size(); ++i)
	{
		if (matches[i].size() < 3)
			continue;

		const cv::DMatch &m1 = matches[i][0];
		const cv::DMatch &m2 = matches[i][1];

		if (m1.distance <= inKnnRatio * m2.distance)
			good_matches.push_back(m1);
	}

	if ((good_matches.size() >= inMinMatches))
	{
		std::vector< cv::Point2f > obj;
		std::vector< cv::Point2f > scene;

		for (unsigned int i = 0; i < good_matches.size(); i++)
		{
			// Get the keypoints from the good matches
			obj.push_back(keypointsO[good_matches[i].queryIdx].pt);
			scene.push_back(keypointsS[good_matches[i].trainIdx].pt);
		}

		cv::Mat H = findHomography(obj, scene, CV_RANSAC);

		// Get the corners from the image_1 ( the object to be "detected" )
		std::vector< cv::Point2f > obj_corners(4);
		obj_corners[0] = cvPoint(0, 0); obj_corners[1] = cvPoint(inImageObj.cols, 0);
		obj_corners[2] = cvPoint(inImageObj.cols, inImageObj.rows); obj_corners[3] = cvPoint(0, inImageObj.rows);
		std::vector< cv::Point2f > scene_corners(4);

		perspectiveTransform(obj_corners, scene_corners, H);

		// Draw lines between the corners (the mapped object in the scene - image_2 )
		line(outImg, scene_corners[0], scene_corners[1], cv::Scalar(255, 0, 0), 2); //TOP line
		line(outImg, scene_corners[1], scene_corners[2], cv::Scalar(255, 0, 0), 2);
		line(outImg, scene_corners[2], scene_corners[3], cv::Scalar(255, 0, 0), 2);
		line(outImg, scene_corners[3], scene_corners[0], cv::Scalar(255, 0, 0), 2);

		//imshow("Scene", outImg);
		//imshow("Obj", inImageObj);
		//cvWaitKey(5);

		return true;
	}

	return false;
}

///Returns true if an im1 is contained in im2 or viceversa
bool crossCorr(cv::Mat im1, cv::Mat im2)
{
	//im1 roi from the previous frame
	//im2 roi fromcurrent frame
	if (im1.rows <= 0 || im1.cols <= 0 || im2.rows <= 0 || im2.cols <= 0)
		return false;

	cv::Mat result, larger_im, smaller_im;

	/// Create the result matrix
	int result_cols;
	int result_rows;

	//select largest image
	if (im2.cols > im1.cols)
	{
		larger_im = im2;
		smaller_im = im1;
	}
	else
	{
		larger_im = im1;
		smaller_im = im2;
	}
	//check rows to be also larger otherwise crop the smaller to remove extra rows
	if (larger_im.rows < smaller_im.rows)
	{
		//add rows to match sizes
		cv::Mat rows = cv::Mat::ones(smaller_im.rows - larger_im.rows, larger_im.cols, larger_im.type());
		larger_im.push_back(rows);
	}

	result_cols = larger_im.cols - smaller_im.cols + 1;
	result_rows = larger_im.rows - smaller_im.rows + 1;
	result.create(result_cols, result_rows, CV_32FC1);

	/// Do the Matching and Normalize
	matchTemplate(larger_im, smaller_im, result, CV_TM_CCORR_NORMED);
	//normalize(result, result, 0, 1, NORM_MINMAX, -1, cv::Mat());

	/// Localizing the best match with minMaxLoc
	double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
	cv::Point matchLoc;

	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

	matchLoc = maxLoc;

	/// Show me what you got
	cv::Mat scene = larger_im.clone();
	rectangle(scene, matchLoc, cv::Point(matchLoc.x + smaller_im.cols, matchLoc.y + smaller_im.rows), cv::Scalar(0, 0, 255), 2, 8, 0);
	//imshow(image_window, scene);
	//imshow(result_window, result);

	//if (maxVal>0.89 && minVal <0.3)
	bool ret;
	int thresWidth = (larger_im.cols)*.7;
	if ( (maxVal > 0.5) && (smaller_im.cols > thresWidth) )//good threshold and consistent size
	{

		//std::cout << "matched" << endl;
		ret = true;
	}
	else
	{
		//std::cout << "non matched" << endl;
		ret = false;
	}
	//cv::imshow("match1", scene);
	//cv::imshow("match2", smaller_im);

	return ret;
}

void posScaleToBbox(std::vector<kstate> kstates, std::vector<kstate>& trackedDetections)
{
	for (unsigned int i = 0; i < kstates.size(); i++)
	{
		if (kstates[i].active)
		{
			kstate tmp;
			tmp.pos.x = kstates[i].pos.x;// -(kstates[i].pos.width / 2);
			tmp.pos.y = kstates[i].pos.y;// -(kstates[i].pos.height / 2);
			tmp.pos.width = kstates[i].pos.width;
			tmp.pos.height = kstates[i].pos.height;
			tmp.color = kstates[i].color;
			tmp.id = kstates[i].id;
			tmp.score = kstates[i].score;
			tmp.lifespan = kstates[i].lifespan;
			tmp.real_data = kstates[i].real_data;
			tmp.range = kstates[i].range;
			tmp.min_height = kstates[i].min_height;
			tmp.max_height = kstates[i].max_height;

			//fill in also LAtentSvm object
			tmp.obj.rect = tmp.pos;
			tmp.obj.score = tmp.score;

			if (tmp.pos.x < 0)
				tmp.pos.x = 0;
			if (tmp.pos.y < 0)
				tmp.pos.y = 0;

			trackedDetections.push_back(tmp);
		}
	}
}

int getAvailableIndex(std::vector<kstate>& kstates)
{
	unsigned int cur_size = kstates.size();
	std::vector<bool> ids;

	ids.resize(cur_size, false);

	for (unsigned int i=0; i<cur_size;i++)
	{
		ids[kstates[i].id]= true;
	}
	for (unsigned int i=0; i<cur_size;i++)
	{
		if(ids[i] == false)
			return i;
	}
	return cur_size;
}

void initTracking(cv::LatentSvmDetector::ObjectDetection object, std::vector<kstate>& kstates,
		  cv::LatentSvmDetector::ObjectDetection detection,
		  cv::Mat& image, std::vector<cv::Scalar> colors, float range)
{
	kstate new_state;
	//cv::KalmanFilter KF(4, 2, 0);//XY Only
	cv::KalmanFilter KF(8, 4, 0);

	/*cv::Mat_<float> measurement = (cv::Mat_<float>(2, 1) << object.rect.x,//XY Only
		object.rect.y);*/
	cv::Mat_<float> measurement = (cv::Mat_<float>(4, 1) << object.rect.x,
		object.rect.y, object.rect.width, object.rect.height);

	/*KF.transitioncv::Matrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,//XY Only
												0, 1, 0, 1,
												0, 0, 1, 0,
												0, 0, 0, 1);*/
	KF.transitionMatrix = (cv::Mat_<float>(8, 8)
	<<	1, 0, 0, 0, 1, 0, 0, 0,
		0, 1, 0, 0, 0, 1, 0, 0,
		0, 0, 1, 0, 0, 0, 1, 0,
		0, 0, 0, 1, 0, 0, 0, 1,
		0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 1);

	//init pre
	KF.statePre.at<float>(0) = object.rect.x;
	KF.statePre.at<float>(1) = object.rect.y;
	KF.statePre.at<float>(2) = object.rect.width;//XY Only
	KF.statePre.at<float>(3) = object.rect.height;//XY Only
	//init post
	KF.statePost.at<float>(0) = object.rect.x;
	KF.statePost.at<float>(1) = object.rect.y;
	KF.statePost.at<float>(2) = object.rect.width;//XY Only
	KF.statePost.at<float>(3) = object.rect.height;//XY Only

	cv::setIdentity(KF.measurementMatrix);
	cv::setIdentity(KF.processNoiseCov, cv:: Scalar::all(NOISE_COV));//1e-4
	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(MEAS_NOISE_COV));//1e-3
	cv::setIdentity(KF.errorCovPost, cv::Scalar::all(ERROR_ESTIMATE_COV));//100

	//clip detection
	//check that predicted positions are inside the image
	if (detection.rect.x < 0)
		detection.rect.x = 0;
	if (detection.rect.x > image.cols)
		detection.rect.x = image.cols - 1;
	if (detection.rect.y < 0)
		detection.rect.y = 0;
	if (detection.rect.height > image.rows)
		detection.rect.height = image.rows - 1;
	if (detection.rect.width + detection.rect.x > image.cols)
		detection.rect.width = image.cols - detection.rect.x;
	if (detection.rect.height + detection.rect.y > image.rows)
		detection.rect.height = image.rows - detection.rect.y;

	//save data to kstate
	new_state.active = true;
	new_state.image = image(cv::Rect(detection.rect.x,
		detection.rect.y,
		detection.rect.width,
		detection.rect.height)).clone();//Crop image and obtain only object (ROI)
	new_state.KF = KF;
	new_state.lifespan = INITIAL_LIFESPAN;//start only with 1
	new_state.pos = object.rect;
	new_state.score = object.score;
	new_state.id = getAvailableIndex(kstates);
	new_state.color = colors[new_state.id];
	new_state.real_data = 1;
	new_state.range = range;

	//extractOrbFeatures(new_state.image, new_state.orbKeypoints, new_state.orbDescriptors, ORB_NUM_FEATURES);

	kstates.push_back(new_state);

}

//checks whether an index was previously removed
bool isInRemoved(std::vector<unsigned int> removedIndices, unsigned int index)
{
	for (unsigned int i=0; i< removedIndices.size(); i++)
	{
		if (index == removedIndices[i])
			return true;
	}
	return false;
}

void removeUnusedObjects(std::vector<kstate>& states)
{
	std::vector<kstate>::iterator it;
	for(it = states.begin(); it != states.end();)
	{
		if (!(it->active))
			it = states.erase(it);
		else
			it++;
	}
}

bool alreadyMatched(int check_index, std::vector<int>& matched_indices)
{
	for (unsigned int i = 0; i < matched_indices.size(); i++)
	{
		if (matched_indices[i] == check_index)
			return true;
	}
	return false;
}

void Sort(const std::vector<float> in_scores, std::vector<unsigned int>& in_out_indices)
{
	for (unsigned int i = 0; i < in_scores.size(); i++)
		for (unsigned int j = i + 1; j < in_scores.size(); j++)
		{
			if (in_scores[in_out_indices[j]] > in_scores[in_out_indices[i]])
			{
				//float x_tmp = x[i];
				int index_tmp = in_out_indices[i];
				//x[i] = x[j];
				in_out_indices[i] = in_out_indices[j];
				//x[j] = x_tmp;
				in_out_indices[j] = index_tmp;
			}
		}
}

void ApplyNonMaximumSuppresion(std::vector< kstate >& in_source, float in_nms_threshold)
{
	std::vector< kstate > tmp_source = in_source;

	if (tmp_source.empty())
		return ;

	unsigned int size = in_source.size();

	std::vector<float> area(size);
	std::vector<float> scores(size);
	std::vector<int> x1(size);
	std::vector<int> y1(size);
	std::vector<int> x2(size);
	std::vector<int> y2(size);
	std::vector<unsigned int> indices(size);
	std::vector<bool> is_suppresed(size);

	for(unsigned int i = 0; i< in_source.size(); i++)
	{
		kstate tmp = in_source[i];
		area[i] = tmp.pos.width * tmp.pos.height;
		indices[i] = i;
		is_suppresed[i] = false;
		scores[i] = tmp.score;
		x1[i] = tmp.pos.x;
		y1[i] = tmp.pos.y;
		x2[i] = tmp.pos.width + tmp.pos.x;
		y2[i] = tmp.pos.height + tmp.pos.y;
	}

	Sort(scores, indices);//returns indices ordered based on scores

	for(unsigned int i=0; i< size; i++)
	{
		if(!is_suppresed[indices[i]])
		{
			for(unsigned int j= i+1; j< size; j++)
			{
				int x1_max = std::max(x1[indices[i]], x1[indices[j]]);
				int x2_min = std::min(x2[indices[i]], x2[indices[j]]);
				int y1_max = std::max(y1[indices[i]], y1[indices[j]]);
				int y2_min = std::min(y2[indices[i]], y2[indices[j]]);
				int overlap_width = x2_min - x1_max + 1;
				int overlap_height = y2_min - y1_max + 1;
				if(overlap_width > 0 && overlap_height>0)
				{
					float overlap_part = (overlap_width*overlap_height)/area[indices[j]];
					if(overlap_part > in_nms_threshold)
					{
						is_suppresed[indices[j]] = true;
					}
				}
			}
		}
	}

	unsigned int size_out = 0;
	for (unsigned int i = 0; i < size; i++)
	{
		if (!is_suppresed[i])
			size_out++;
	}

	std::vector< kstate > filtered_detections(size_out);

	unsigned int index = 0;
	for(unsigned int i = 0 ; i < size_out; i++)
	{
		if(!is_suppresed[indices[i]])
		{
			filtered_detections[index] = in_source[indices[i]];//x1[indices[i]];
			index++;
		}
	}
	in_source = filtered_detections;
}

void doTracking(std::vector<cv::LatentSvmDetector::ObjectDetection>& detections, int frameNumber,
		std::vector<kstate>& kstates, std::vector<bool>& active, cv::Mat& image,
		std::vector<kstate>& trackedDetections, std::vector<cv::Scalar> & colors)
{
	std::vector<cv::LatentSvmDetector::ObjectDetection> objects;
	//vector<LatentSvmDetector::ObjectDetection> tracked_objects;
	std::vector<bool> predict_indices;//this will correspond to kstates i
	std::vector<bool> correct_indices;//this will correspond to kstates i
	std::vector<int> correct_detection_indices;//this will correspond to kstates i, used to store the index of the corresponding object
	std::vector<bool> add_as_new_indices;//this will correspond to detections j

	//predict_indices.assign(kstates.size(), true);//always predict
	correct_indices.assign(kstates.size(), false);//correct only those matched
	correct_detection_indices.assign(kstates.size(), false);//correct only those matched
	add_as_new_indices.assign(detections.size(), true);//if the detection was not found add as new

	//Convert Bounding box coordinates from (x1,y1,w,h) to (BoxCenterX, BoxCenterY, width, height)
	objects = detections;//bboxToPosScale(detections);

	std::vector<int> already_matched;
	//compare detections from this frame with tracked objects
	for (unsigned int j = 0; j < detections.size(); j++)
	{
		for (unsigned int i = 0; i < kstates.size(); i++)
		{
			//compare only to active tracked objects(not too old)
			if (kstates[i].active)
			{
				//extend the roi 20%
				int new_x = (detections[j].rect.x - detections[j].rect.width*.1);
				int new_y = (detections[j].rect.y - detections[j].rect.height*.1);

				if (new_x < 0)			new_x = 0;
				if (new_x > image.cols)	new_x = image.cols;
				if (new_y < 0)			new_y = 0;
				if (new_y > image.rows) new_y = image.rows;

				int new_width = detections[j].rect.width*1.2;
				int new_height = detections[j].rect.height*1.2;

				if (new_width  + new_x > image.cols)	new_width  = image.cols - new_x;
				if (new_height + new_y > image.rows)	new_height = image.rows - new_y;

				cv::Rect roi_20(new_x, new_y, new_width, new_height);
				//cv::Rect roi(detections[j].rect);
				cv::Rect roi(roi_20);
				cv::Mat currentObjectROI = image(roi).clone();//Crop image and obtain only object (ROI)

				//cv::Rect intersect = detections[j].rect & kstates[i].pos;//check overlapping

				cv::Rect boundingbox;
				bool matched = false;
				//try to match with previous frame
				if ( !USE_ORB )
					matched = ( !alreadyMatched(j, already_matched) && crossCorr(kstates[i].image, currentObjectROI));
				else
					matched = (!alreadyMatched(j, already_matched) && orbMatch(currentObjectROI, kstates[i].image, boundingbox, ORB_MIN_MATCHES, ORB_KNN_RATIO));

				if(matched)
				{
					correct_indices[i] = true;//if ROI on this frame is matched to a previous object, correct
					correct_detection_indices[i] = j;//store the index of the detection corresponding to matched kstate
					add_as_new_indices[j] = false;//if matched do not add as new
					//kstates[i].image = currentObjectROI;//update image with current frame data
					kstates[i].score = detections[j].score;
					kstates[i].range = _ranges[j];
					already_matched.push_back(j);
				}//crossCorr

			}//kstates[i].active
		}//for (int i = 0; i < kstates.size(); i++)
	}//for (int j = 0; j < detections.size(); j++)


	//do prediction and correction for the marked states
	for (unsigned int i = 0; i < kstates.size(); i++)
	{
		if (kstates[i].active)//predict and correct only active states
		{
			//update params before predicting
			cv::setIdentity(kstates[i].KF.measurementMatrix);
			cv::setIdentity(kstates[i].KF.processNoiseCov, cv::Scalar::all(NOISE_COV));//1e-4
			cv::setIdentity(kstates[i].KF.measurementNoiseCov, cv::Scalar::all(MEAS_NOISE_COV));//1e-3
			cv::setIdentity(kstates[i].KF.errorCovPost, cv::Scalar::all(ERROR_ESTIMATE_COV));//100

			cv::Mat prediction = kstates[i].KF.predict();
			cv::Mat correction;
			kstates[i].pos.x = prediction.at<float>(0);
			kstates[i].pos.y = prediction.at<float>(1);
			kstates[i].pos.width = prediction.at<float>(2);
			kstates[i].pos.height = prediction.at<float>(3);
			kstates[i].real_data = 0;
			kstates[i].range = 0.0f;//fixed to zero temporarily as this is not real_data
			kstates[i].min_height = 0.0f;//fixed to zero temporarily as this is not real_data
			kstates[i].max_height = 0.0f;//fixed to zero temporarily as this is not real_data

			//now do respective corrections on KFs (updates)
			if (correct_indices[i])
			{
				//a match was found hence update KF measurement
				int j = correct_detection_indices[i];//obtain the index of the detection

				//cv::Mat_<float> measurement = (cv::Mat_<float>(2, 1) << objects[j].rect.x, //XY ONLY
				//												objects[j].rect.y);
				cv::Mat_<float> measurement = (cv::Mat_<float>(4, 1) << objects[j].rect.x,
					objects[j].rect.y,
					objects[j].rect.width,
					objects[j].rect.height);

				correction = kstates[i].KF.correct(measurement);//UPDATE KF with new info
				kstates[i].lifespan = DEFAULT_LIFESPAN; //RESET Lifespan of object

				//kstates[i].pos.width = objects[j].rect.width;//XY ONLY
				//kstates[i].pos.height = objects[j].rect.height;//XY ONLY

				//use real data instead of predicted if set
				kstates[i].pos.x = objects[j].rect.x;
				kstates[i].pos.y = objects[j].rect.y;
				kstates[i].pos.width = objects[j].rect.width;
				kstates[i].pos.height = objects[j].rect.height;
				kstates[i].real_data = 1;
				//cv::Mat im1 = image(kstates[i].pos);
				//cv::Mat im2 = image(objects[j].rect);
				kstates[i].range = _ranges[j];
				kstates[i].min_height = _min_heights[j];
				kstates[i].max_height = _max_heights[j];
			}


			//check that new widths and heights don't go beyond the image size
			if (kstates[i].pos.width + kstates[i].pos.x > image.cols)
				kstates[i].pos.width = image.cols - kstates[i].pos.x;
			if (kstates[i].pos.height + kstates[i].pos.y > image.rows)
				kstates[i].pos.height = image.rows - kstates[i].pos.y;

			//check that predicted positions are inside the image
			if (kstates[i].pos.x < 0)
				kstates[i].pos.x = 0;
			if (kstates[i].pos.x > image.cols)
				kstates[i].pos.x = image.cols;
			if (kstates[i].pos.y < 0)
				kstates[i].pos.y = 0;
			if (kstates[i].pos.y > image.rows)
				kstates[i].pos.y = image.rows;

			//remove those where the dimensions of are unlikely to be real
			if (kstates[i].pos.width > kstates[i].pos.height*4)
				kstates[i].active = false;

			if (kstates[i].pos.height > kstates[i].pos.width*2)
				kstates[i].active = false;

			kstates[i].lifespan--;//reduce lifespan
			if (kstates[i].lifespan <= 0)
			{
				kstates[i].active = false; //Too old, stop tracking.
			}
		}
	}

	//finally add non matched detections as new
	for (unsigned int i = 0; i < add_as_new_indices.size(); i++)
	{
		if (add_as_new_indices[i])
		{
			initTracking(objects[i], kstates, detections[i], image, colors, _ranges[i]);
		}
	}
	/*
	//check overlapping states and remove them
	float overlap = (OVERLAPPING_PERC/100);
	std::vector<unsigned int> removedIndices;
	for (unsigned int i = 0; i < kstates.size() ; i++)
	{
		for (unsigned int j = kstates.size() - 1; j > 0; j--)
		{
			if (i==j || isInRemoved(removedIndices, i) || isInRemoved(removedIndices, j))
				continue;
			//cout << "i:" << i << " j:" << j << endl;
			cv::Rect intersection = kstates[i].pos & kstates[j].pos;

			if ( ( (intersection.width >= kstates[i].pos.width * overlap) && (intersection.height >= kstates[i].pos.height * overlap) ) ||
				( (intersection.width >= kstates[j].pos.width * overlap) && (intersection.height >= kstates[j].pos.height * overlap) ) )
			{
				//if one state is overlapped by "overlap" % remove it (mark it as unused
				if (kstates[i].real_data && !(kstates[j].real_data))
				{
					kstates[j].active = false;
					removedIndices.push_back(j);
				}
				else if (!(kstates[i].real_data) && (kstates[j].real_data))
				{
					kstates[i].active = false;
					removedIndices.push_back(i);
				}
				else
				{
					kstates[j].active = false;
					removedIndices.push_back(j);
				}
			}
		}
	}*/
	ApplyNonMaximumSuppresion(kstates, OVERLAPPING_PERC);

	removeUnusedObjects(kstates);

	//return to x,y,w,h
	posScaleToBbox(kstates, trackedDetections);

}

void publish_if_possible()
{
	if (track_ready_ && detect_ready_)
	{
		image_objects.publish(kf_objects_msg_);
		track_ready_ = false;
		detect_ready_ = false;
	}
}

void trackAndDrawObjects(cv::Mat& image, int frameNumber, std::vector<cv::LatentSvmDetector::ObjectDetection> detections,
			 std::vector<kstate>& kstates, std::vector<bool>& active,
			 std::vector<cv::Scalar> colors, const sensor_msgs::Image& image_source)
{
	std::vector<kstate> tracked_detections;

	cv::TickMeter tm;
	tm.start();
	//std::cout << endl << "START tracking...";
	doTracking(detections, frameNumber, kstates, active, image, tracked_detections, colors);
	tm.stop();
	//std::cout << "END Tracking time = " << tm.getTimeSec() << " sec" << endl;

	//ROS
	int num = tracked_detections.size();
	std::vector<cv_tracker::image_rect_ranged> rect_ranged_array;
	std::vector<int> real_data(num,0);
	std::vector<int> obj_id(num, 0);
	std::vector<int> lifespan(num, 0);
	//ENDROS

	for (size_t i = 0; i < tracked_detections.size(); i++)
	{
		kstate od = tracked_detections[i];
		cv_tracker::image_rect_ranged rect_ranged_;

		//od.rect contains x,y, width, height
		rectangle(image, od.pos, od.color, 3);
		putText(image, SSTR(od.id), cv::Point(od.pos.x + 4, od.pos.y + 13), cv::FONT_HERSHEY_SIMPLEX, 0.55, od.color, 2);
		//ROS
		obj_id[i] = od.id; // ?
		rect_ranged_.rect.x	= od.pos.x;
		rect_ranged_.rect.y	= od.pos.y;
		rect_ranged_.rect.width	= od.pos.width;
		rect_ranged_.rect.height = od.pos.height;
		rect_ranged_.range	= od.range;
		rect_ranged_.min_height	= od.min_height;
		rect_ranged_.max_height	= od.max_height;

		rect_ranged_array.push_back(rect_ranged_);

		real_data[i] = od.real_data;
		lifespan[i] = od.lifespan;
		//ENDROS
	}
	//more ros
	kf_objects_msg_.type = object_type;
	kf_objects_msg_.total_num = num;
	copy(rect_ranged_array.begin(), rect_ranged_array.end(), back_inserter(kf_objects_msg_.rect_ranged)); // copy vector
	copy(real_data.begin(), real_data.end(), back_inserter(kf_objects_msg_.real_data)); // copy vector
	copy(obj_id.begin(), obj_id.end(), back_inserter(kf_objects_msg_.obj_id)); // copy vector
	copy(lifespan.begin(), lifespan.end(), back_inserter(kf_objects_msg_.lifespan)); // copy vector

//	kf_objects_msg_.header = image_source.header;
	kf_objects_msg_.header = image_objects_header;

	publish_if_possible();

	//cout << "."<< endl;
}

void image_callback(const sensor_msgs::Image& image_source)
{
	if (!_ready)
		return;

	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
	cv::Mat imageTrack = cv_image->image;
	trackAndDrawObjects(imageTrack, _counter, _dpm_detections, _kstates, _active, _colors, image_source);
	//_ready=false;
	//imshow("Tracked", imageTrack);

	_counter++;
}

void detections_callback(cv_tracker::image_obj_ranged image_objects_msg)
{
	if(!detect_ready_)
	{
		unsigned int num = image_objects_msg.obj.size();
		std::vector<cv_tracker::image_rect_ranged> objects = image_objects_msg.obj;
		object_type = image_objects_msg.type;
		image_objects_header = image_objects_msg.header;
		//points are X,Y,W,H and repeat for each instance
		_dpm_detections.clear();
		_ranges.clear();
		_min_heights.clear();
		_max_heights.clear();

		for (unsigned int i=0; i<num;i++)
		{
			cv::Rect tmp;
			tmp.x = objects.at(i).rect.x;
			tmp.y = objects.at(i).rect.y;
			tmp.width = objects.at(i).rect.width;
			tmp.height = objects.at(i).rect.height;
			_dpm_detections.push_back(cv::LatentSvmDetector::ObjectDetection(tmp, 0));
			_ranges.push_back(objects.at(i).range);
			_min_heights.push_back(objects.at(i).min_height);
			_max_heights.push_back(objects.at(i).max_height);
		}
		//_ready = true;
		detect_ready_ = true;
	}
	//cout << "received pos" << endl;

	publish_if_possible();
}

static void kf_config_cb(const runtime_manager::ConfigCarKf::ConstPtr& param)
{
	if (param->initial_lifespan > 0)
		INITIAL_LIFESPAN	= param->initial_lifespan;
	if (param->default_lifespan > 0)
		DEFAULT_LIFESPAN	= param->default_lifespan;
	if(param->noise_covariance > 0)
		NOISE_COV			= param->noise_covariance;
	if(param->measurement_noise_covariance > 0)
		MEAS_NOISE_COV		= param->measurement_noise_covariance;
	if(param->error_estimate_covariance > 0)
		ERROR_ESTIMATE_COV	= param->error_estimate_covariance;
	if(param->percentage_of_overlapping > 0)
		OVERLAPPING_PERC	= param->percentage_of_overlapping;

	ORB_NUM_FEATURES	= 2000;
	ORB_MIN_MATCHES		= 3;
	ORB_KNN_RATIO		= 0.7;

	USE_ORB				= param->use_orb;
}

void init_params()
{
	DEFAULT_LIFESPAN	= 8;
	INITIAL_LIFESPAN	= 4;
	NOISE_COV			= 1;
	MEAS_NOISE_COV		= 25;
	ERROR_ESTIMATE_COV	= 1000000;
	OVERLAPPING_PERC	= 80.0;
	SHOW_PREDICTIONS	= false;

	ORB_NUM_FEATURES	= 2000;
	ORB_MIN_MATCHES		= 3;
	ORB_KNN_RATIO		= 0.7;
	USE_ORB				= false;
}

int kf_main(int argc, char* argv[])
{
	ros::init(argc, argv, "kf");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	image_objects = n.advertise<cv_tracker::image_obj_tracked>("image_obj_tracked", 1);

	cv::generateColors(_colors, 25);

	std::string image_topic;
	std::string obj_topic;
	if (private_nh.getParam("image_node", image_topic))
    	{
        	ROS_INFO("Setting image node to %s", image_topic.c_str());
    	}
	else
	{
		ROS_INFO("No image node received, defaulting to image_raw, you can use _image_node:=YOUR_TOPIC");
		image_topic = "/image_raw";
	}
	if (private_nh.getParam("object_node", image_topic))
    	{
        	ROS_INFO("Setting object node to %s", image_topic.c_str());
    	}
	else
	{
		ROS_INFO("No object node received, defaulting to image_obj_ranged, you can use _object_node:=YOUR_TOPIC");
		obj_topic = "image_obj_ranged";
	}

	init_params();

	ros::Subscriber sub_image = n.subscribe(image_topic, 1, image_callback);
	ros::Subscriber sub_dpm = n.subscribe(obj_topic, 1, detections_callback);


	std::string config_topic("/config");
	config_topic += ros::this_node::getNamespace() + "/kf";
	ros::Subscriber config_subscriber = n.subscribe(config_topic, 1, kf_config_cb);

	//TimeSynchronizer<Image, dpm::ImageObjects> sync(image_sub, pos_sub, 10);

	//sync.registerCallback(boost::bind(&sync_callback, _1, _2));

	ros::spin();
	return 0;
}
