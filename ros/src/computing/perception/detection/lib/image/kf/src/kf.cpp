//ROS STUFF
#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "dpm/ImageObjects.h"

//TRACKING STUFF
#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/video/tracking.hpp"

#include <iostream>
#include <stdio.h>

#include <sstream>

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

#define DEFAULT_LIFESPAN 4 //LIFESPAN of objects before stop being tracked, in frames
#define INITIAL_LIFESPAN 3 //LIFESPAN of objects before stop being tracked, in frames

using namespace std;
using namespace cv;

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher image_objects;//ROS

struct kstate
{
	KalmanFilter		KF;//KalmanFilter for this object
	Rect			pos;//position of the object centerx, centery, width, height
	float			score;//DPM score
	bool			active;//if too old (lifespan) don't use
	int			id;//id of this tracked object
	Mat			image;//image containing the detected and tracked object
	int			lifespan;//remaining lifespan before deprecate
	LatentSvmDetector::ObjectDetection obj;//currently not used
	Scalar			color;
};

vector<kstate> _kstates;
vector<bool> _active;
vector<LatentSvmDetector::ObjectDetection> _detections;



///Returns true if an im1 is contained in im2 or viceversa
bool crossCorr(Mat im1, Mat im2)
{
	//im1 roi from the previous frame
	//im2 roi fromcurrent frame
	if (im1.rows <= 0 || im1.cols <= 0 || im2.rows <= 0 || im2.cols <= 0)
		return false;

	Mat result, larger_im, smaller_im;

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
		Mat rows = Mat::ones(smaller_im.rows - larger_im.rows, larger_im.cols, larger_im.type());
		larger_im.push_back(rows);
	}

	result_cols = larger_im.cols - smaller_im.cols + 1;
	result_rows = larger_im.rows - smaller_im.rows + 1;
	result.create(result_cols, result_rows, CV_32FC1);

	/// Do the Matching and Normalize
	matchTemplate(larger_im, smaller_im, result, CV_TM_CCORR_NORMED);
	//normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

	/// Localizing the best match with minMaxLoc
	double minVal; double maxVal; Point minLoc; Point maxLoc;
	Point matchLoc;

	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

	matchLoc = maxLoc;

	/// Show me what you got
	Mat scene = larger_im.clone();
	rectangle(scene, matchLoc, Point(matchLoc.x + smaller_im.cols, matchLoc.y + smaller_im.rows), Scalar(0, 0, 255), 2, 8, 0);
	//imshow(image_window, scene);
	//imshow(result_window, result);

	//if (maxVal>0.89 && minVal <0.3)
	bool ret;
	int thresWidth = (larger_im.cols)*.7;
	if ( (maxVal > 0.5) && (smaller_im.cols > thresWidth) )//good threshold and consistent size
	{

		std::cout << "matched" << endl;
		ret = true;
	}
	else
	{
		std::cout << "non matched" << endl;
		ret = false;
	}
	//cv::imshow("match1", scene);
	//cv::imshow("match2", smaller_im);

	return ret;
}

void posScaleToBbox(vector<kstate> kstates, vector<kstate>& trackedDetections)
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


void initTracking(LatentSvmDetector::ObjectDetection object, vector<kstate>& kstates, LatentSvmDetector::ObjectDetection detection, Mat& image, vector<Scalar> colors)
{
	kstate new_state;
	//KalmanFilter KF(4, 2, 0);//XY Only
	KalmanFilter KF(8, 4, 0);

	/*Mat_<float> measurement = (Mat_<float>(2, 1) << object.rect.x,//XY Only
		object.rect.y);*/
	Mat_<float> measurement = (Mat_<float>(4, 1) << object.rect.x,
		object.rect.y, object.rect.width, object.rect.height);

	/*KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0,//XY Only
												0, 1, 0, 1,
												0, 0, 1, 0,
												0, 0, 0, 1);*/
	KF.transitionMatrix = (Mat_<float>(8, 8)
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
	//KF.measurementMatrix = measurement;
	cv::setIdentity(KF.measurementMatrix);
	cv::setIdentity(KF.processNoiseCov, Scalar::all(1));//1e-4
	cv::setIdentity(KF.measurementNoiseCov, Scalar::all(25));//1e-3
	cv::setIdentity(KF.errorCovPost, Scalar::all(1000000));//100
	//setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	//setIdentity(KF.measurementNoiseCov, Scalar::all(1e-3));
	//setIdentity(KF.errorCovPost, Scalar::all(100));

	//KF.correct(measurement);
	//KF.correct(measurement);

	//save data to kstate
	new_state.active = true;
	new_state.image = image(Rect(detection.rect.x,
		detection.rect.y,
		detection.rect.width,
		detection.rect.height)).clone();//Crop image and obtain only object (ROI)
	new_state.KF = KF;
	new_state.lifespan = INITIAL_LIFESPAN;//start only with 1
	new_state.pos = object.rect;
	new_state.score = object.score;
	new_state.id = (kstates.size());
	new_state.color = colors[new_state.id];

	kstates.push_back(new_state);

}


void doTracking(vector<LatentSvmDetector::ObjectDetection>& detections, int frameNumber,
	vector<kstate>& kstates, vector<bool>& active, Mat& image, vector<kstate>& trackedDetections, vector<Scalar> & colors)
{
	vector<LatentSvmDetector::ObjectDetection> objects;
	//vector<LatentSvmDetector::ObjectDetection> tracked_objects;
	vector<bool> predict_indices;//this will correspond to kstates i
	vector<bool> correct_indices;//this will correspond to kstates i
	vector<int> correct_detection_indices;//this will correspond to kstates i, used to store the index of the corresponding object
	vector<bool> add_as_new_indices;//this will correspond to detections j

	//predict_indices.assign(kstates.size(), true);//always predict
	correct_indices.assign(kstates.size(), false);//correct only those matched
	correct_detection_indices.assign(kstates.size(), false);//correct only those matched
	add_as_new_indices.assign(detections.size(), true);//if the detection was not found add as new

	//Convert Bounding box coordinates from (x1,y1,w,h) to (BoxCenterX, BoxCenterY, width, height)
	objects = detections;//bboxToPosScale(detections);
	//compare detections from this frame with tracked objects
	for (unsigned int j = 0; j < detections.size(); j++)
	{
		for (unsigned int i = 0; i < kstates.size(); i++)
		{
			//compare only to active tracked objects(not too old)
			if (kstates[i].active)
			{
				//try to match with previous frame
				Rect roi_20; //extend the roi 20%
				//roi_20.x = (detections[j].rect.x - detections[j].rect.width*.1)<0 ? 0 : detections[j].rect.x - detections[j].rect.width*.1;
				//roi_20.y = (detections[j].rect.y - detections[j].rect.height*.1)<0 ? 0 : detections[j].rect.y - detections[j].rect.height*.1;
				//roi_20.width = (roi_20.x + detections[j].rect.width*1.2 > image.cols) ? image.cols - roi_20.x : detections[j].rect.width*1.2;
				//roi_20.height = (roi_20.y + detections[j].rect.height*1.2 > image.rows) ? image.rows - roi_20.y : detections[j].rect.height*1.2;

				Rect roi(detections[j].rect);

				Mat currentObjectROI = image(roi).clone();//Crop image and obtain only object (ROI)

				Rect intersect = detections[j].rect & kstates[i].pos;//check overlapping

				if (intersect.width > 0
					&& crossCorr(kstates[i].image, currentObjectROI)
					)
				{
					correct_indices[i] = true;//if ROI on this frame is matched to a previous object, correct
					correct_detection_indices[i] = j;//store the index of the detection corresponding to matched kstate
					add_as_new_indices[j] = false;//if matched do not add as new
					kstates[i].image = currentObjectROI;//update image with current frame data
					kstates[i].score = detections[j].score;
				}//crossCorr

			}//kstates[i].active
		}//for (int i = 0; i < kstates.size(); i++)
	}//for (int j = 0; j < detections.size(); j++)

	//do prediction and correction for the marked states
	for (unsigned int i = 0; i < kstates.size(); i++)
	{
		if (kstates[i].active)//predict and correct only active states
		{
			Mat prediction = kstates[i].KF.predict();
			kstates[i].pos.x = prediction.at<float>(0);
			kstates[i].pos.y = prediction.at<float>(1);
			kstates[i].pos.width = prediction.at<float>(2);
			kstates[i].pos.height = prediction.at<float>(3);

			//check that predicted positions are inside the image
			if (kstates[i].pos.x < 0)
				kstates[i].pos.x = 0;
			if (kstates[i].pos.x > image.cols)
				kstates[i].pos.x = image.cols;
			if (kstates[i].pos.y < 0)
				kstates[i].pos.y = 0;
			if (kstates[i].pos.y > image.rows)
				kstates[i].pos.y = image.rows;

			kstates[i].lifespan--;//reduce lifespan
			if (kstates[i].lifespan <= 0)
			{
				kstates[i].active = false; //Too old, stop tracking.
			}

			//now do respective corrections on KFs (updates)
			if (correct_indices[i])
			{
				//a match was found hence update KF measurement
				int j = correct_detection_indices[i];//obtain the index of the detection

				/*Mat_<float> measurement = (Mat_<float>(2, 1) << objects[j].rect.x, //XY ONLY
																objects[j].rect.y);*/
				Mat_<float> measurement = (Mat_<float>(4, 1) << objects[j].rect.x,
					objects[j].rect.y,
					objects[j].rect.width,
					objects[j].rect.height);

				//use real data instead of predicted data when data is available
				kstates[i].pos.x = objects[j].rect.x;
				kstates[i].pos.y = objects[j].rect.y;
				kstates[i].pos.width = objects[j].rect.width;
				kstates[i].pos.height = objects[j].rect.height;

				kstates[i].KF.correct(measurement);//UPDATE KF with new info
				kstates[i].lifespan = DEFAULT_LIFESPAN; //RESET Lifespan of object

				//kstates[i].pos.width = objects[j].rect.width;//XY ONLY
				//kstates[i].pos.height = objects[j].rect.height;//XY ONLY

				//check that new widths and heights don't go beyond the image size
				if (kstates[i].pos.width + kstates[i].pos.x > image.cols)
					kstates[i].pos.width = image.cols - kstates[i].pos.x;
				if (kstates[i].pos.height + kstates[i].pos.y > image.rows)
					kstates[i].pos.height = image.rows - kstates[i].pos.y;

				Mat im1 = image(kstates[i].pos);
				Mat im2 = image(objects[j].rect);

			}
		}
	}

	//finally add non matched detections as new
	for (unsigned int i = 0; i < add_as_new_indices.size(); i++)
	{
		if (add_as_new_indices[i])
		{
			initTracking(objects[i], kstates, detections[i], image, colors);
		}
	}

	//return to x,y,w,h
	posScaleToBbox(kstates, trackedDetections);

}

void trackAndDrawObjects(Mat& image, int frameNumber, vector<LatentSvmDetector::ObjectDetection> detections,
	vector<kstate>& kstates, vector<bool>& active, vector<Scalar> colors)
{
	vector<kstate> tracked_detections;


	TickMeter tm;
	tm.start();
	std::cout << endl << "START tracking" << endl;
	doTracking(detections, frameNumber, kstates, active, image, tracked_detections, colors);
	tm.stop();
	std::cout << "END Tracking time = " << tm.getTimeSec() << " sec" << endl;

	//ROS
	int num = tracked_detections.size();
    	std::vector<int> corner_point_array(num * 4,0);
    	std::vector<int> car_type_array(num, 0);
	//ENDROS

	for (size_t i = 0; i < tracked_detections.size(); i++)
	{
		kstate od = tracked_detections[i];
		//od.rect contains x,y, width, height
		rectangle(image, od.pos, od.color, 3);
		putText(image, SSTR(od.id), Point(od.pos.x + 4, od.pos.y + 13), FONT_HERSHEY_SIMPLEX, 0.55, od.color, 2);
		//ROS		
		car_type_array[i] = od.id; // ?
		corner_point_array[0+i*4] = od.pos.x;
		corner_point_array[1+i*4] = od.pos.y;
		corner_point_array[2+i*4] = od.pos.width;//updated to show width instead of 2nd point
		corner_point_array[3+i*4] = od.pos.height;//updated to show height instead of 2nd point
		//ENDROS
	}
	dpm::ImageObjects image_objects_msg;
	image_objects_msg.car_num = num;
	image_objects_msg.corner_point = corner_point_array;
	image_objects_msg.car_type = car_type_array;

	image_objects.publish(image_objects_msg);
}
void sync_callback(const sensor_msgs::ImageConstPtr& image_source,
			const dpm::ImageObjectsConstPtr &image_objects_msg_ptr)
{
	dpm::ImageObjects image_objects_msg = *image_objects_msg_ptr;

	const auto& encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source,
							     encoding);
	IplImage frame = cv_image->image;
	
	Mat image(&frame, true);

	int num = image_objects_msg.car_num;
	vector<int> points = image_objects_msg.corner_point; 

		
	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kf");
	ros::NodeHandle nh;

	//std::string published_node;//future_use

	image_objects = nh.advertise<dpm::ImageObjects>("car_pos_xy_tracked", 1);

	message_filters::Subscriber<Image> image_sub(nh, "/image_raw", 1);
	message_filters::Subscriber<dpm::ImageObjects> pos_sub(nh, "/car_pos_xy", 1);
	TimeSynchronizer<Image, dpm::ImageObjects> sync(image_sub, pos_sub, 10);
	
	sync.registerCallback(boost::bind(&sync_callback, _1, _2));
	

	ros::spin();
	return 0;
}

