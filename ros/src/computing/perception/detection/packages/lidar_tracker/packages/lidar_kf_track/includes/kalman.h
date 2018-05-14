#pragma once
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

// http://www.morethantechnical.com/2011/06/17/simple-kalman-filter-for-tracking-using-opencv-2-2-w-code/
class TKalmanFilter
{
public:
	cv::KalmanFilter kalman;
	float deltatime; //приращение времени
	cv::Point2f LastResult;
	TKalmanFilter(cv::Point2f in_point, float in_time_delta = 0.2, float in_acceleration_noise_magnitude = 0.5);
	~TKalmanFilter();
	cv::Point2f GetPrediction();
	cv::Point2f Update(cv::Point2f in_point, bool in_data_correct);
};
