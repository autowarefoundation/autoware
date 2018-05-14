#include "kalman.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
TKalmanFilter::TKalmanFilter(cv::Point2f pt, float dt, float Accel_noise_mag)
{
	//time increment (lower values makes target more "massive")
	deltatime = dt; //0.2

	// We don't know acceleration, so, assume it to process noise.
	// But we can guess, the range of acceleration values thich can be achieved by tracked object.
    // Process noise. (standard deviation of acceleration: m/s^2)
	// shows, woh much target can accelerate.
	//float Accel_noise_mag = 0.5;

	//4 state variables, 2 measurements
	kalman.init( 4, 2, 0 );
	// Transition cv::Matrix
	kalman.transitionMatrix = (cv::Mat_<float>(4, 4) << 	1, 0, deltatime, 0,
															0, 1, 0, deltatime,
															0, 0, 1, 0,
															0, 0, 0, 1);

	// init...
	LastResult = pt;
	kalman.statePre.at<float>(0) = pt.x; // x
	kalman.statePre.at<float>(1) = pt.y; // y

	kalman.statePre.at<float>(2) = 0;
	kalman.statePre.at<float>(3) = 0;

	kalman.statePost.at<float>(0) = pt.x;
	kalman.statePost.at<float>(1) = pt.y;

	cv::setIdentity(kalman.measurementMatrix);

	kalman.processNoiseCov = (cv::Mat_<float>(4, 4) <<
		pow(deltatime,4.0)/4.0	, 0						, pow(deltatime,3.0)/2.0	, 0,
		0						, pow(deltatime,4.0)/4.0,0							, pow(deltatime,3.0)/2.0,
		pow(deltatime,3.0)/2.0	, 0						, pow(deltatime,2.0)		, 0,
		0						, pow(deltatime,3.0)/2.0,0							, pow(deltatime,2.0));


	kalman.processNoiseCov*=Accel_noise_mag;

	setIdentity(kalman.measurementNoiseCov, cv::Scalar::all(0.1));

	setIdentity(kalman.errorCovPost, cv::Scalar::all(.1));

}
//---------------------------------------------------------------------------
TKalmanFilter::~TKalmanFilter()
{
	//delete kalman;
}

//---------------------------------------------------------------------------
cv::Point2f TKalmanFilter::GetPrediction()
{
	cv::Mat_<float> prediction = kalman.predict();
	LastResult = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
	return LastResult;
}
//---------------------------------------------------------------------------
cv::Point2f TKalmanFilter::Update(cv::Point2f p, bool DataCorrect)
{
	cv::Mat_<float> measurement(2, 1, CV_32FC(1));
	if(!DataCorrect)
	{
		measurement.at<float>(0) = LastResult.x;  //update using prediction
		measurement.at<float>(1) = LastResult.y;
	}
	else
	{
		measurement.at<float>(0) = p.x;  //update using measurements
		measurement.at<float>(1) = p.y;
	}
	// Correction
	cv::Mat_<float> estiMated = kalman.correct(measurement);
	LastResult.x = estiMated.at<float>(0);   //update using measurements
	LastResult.y = estiMated.at<float>(1);
	return LastResult;
}
//---------------------------------------------------------------------------
