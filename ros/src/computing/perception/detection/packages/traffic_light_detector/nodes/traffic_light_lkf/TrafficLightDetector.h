#ifndef TRAFFIC_LIGHT_DETECTOR_H
#define TRAFFIC_LIGHT_DETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "Context.h"

using namespace cv;

#define MY_COLOR_PURPLE Scalar(255,0,255)
#define MY_COLOR_RED Scalar(0,0,255)
#define MY_COLOR_GREEN Scalar(0,255,0)
#define MY_COLOR_YELLOW Scalar(60,255,255)
#define MY_COLOR_BLUE Scalar(255,0,0)
#define MY_COLOR_WHITE Scalar(255,255,255)

#if 0
/* const LightState STATE_TRANSITION_MATRIX[5][8] = {  */
/* 	{ GREEN, RED, YELLOW, REDYELLOW, GREEN, GREEN, GREEN, UNDEFINED }, */
/* 	{ YELLOW, RED, YELLOW, REDYELLOW, GREEN, GREEN, GREEN, UNDEFINED }, */
/* 	{ RED, RED, REDYELLOW, REDYELLOW, GREEN, GREEN, GREEN, UNDEFINED }, */
/* 	{ REDYELLOW, REDYELLOW, REDYELLOW, REDYELLOW, GREEN, GREEN, GREEN, UNDEFINED }, */
/* 	{ UNDEFINED, RED, YELLOW, REDYELLOW, GREEN, GREEN, GREEN, UNDEFINED } */
/* }; */
#endif
const LightState STATE_TRANSITION_MATRIX[4][8] = {
	{ GREEN,     RED, YELLOW, YELLOW, GREEN, GREEN, YELLOW, UNDEFINED },
	{ YELLOW,    RED, YELLOW, RED,    GREEN, RED,   YELLOW, UNDEFINED },
	{ RED,       RED, YELLOW, RED,    GREEN, RED,   GREEN,  UNDEFINED },
	{ UNDEFINED, RED, YELLOW, RED,    GREEN, RED,   YELLOW, UNDEFINED }
};

double getBrightnessRatioInCircle(const Mat &input, const Point center, const int radius);
int getCurrentLightsCode(bool display_red, bool display_yellow, bool display_green);
LightState determineState(LightState previousState, int currentLightsCode);

class TrafficLightDetector {
public:
	TrafficLightDetector();
	LightState brightnessDetect(const Mat &input);
	void colorDetect(const Mat &input, Mat &output, const Rect coords, int Hmin, int Hmax);
	vector<Context> contexts;
};

#endif
