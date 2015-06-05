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
	void brightnessDetect(const Mat &input);
	void colorDetect(const Mat &input, Mat &output, const Rect coords, int Hmin, int Hmax);
	vector<Context> contexts;
};

/* /\* */
/*   think in OpenCV value range */
/*   H : 0 <= H <= 180 (H_opencv = H_def / 2) */
/*   S : 0 <= S <= 255 (S_opencv = S_def * 255) */
/*   V : 0 <= V <= 255 (V_opencv = V_dev * 255) */
/* *\/ */
/* enum daytime_Hue_threshold { */
/*   DAYTIME_RED_LOWER    = 340/2, */
/*   //DAYTIME_RED_UPPER    = 360/2, */
/*   DAYTIME_RED_UPPER    = 20/2, */
/*   //DAYTIME_YELLOW_LOWER = 0/2, */
/*   DAYTIME_YELLOW_LOWER = 40/2, */
/*   DAYTIME_YELLOW_UPPER = 60/2, */
/*   DAYTIME_BLUE_LOWER   = 140/2, */
/*   DAYTIME_BLUE_UPPER   = 180/2, */
/* }; */

/* #define DAYTIME_S_SIGNAL_THRESHOLD (0.27*255) */
/* //#define DAYTIME_V_SIGNAL_THRESHOLD 110 */
/* #define DAYTIME_V_SIGNAL_THRESHOLD 80 */
enum daytime_Hue_threshold {
    DAYTIME_RED_LOWER    = 340,
    DAYTIME_RED_UPPER    = 40,
    DAYTIME_YELLOW_LOWER = 40,
    DAYTIME_YELLOW_UPPER = 60,
    DAYTIME_GREEN_LOWER  = 70,//120,//140,
    DAYTIME_GREEN_UPPER  = 180,
};

#define DAYTIME_S_SIGNAL_THRESHOLD ((double)0.1)//((double)0.2) //0.27
#define DAYTIME_V_SIGNAL_THRESHOLD ((double)100/255) //((double)110/255)

#define NOISE_REDUCTION_TIME 1

#define CIRCLE_LEVEL_THRESHOLD 0.65
#define CIRCLE_AREA_THRESHOLD 10

/* utility functions to convert HSV value range from OpenCV to definition */
inline double Actual_Hue(uchar hue_opencv)
{
    return ((double)2 * hue_opencv);
} /* static inline double Actual_Hue() */


inline double Actual_Sat(uchar sat_opencv)
{
    return ((double)sat_opencv / 255);
} /* static inline double Actual_Sat() */


inline double Actual_Val(uchar val_opencv)
{
    return ((double)val_opencv / 255);
} /* static inline double Actual_Val() */




#endif
