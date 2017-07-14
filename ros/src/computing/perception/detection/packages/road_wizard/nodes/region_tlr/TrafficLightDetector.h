#ifndef TRAFFIC_LIGHT_DETECTOR_H
#define TRAFFIC_LIGHT_DETECTOR_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "Context.h"

#define MY_COLOR_PURPLE	cv::Scalar(255,0,255)
#define MY_COLOR_RED	cv::Scalar(0,0,255)
#define MY_COLOR_GREEN	cv::Scalar(0,255,0)
#define MY_COLOR_YELLOW	cv::Scalar(60,255,255)
#define MY_COLOR_BLUE	cv::Scalar(255,0,0)
#define MY_COLOR_WHITE	cv::Scalar(255,255,255)

const LightState STATE_TRANSITION_MATRIX[4][8] = {
	/* current GYR: 000, 001, 010, 011, 100, 101, 110, 111 */
	{ GREEN,     UNDEFINED, YELLOW,    YELLOW, GREEN,     GREEN,     YELLOW, UNDEFINED }, /* pre = GREEN  */
	{ YELLOW,    RED,       YELLOW,    RED,    UNDEFINED, UNDEFINED, YELLOW, UNDEFINED }, /* pre = YELLOW */
	{ RED,       RED,       UNDEFINED, RED,    GREEN,     RED,       GREEN,  UNDEFINED }, /* pre = RED */
	{ UNDEFINED, RED,       YELLOW,    RED,    GREEN,     RED,       YELLOW, UNDEFINED }  /* pre = UNDEFINED */
};

double getBrightnessRatioInCircle(const cv::Mat &input, const cv::Point center, const int radius);
int getCurrentLightsCode(bool display_red, bool display_yellow, bool display_green);
LightState determineState(LightState previousState, int currentLightsCode, int* stateJudgeCount);

class TrafficLightDetector {
public:
	TrafficLightDetector();
	void brightnessDetect(const cv::Mat &input);
	void colorDetect(const cv::Mat &input, cv::Mat &output, const cv::Rect coords, int Hmin, int Hmax);
	std::vector<Context> contexts;
};

enum daytime_Hue_threshold {
    DAYTIME_RED_LOWER    = 340,
    DAYTIME_RED_UPPER    = 50,
    DAYTIME_YELLOW_LOWER = 50,
    DAYTIME_YELLOW_UPPER = 70,
    DAYTIME_GREEN_LOWER  = 80,//120,//140,
    DAYTIME_GREEN_UPPER  = 190,//180,
};

#define DAYTIME_S_SIGNAL_THRESHOLD ((double)0.37)//((double)0.27)
#define DAYTIME_V_SIGNAL_THRESHOLD ((double)140/255) //((double)90/255) //((double)110/255)

#define NOISE_REDUCTION_TIME 1

#define CIRCLE_LEVEL_THRESHOLD 0.75 //0.65
#define CIRCLE_AREA_THRESHOLD 0.5 //1 //5

#define CHANGE_STATE_THRESHOLD 10

/* utility functions to convert HSV value range from OpenCV to definition */
static inline double Actual_Hue(uchar hue_opencv)
{
    return ((double)2 * hue_opencv);
} /* static inline double Actual_Hue() */

static inline double Actual_Sat(uchar sat_opencv)
{
    return ((double)sat_opencv / 255);
} /* static inline double Actual_Sat() */


static inline double Actual_Val(uchar val_opencv)
{
    return ((double)val_opencv / 255);
} /* static inline double Actual_Val() */

#endif
