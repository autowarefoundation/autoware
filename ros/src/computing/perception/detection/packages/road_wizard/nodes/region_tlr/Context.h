#ifndef CONTEXT_H
#define CONTEXT_H

#include <opencv2/core/core.hpp>

enum LightState { GREEN, YELLOW, RED, UNDEFINED };

class Context {
public:
	Context(){};
	Context(cv::Point aRedCenter, cv::Point aYellowCenter, cv::Point aGreenCenter,
		int aLampRadius, cv::Point aTopLeft, cv::Point aBotRight);
	cv::Point redCenter;
	cv::Point yellowCenter;
	cv::Point greenCenter;
	cv::Point3d redCenter3d;
	cv::Point3d yellowCenter3d;
	cv::Point3d greenCenter3d;
	int lampRadius;
	cv::Point topLeft;
	cv::Point botRight;
	LightState lightState;
	int signalID;
	int stateJudgeCount;
};

#endif
