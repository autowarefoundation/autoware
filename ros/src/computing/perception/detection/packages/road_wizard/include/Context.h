#ifndef CONTEXT_H
#define CONTEXT_H

#include <vector>
#include <opencv2/core/core.hpp>
#include "road_wizard/Signals.h"

enum LightState { GREEN, YELLOW, RED, UNDEFINED };

class Context {
public:
	Context(){};
	Context(cv::Point aRedCenter, cv::Point aYellowCenter, cv::Point aGreenCenter,
		int aLampRadius, cv::Point aTopLeft, cv::Point aBotRight);
        static void SetContexts(std::vector<Context> &contexts, 
                                const road_wizard::Signals::ConstPtr &extracted_pos,
                                const int frame_row,
                                const int frame_colmuns);

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

 private:
        static bool CompareContext(const Context left, const Context right);
};

#endif
