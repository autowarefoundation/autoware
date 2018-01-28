#ifndef CONTEXT_H
#define CONTEXT_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <autoware_msgs/Signals.h>

enum LightState
{
	GREEN, YELLOW, RED, UNDEFINED
};


class Context
{
public:
	Context()
	{
	};

	Context(cv::Point aRedCenter, cv::Point aYellowCenter, cv::Point aGreenCenter,
	        int aLampRadius, cv::Point aTopLeft, cv::Point aBotRight);

	static void SetContexts(std::vector<Context> &out_contexts,
	                        const autoware_msgs::Signals::ConstPtr &in_extracted_pos,
	                        const int in_image_height,
	                        const int in_image_width);

	cv::Point   redCenter;
	cv::Point   yellowCenter;
	cv::Point   greenCenter;
	cv::Point3d redCenter3d;
	cv::Point3d yellowCenter3d;
	cv::Point3d greenCenter3d;
	int         lampRadius;
	cv::Point   topLeft;
	cv::Point   botRight;
	LightState  lightState;
	LightState  newCandidateLightState;
	int         signalID;
	int         stateJudgeCount;
	bool        leftTurnSignal;
	bool        rightTurnSignal;
	int         closestLaneId;
private:
	static bool CompareContext(const Context in_context_a, const Context in_context_b);
};

#endif
