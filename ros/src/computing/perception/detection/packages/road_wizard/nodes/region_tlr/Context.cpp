#include "Context.h"

Context::Context(cv::Point aRedCenter, cv::Point aYellowCenter, cv::Point aGreenCenter,
		 int aLampRadius, cv::Point aTopLeft, cv::Point aBotRight)
{
	redCenter = aRedCenter;
	yellowCenter = aYellowCenter;
	greenCenter = aGreenCenter;
	lampRadius = aLampRadius;
	topLeft = aTopLeft;
	botRight = aBotRight;
}
