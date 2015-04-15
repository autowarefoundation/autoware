#ifndef CONTEXT_H
#define CONTEXT_H

#include <opencv2/core/core.hpp>

using namespace cv;

#if 0
//enum LightState { GREEN, YELLOW, RED, REDYELLOW, UNDEFINED };
#endif
enum LightState { GREEN, YELLOW, RED, UNDEFINED };

class Context {
public:
	Context(Point aRedCenter, Point aYellowCenter, Point aGreenCenter, int aLampRadius, Point aTopLeft, Point aBotRight);
	Point redCenter;
	Point yellowCenter;
	Point greenCenter;
	int lampRadius;
	Point topLeft;
	Point botRight;
	LightState lightState;
};

#endif
