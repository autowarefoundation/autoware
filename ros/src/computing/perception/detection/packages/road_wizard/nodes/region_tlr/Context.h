#ifndef CONTEXT_H
#define CONTEXT_H

#include <opencv2/core/core.hpp>

using namespace cv;

enum LightState { GREEN, YELLOW, RED, UNDEFINED };

class Context {
public:
	Context(){};
	Context(Point aRedCenter, Point aYellowCenter, Point aGreenCenter, int aLampRadius, Point aTopLeft, Point aBotRight);
	Point redCenter;
	Point yellowCenter;
	Point greenCenter;
    Point3d redCenter3d;
    Point3d yellowCenter3d;
    Point3d greenCenter3d;
	int lampRadius;
	Point topLeft;
	Point botRight;
	LightState lightState;
    int signalID;
    int stateJudgeCount;
};

#endif
