#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

/* External includes */
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

/* Internal includes */
#include "TrafficLightDetector.h"

/* Extra includes */
#include "traffic_light_detector/Signals.h"

#define MAIN_WINDOW_NAME "Main"
#define SETTINGS_WINDOW_NAME "Settings"

/* Functions declarations */
void setContexts(TrafficLightDetector &detector, const traffic_light_detector::Signals::ConstPtr& extractedPos);
void initMasks(char *pathToShowMask);
void drawTrafficLights(Mat &targetImg, LightState lightState);
void drawEnforcement(Mat &targetImg, bool isEnforced, LightState lightState);
void drawBoundedRects(Mat &targetImg, vector<Rect> boundedRects);

#define RED_DRAW_CENTER Point(465,465)
#define YELLOW_DRAW_CENTER Point(465,500)
#define GREEN_DRAW_CENTER Point(465,535)
#define LIGHT_DRAW_RADIUS 15

#define MINIMAM_RADIUS 3
#define ROI_MARGINE 5

#endif
