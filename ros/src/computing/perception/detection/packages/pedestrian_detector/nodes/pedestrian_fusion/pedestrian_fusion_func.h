#ifndef INCLUDED_MFunctions_
#define INCLUDED_MFunctions_

#ifndef _DEBUG
#define _DEBUG 0
#endif

//openCV library
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
//Debug mode
#ifndef ROS
#ifdef _DEBUG
#pragma comment(lib,"cv200d.lib")
#pragma comment(lib,"cxcore200d.lib")
#pragma comment(lib,"cvaux200d.lib")
#pragma comment(lib,"highgui200d.lib")
#else
//Release mode
#pragma comment(lib,"cv200.lib")
#pragma comment(lib,"cxcore200.lib")
#pragma comment(lib,"cvaux200.lib")
#pragma comment(lib,"highgui200.lib")
#endif
#endif
//C++ library
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <boost/array.hpp>
//ORIGINAL header files
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
//ROS
#include "dpm/ImageObjects.h"
#include "scan_to_image/ScanImage.h"
#include "points_to_image/PointsImage.h"
#include "car_detector/FusedObjects.h"

#define IMAGE_WIDTH 800
#define IMAGE_HEIGHT 600
#define NO_DATA 0
#if _DEBUG
#define IMAGE_TOPIC "/image_raw"
#define IMAGE_CALLBACK imageCallback
#endif

struct Scan_image{
    float distance[IMAGE_WIDTH][IMAGE_HEIGHT];
    float intensity[IMAGE_WIDTH][IMAGE_HEIGHT];
    int max_y;
    int min_y;
};

extern void calcDistance();
extern void setImageObjects(const dpm::ImageObjects& image_objects);
extern void setScanImage(const scan_to_image::ScanImage& scan_image);
extern void setPointsImage(const points_to_image::PointsImage& points_image);
extern int getObjectNum();
extern std::vector<int> getCornerPoint();
extern std::vector<float> getDistance();
extern void init();
extern void destroy();
#if _DEBUG
extern void imageCallback(const sensor_msgs::Image& image_source);
#endif

#endif
