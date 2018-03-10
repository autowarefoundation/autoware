#ifndef __BGAPI_INIT_H__
#define __BGAPI_INIT_H__

#include <vector>
#include <iostream>
#include <sstream>

#include <stdlib.h>
#include <stdio.h>

// OpenCV includes
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int init_systems(int * system_count, vector<BGAPI::System*> * externppSystem);
int init_cameras( int system_count, vector<BGAPI::System*> * externppSystem, int * pCurrSystem, int& numCameras, std::vector<BGAPI::Camera*>& cameraObjects);
int release_systems( std::vector<BGAPI::System*> * ppSystem );
int release_images( std::vector<BGAPI::Image*> * ppImage );
bool setup_cameras(std::vector<BGAPI::Camera*> & cameraObjects, std::string camera_pixel_format_str);
bool start_cameras(std::vector<BGAPI::Camera*> & cameraObjects);
void stop_cameras(std::vector<BGAPI::Camera*> & cameraObjects);
#endif // __BGAPI_INIT_H__
