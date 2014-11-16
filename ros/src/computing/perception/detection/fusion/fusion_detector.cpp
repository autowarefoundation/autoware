/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"cv200d.lib")
#pragma comment(lib,"cxcore200d.lib")
#pragma comment(lib,"cvaux200d.lib")
#pragma comment(lib,"highgui200d.lib")
#else
//Releaseモードの場合
#pragma comment(lib,"cv200.lib")
#pragma comment(lib,"cxcore200.lib")
#pragma comment(lib,"cvaux200.lib")
#pragma comment(lib,"highgui200.lib")
#endif
//C++ library
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <math.h>

//ORIGINAL header files
#if 1 // AXE
#include "src/car_det_func.h"
#else
#include "Laser_func.h"
#include "car_det_func.h"
#include "Common.h"
#include "Depth_points_func.h"
#endif

#include "std_msgs/String.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>//C++ library
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#if 1 // AXE
#include "dpm/ImageObjects.h"
#include "scan_to_image/ScanImage.h"
#include "fusion/FusedObjects.h"
#else
#include "sensors_fusion/ObstaclePosition.h"
#include "sensors_fusion/TransformedPointData.h"
#endif
#include <boost/array.hpp>
//end yukky there are not hedarfile

char WINDOW_NAME[] = "CAR_TRACK";
double ratio = 1;	//resize ratio
IplImage *IM_D;
IplImage *IM_D_clone;
int corner_point[128]; //max corner point
int car_type[64];
int car_num;

#if 1 // AXE
ros::Publisher fused_objects;
#endif

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
#if 1 // AXE
void obstacle_detectionCallback(const dpm::ImageObjects& obstacle_position)
#else
void obstacle_detectionCallback(const sensors_fusion::ObstaclePosition& obstacle_position)
#endif
{
    int i;

    car_num = obstacle_position.car_num;
    for (i = 0 ;i < obstacle_position.car_num; i++) {
        car_type[i] = obstacle_position.car_type[i];
        corner_point[0+i*4] = obstacle_position.corner_point[0+i*4];
        corner_point[1+i*4] = obstacle_position.corner_point[1+i*4];
        corner_point[2+i*4] = obstacle_position.corner_point[2+i*4];
        corner_point[3+i*4] = obstacle_position.corner_point[3+i*4];
    }
}
#if 1 // AXE
void distance_measurementCallback(const scan_to_image::ScanImage& transformed_point_data)
#else
void distance_measurementCallback(const sensors_fusion::TransformedPointData& transformed_point_data)
#endif
{
    CvSeq *points;
    CvPoint pt;
    CvMemStorage *storage = cvCreateMemStorage (0);
    int i, j;
#if 1 // AXE
    if(IM_D_clone == NULL){
      return;
    }
#endif
    IplImage *IM_D_clone_plot;

    IM_D_clone_plot = cvCloneImage(IM_D_clone);

    //画像に点群データをプロット
    points = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), storage);

    for (i = 0; i < transformed_point_data.scan.ranges.size(); i++) {
        if(0 > transformed_point_data.image_point_x[i] || transformed_point_data.image_point_x[i] > 639) {
            continue;
        }
        if(0 > transformed_point_data.image_point_y[i] || transformed_point_data.image_point_y[i] > 479) {
            continue;
        }
        pt.x = transformed_point_data.image_point_x[i];
        pt.y = transformed_point_data.image_point_y[i];

        cvSeqPush (points, &pt);
        cvCircle(IM_D_clone_plot, pt, 2, CV_RGB (0, 255, 0), CV_FILLED, 8, 0);
    }

#if 1 // AXE
    std::vector<int> corner_point_array;
    std::vector<int> car_type_array;
#endif
	for(i = 0; i < car_num; i++)
	{
        double obstacle_distance = -1;
        char distance_string[32];
        CvScalar col = cvScalar(0.0, 0.0, 255.0);

        printf("%d, %d, %d, %d\n", corner_point[0+i*4], corner_point[1+i*4], corner_point[2+i*4], corner_point[3+i*4]);
        for(j = 0; j < transformed_point_data.scan.ranges.size(); j++) {
            if(transformed_point_data.image_point_x[j] > corner_point[0+i*4] && transformed_point_data.image_point_x[j] < corner_point[2+i*4]) {
                if(transformed_point_data.image_point_y[j] > corner_point[1+i*4] && transformed_point_data.image_point_y[j] < corner_point[3+i*4]) {
                    if(obstacle_distance > transformed_point_data.distance[j] || obstacle_distance == -1) {
                        obstacle_distance = transformed_point_data.distance[j];
                    }
                }
            }
        }
        CvFont dfont;
        float hscale      = 1.0f;
        float vscale      = 1.0f;
        float italicscale = 0.0f;
        int  thickness    = 2;

        if(obstacle_distance != -1) {
            cvInitFont (&dfont, CV_FONT_HERSHEY_SIMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "%.2f m", obstacle_distance);
            cvPutText(IM_D_clone_plot, distance_string, cvPoint(corner_point[0+i*4] , corner_point[3+i*4]), &dfont, CV_RGB(255, 0, 0));
#if 1 // AXE
	    car_type_array.push_back(car_type[i]);
	    for(j=0; j<4; j++){
	      corner_point_array.push_back(corner_point[i+4+j]);
	    }
#endif
        } else {
            cvInitFont (&dfont, CV_FONT_HERSHEY_SIMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "No data");
            cvPutText(IM_D_clone_plot, distance_string, cvPoint(corner_point[0+i*4] , corner_point[3+i*4]), &dfont, CV_RGB(255, 0, 0));
        }
    }
#if 1 // AXE
    fusion::FusedObjects fused_objects_msg;
    fused_objects_msg.car_num = car_type_array.size();
    fused_objects_msg.car_type = car_type_array;
    fused_objects_msg.corner_point = corner_point_array;
    fused_objects.publish(fused_objects_msg);
#endif
    cvShowImage(WINDOW_NAME,IM_D_clone_plot);
    cvWaitKey(2);
    cvClearSeq(points);
    cvReleaseMemStorage(&storage);
    cvReleaseImage(&IM_D_clone_plot);
}

/* メモリリークを防ぐための苦肉の策 */
cv_bridge::CvImagePtr cv_image;
IplImage temp;
void imageCallback(const sensor_msgs::Image& image_source)
{
    /* メモリリークを起こす */
    // cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
    // IplImage temp = cv_image->image;
    // IM_D = &temp;
    // show_rects(IM_D, car_num, corner_point, car_type, ratio);
    // IM_D_clone = cvCloneImage(IM_D);

    /* メモリリークを起こさない */
    cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
    temp = cv_image->image;

    IM_D_clone = &temp;
    show_rects(IM_D_clone, car_num, corner_point, car_type, ratio);
}



// %EndTag(CALLBACK)%



int main(int argc, char **argv)
{

//    cvNamedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
    cvNamedWindow(WINDOW_NAME, 2);
#if 1 // AXE
    IM_D_clone = NULL;
    car_num = 0;
#else
    IM_D_clone = cvLoadImage("/home/yukky/workspace/cartrack/gccDebug/CAR_TRACKING/Test_Images/Daytime_Image_PNG/5.png",CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
#endif

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
#if 1 // AXE
  ros::init(argc, argv, "fusion_detector");
#else
  ros::init(argc, argv, "obstacle_detection_and_distance_measurement");
#endif

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;


  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
#if 1 // AXE
  ros::Subscriber obstacle_detection = n.subscribe("image_objects", 1, obstacle_detectionCallback);
  ros::Subscriber distance_measurement = n.subscribe("scan_image", 1, distance_measurementCallback);
  ros::Subscriber image = n.subscribe("/image_raw", 1, imageCallback);
  
  fused_objects = n.advertise<fusion::FusedObjects>("fused_objects", 10);
#else
  ros::Subscriber obstacle_detection = n.subscribe("obstacle_position", 1, obstacle_detectionCallback);
  ros::Subscriber distance_measurement = n.subscribe("transformed_point_data", 1, distance_measurementCallback);
  ros::Subscriber image = n.subscribe("/usb_cam/image_raw", 1, imageCallback);
#endif

//  image_transport::ImageTransport it(n);
//  image_transport::Subscriber sub = it.subscribe("imageraw", 1, chatterCallback);
// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%
  cvDestroyWindow(WINDOW_NAME);
/* yukky
  //release car-detector-model
  free_model(MO);
  //release detection result
  release_result(LR);
  //close and release file information
  fclose(fp);			//close laser_file
  //cvReleaseCapture(&capt);
  s_free(FPASS);		//release laser_file pass
*/
  return 0;
}
// %EndTag(FULLTEXT)%
