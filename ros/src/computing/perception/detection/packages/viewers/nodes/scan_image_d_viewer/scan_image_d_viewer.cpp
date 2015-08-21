/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//openCV library
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <float.h>
#include "cv_tracker/image_obj_ranged.h"
#include "scan2image/ScanImage.h"

#define IMAGE_WIDTH 800
#define IMAGE_HEIGHT 600
#define NO_DATA 0

char window_name[] = "SCAN_IMAGE_VIEWER";
//for imageCallback
cv_bridge::CvImagePtr cv_image;
IplImage image;
scan2image::ScanImage scan_image;
bool exist_image = false;
bool exist_scan = false;
cv::Mat colormap;

cv_tracker::image_obj_ranged car_fused_objects;
cv_tracker::image_obj_ranged pedestrian_fused_objects;
static const int OBJ_RECT_THICKNESS = 3;

/* check whether floating value x is nearly 0 or not */
static inline bool isNearlyNODATA(float x)
{
    float abs_x  = (float)fabs(x);
    const int rangeScale = 100;
    return(abs_x < FLT_MIN*rangeScale);
}

static void putDistance(IplImage *Image,
                        std::vector<cv_tracker::image_rect_ranged> objects,
                        int threshold_height,
                        const char* objectLabel)
{
  char distance_string[32];
  CvFont dfont;
  float hscale	    = 0.7f;
  float vscale	    = 0.7f;
  float italicscale = 0.0f;
  int	thickness   = 1;

  CvFont      dfont_label;
  float       hscale_label = 0.5f;
  float       vscale_label = 0.5f;
  CvSize      text_size;
  int         baseline     = 0;

  cvInitFont(&dfont_label, CV_FONT_HERSHEY_COMPLEX, hscale_label, vscale_label, italicscale, thickness, CV_AA);
  cvGetTextSize(objectLabel,
                &dfont_label,
                &text_size,
                &baseline);

  for (unsigned int i=0; i<objects.size(); i++)
    {
      if (objects.at(i).rect.y > threshold_height) // temporal way to avoid drawing detections in the sky
        {
          if (!isNearlyNODATA(objects.at(i).range))
            {
              /* put label */
              CvPoint labelOrg = cvPoint(objects.at(i).rect.x - OBJ_RECT_THICKNESS,
                                         objects.at(i).rect.y - baseline - OBJ_RECT_THICKNESS);

              cvRectangle(Image,
                          cvPoint(labelOrg.x + 0, labelOrg.y + baseline),
                          cvPoint(labelOrg.x + text_size.width, labelOrg.y - text_size.height),
                          CV_RGB(0, 0, 0), // label background is black
                          -1, 8, 0
                          );
              cvPutText(Image,
                        objectLabel,
                        labelOrg,
                        &dfont_label,
                        CV_RGB(255, 255, 255) // label text color is white
                        );

              /* put distance data */
              cvRectangle(Image,
                          cv::Point(objects.at(i).rect.x + (objects.at(i).rect.width/2) - (((int)log10(objects.at(i).range/100)+1) * 5 + 45),
                                    objects.at(i).rect.y + objects.at(i).rect.height + 5),
                          cv::Point(objects.at(i).rect.x + (objects.at(i).rect.width/2) + (((int)log10(objects.at(i).range/100)+1) * 8 + 38),
                                    objects.at(i).rect.y + objects.at(i).rect.height + 30),
                          cv::Scalar(255,255,255),
                          -1);

              cvInitFont (&dfont,
                          CV_FONT_HERSHEY_COMPLEX,
                          hscale,
                          vscale,
                          italicscale,
                          thickness,
                          CV_AA);

              sprintf(distance_string, "%.2f m", objects.at(i).range / 100); //unit of length is meter
              cvPutText(Image,
                        distance_string,
                        cvPoint(objects.at(i).rect.x + (objects.at(i).rect.width/2) - (((int)log10(objects.at(i).range/100)+1) * 5 + 40),
                                objects.at(i).rect.y + objects.at(i).rect.height + 25),
                        &dfont,
                        CV_RGB(255, 0, 0));
            }
        }
    }
}

static void drawRects(IplImage *Image,
                      std::vector<cv_tracker::image_rect_ranged> objects,
                      CvScalar color,
                      int threshold_height)
{
    unsigned int object_num = objects.size();
    for(unsigned int i = 0; i < object_num; i++)
    {
        if (objects.at(i).rect.y > threshold_height && !isNearlyNODATA(objects.at(i).range)) // temporal way to avoid drawing detections in the sky
        {
            CvPoint p1=cvPoint(objects.at(i).rect.x, objects.at(i).rect.y);
            CvPoint p2=cvPoint(objects.at(i).rect.x + objects.at(i).rect.width, objects.at(i).rect.y + objects.at(i).rect.height);
            cvRectangle(Image,p1,p2,color,OBJ_RECT_THICKNESS);
        }
    }
}

static void show()
{
    if(!exist_image || !exist_scan){
        return;
    }

    IplImage* image_view = cvCreateImage(cvGetSize(&image), image.depth, image.nChannels);
    cvCopy(&image, image_view);

	float min_d, max_d;
	min_d = max_d = scan_image.distance.at(0);
	for(int i = 1; i < IMAGE_WIDTH * IMAGE_HEIGHT; i++){
		float di = scan_image.distance.at(i);
		max_d = di > max_d ? di : max_d;
		min_d = di < min_d ? di : min_d;
	}
	float wid_d = max_d - min_d;

    /*
     * Plot depth points on an image
     */
    CvPoint pt;
    int height, width;
    for(int i = 0; i < (int)scan_image.distance.size(); i++) {
        height = (int)(i % IMAGE_HEIGHT);
        width = (int)(i / IMAGE_HEIGHT);
        if(scan_image.distance.at(i) != 0.0) {
            pt.x = width;
            pt.y = height;
			int colorid= wid_d ? ( (scan_image.distance.at(i) - min_d) * 255 / wid_d ) : 128;
			cv::Vec3b color=colormap.at<cv::Vec3b>(colorid);
			int g = color[1];
			int b = color[2];
			int r = color[0];
            cvCircle(image_view, pt, 2, CV_RGB (r, g, b), CV_FILLED, 8, 0);
        }
    }


  drawRects(image_view,
            car_fused_objects.obj,
            cvScalar(255.0, 255.0, 0,0),
            (image_view->height)*.3);

  drawRects(image_view,
            pedestrian_fused_objects.obj,
            cvScalar(0.0, 255.0, 0,0),
            (image_view->height)*.3);
  /* PUT DISTANCE text on image */
  putDistance(image_view,
              car_fused_objects.obj,
              (image_view->height)*.3,
              car_fused_objects.type.c_str());
  putDistance(image_view,
              pedestrian_fused_objects.obj,
              (image_view->height)*.3,
              pedestrian_fused_objects.type.c_str());

    /*
     * Show image
     */
    cvShowImage(window_name, image_view);
    cvWaitKey(2);
    cvReleaseImage(&image_view);
}

static void scan_image_callback(const scan2image::ScanImage& scan_image_msg)
{
    scan_image = scan_image_msg;
    exist_scan = true;
    show();
}

static void car_fusion_callback(const cv_tracker::image_obj_ranged& fused_car_msg)
{
  car_fused_objects = fused_car_msg;
//  show();
}

static void ped_fusion_callback(const cv_tracker::image_obj_ranged& fused_pds_msg)
{
  pedestrian_fused_objects = fused_pds_msg;
//  show();
}

static void image_callback(const sensor_msgs::Image& image_msg)
{
    cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    image = cv_image->image;
    exist_image = true;
    show();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sca_image_d_viewer");
    ros::NodeHandle n;

    ros::Subscriber scan_image_sub = n.subscribe("/scan_image", 1, scan_image_callback);
    ros::Subscriber image_sub = n.subscribe("/image_raw", 1, image_callback);
    ros::Subscriber car_fusion_sub = n.subscribe("/obj_car/image_obj_ranged", 1, car_fusion_callback);
    ros::Subscriber pedestrian_fusion_sub = n.subscribe("/obj_person/image_obj_ranged", 1, ped_fusion_callback);

    cv::Mat grayscale(256,1,CV_8UC1);
    for(int i = 0; i < 256; i++) {
        grayscale.at<uchar>(i)=i;
    }
    cv::applyColorMap(grayscale, colormap, cv::COLORMAP_JET);
    cvNamedWindow(window_name, 2);

    ros::spin();

    cvDestroyWindow(window_name);
    return 0;
}
