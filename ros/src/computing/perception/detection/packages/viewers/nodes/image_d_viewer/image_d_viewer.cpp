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

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include "cv_tracker/image_obj_ranged.h"
#include <math.h>
#include <float.h>
#define NO_DATA 0

static char window_name_base[] = "image_d_viewer";
static std::string window_name;
//for imageCallback
static cv_bridge::CvImagePtr cv_image;
static IplImage temp;
static IplImage *image;
static double ratio = 1;	//resize ratio

static cv_tracker::image_obj_ranged car_fused_objects;
static cv_tracker::image_obj_ranged pedestrian_fused_objects;

static const int OBJ_RECT_THICKNESS = 3;
static void showImage();

/* check whether floating value x is nearly 0 or not */
static inline bool isNearlyNODATA(float x)
{
  float abs_x  = (float)fabs(x);
  const int rangeScale = 100;
  return(abs_x < FLT_MIN*rangeScale);
}

void showRects(IplImage *Image,
               std::vector<cv_tracker::image_rect_ranged> objects,
               double ratio,
               CvScalar col)
{
    unsigned int object_num = objects.size();
    for(unsigned int i = 0; i < object_num; i++)
    {
        if (!isNearlyNODATA(objects.at(i).range))
        {
            CvPoint p1=cvPoint(objects.at(i).rect.x, objects.at(i).rect.y);
            CvPoint p2=cvPoint(objects.at(i).rect.x + objects.at(i).rect.width, objects.at(i).rect.y + objects.at(i).rect.height);
            cvRectangle(Image,p1,p2,col,OBJ_RECT_THICKNESS);
        }
    }
}

static void obj_carCallback(const cv_tracker::image_obj_ranged& fused_objects)
{
    if(image == NULL){
      return;
    }
    car_fused_objects = fused_objects;
    showImage();
}

static void obj_personCallback(const cv_tracker::image_obj_ranged& fused_objects)
{
    if(image == NULL){
      return;
    }
    pedestrian_fused_objects = fused_objects;
    showImage();
}

static void imageCallback(const sensor_msgs::Image& image_source)
{
    cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
    temp = cv_image->image;
    image = &temp;
    showImage();
}

static void showImage()
{
    IplImage* image_clone = cvCloneImage(image);
    char distance_string[32];
    CvFont dfont;
    float hscale      = 0.7f;
    float vscale      = 0.7f;
    float italicscale = 0.0f;
    int  thickness    = 1;

    std::string objectLabel;
    CvFont      dfont_label;
    float       hscale_label = 0.5f;
    float       vscale_label = 0.5f;
    CvSize      text_size;
    int         baseline     = 0;

    cvInitFont(&dfont_label, CV_FONT_HERSHEY_COMPLEX, hscale_label, vscale_label, italicscale, thickness, CV_AA);
    objectLabel = car_fused_objects.type;
    cvGetTextSize(objectLabel.data(),
                  &dfont_label,
                  &text_size,
                  &baseline);

    /*
     * Plot obstacle frame
     */
    showRects(image_clone,
              car_fused_objects.obj,
              ratio,
              cvScalar(255.0,255.0,0.0));
    showRects(image_clone,
              pedestrian_fused_objects.obj,
              ratio,
              cvScalar(0.0,255.0,0.0));


    /*
     * Plot car distance data on image
     */
    for (unsigned int i = 0; i < car_fused_objects.obj.size(); i++) {
      if(!isNearlyNODATA(car_fused_objects.obj.at(i).range)) {
          int rect_x      = car_fused_objects.obj.at(i).rect.x;
          int rect_y      = car_fused_objects.obj.at(i).rect.y;
          int rect_width  = car_fused_objects.obj.at(i).rect.width;
          int rect_height = car_fused_objects.obj.at(i).rect.height;
          float range     = car_fused_objects.obj.at(i).range;

          /* put label */
          CvPoint labelOrg = cvPoint(rect_x - OBJ_RECT_THICKNESS,
                                     rect_y - baseline - OBJ_RECT_THICKNESS);
          cvRectangle(image_clone,
                      cvPoint(labelOrg.x + 0, labelOrg.y + baseline),
                      cvPoint(labelOrg.x + text_size.width, labelOrg.y - text_size.height),
                      CV_RGB(0, 0, 0), // label background color is black
                      -1, 8, 0
                      );
          cvPutText(image_clone,
                    objectLabel.data(),
                    labelOrg,
                    &dfont_label,
                    CV_RGB(255, 255, 255) // label text color is white
                    );

          /* put distance data */
            cvRectangle(image_clone,
                        cv::Point(rect_x + (rect_width/2) - (((int)log10(range/100)+1) * 5 + 45),
                                  rect_y + rect_height + 5),
                        cv::Point(rect_x + (rect_width/2) + (((int)log10(range/100)+1) * 8 + 38),
                                  rect_y + rect_height + 30),
                        cv::Scalar(255,255,255), -1);
            cvInitFont (&dfont, CV_FONT_HERSHEY_COMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "%.2f m", range / 100); //unit of length is meter
            cvPutText(image_clone,
                      distance_string,
                      cvPoint(rect_x + (rect_width/2) - (((int)log10(range/100)+1) * 5 + 40),
                              rect_y + rect_height + 25),
                      &dfont,
                      CV_RGB(255, 0, 0));
        }
    }

    objectLabel = pedestrian_fused_objects.type;
    cvGetTextSize(objectLabel.data(),
                  &dfont_label,
                  &text_size,
                  &baseline);

    /*
     * Plot pedestrian distance data on image
     */
    for (unsigned int i = 0; i < pedestrian_fused_objects.obj.size(); i++) {
      if(!isNearlyNODATA(pedestrian_fused_objects.obj.at(i).range)) {
          int rect_x      = pedestrian_fused_objects.obj.at(i).rect.x;
          int rect_y      = pedestrian_fused_objects.obj.at(i).rect.y;
          int rect_width  = pedestrian_fused_objects.obj.at(i).rect.width;
          int rect_height = pedestrian_fused_objects.obj.at(i).rect.height;
          float range     = pedestrian_fused_objects.obj.at(i).range;

          /* put label */
          CvPoint labelOrg = cvPoint(rect_x - OBJ_RECT_THICKNESS,
                                     rect_y - baseline - OBJ_RECT_THICKNESS);
          cvRectangle(image_clone,
                      cvPoint(labelOrg.x + 0, labelOrg.y + baseline),
                      cvPoint(labelOrg.x + text_size.width, labelOrg.y - text_size.height),
                      CV_RGB(0, 0, 0), // label background color is black
                      -1, 8, 0
                      );
          cvPutText(image_clone,
                    objectLabel.data(),
                    labelOrg,
                    &dfont_label,
                    CV_RGB(255, 255, 255) // label text color is white
                    );

          /* put distance data */
            cvRectangle(image_clone,
                        cv::Point(rect_x + (rect_width/2) - (((int)log10(range/100)+1) * 5 + 45),
                                  rect_y + rect_height + 5),
                        cv::Point(rect_x + (rect_width/2) + (((int)log10(range/100)+1) * 8 + 38),
                                  rect_y + rect_height + 30),
                        cv::Scalar(255,255,255), -1);
            cvInitFont (&dfont, CV_FONT_HERSHEY_COMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "%.2f m", range / 100); //unit of length is meter
            cvPutText(image_clone,
                      distance_string,
                      cvPoint(rect_x + (rect_width/2) - (((int)log10(range/100)+1) * 5 + 40),
                              rect_y + rect_height + 25),
                      &dfont,
                      CV_RGB(255, 0, 0));
        }
    }

    /*
     * Show image
     */
    if (cvGetWindowHandle(window_name.c_str()) != NULL) // Guard not to write destroyed window by using close button on the window
      {
        cvShowImage(window_name.c_str(), image_clone);
        cvWaitKey(2);
      }
    cvReleaseImage(&image_clone);
}

int main(int argc, char **argv)
{
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


    ros::init(argc, argv, "image_d_viewer");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called Callback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    std::string image_topic;
    std::string car_topic;
    std::string person_topic;

    if (!private_nh.getParam("image_topic", image_topic)) {
      image_topic = "/image_raw";
    }

    if (!private_nh.getParam("car_topic", car_topic)) {
      car_topic = "/obj_car/image_obj_ranged";
    }

    if (!private_nh.getParam("person_topic", person_topic)) {
      person_topic = "/obj_person/image_obj_ranged";
    }

    std::string name_space_str = ros::this_node::getNamespace();
    if (name_space_str != "/") {
      window_name = std::string(window_name_base) + " (" + ros::this_node::getNamespace() + ")";
    }
    else {
      window_name = std::string(window_name_base);
    }
    cvNamedWindow(window_name.c_str(), 2);
    cvStartWindowThread();
    image = NULL;
    car_fused_objects.obj.clear();
    pedestrian_fused_objects.obj.clear();

    ros::Subscriber image_sub = n.subscribe(image_topic, 1, imageCallback);
    ros::Subscriber obj_car_sub = n.subscribe(car_topic, 1, obj_carCallback);
    ros::Subscriber obj_person_sub = n.subscribe(person_topic, 1, obj_personCallback);

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();
    cvDestroyWindow(window_name.c_str());

    return 0;
}
