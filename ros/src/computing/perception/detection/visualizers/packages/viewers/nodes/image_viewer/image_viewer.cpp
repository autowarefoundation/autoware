/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *	list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *	this list of conditions and the following disclaimer in the documentation
 *	and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *	contributors may be used to endorse or promote products derived from
 *	this software without specific prior written permission.
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

#include <string>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>

#if (CV_MAJOR_VERSION == 3)
#include "gencolors.cpp"
#else

#include <opencv2/contrib/contrib.hpp>

#endif

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <autoware_msgs/DetectedObjectArray.h>

static std::vector<cv::Scalar> _colors;

static const int OBJ_RECT_THICKNESS = 3;

static bool _drawing = false;

static const std::string window_name = "Image Viewer";

autoware_msgs::DetectedObjectArray detected_objects_;

const int kRectangleThickness = 3;

/*static void dashed_rectangle(cv::Mat& img, const cv::Rect& r, const cv::Scalar& color,
			     int thickness = 2, int dash_length = 10)
{
	//draw horizontal dashed lines
	for (int i = 0; i < r.width; i+=dash_length) {
		cv::line(img, cv::Point(r.x+i, r.y),  cv::Point(r.x+i+(dash_length/2), r.y), color, thickness);
		cv::line(img, cv::Point(r.x+i, r.y + r.height), cv::Point(r.x+i+(dash_length/2), r.y + r.height), color, thickness);
	}

	//draw vertical dashes lines
	for (int i = 0; i < r.height; i+=dash_length) {
		cv::line(img, cv::Point(r.x, r.y+i), cv::Point(r.x, r.y+i+(dash_length/2)), color, thickness);
		cv::line(img, cv::Point(r.x +r.width, r.y+i), cv::Point(r.x+ r.width, r.y+i+(dash_length/2)), color, thickness);
	}
}*/
static void DrawLabel(const std::string &label,
                      const cv::Point &rectangle_origin,
                      cv::Mat &image) {
    // label's property
    const int font_face = cv::FONT_HERSHEY_COMPLEX;
    const double font_scale = 0.5;
    const int font_thickness = 1;
    int font_baseline = 0;

    cv::Size label_size = cv::getTextSize(label,
                                          font_face,
                                          font_scale,
                                          font_thickness,
                                          &font_baseline);

    cv::Point label_origin = cv::Point(rectangle_origin.x - kRectangleThickness,
                                       rectangle_origin.y - font_baseline - kRectangleThickness);

    // Fill label's background by black
    cv::rectangle(image,
                  cv::Point(label_origin.x, label_origin.y + font_baseline),
                  cv::Point(label_origin.x + label_size.width, label_origin.y - label_size.height),
                  CV_RGB(0, 0, 0),
                  CV_FILLED);

    // Draw label text by white
    cv::putText(image,
                label,
                label_origin,
                font_face,
                font_scale,
                CV_RGB(255, 255, 255));

}

static void drawDetections(autoware_msgs::DetectedObjectArray detected_objects, cv::Mat frame) {
    /* variables for object label */
    for (const auto detected_object : detected_objects.objects) {
        // Make label shown on a rectangle
        std::ostringstream label;
        label << detected_object.label << ":" << std::setprecision(2) << detected_object.score;

        // Draw object information label
        DrawLabel(label.str(), cv::Point(detected_object.x, detected_object.y), frame);

        // Draw rectangle
        cv::rectangle(frame,
                      cv::Point(detected_object.x, detected_object.y),
                      cv::Point(detected_object.x + detected_object.width, detected_object.y + detected_object.height),
                      cv::Scalar(detected_object.color.r, detected_object.color.g, detected_object.color.b),
                      kRectangleThickness,
                      CV_AA,
                      0);
    }
}

static void image_viewer_callback(const sensor_msgs::Image &image_source) {
    _drawing = true;

    const auto &encoding = sensor_msgs::image_encodings::BGR8;
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source,
                                                         encoding);

    cv::Mat matImage(cv_image->image);

    //UNTRACKED

    drawDetections(detected_objects_, matImage);


    if (cvGetWindowHandle(window_name.c_str()) !=
        NULL) // Guard not to write destroyed window by using close button on the window
    {
        imshow(window_name, matImage);
        cvWaitKey(2);
    }

    _drawing = false;
}

static void image_obj_update_cb(const autoware_msgs::DetectedObjectArray &image_objs) {
    if (_drawing)
        return;

    detected_objects_ = image_objs;
}

int main(int argc, char **argv) {

    /* create resizable window */
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::startWindowThread();

    ros::init(argc, argv, "image_viewer");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    std::string image_topic_name;

    if (private_nh.getParam("image_raw_topic", image_topic_name)) {
        ROS_INFO("Setting image topic to %s", image_topic_name.c_str());
    } else {
        ROS_INFO("No image topic received, defaulting to image_raw, you can use _image_raw_topic:=YOUR_NODE");
        image_topic_name = "/image_raw";
    }

#if (CV_MAJOR_VERSION == 3)
    generateColors(_colors, 25);
#else
    cv::generateColors(_colors, 25);
#endif

    ros::Subscriber scriber = n.subscribe(image_topic_name, 1, image_viewer_callback);

    ros::Subscriber scriber_obj = n.subscribe("/detected_objects", 1,
                                              image_obj_update_cb);

    ros::spin();

    /* destroy window */
    cv::destroyWindow(window_name);

    return 0;
}
