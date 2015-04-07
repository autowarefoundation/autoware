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

void show()
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
    /*
     * Show image
     */
    cvShowImage(window_name, image_view);
    cvWaitKey(2);
    cvReleaseImage(&image_view);
}

void scan_image_callback(const scan2image::ScanImage& scan_image_msg)
{
    scan_image = scan_image_msg;
    exist_scan = true;
    show();
}

void image_callback(const sensor_msgs::Image& image_msg)
{
    cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    image = cv_image->image;
    exist_image = true;
    show();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sca_image_viewer");
    ros::NodeHandle n;

    ros::Subscriber scan_image_sub = n.subscribe("/scan_image", 1, scan_image_callback);
    ros::Subscriber image_sub = n.subscribe("/image_raw", 1, image_callback);

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
