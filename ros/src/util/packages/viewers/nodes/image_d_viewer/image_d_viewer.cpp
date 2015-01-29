#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include "car_detector/FusedObjects.h"
#define NO_DATA 0

char window_name[] = "image_d_viewer";
//for imageCallback
cv_bridge::CvImagePtr cv_image;
IplImage temp;
IplImage *image;
double ratio = 1;	//resize ratio

car_detector::FusedObjects car_fused_objects;
car_detector::FusedObjects pedestrian_fused_objects;

void showImage();

void showRects(IplImage *Image,int object_num, std::vector<int> corner_point, double ratio, CvScalar col)
{
	for(int i = 0; i < object_num; i++)
	{
		CvPoint p1=cvPoint(corner_point[0+i*4], corner_point[1+i*4]);
		CvPoint p2=cvPoint(corner_point[0+i*4] + corner_point[2+i*4], corner_point[1+i*4] + corner_point[3+i*4]);
		cvRectangle(Image,p1,p2,col,3);
	}
}

void car_pixel_xyzCallback(const car_detector::FusedObjects& fused_objects)
{
    if(image == NULL){
      return;
    }
    car_fused_objects = fused_objects;
    showImage();
}

void pedestrian_pixel_xyzCallback(const car_detector::FusedObjects& fused_objects)
{
    if(image == NULL){
      return;
    }
    pedestrian_fused_objects = fused_objects;
    showImage();
}


void imageCallback(const sensor_msgs::Image& image_source)
{
    cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
    temp = cv_image->image;
    image = &temp;
    showImage();
}

void showImage()
{
    int i;
    IplImage* image_clone = cvCloneImage(image);
    char distance_string[32];
    CvFont dfont;
    float hscale      = 0.7f;
    float vscale      = 0.7f;
    float italicscale = 0.0f;
    int  thickness    = 1;

    /*
     * Plot obstacle frame
     */
    showRects(image_clone, car_fused_objects.car_num, car_fused_objects.corner_point, ratio, cvScalar(255.0,255.0,0.0));
    showRects(image_clone, pedestrian_fused_objects.car_num, pedestrian_fused_objects.corner_point, ratio, cvScalar(0.0,255.0,0.0));

    /*
     * Plot car distance data on image
     */
    for (i = 0; i < car_fused_objects.car_num; i++) {
        if(car_fused_objects.distance.at(i) != NO_DATA) {
            cvRectangle(image_clone,
                        cv::Point(car_fused_objects.corner_point[0+i*4] + (car_fused_objects.corner_point[2+i*4]/2) - (((int)log10(car_fused_objects.distance.at(i)/100)+1) * 5 + 45),
                                  car_fused_objects.corner_point[1+i*4] + car_fused_objects.corner_point[3+i*4] + 5),
                        cv::Point(car_fused_objects.corner_point[0+i*4] + (car_fused_objects.corner_point[2+i*4]/2) + (((int)log10(car_fused_objects.distance.at(i)/100)+1) * 8 + 38),
                                  car_fused_objects.corner_point[1+i*4] + car_fused_objects.corner_point[3+i*4] + 30),
                        cv::Scalar(255,255,255), -1);
            cvInitFont (&dfont, CV_FONT_HERSHEY_COMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "%.2f m", car_fused_objects.distance.at(i) / 100); //unit of length is meter
            cvPutText(image_clone,
                      distance_string,
                      cvPoint(car_fused_objects.corner_point[0+i*4] + (car_fused_objects.corner_point[2+i*4]/2) - (((int)log10(car_fused_objects.distance.at(i)/100)+1) * 5 + 40),
                              car_fused_objects.corner_point[1+i*4] + car_fused_objects.corner_point[3+i*4] + 25),
                      &dfont,
                      CV_RGB(255, 0, 0));
        } else {
            cvRectangle(image_clone,
                        cv::Point(car_fused_objects.corner_point[0+i*4] + (car_fused_objects.corner_point[2+i*4]/2) - 50,
                                  car_fused_objects.corner_point[1+i*4] + car_fused_objects.corner_point[3+i*4] + 5),
                        cv::Point(car_fused_objects.corner_point[0+i*4] + (car_fused_objects.corner_point[2+i*4]/2) + 55,
                                  car_fused_objects.corner_point[1+i*4] + car_fused_objects.corner_point[3+i*4] + 30),
                        cv::Scalar(255,255,255), -1);
            cvInitFont (&dfont, CV_FONT_HERSHEY_COMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "No data");
            cvPutText(image_clone,
                      distance_string,
                      cvPoint(car_fused_objects.corner_point[0+i*4] + (car_fused_objects.corner_point[2+i*4]/2) - 45,
                              car_fused_objects.corner_point[1+i*4] + car_fused_objects.corner_point[3+i*4] + 25),
                      &dfont,
                      CV_RGB(255, 0, 0));
        }
    }

    /*
     * Plot pedestrian distance data on image
     */
    for (i = 0; i < pedestrian_fused_objects.car_num; i++) {
        if(pedestrian_fused_objects.distance.at(i) != NO_DATA) {
            cvRectangle(image_clone,
                        cv::Point(pedestrian_fused_objects.corner_point[0+i*4] + (pedestrian_fused_objects.corner_point[2+i*4]/2) - (((int)log10(pedestrian_fused_objects.distance.at(i)/100)+1) * 5 + 45),
                                  pedestrian_fused_objects.corner_point[1+i*4] + pedestrian_fused_objects.corner_point[3+i*4] + 5),
                        cv::Point(pedestrian_fused_objects.corner_point[0+i*4] + (pedestrian_fused_objects.corner_point[2+i*4]/2) + (((int)log10(pedestrian_fused_objects.distance.at(i)/100)+1) * 8 + 38),
                                  pedestrian_fused_objects.corner_point[1+i*4] + pedestrian_fused_objects.corner_point[3+i*4] + 30),
                        cv::Scalar(255,255,255), -1);
            cvInitFont (&dfont, CV_FONT_HERSHEY_COMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "%.2f m", pedestrian_fused_objects.distance.at(i) / 100); //unit of length is meter
            cvPutText(image_clone,
                      distance_string,
                      cvPoint(pedestrian_fused_objects.corner_point[0+i*4] + (pedestrian_fused_objects.corner_point[2+i*4]/2) - (((int)log10(pedestrian_fused_objects.distance.at(i)/100)+1) * 5 + 40),
                              pedestrian_fused_objects.corner_point[1+i*4] + pedestrian_fused_objects.corner_point[3+i*4] + 25),
                      &dfont,
                      CV_RGB(255, 0, 0));
        } else {
            cvRectangle(image_clone,
                        cv::Point(pedestrian_fused_objects.corner_point[0+i*4] + (pedestrian_fused_objects.corner_point[2+i*4]/2) - 50,
                                  pedestrian_fused_objects.corner_point[1+i*4] + pedestrian_fused_objects.corner_point[3+i*4] + 5),
                        cv::Point(pedestrian_fused_objects.corner_point[0+i*4] + (pedestrian_fused_objects.corner_point[2+i*4]/2) + 55,
                                  pedestrian_fused_objects.corner_point[1+i*4] + pedestrian_fused_objects.corner_point[3+i*4] + 30),
                        cv::Scalar(255,255,255), -1);
            cvInitFont (&dfont, CV_FONT_HERSHEY_COMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "No data");
            cvPutText(image_clone,
                      distance_string,
                      cvPoint(pedestrian_fused_objects.corner_point[0+i*4] + (pedestrian_fused_objects.corner_point[2+i*4]/2) - 45,
                              pedestrian_fused_objects.corner_point[1+i*4] + pedestrian_fused_objects.corner_point[3+i*4] + 25),
                      &dfont,
                      CV_RGB(255, 0, 0));
        }
    }

    /*
     * Show image
     */
    cvShowImage(window_name, image_clone);
    cvWaitKey(2);
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

    cvNamedWindow(window_name, 2);
    image = NULL;
    car_fused_objects.car_num = 0;
    pedestrian_fused_objects.car_num = 0;

    ros::init(argc, argv, "image_d_viewer");

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

    ros::Subscriber image_sub = n.subscribe("/image_raw", 1, imageCallback);
    ros::Subscriber car_pixel_xyz_sub = n.subscribe("/car_pixel_xyz", 1, car_pixel_xyzCallback);
    ros::Subscriber pedestrian_pixel_xyz_sub = n.subscribe("/pedestrian_pixel_xyz", 1, pedestrian_pixel_xyzCallback);

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();
    cvDestroyWindow(window_name);

    return 0;

}
