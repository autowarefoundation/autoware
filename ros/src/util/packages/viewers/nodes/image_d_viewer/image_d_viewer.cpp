#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include "car_detector/FusedObjects.h"
#define NO_DATA 0

char window_name[] = "CAR_TRACK";
//for imageCallback
cv_bridge::CvImagePtr cv_image;
IplImage temp;
IplImage *image;
std::vector<int> depth_point_x_on_image;
std::vector<int> depth_point_y_on_image;
double ratio = 1;	//resize ratio

void showRects(IplImage *Image,int object_num, std::vector<int> corner_point, double ratio)
{
	for(int i = 0; i < object_num; i++)
	{
		CvScalar col = cvScalar(255.0,255.0,0.0);
		CvPoint p1=cvPoint(corner_point[0+i*4], corner_point[1+i*4]);
		CvPoint p2=cvPoint(corner_point[0+i*4] + corner_point[2+i*4], corner_point[1+i*4] + corner_point[3+i*4]);
		cvRectangle(Image,p1,p2,col,3);
	}
}

void fused_objectsCallback(const car_detector::FusedObjects& fused_objects)
{
    if(image == NULL){
      return;
    }

    IplImage *image_clone = cvCloneImage(image);

    int i;

    for (i = 0; i < fused_objects.car_num; i++) {
        if(fused_objects.distance.at(i) != NO_DATA) {
            printf("%f\n", fused_objects.distance.at(i));
        } else {
            printf("no data\n");
        }

        char distance_string[32];
        CvFont dfont;
        float hscale      = 1.0f;
        float vscale      = 1.0f;
        float italicscale = 0.0f;
        int  thickness    = 2;

        /*
         * Plot distances on an image
         */
        if(fused_objects.distance.at(i) != NO_DATA) {
            cvInitFont (&dfont, CV_FONT_HERSHEY_SIMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "%.2f m", fused_objects.distance.at(i));
            cvPutText(image_clone, distance_string, cvPoint(fused_objects.corner_point[0+i*4] , fused_objects.corner_point[1+i*4] + fused_objects.corner_point[3+i*4]), &dfont, CV_RGB(255, 0, 0));
        } else {
            cvInitFont (&dfont, CV_FONT_HERSHEY_SIMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "No data");
            cvPutText(image_clone, distance_string, cvPoint(fused_objects.corner_point[0+i*4] , fused_objects.corner_point[1+i*4] + fused_objects.corner_point[3+i*4]), &dfont, CV_RGB(255, 0, 0));
        }
    }

    showRects(image_clone, fused_objects.car_num, fused_objects.corner_point, ratio);

    /*
     * Show image
     */
    cvShowImage(window_name, image_clone);
    cvWaitKey(2);
    cvReleaseImage(&image_clone);
}

void imageCallback(const sensor_msgs::Image& image_source)
{
    cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
    temp = cv_image->image;
    image = &temp;
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
    ros::Subscriber fused_objects_sub = n.subscribe("/car_pos_xyz", 1, fused_objectsCallback);

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();
    cvDestroyWindow(window_name);

    return 0;

}
