#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "dpm/ImageObjects.h"

using namespace std;
using namespace cv;

vector<Rect> cars;
vector<Rect> peds;

static void image_viewer_callback(const sensor_msgs::Image& image_source)
{
	const auto& encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source,
							     encoding);
	IplImage frame = cv_image->image;
	
	Mat matImage(&frame, false);	
	//cvtColor(matImage, matImage, CV_BGR2RGB);
	
	for(std::size_t i=0; i<cars.size();i++)
	{
		
		cvRectangle( &frame, 
				cvPoint(cars[i].x, cars[i].y),
				cvPoint(cars[i].x+cars[i].width, cars[i].y+cars[i].height),
				Scalar( 0, 255, 0,0 ), 3, 8,0 );
	}
	for(std::size_t i=0; i<peds.size();i++)
	{
		
		cvRectangle( &frame, 
				cvPoint(peds[i].x, peds[i].y),
				cvPoint(peds[i].x+peds[i].width, peds[i].y+peds[i].height),
				Scalar( 255, 0,0, 0 ), 3, 8,0 );
	}
	

	cvShowImage("Image Viewer", &frame);
	cvWaitKey(2);
}

void car_updater_callback(dpm::ImageObjects image_objects_msg)
{
	int num = image_objects_msg.car_num;
	vector<int> points = image_objects_msg.corner_point; 
	//points are X,Y,W,H and repeat for each instance
	cars.clear();
	
	for (int i=0; i<num;i++)
	{
		Rect tmp;
		tmp.x = points[i*4 + 0];
		tmp.y = points[i*4 + 1];
		tmp.width = points[i*4 + 2];
		tmp.height = points[i*4 + 3];
		cars.push_back(tmp);
		
	}
	
}
void ped_updater_callback(dpm::ImageObjects image_objects_msg)
{
	int num = image_objects_msg.car_num;
	vector<int> points = image_objects_msg.corner_point; 
	//points are X,Y,W,H and repeat for each instance
	peds.clear();
	
	for (int i=0; i<num;i++)
	{
		Rect tmp;
		tmp.x = points[i*4 + 0];
		tmp.y = points[i*4 + 1];
		tmp.width = points[i*4 + 2];
		tmp.height = points[i*4 + 3];
		peds.push_back(tmp);
		
	}
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_viewer");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	string image_node;
	string car_node;
	string pedestrian_node;

	if (private_nh.getParam("image_node", image_node))
    	{
        	ROS_INFO("Setting image node to %s", image_node.c_str());
    	}
	else
	{
		ROS_INFO("No image node received, defaulting to image_raw, you can use _image_node:=YOUR_NODE");
		image_node = "/image_raw";
	}

	if (private_nh.getParam("cars_node", car_node))
    	{
        	ROS_INFO("Setting car positions node to %s", car_node.c_str());
    	}
	else
	{
		ROS_INFO("No car positions node received, defaulting to car_pos_xy, you can use _car_node:=YOUR_TOPIC");
		car_node = "/car_pos_xy";
	}

	if (private_nh.getParam("pedestrian_node", pedestrian_node))
    	{
        	ROS_INFO("Setting pedestrian positions node to %s", pedestrian_node.c_str());
    	}
	else
	{
		ROS_INFO("No pedestrian positions node received, defaulting to pedestrian_pos_xy, you can use _pedestrian_node:=YOUR_TOPIC");
		pedestrian_node = "/pedestrian_pos_xy";
	}

	ros::Subscriber scriber = n.subscribe(image_node, 1,
					      image_viewer_callback);
	ros::Subscriber scriber_car = n.subscribe(car_node, 1,
					      car_updater_callback);
	ros::Subscriber scriber_ped = n.subscribe(car_node, 1,
					      ped_updater_callback); 

	ros::spin();
	return 0;
}
