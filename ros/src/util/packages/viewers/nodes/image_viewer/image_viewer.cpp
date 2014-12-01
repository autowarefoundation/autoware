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

static void image_viewer_callback(const sensor_msgs::Image& image_source)
{
	const auto& encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source,
							     encoding);
	IplImage frame = cv_image->image;
	
	cout << "size:"<<cars.size()<<endl;
	for(int i=0; i<cars.size();i++)
	{
		
		cvRectangle( &frame, 
				cvPoint(cars[i].x, cars[i].y),
				cvPoint(cars[i].x+cars[i].width, cars[i].y+cars[i].height),
				Scalar( 0, 255, 0,0 ), 1, 8,0 );
	}
	

	cvShowImage("Image Viewer", &frame);
	cvWaitKey(2);
}

void rect_updater_callback(dpm::ImageObjects image_objects_msg)
{
	int num = image_objects_msg.car_num;
	vector<int> points = image_objects_msg.corner_point; 
	//points are X,Y,W,H and repeat for each instance
	
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_viewer");
	ros::NodeHandle n;
	ros::Subscriber scriber = n.subscribe("/image_raw", 1,
					      image_viewer_callback);
	ros::Subscriber scriber_car = n.subscribe("/car_pos_xy", 1,
					      rect_updater_callback); 

	ros::spin();
	return 0;
}

