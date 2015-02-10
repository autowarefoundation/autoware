#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "points_to_image/PointsImage.h"
#include "points_to_image/CameraExtrinsic.h"

#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

#define IMAGE_WIDTH 800
#define IMAGE_HEIGHT 640

cv::Mat cameraExtrinsicMat;
cv::Mat cameraMat;
cv::Mat distCoeff;
cv::Size imageSize;

ros::Publisher pub;
ros::Publisher cpub;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	int w = imageSize.width;
	int h = imageSize.height;

	points_to_image::PointsImage pub_msg;
	points_to_image::CameraExtrinsic cpub_msg;

	//pub_msg.header = msg->header;

	pub_msg.intensity.assign(w * h, 0);
	pub_msg.distance.assign(w * h, 0);
	cv::Mat invR = cameraExtrinsicMat(cv::Rect(0,0,3,3)).t();
	cv::Mat invT = -invR*(cameraExtrinsicMat(cv::Rect(3,0,1,3)));
	char* cp = (char*)msg->data.data();

	pub_msg.max_y = -1;
	pub_msg.min_y = h;
	int x, y;
	for(y=0; y<msg->height; y++){
		for(x=0; x<msg->width; x++){
			float* fp = (float *)(cp + msg->row_step * y + msg->point_step * x);
			double intensity = fp[4];

			cv::Mat point(1, 3, CV_64F);
			point.at<double>(0) = double(fp[0]);
			point.at<double>(1) = double(fp[1]);
			point.at<double>(2) = double(fp[2]);
			point = point * invR.t() + invT.t();

			if (point.at<double>(2) <= 0) {
				continue;
			}

			double tmpx = point.at<double>(0) / point.at<double>(2);
			double tmpy = point.at<double>(1)/point.at<double>(2);
			double r2 = tmpx * tmpx + tmpy * tmpy;
			double tmpdist = 1 + distCoeff.at<double>(0) * r2
				+ distCoeff.at<double>(1) * r2 * r2
				+ distCoeff.at<double>(4) * r2 * r2 * r2;

			cv::Point2d imagepoint;
			imagepoint.x = tmpx * tmpdist 
				+ 2 * distCoeff.at<double>(2) * tmpx * tmpy
				+ distCoeff.at<double>(3) * (r2 + 2 * tmpx * tmpx);
			imagepoint.y = tmpy * tmpdist
				+ distCoeff.at<double>(2) * (r2 + 2 * tmpy * tmpy)
				+ 2 * distCoeff.at<double>(3) * tmpx * tmpy;
			imagepoint.x = cameraMat.at<double>(0,0) * imagepoint.x + cameraMat.at<double>(0,2);
			imagepoint.y = cameraMat.at<double>(1,1) * imagepoint.y + cameraMat.at<double>(1,2);

			int px = int(imagepoint.x + 0.5);
			int py = int(imagepoint.y + 0.5);
			if(0 <= px && px < w && 0 <= py && py < h){
				int pid = py * w + px;
				if(pub_msg.distance[pid] == 0 ||
				   pub_msg.distance[pid] > point.at<double>(2)){
					pub_msg.distance[pid] = float(point.at<double>(2) * 100);
					pub_msg.intensity[pid] = float(intensity);

					pub_msg.max_y = py > pub_msg.max_y ? py : pub_msg.max_y;
					pub_msg.min_y = py < pub_msg.min_y ? py : pub_msg.min_y;
				}
			}
		}
	}
	pub.publish(pub_msg);

	//publish calibration value
	std::vector<float> cali;
	for(int i=0; i<cpub_msg.ysize ; i++){
	  for(int j=0; j<cpub_msg.xsize ; j++){
	    cali.push_back(cameraExtrinsicMat.at<float>(i,j));
	  }
	}
	cpub_msg.calibration = cali;

	cpub.publish(cpub_msg);
       
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "points_to_image");
	ros::NodeHandle n;

	if(argc < 2){
		std::cout<<"Need calibration filename as the first parameter.";
		return 0;
	}

	cv::FileStorage fs(argv[1], cv::FileStorage::READ);
	if(!fs.isOpened()){
		std::cout<<"Invalid calibration filename.";
		return 0;
	}

	fs[CAMERAEXTRINSICMAT] >> cameraExtrinsicMat;
	fs[CAMERAMAT] >> cameraMat;
	fs[DISTCOEFF] >> distCoeff;
	//fs[IMAGESIZE] >> imageSize;
	imageSize.width = IMAGE_WIDTH;
	imageSize.height = IMAGE_HEIGHT;

	pub = n.advertise<points_to_image::PointsImage>("points_image", 10);
	cpub = n.advertise<points_to_image::CameraExtrinsic>("camera_calibration", 1);
	ros::Subscriber sub = n.subscribe("velodyne_points", 1, callback);

	ros::spin();
	return 0;
}
