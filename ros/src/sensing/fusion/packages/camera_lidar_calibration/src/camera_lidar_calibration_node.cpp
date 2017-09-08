/*
 * camera_lidar_calibration.cpp
 *
 *  Created on: Sep 4, 2017
 *      Author: ne0
 */

#include <string>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <autoware_msgs/CloudClusterArray.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

class RosCameraLidarApp
{
	ros::NodeHandle 	node_handle_;

	ros::Subscriber 	subscriber_image_raw_;
	ros::Subscriber 	subscriber_points_raw_;
    ros::Subscriber     subscriber_clusters_;

	ros::Publisher 		publisher_image_rect_;
	ros::Publisher 		publisher_plane_points_;

	int 				chessboard_width_;
	int	 				chessboard_height_;

	void PublishCloud(const ros::Publisher* in_publisher,
					  const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
					  std_msgs::Header in_header)
	{
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
		cloud_msg.header = in_header;
		in_publisher->publish(cloud_msg);
	}

	void SegmentPlanes(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
					   pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr)
	{
		std::vector<int> inliers;

		pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr
				model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZI> (in_cloud_ptr));

		pcl::RandomSampleConsensus<pcl::PointXYZI> ransac (model_p);
		ransac.setDistanceThreshold (.01);
		ransac.computeModel();
		ransac.getInliers(inliers);

		// copies all inliers of the model computed to another PointCloud
		pcl::copyPointCloud<pcl::PointXYZI>(*in_cloud_ptr, inliers, *out_cloud_ptr);

	}

	void ImageCallback(const sensor_msgs::Image& in_image_sensor)
	{
		//Receive Image, convert it to OpenCV Mat
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_sensor, "bgr8");
		cv::Mat image = cv_image->image;

		cv::Size pattern_size(chessboard_width_,chessboard_height_); //interior number of corners
		cv::Mat gray; //source image
		cv::cvtColor(image, gray, CV_BGR2GRAY);
		std::vector<cv::Point2f> corners; //this will be filled by the detected corners

		//CALIB_CB_FAST_CHECK saves a lot of time on images
		//that do not contain any chessboard corners
		bool chessboard_found = findChessboardCorners(gray, pattern_size, corners,
												  cv::CALIB_CB_ADAPTIVE_THRESH
												  + cv::CALIB_CB_NORMALIZE_IMAGE
												  //+ cv::CALIB_CB_FAST_CHECK
												);

		if(chessboard_found)
		{
			ROS_INFO("Chessboard found.");
			cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
						 cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.1));
		}

		cv::drawChessboardCorners(image, pattern_size, cv::Mat(corners), chessboard_found);

		cv_bridge::CvImage out_msg;
		out_msg.header   = in_image_sensor.header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image    = image; // Your cv::Mat

		publisher_image_rect_.publish(out_msg.toImageMsg());

	}

	void PointsCallback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(*in_sensor_cloud, *velodyne_cloud_ptr);

		pcl::PointCloud<pcl::PointXYZI>::Ptr planes_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);

		//SegmentPlanes(velodyne_cloud_ptr, planes_cloud_ptr);

		PublishCloud(&publisher_plane_points_, planes_cloud_ptr, in_sensor_cloud->header);
	}

    void ClustersCallback(const autoware_msgs::CloudClusterArrayPtr& in_clusters)
    {

    }

public:
	void Run()
	{
		ros::NodeHandle private_node_handle("~");//to receive args

		std::string image_raw_topic_str, points_raw_topic_str, clusters_topic_str;

        private_node_handle.param<std::string>("image_src", clusters_topic_str, "image_raw");
        ROS_INFO("image_src: %s", image_raw_topic_str.c_str());

        private_node_handle.param<std::string>("points_src", points_raw_topic_str, "points_raw");
        ROS_INFO("points_src: %s", points_raw_topic_str.c_str());

		private_node_handle.param("chessboard_width", chessboard_width_, 6);
		ROS_INFO("chessboard_width: %d", chessboard_width_);

		private_node_handle.param("chessboard_height", chessboard_height_, 8);
		ROS_INFO("chessboard_height: %d", chessboard_height_);

        private_node_handle.param<std::string>("clusters_topic_str", clusters_topic_str, "cloud_clusters");
        ROS_INFO("clusters_topic: %s", clusters_topic_str.c_str());

		ROS_INFO("Camera Lidar calibrator initialized.");

		ROS_INFO("Subscribing to... %s", image_raw_topic_str.c_str());
		subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &RosCameraLidarApp::ImageCallback, this);

		ROS_INFO("Subscribing to... %s", points_raw_topic_str.c_str());
		subscriber_points_raw_ = node_handle_.subscribe(points_raw_topic_str, 1, &RosCameraLidarApp::PointsCallback, this);

        ROS_INFO("Subscribing to... %s", points_raw_topic_str.c_str());
        subscriber_clusters_ = node_handle_.subscribe(clusters_topic_str, 1, &RosCameraLidarApp::ClustersCallback, this);

		publisher_image_rect_ = node_handle_.advertise<sensor_msgs::Image>("/image_chessboard", 1);
		ROS_INFO("Publishing Rectified image in /image_chessboard");

		publisher_plane_points_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/points_chess_plane", 1);
		ROS_INFO("Publishing Plane points cloud in /points_chess_plane");

		ROS_INFO("Ready. Waiting for data..."
                  "\nRemember to Launch Euclidean Clustering Node with integrated ground removal, and no downsampling");
		ros::spin();
		ROS_INFO("END Calibrator");
	}

	~RosCameraLidarApp()
	{

	}

	RosCameraLidarApp()
	{

	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_lidar_calibration");

	RosCameraLidarApp app;

	app.Run();

	return 0;
}
