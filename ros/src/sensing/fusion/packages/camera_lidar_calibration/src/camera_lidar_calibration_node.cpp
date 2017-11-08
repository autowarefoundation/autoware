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

	int 				chessboard_squares_width_;
	int	 				chessboard_squares_height_;
	double 				chessboard_width_;//real dimensions [meters]
	double	 			chessboard_height_;//real dimensions [meters]
	int 				chessboard_min_points_;
	int	 				chessboard_max_points_;

	void PublishCloud(const ros::Publisher* in_publisher,
					  const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
					  std_msgs::Header in_header)
	{
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
		cloud_msg.header = in_header;
		in_publisher->publish(cloud_msg);
	}
	void PublishColorCloud(const ros::Publisher* in_publisher,
					  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr,
					  std_msgs::Header in_header)
	{
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
		cloud_msg.header = in_header;
		in_publisher->publish(cloud_msg);
	}

	void GetCentroidMinMaxPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_ptr,
								pcl::PointXYZ& out_centroid,
								pcl::PointXYZ& out_min_point,
								pcl::PointXYZ& out_max_point)
	{
		float min_x=std::numeric_limits<float>::max();float max_x=-std::numeric_limits<float>::max();
		float min_y=std::numeric_limits<float>::max();float max_y=-std::numeric_limits<float>::max();
		float min_z=std::numeric_limits<float>::max();float max_z=-std::numeric_limits<float>::max();
		for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
		{
			//fill new colored cluster point by point
			pcl::PointXYZRGB p;
			p.x = in_cloud_ptr->points[i].x;
			p.y = in_cloud_ptr->points[i].y;
			p.z = in_cloud_ptr->points[i].z;

			out_centroid.x += p.x; out_centroid.y += p.y;	out_centroid.z += p.z;

			if(p.x<min_x)	min_x = p.x;
			if(p.y<min_y)	min_y = p.y;
			if(p.z<min_z)	min_z = p.z;
			if(p.x>max_x)	max_x = p.x;
			if(p.y>max_y)	max_y = p.y;
			if(p.z>max_z)	max_z = p.z;
		}
		if (in_cloud_ptr->points.size() > 0)
		{
			out_centroid.x /= in_cloud_ptr->points.size();
			out_centroid.y /= in_cloud_ptr->points.size();
			out_centroid.z /= in_cloud_ptr->points.size();
		}
		out_min_point.x = min_x; out_min_point.y = min_y; out_min_point.z= min_z;
		out_max_point.x = max_x; out_max_point.y = max_y; out_max_point.z= max_z;
	}

	void FitPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_ptr,
					   pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr)
	{
		std::vector<int> inliers;
		Eigen::VectorXf plane_coeff;

		pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
				model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (in_cloud_ptr));

		pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
		ransac.setDistanceThreshold (.05);
		ransac.computeModel();
		ransac.getInliers(inliers);

		ransac.getModelCoefficients(plane_coeff);

		// copies all inliers of the model computed to another PointCloud
		pcl::copyPointCloud<pcl::PointXYZRGB>(*in_cloud_ptr, inliers, *out_cloud_ptr);

		//////////////////////////////////////////////////////////
		//copy the resulting plane
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::copyPointCloud<pcl::PointXYZRGB>(*out_cloud_ptr, *origin_cloud_ptr);

		///////
		pcl::PointXYZ centroid, min_point, max_point;
		GetCentroidMinMaxPoints(out_cloud_ptr, centroid, min_point, max_point);

		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
		transform_2.translation() << -centroid.x, -centroid.y, -centroid.z;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::transformPointCloud (*out_cloud_ptr, *transformed_cloud, transform_2);

		*out_cloud_ptr += *transformed_cloud;
	}

	void GetCloudDimensions(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_ptr, cv::Point3f& out_dimensions)
	{

	}
	void ImageCallback(const sensor_msgs::Image& in_image_sensor)
	{
		//Receive Image, convert it to OpenCV Mat
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_sensor, "bgr8");
		cv::Mat image = cv_image->image;

		cv::Size pattern_size(chessboard_squares_width_,chessboard_squares_height_); //interior number of corners
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
		//pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
		//pcl::fromROSMsg(*in_sensor_cloud, *velodyne_cloud_ptr);

		//pcl::PointCloud<pcl::PointXYZI>::Ptr planes_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);

		//SegmentPlanes(velodyne_cloud_ptr, planes_cloud_ptr);


	}

	void ClustersCallback(const autoware_msgs::CloudClusterArrayPtr& in_clusters)
	{
		pcl::PointCloud<pcl::PointXYZRGB> planes_cloud;
		planes_cloud.points.clear();

		for(size_t i = 0; i <in_clusters->clusters.size() ; i++)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::fromROSMsg(in_clusters->clusters[i].cloud, *cluster_cloud_ptr);

			if(cluster_cloud_ptr->points.size() >= chessboard_min_points_
			   && cluster_cloud_ptr->points.size() <= chessboard_max_points_ )
			{
				FitPlane(cluster_cloud_ptr, plane_cloud_ptr);
				//GetCloudDimensions(plane_cloud_ptr);
				planes_cloud += *plane_cloud_ptr;
			}
		}
		PublishColorCloud(&publisher_plane_points_, planes_cloud.makeShared(), in_clusters->header);
	}

public:
	void Run()
	{
		ros::NodeHandle private_node_handle("~");//to receive args

		std::string image_raw_topic_str, points_raw_topic_str, clusters_topic_str;

		private_node_handle.param<std::string>("image_src", image_raw_topic_str, "image_raw");
		ROS_INFO("image_src: %s", image_raw_topic_str.c_str());

		private_node_handle.param<std::string>("points_src", points_raw_topic_str, "points_raw");
		ROS_INFO("points_src: %s", points_raw_topic_str.c_str());

		private_node_handle.param<std::string>("clusters_topic_str", clusters_topic_str, "cloud_clusters");
		ROS_INFO("clusters_topic: %s", clusters_topic_str.c_str());

		private_node_handle.param("chessboard_squares_width", chessboard_squares_width_, 6);
		ROS_INFO("chessboard_squares_width: %d", chessboard_squares_width_);

		private_node_handle.param("chessboard_squares_height", chessboard_squares_height_, 8);
		ROS_INFO("chessboard_squares_height: %d", chessboard_squares_height_);

		private_node_handle.param("chessboard_width", chessboard_width_, 1.5);
		ROS_INFO("chessboard_width: %f", chessboard_width_);

		private_node_handle.param("chessboard_height", chessboard_height_, 1.0);
		ROS_INFO("chessboard_height: %f", chessboard_height_);

		private_node_handle.param("chessboard_min_points", chessboard_min_points_, 500);
		ROS_INFO("chessboard_min_points: %d", chessboard_min_points_);

		private_node_handle.param("chessboard_max_points", chessboard_max_points_, 2000);
		ROS_INFO("chessboard_max_points: %d", chessboard_max_points_);

		ROS_INFO("Subscribing to... %s", image_raw_topic_str.c_str());
		subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &RosCameraLidarApp::ImageCallback, this);

		ROS_INFO("Subscribing to... %s", points_raw_topic_str.c_str());
		subscriber_points_raw_ = node_handle_.subscribe(points_raw_topic_str, 1, &RosCameraLidarApp::PointsCallback, this);

		ROS_INFO("Subscribing to... %s", clusters_topic_str.c_str());
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
