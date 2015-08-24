#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

#include <visualization_msgs/Marker.h>
#include <lidar_tracker/centroids.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/contrib/contrib.hpp>

#include <chrono>
#include <iostream>

using namespace cv;

std::vector<cv::Scalar> _colors;
ros::Publisher pub;
ros::Publisher centroid_pub;
ros::Publisher marker_pub;
visualization_msgs::Marker marker;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	// Convert to PCL data type
	pcl_conversions::toPCL(*input, *cloud);

	//Store PCL and PCL2 formats
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloud, *cloud1);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>());

	/////////////////////////////////
	//---	1. Remove planes (floor)
	/////////////////////////////////

	float distance = 0.5;//this may be a parameter,so we must tune it
	/*seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (distance);

	int nr_points = (int) cloud1->points.size ();
	while (cloud1->points.size () > distance * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud1);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud1);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter (*cloud_plane);

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud1 = *cloud_f;
	}
*/
	/////////////////////////////////
	//---	2. Euclidean Clustering
	/////////////////////////////////
	auto start = std::chrono::system_clock::now(); //start time
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	tree->setInputCloud (cloud1);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (distance); //
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (2500);
	ec.setSearchMethod(tree);
	ec.setInputCloud (cloud1);
	ec.extract (cluster_indices);
	auto end = std::chrono::system_clock::now(); //end time
	auto dur = end - start; //processing time
	double time = std::chrono::duration_cast < std::chrono::microseconds > (dur).count(); //micro sec
	std::cout << "Euclidean Clustering : " << time * 0.001 << " milli sec" << std::endl;

	/////////////////////////////////
	//---	3. Color clustered points
	/////////////////////////////////
	auto start2 = std::chrono::system_clock::now(); //start time

	int j = 0;
	unsigned int k = 0;

	lidar_tracker::centroids centroids;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

		geometry_msgs::Point centroid;
		//assign color to each cluster
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
			//fill new colored cluster point by point
			pcl::PointXYZRGB p;
			p.x = cloud1->points[*pit].x;
			p.y = cloud1->points[*pit].y;
			p.z = cloud1->points[*pit].z;
			p.r = _colors[k].val[0];
			p.g = _colors[k].val[1];
			p.b = _colors[k].val[2];

			centroid.x += cloud1->points[*pit].x;
			centroid.y += cloud1->points[*pit].y;
			centroid.z += cloud1->points[*pit].z;

			cloud_cluster->points.push_back (p);
		}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		*final_cluster = *final_cluster + *cloud_cluster;//sum up all the colored cluster into a complete pc

		j++; k++;

		centroid.x /= it->indices.size();
		centroid.y /= it->indices.size();
		centroid.z /= it->indices.size();
		centroids.points.push_back(centroid);
		marker.points.push_back(centroid);

	}
	auto end2 = std::chrono::system_clock::now(); //end time
		auto dur2 = end2 - start2; //processing time
		double time2 = std::chrono::duration_cast < std::chrono::microseconds > (dur2).count(); //micro sec
		std::cout << "Color clustered points : " << time2 * 0.001 << " milli sec" << std::endl;

	//---	4. Publish
	//convert back to ros
	pcl_conversions::toPCL(input->header, final_cluster->header);
	// Publish the data
	pub.publish (final_cluster);

	centroids.header = input->header;
	centroid_pub.publish(centroids);


  marker_pub.publish(marker);
  marker.points.clear();
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "euclidean_cluster");

	ros::NodeHandle h;
	ros::NodeHandle private_nh("~");

	cv::generateColors(_colors, 100);

	pub = h.advertise<sensor_msgs::PointCloud2>("/points_cluster",1);
	centroid_pub = h.advertise<lidar_tracker::centroids>("/cluster_centroids",1);
	marker_pub = h.advertise<visualization_msgs::Marker>("centroid_marker",1);


	string points_topic;

	if (private_nh.getParam("points_node", points_topic))
	{
		ROS_INFO("euclidean_cluster > Setting points node to %s", points_topic.c_str());
	}
	else
	{
		ROS_INFO("euclidean_cluster > No points node received, defaulting to velodyne_points, you can use _points_node:=YOUR_TOPIC");
		points_topic = "/points_raw";
	}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = h.subscribe (points_topic, 1, cloud_cb);


	    marker.header.frame_id = "velodyne";
	    marker.header.stamp = ros::Time();
	    marker.ns = "my_namespace";
	    marker.id = 0;
	    marker.type = visualization_msgs::Marker::SPHERE_LIST;
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.scale.x = 1.0;
	    marker.scale.y = 1.0;
	    marker.scale.z = 1.0;
	    marker.color.a = 1.0;
	    marker.color.r = 0.0;
	    marker.color.g = 0.0;
	    marker.color.b = 1.0;
	   // marker.lifetime = ros::Duration(0.1);
	    marker.frame_locked = true;



	// Spin
	ros::spin ();
}
