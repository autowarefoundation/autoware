#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>

#include <visualization_msgs/Marker.h>
#include <lidar_tracker/centroids.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <limits>
#include <cmath>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/contrib/contrib.hpp>

#include <chrono>
#include <iostream>

//#include <vector_map/vector_map.h>
//#include <vector_map_server/GetSignal.h>

using namespace cv;

std::vector<cv::Scalar> _colors;
ros::Publisher _pub_cluster_cloud;
ros::Publisher _pub_ground_cloud;
ros::Publisher _pub_filtered;
ros::Publisher _pub_ground;
ros::Publisher _centroid_pub;
ros::Publisher _marker_pub;
visualization_msgs::Marker _visualization_marker;

ros::Publisher _pub_points_lanes_cloud;
ros::Publisher _pub_jsk_boundingboxes;

std_msgs::Header _velodyne_header;

pcl::PointCloud<pcl::PointXYZ> _sensor_cloud;

/* parameters for tuning */
static bool _downsample_cloud;
static bool _pose_estimation;
static double _distance;
static double _leaf_size;
static int _cluster_size_min;
static int _cluster_size_max;

static bool _publish_ground;	//only ground
static bool _publish_filtered;	//pc with no ground

static bool _using_sensor_cloud;
static bool _use_diffnormals;

void publishCloud(ros::Publisher* in_publisher, pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	in_publisher->publish(cloud_msg);
}

void publishColorCloud(ros::Publisher* in_publisher, pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	cloud_msg.header=_velodyne_header;
	in_publisher->publish(cloud_msg);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	//ORIGINAL EUCLIDEAN CLUSTERING CODE
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

	if (_downsample_cloud)
	{
		/////////////////////////////////
		//---	ex. Down Sampling
		/////////////////////////////////

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud1);
		sor.setLeafSize((float)_leaf_size, (float)_leaf_size, (float)_leaf_size);
		sor.filter(*cloud_filtered);

		*cloud1 = *cloud_filtered;
	}

	/////////////////////////////////
	//---	1. Remove planes (floor)
	/////////////////////////////////

	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold ((float)_distance);
	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud (cloud1);
	seg.segment (*inliers, *coefficients);
	if (inliers->indices.size () == 0)
	{
		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	}
	else
	{
		pcl::copyPointCloud(*cloud1, *inliers, *cloud_plane);
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

	/////////////////////////////////
	//---	2. Euclidean Clustering
	/////////////////////////////////
	auto start = std::chrono::system_clock::now(); //start time
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	tree->setInputCloud (cloud_f);  // pass ground-removed points

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.8); //
	ec.setMinClusterSize (_cluster_size_min);
	ec.setMaxClusterSize (_cluster_size_max);
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
		_visualization_marker.points.push_back(centroid);

	}
	auto end2 = std::chrono::system_clock::now(); //end time
		auto dur2 = end2 - start2; //processing time
		double time2 = std::chrono::duration_cast < std::chrono::microseconds > (dur2).count(); //micro sec
		std::cout << "Color clustered points : " << time2 * 0.001 << " milli sec" << std::endl;

	//---	4. Publish
	//convert back to ros
	pcl_conversions::toPCL(input->header, final_cluster->header);
	// Publish the data
	//pub.publish (final_cluster);

///////////////////////////////////////////////
	//4.5 Publish Filtered PointClouds if requested 
	//////////////////////////////////////////////
	if(_publish_filtered)	//points, no ground
	{
		pcl_conversions::toPCL(input->header, cloud_f->header);
		// Publish the data
		_pub_filtered.publish (cloud_f);
	}
	if(_publish_ground)		//only ground
	{
		pcl_conversions::toPCL(input->header, cloud_plane->header);
		// Publish the data
		_pub_ground.publish (cloud_plane);
	}

	centroids.header = input->header;
	_centroid_pub.publish(centroids);


	_marker_pub.publish(_visualization_marker);
	_visualization_marker.points.clear();
}

void keepLanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
					pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
					float in_x_threshold = 40, //keep upto 50 meters ahead and behind
					float in_y_threshold = 5, //lane's width is about 3.5 (adjacent) + 1.5 (current center) Japan's lane 3.25~3.5m width
					float in_z_threshold = 2) // keep upto 2 meters above
{
	pcl::PointIndices::Ptr far_indices (new pcl::PointIndices);
	for(unsigned int i=0; i< in_cloud_ptr->points.size(); i++)
	{
		pcl::PointXYZ current_point;
		current_point.x=in_cloud_ptr->points[i].x;
		current_point.y=in_cloud_ptr->points[i].y;
		current_point.z=in_cloud_ptr->points[i].z;

		if (	current_point.x > in_x_threshold || current_point.x < -1.0*in_x_threshold ||
				current_point.y > in_y_threshold || current_point.y < -1.0*in_y_threshold ||
				current_point.z < -1.8 || current_point.z > 2.0)//remove points which do not belong to an adjacent lane
		{
			far_indices->indices.push_back(i);
		}
	}

	out_cloud_ptr->points.clear();

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (in_cloud_ptr);
	extract.setIndices(far_indices);
	extract.setNegative(true);//true removes the indices, false leaves only the indices
	extract.filter(*out_cloud_ptr);
}

bool independentDistance (const pcl::PointXYZ& in_point_a, const pcl::PointXYZ& in_point_b, float squared_distance)
{
	if (fabs (in_point_a.x - in_point_b.x) <= (_distance *1.0f) &&
			fabs (in_point_a.y - in_point_b.y) <= (_distance *1.0f) &&
			fabs (in_point_a.z - in_point_b.z) <= (_distance *2.0f))
	{
		return (true);
	}
	else
		return (false);
}

void clusterAndColor(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
		jsk_recognition_msgs::BoundingBoxArray& in_boundingbox_array,
		lidar_tracker::centroids& in_centroids,
		double in_max_cluster_distance=0.5)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	tree->setInputCloud (in_cloud_ptr);

	std::vector<pcl::PointIndices> cluster_indices;

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (in_max_cluster_distance); //
	ec.setMinClusterSize (_cluster_size_min);
	ec.setMaxClusterSize (_cluster_size_max);
	ec.setSearchMethod(tree);
	ec.setInputCloud (in_cloud_ptr);
	ec.extract (cluster_indices);


	/*pcl::ConditionalEuclideanClustering<pcl::PointXYZ> cec (true);
	cec.setInputCloud (in_cloud_ptr);
	cec.setConditionFunction (&independentDistance);
	cec.setMinClusterSize (cluster_size_min);
	cec.setMaxClusterSize (cluster_size_max);
	cec.setClusterTolerance (_distance*2.0f);
	cec.segment (cluster_indices);*/

	/////////////////////////////////
	//---	3. Color clustered points
	/////////////////////////////////
	int j = 0;
	unsigned int k = 0;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects_cloud_clusters;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);//coord + color cluster
	for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);//coord + color cluster
		//assign color to each cluster
		geometry_msgs::Point centroid;
		for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			//fill new colored cluster point by point
			pcl::PointXYZRGB p;
			p.x = in_cloud_ptr->points[*pit].x;
			p.y = in_cloud_ptr->points[*pit].y;
			p.z = in_cloud_ptr->points[*pit].z;
			p.r = _colors[k].val[0];
			p.g = _colors[k].val[1];
			p.b = _colors[k].val[2];

			centroid.x += in_cloud_ptr->points[*pit].x;
			centroid.y += in_cloud_ptr->points[*pit].y;
			centroid.z += in_cloud_ptr->points[*pit].z;

			if (p.z > -1.3 && p.z < 0.5)
			{
				current_cluster->points.push_back(p);
			}
		}

		centroid.x /= it->indices.size();
		centroid.y /= it->indices.size();
		centroid.z /= it->indices.size();


		//get min, max
		float min_x=std::numeric_limits<float>::max();float max_x=-std::numeric_limits<float>::max();
		float min_y=std::numeric_limits<float>::max();float max_y=-std::numeric_limits<float>::max();
		float min_z=std::numeric_limits<float>::max();float max_z=-std::numeric_limits<float>::max();

		for(unsigned int i=0; i<current_cluster->points.size();i++)
		{
			if(current_cluster->points[i].x<min_x)	min_x = current_cluster->points[i].x;
			if(current_cluster->points[i].y<min_y)	min_y = current_cluster->points[i].y;
			if(current_cluster->points[i].z<min_z)	min_z = current_cluster->points[i].z;
			if(current_cluster->points[i].x>max_x)	max_x = current_cluster->points[i].x;
			if(current_cluster->points[i].y>max_y)	max_y = current_cluster->points[i].y;
			if(current_cluster->points[i].z>max_z)	max_z = current_cluster->points[i].z;
		}

		pcl::PointXYZ min_point(min_x, min_y, min_z), max_point(max_x, max_y, max_z);

		float l = max_point.x - min_point.x;
		float w = max_point.y - min_point.y;
		float h = max_point.z - min_point.z;

		jsk_recognition_msgs::BoundingBox bounding_box;
		bounding_box.header = _velodyne_header;

		bounding_box.pose.position.x = min_point.x + l/2;
		bounding_box.pose.position.y = min_point.y + w/2;
		bounding_box.pose.position.z = min_point.z + h/2;

		bounding_box.dimensions.x = ((l<0)?-1*l:l);
		bounding_box.dimensions.y = ((w<0)?-1*w:w);
		bounding_box.dimensions.z = ((h<0)?-1*h:h);

		double rz = 0;

		if (_pose_estimation)
		{
			std::vector<cv::Point2f> inner_points;
			for (unsigned int i=0; i<current_cluster->points.size(); i++)
			{
				inner_points.push_back(cv::Point2f((current_cluster->points[i].x + fabs(min_point.x))*8, (current_cluster->points[i].y + fabs(min_point.y) ))*8);
			}

			cv::Mat points_mat = cv::Mat(inner_points);

			if (inner_points.size() > 0)
			{
				cv::RotatedRect rot_box = cv::minAreaRect(points_mat);
				rz = atan(rot_box.angle);
			}
		}


		tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);
		//tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);

		tf::quaternionTFToMsg(quat, bounding_box.pose.orientation);

		if (	//(fabs(bounding_box.pose.position.x) > 2.1 && fabs(bounding_box.pose.position.y) > 0.8 ) && //ignore points that belong to our car
				bounding_box.dimensions.x >0 && bounding_box.dimensions.y >0 && bounding_box.dimensions.z > 0 &&
				bounding_box.dimensions.x < 15 && bounding_box.dimensions.y >0 && bounding_box.dimensions.y < 15 &&
				max_point.z > -1.5 && min_point.z > -1.5 && min_point.z < 1.0 )
		{
			in_boundingbox_array.boxes.push_back(bounding_box);
			in_centroids.points.push_back(centroid);
			_visualization_marker.points.push_back(centroid);
		}

		current_cluster->width = current_cluster->points.size();
		current_cluster->height = 1;
		current_cluster->is_dense = true;

		//objects_cloud_clusters.push_back(cloud_cluster); //CLUSTERS in vector TODO

		*out_cloud_ptr = *out_cloud_ptr + *current_cluster;//sum up all the colored cluster into a complete pc

		j++; k++;
	}
	//std::cout << "Clusters: " << k << std::endl;


}

void segmentByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
		jsk_recognition_msgs::BoundingBoxArray& in_boundingbox_array,
		lidar_tracker::centroids& in_centroids,
		double in_max_cluster_distance=0.5)
{
	//cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
	//in this way, the points farther in the pc will also be clustered

	//0 => 0-15m d=0.5
	//1 => 15-30 d=1
	//2 => 30-45 d=1.6
	//3 => 45-60 d=2.1
	//4 => >60   d=2.6

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(5);
	std::vector<double> thresholds = {0.5, 1.1, 1.6, 2.3, 2.0f};

	for(unsigned int i=0; i<cloud_segments_array.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_segments_array[i] = tmp_cloud;
	}

	for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
	{
		pcl::PointXYZ current_point;
		current_point.x = in_cloud_ptr->points[i].x;
		current_point.y = in_cloud_ptr->points[i].y;
		current_point.z = in_cloud_ptr->points[i].z;

		float origin_distance = sqrt(current_point.x*current_point.x + current_point.y+current_point.y);

		if 		(origin_distance < 15 )	{cloud_segments_array[0]->points.push_back (current_point);}
		else if(origin_distance < 30)	{cloud_segments_array[1]->points.push_back (current_point);}
		else if(origin_distance < 45)	{cloud_segments_array[2]->points.push_back (current_point);}
		else if(origin_distance < 60)	{cloud_segments_array[3]->points.push_back (current_point);}
		else							{cloud_segments_array[4]->points.push_back (current_point);}
	}

	for(unsigned int i=0; i<cloud_segments_array.size(); i++)
	{
		clusterAndColor(cloud_segments_array[i], out_cloud_ptr, in_boundingbox_array, in_centroids, thresholds[i]);
	}

}

void removeFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr, float in_max_height=0.2, float in_floor_max_angle=0.35)
{
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

	seg.setOptimizeCoefficients (true);
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setAxis(Eigen::Vector3f(0,0,1));
	seg.setEpsAngle(in_floor_max_angle);

	seg.setDistanceThreshold (in_max_height);//floor distance
	seg.setOptimizeCoefficients(true);
	seg.setInputCloud(in_cloud_ptr);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size () == 0)
	{
		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	}

	/*REMOVE THE FLOOR FROM THE CLOUD*/
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (in_cloud_ptr);
	extract.setIndices(inliers);
	extract.setNegative(true);//true removes the indices, false leaves only the indices
	extract.filter(*out_nofloor_cloud_ptr);

	/*EXTRACT THE FLOOR FROM THE CLOUD*/
	extract.setNegative(false);//true removes the indices, false leaves only the indices
	extract.filter(*out_onlyfloor_cloud_ptr);
}

void downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size=0.2)
{

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(in_cloud_ptr);
	sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
	sor.filter(*out_cloud_ptr);

}

void differenceNormalsSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{

	float small_scale=0.5;
	float large_scale=2.0;
	float angle_threshold=0.5;
	pcl::search::Search<pcl::PointXYZ>::Ptr tree;
	if (in_cloud_ptr->isOrganized ())
	{
		tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
	}
	else
	{
		tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
	}

	// Set the input pointcloud for the search tree
	tree->setInputCloud (in_cloud_ptr);

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
	//pcl::gpu::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
	normal_estimation.setInputCloud (in_cloud_ptr);
	normal_estimation.setSearchMethod (tree);

	normal_estimation.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

	pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

	normal_estimation.setRadiusSearch (small_scale);
	normal_estimation.compute (*normals_small_scale);

	normal_estimation.setRadiusSearch (large_scale);
	normal_estimation.compute (*normals_large_scale);

	pcl::PointCloud<pcl::PointNormal>::Ptr diffnormals_cloud (new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*in_cloud_ptr, *diffnormals_cloud);

	// Create DoN operator
	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> diffnormals_estimator;
	diffnormals_estimator.setInputCloud (in_cloud_ptr);
	diffnormals_estimator.setNormalScaleLarge (normals_large_scale);
	diffnormals_estimator.setNormalScaleSmall (normals_small_scale);

	diffnormals_estimator.initCompute();

	diffnormals_estimator.computeFeature(*diffnormals_cloud);

	pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (new pcl::ConditionOr<pcl::PointNormal>() );
	range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
			new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, angle_threshold) )
			);
	// Build the filter
	pcl::ConditionalRemoval<pcl::PointNormal> cond_removal;
	cond_removal.setCondition(range_cond);
	cond_removal.setInputCloud (diffnormals_cloud);

	pcl::PointCloud<pcl::PointNormal>::Ptr diffnormals_cloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

	// Apply filter
	cond_removal.filter (*diffnormals_cloud_filtered);

	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*diffnormals_cloud, *out_cloud_ptr);
}

void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{
	if (!_using_sensor_cloud)
	{
		_using_sensor_cloud = true;

		pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr inlanes_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr nofloor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr onlyfloor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr diffnormals_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

		lidar_tracker::centroids centroids;
		jsk_recognition_msgs::BoundingBoxArray boundingbox_array;

		pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);

		_velodyne_header = in_sensor_cloud->header;

		if (_downsample_cloud)
			downsampleCloud(current_sensor_cloud_ptr, downsampled_cloud_ptr, _leaf_size);
		else
			downsampled_cloud_ptr=current_sensor_cloud_ptr;

		//keepLanePoints(downsampled_cloud_ptr, inlanes_cloud_ptr);
		//keepLanePoints(current_sensor_cloud_ptr, inlanes_cloud_ptr);

		removeFloor(current_sensor_cloud_ptr, nofloor_cloud_ptr, onlyfloor_cloud_ptr);
		//removeFloor(inlanes_cloud_ptr, nofloor_cloud_ptr);

		publishCloud(&_pub_points_lanes_cloud, nofloor_cloud_ptr);

		publishCloud(&_pub_ground_cloud, onlyfloor_cloud_ptr);

		//clusterAndColor(nofloor_cloud_ptr, colored_clustered_cloud_ptr, boundingbox_array, centroids, _distance);

		if (_use_diffnormals)
			differenceNormalsSegmentation(nofloor_cloud_ptr, diffnormals_cloud_ptr);
		else
			diffnormals_cloud_ptr = nofloor_cloud_ptr;

		//clusterAndColor(diffnormals_cloud_ptr, colored_clustered_cloud_ptr, boundingbox_array, centroids, _distance);

		segmentByDistance(diffnormals_cloud_ptr, colored_clustered_cloud_ptr, boundingbox_array, centroids, _distance);

		publishColorCloud(&_pub_cluster_cloud, colored_clustered_cloud_ptr);

		// Publish BB
		boundingbox_array.header = _velodyne_header;
		_pub_jsk_boundingboxes.publish(boundingbox_array);

		centroids.header = _velodyne_header;
		_centroid_pub.publish(centroids);

		_marker_pub.publish(_visualization_marker);
		_visualization_marker.points.clear();

		_using_sensor_cloud = false;
	}
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "euclidean_cluster");

	ros::NodeHandle h;
	ros::NodeHandle private_nh("~");

	cv::generateColors(_colors, 100);

	_pub_cluster_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_cluster",1);
	_pub_ground_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_ground",1);
	_centroid_pub = h.advertise<lidar_tracker::centroids>("/cluster_centroids",1);
	_marker_pub = h.advertise<visualization_msgs::Marker>("centroid_marker",1);

	_pub_points_lanes_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_lanes",1);
	_pub_jsk_boundingboxes = h.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes",1);

	std::string points_topic;

	_using_sensor_cloud = false;

	if (private_nh.getParam("points_node", points_topic))
	{
		ROS_INFO("euclidean_cluster > Setting points node to %s", points_topic.c_str());
	}
	else
	{
		ROS_INFO("euclidean_cluster > No points node received, defaulting to points_raw, you can use _points_node:=YOUR_TOPIC");
		points_topic = "/points_raw";
	}
	_publish_ground = false;
	if (private_nh.getParam("publish_ground", _publish_ground))
	{
		ROS_INFO("Publishing /points_ground point cloud...");
		_pub_ground = h.advertise<sensor_msgs::PointCloud2>("/points_ground",1);
	}
	_publish_filtered = false;
	if (private_nh.getParam("publish_filtered", _publish_filtered))
	{
		ROS_INFO("Publishing /points_filtered point cloud...");
		_pub_filtered = h.advertise<sensor_msgs::PointCloud2>("/points_filtered",1);
	}

	_use_diffnormals = false;
	if (private_nh.getParam("use_diffnormals", _use_diffnormals))
	{
		if (_use_diffnormals)
			ROS_INFO("Euclidean Clustering: Applying difference of normals on clustering pipeline");
		else
			ROS_INFO("Euclidean Clustering: Difference of Normals will not be used.");
	}

	/* Initialize tuning parameter */
	private_nh.param("downsample_cloud", _downsample_cloud, false);
	private_nh.param("distance", _distance, 0.5);
	private_nh.param("leaf_size", _leaf_size, 0.1);
	private_nh.param("cluster_size_min", _cluster_size_min, 20);
	private_nh.param("cluster_size_max", _cluster_size_max, 100000);
	private_nh.param("pose_estimation", _pose_estimation, false);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = h.subscribe (points_topic, 1, velodyne_callback);


	_visualization_marker.header.frame_id = "velodyne";
	_visualization_marker.header.stamp = ros::Time();
	_visualization_marker.ns = "my_namespace";
	_visualization_marker.id = 0;
	_visualization_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	_visualization_marker.action = visualization_msgs::Marker::ADD;
	_visualization_marker.scale.x = 1.0;
	_visualization_marker.scale.y = 1.0;
	_visualization_marker.scale.z = 1.0;
	_visualization_marker.color.a = 1.0;
	_visualization_marker.color.r = 0.0;
	_visualization_marker.color.g = 0.0;
	_visualization_marker.color.b = 1.0;
	// marker.lifetime = ros::Duration(0.1);
	_visualization_marker.frame_locked = true;


	// Spin
	ros::spin ();
}
