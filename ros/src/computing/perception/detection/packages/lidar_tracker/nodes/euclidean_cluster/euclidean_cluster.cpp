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
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>

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
ros::Publisher pub_cluster_cloud;
ros::Publisher pub_filtered;
ros::Publisher pub_ground;
ros::Publisher centroid_pub;
ros::Publisher marker_pub;
visualization_msgs::Marker visualization_marker;

ros::Publisher pub_points_lanes_cloud;
ros::Publisher pub_jsk_boundingboxes;

std_msgs::Header _velodyne_header;

pcl::PointCloud<pcl::PointXYZ> _sensor_cloud;

/* parameters for tuning */
static bool is_downsampling;
static double distance;
static double leaf_size;
static int cluster_size_min;
static int cluster_size_max;

static bool publish_ground;	//only ground
static bool publish_filtered;	//pc with no ground

static bool _using_sensor_cloud;

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

	if (is_downsampling)
	  {
		/////////////////////////////////
		//---	ex. Down Sampling
		/////////////////////////////////

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud1);
		sor.setLeafSize((float)leaf_size, (float)leaf_size, (float)leaf_size);
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
	seg.setDistanceThreshold ((float)distance);
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
	ec.setMinClusterSize (cluster_size_min);
	ec.setMaxClusterSize (cluster_size_max);
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
		visualization_marker.points.push_back(centroid);

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
	if(publish_filtered)	//points, no ground
	{
		pcl_conversions::toPCL(input->header, cloud_f->header);
		// Publish the data
		pub_filtered.publish (cloud_f);
	}
	if(publish_ground)		//only ground
	{
		pcl_conversions::toPCL(input->header, cloud_plane->header);
		// Publish the data
		pub_ground.publish (cloud_plane);
	}

	centroids.header = input->header;
	centroid_pub.publish(centroids);


  marker_pub.publish(visualization_marker);
  visualization_marker.points.clear();
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
	if (fabs (in_point_a.x - in_point_b.x) <= (distance *1.0f) &&
			fabs (in_point_a.y - in_point_b.y) <= (distance *1.0f) &&
			fabs (in_point_a.z - in_point_b.z) <= (distance *2.0f))
	{
		return (true);
	}
	else
		return (false);
}
void clusterAndColor(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr, double in_max_cluster_distance=0.5)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	tree->setInputCloud (in_cloud_ptr);

	std::vector<pcl::PointIndices> cluster_indices;

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (distance); //
	ec.setMinClusterSize (cluster_size_min);
	ec.setMaxClusterSize (cluster_size_max);
	ec.setSearchMethod(tree);
	ec.setInputCloud (in_cloud_ptr);
	ec.extract (cluster_indices);


	/*pcl::ConditionalEuclideanClustering<pcl::PointXYZ> cec (true);
	cec.setInputCloud (in_cloud_ptr);
	cec.setConditionFunction (&independentDistance);
	cec.setMinClusterSize (cluster_size_min);
	cec.setMaxClusterSize (cluster_size_max);
	cec.setClusterTolerance (distance*2.0f);
	cec.segment (cluster_indices);*/

	/////////////////////////////////
	//---	3. Color clustered points
	/////////////////////////////////
	int j = 0;
	unsigned int k = 0;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

	jsk_recognition_msgs::BoundingBoxArray boundingbox_array;
	boundingbox_array.header = _velodyne_header;

	lidar_tracker::centroids centroids;

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

		//pose estimation for the cluster
		//test using linear regression
		//Slope(b) = (NΣXY - (ΣX)(ΣY)) / (NΣX2 - (ΣX)2)
		/*float sum_x=0, sum_y=0, sum_xy=0, sum_xx=0;
		for (unsigned int i=0; i<current_cluster->points.size(); i++)
		{
			sum_x+= current_cluster->points[i].x;
			sum_y+= current_cluster->points[i].y;
			sum_xy+= current_cluster->points[i].x*current_cluster->points[i].y;
			sum_xx+= current_cluster->points[i].x*current_cluster->points[i].x;
		}
		double slope= (current_cluster->points.size()*sum_xy - (sum_x*sum_y))/(current_cluster->points.size()*sum_xx - sum_x*sum_x);

		double rz = atan(slope);
		tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);*/
		tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);

		tf::quaternionTFToMsg(quat, bounding_box.pose.orientation);

		if (	//(fabs(bounding_box.pose.position.x) > 2.1 && fabs(bounding_box.pose.position.y) > 0.8 ) && //ignore points that belong to our car
				bounding_box.dimensions.x >0 && bounding_box.dimensions.y >0 && bounding_box.dimensions.z > 0 &&
				bounding_box.dimensions.x < 15 && bounding_box.dimensions.y >0 && bounding_box.dimensions.y < 15 &&
				max_point.z > -1.5 && min_point.z > -1.5 && min_point.z < 1.0 )
		{
			boundingbox_array.boxes.push_back(bounding_box);
			centroids.points.push_back(centroid);
			visualization_marker.points.push_back(centroid);
		}

		current_cluster->width = current_cluster->points.size();
		current_cluster->height = 1;
		current_cluster->is_dense = true;

		//objects_cloud_clusters.push_back(cloud_cluster); //CLUSTERS in vector TODO

		*out_cloud_ptr = *out_cloud_ptr + *current_cluster;//sum up all the colored cluster into a complete pc

		j++; k++;
	}
	std::cout << "Clusters: " << k << std::endl;

	//---	4. Publish
	// Publish BB
	pub_jsk_boundingboxes.publish(boundingbox_array);
	centroids.header = _velodyne_header;
	centroid_pub.publish(centroids);

	marker_pub.publish(visualization_marker);
	visualization_marker.points.clear();
}


void removeFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_max_height=0.2, float in_floor_max_angle=0.35)
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
	extract.filter(*out_cloud_ptr);
}

void downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size=0.2)
{

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(in_cloud_ptr);
	sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
	sor.filter(*out_cloud_ptr);

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
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);

		_velodyne_header = in_sensor_cloud->header;

		//downsampleCloud(current_sensor_cloud_ptr, downsampled_cloud_ptr, leaf_size);

		//keepLanePoints(downsampled_cloud_ptr, inlanes_cloud_ptr);
		//keepLanePoints(current_sensor_cloud_ptr, inlanes_cloud_ptr);

		removeFloor(current_sensor_cloud_ptr, nofloor_cloud_ptr);
		//removeFloor(inlanes_cloud_ptr, nofloor_cloud_ptr);

		publishCloud(&pub_points_lanes_cloud, nofloor_cloud_ptr);

		clusterAndColor(nofloor_cloud_ptr, colored_clustered_cloud_ptr, distance);

		publishColorCloud(&pub_cluster_cloud, colored_clustered_cloud_ptr);

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

	pub_cluster_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_cluster",1);
	centroid_pub = h.advertise<lidar_tracker::centroids>("/cluster_centroids",1);
	marker_pub = h.advertise<visualization_msgs::Marker>("centroid_marker",1);

	pub_points_lanes_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_lanes",1);
	pub_jsk_boundingboxes = h.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes",1);

	string points_topic;

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
	publish_ground = false;
	if (private_nh.getParam("publish_ground", publish_ground))
	{
		ROS_INFO("Publishing /points_ground point cloud...");
		pub_ground = h.advertise<sensor_msgs::PointCloud2>("/points_ground",1);
	}
	publish_filtered = false;
	if (private_nh.getParam("publish_filtered", publish_filtered))
	{
		ROS_INFO("Publishing /points_filtered point cloud...");
		pub_filtered = h.advertise<sensor_msgs::PointCloud2>("/points_filtered",1);
	}

	/* Initialize tuning parameter */
	private_nh.param("is_downsampling", is_downsampling, false);
	private_nh.param("distance", distance, 0.5);
	private_nh.param("leaf_size", leaf_size, 0.1);
	private_nh.param("cluster_size_min", cluster_size_min, 10);
	private_nh.param("cluster_size_max", cluster_size_max, 5000);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = h.subscribe (points_topic, 1, velodyne_callback);


	    visualization_marker.header.frame_id = "velodyne";
	    visualization_marker.header.stamp = ros::Time();
	    visualization_marker.ns = "my_namespace";
	    visualization_marker.id = 0;
	    visualization_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	    visualization_marker.action = visualization_msgs::Marker::ADD;
	    visualization_marker.scale.x = 1.0;
	    visualization_marker.scale.y = 1.0;
	    visualization_marker.scale.z = 1.0;
	    visualization_marker.color.a = 1.0;
	    visualization_marker.color.r = 0.0;
	    visualization_marker.color.g = 0.0;
	    visualization_marker.color.b = 1.0;
	   // marker.lifetime = ros::Duration(0.1);
	    visualization_marker.frame_locked = true;



	// Spin
	ros::spin ();
}
