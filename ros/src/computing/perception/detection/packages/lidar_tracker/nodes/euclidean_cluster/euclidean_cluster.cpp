#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
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

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <lidar_tracker/centroids.h>
#include <lidar_tracker/CloudCluster.h>
#include <lidar_tracker/CloudClusterArray.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <tf/tf.h>

#include <limits>
#include <cmath>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/contrib/contrib.hpp>

#include <chrono>
#include <iostream>
#include <vector>

#include "Cluster.h"

//#include <vector_map/vector_map.h>
//#include <vector_map_server/GetSignal.h>

using namespace cv;

std::vector<cv::Scalar> _colors;
ros::Publisher _pub_cluster_cloud;
ros::Publisher _pub_ground_cloud;
ros::Publisher _centroid_pub;
ros::Publisher _marker_pub;
ros::Publisher _pub_clusters_message;
visualization_msgs::Marker _visualization_marker;

ros::Publisher _pub_points_lanes_cloud;
ros::Publisher _pub_jsk_boundingboxes;

std_msgs::Header _velodyne_header;

pcl::PointCloud<pcl::PointXYZ> _sensor_cloud;

std::vector<double> _clustering_thresholds;
std::vector<double> _clustering_distances;

tf::StampedTransform* _transform;
tf::StampedTransform* _velodyne_output_transform;
tf::TransformListener* _transform_listener;

std::string _output_frame;
static bool _velodyne_transform_available;
static bool _downsample_cloud;
static bool _pose_estimation;
static double _leaf_size;
static int _cluster_size_min;
static int _cluster_size_max;

static bool _remove_ground;	//only ground

static bool _using_sensor_cloud;
static bool _use_diffnormals;

static double _clip_min_height;
static double _clip_max_height;

static bool _keep_lanes;
static double _keep_lane_left_distance;
static double _keep_lane_right_distance;

static double _max_boundingbox_side;

void transformBoundingBox(const jsk_recognition_msgs::BoundingBox& in_boundingbox, jsk_recognition_msgs::BoundingBox& out_boundingbox, const std::string& in_target_frame, const std_msgs::Header& in_header)
{
	geometry_msgs::PoseStamped pose_in, pose_out;
	pose_in.header = in_header;
	pose_in.pose = in_boundingbox.pose;
	try
	{
		_transform_listener->transformPose(in_target_frame, ros::Time(), pose_in, in_header.frame_id,  pose_out);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("transformBoundingBox: %s",ex.what());
	}
	out_boundingbox.pose = pose_out.pose;
	out_boundingbox.header = in_header;
	out_boundingbox.header.frame_id = in_target_frame;
	out_boundingbox.dimensions = in_boundingbox.dimensions;
	out_boundingbox.value = in_boundingbox.value;
	out_boundingbox.label = in_boundingbox.label;
}

void publishCloudClusters(const ros::Publisher* in_publisher, const lidar_tracker::CloudClusterArray& in_clusters, const std::string& in_target_frame, const std_msgs::Header& in_header)
{
	if (in_target_frame!=in_header.frame_id)
	{
		lidar_tracker::CloudClusterArray clusters_transformed;
		clusters_transformed.header = in_header;
		clusters_transformed.header.frame_id = in_target_frame;
		for (auto i=clusters_transformed.clusters.begin(); i!= clusters_transformed.clusters.end(); i++)
		{
			lidar_tracker::CloudCluster cluster_transformed;
			cluster_transformed.header = in_header;
			try
			{
				_transform_listener->lookupTransform(in_target_frame, _velodyne_header.frame_id,
										ros::Time(), *_transform);
				pcl_ros::transformPointCloud(in_target_frame, *_transform, i->cloud, cluster_transformed.cloud);

				_transform_listener->transformPoint(in_target_frame, ros::Time(), i->min_point, in_header.frame_id, cluster_transformed.min_point);
				_transform_listener->transformPoint(in_target_frame, ros::Time(), i->max_point, in_header.frame_id, cluster_transformed.max_point);
				_transform_listener->transformPoint(in_target_frame, ros::Time(), i->avg_point, in_header.frame_id, cluster_transformed.avg_point);
				_transform_listener->transformPoint(in_target_frame, ros::Time(), i->centroid_point, in_header.frame_id, cluster_transformed.centroid_point);

				cluster_transformed.dimensions = i->dimensions;
				cluster_transformed.eigen_values = i->eigen_values;
				cluster_transformed.eigen_vectors = i->eigen_vectors;

				transformBoundingBox(i->bounding_box, cluster_transformed.bounding_box, in_target_frame, in_header);

				clusters_transformed.clusters.push_back(cluster_transformed);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("publishCloudClusters: %s",ex.what());
			}
		}
		in_publisher->publish(clusters_transformed);
	}
	else
	{
		in_publisher->publish(in_clusters);
	}
}

void publishCentroids(const ros::Publisher* in_publisher, const lidar_tracker::centroids& in_centroids, const std::string& in_target_frame, const std_msgs::Header& in_header)
{
	if (in_target_frame!=in_header.frame_id)
	{
		lidar_tracker::centroids centroids_transformed;
		centroids_transformed.header = in_header;
		centroids_transformed.header.frame_id = in_target_frame;
		for (auto i=centroids_transformed.points.begin(); i!= centroids_transformed.points.end(); i++)
		{
			geometry_msgs::PointStamped centroid_in, centroid_out;
			centroid_in.header = in_header;
			centroid_in.point = *i;
			try
			{
				_transform_listener->transformPoint(in_target_frame, ros::Time(), centroid_in, in_header.frame_id, centroid_out);

				centroids_transformed.points.push_back(centroid_out.point);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("publishCentroids: %s",ex.what());
			}
		}
		in_publisher->publish(centroids_transformed);
	}
	else
	{
		in_publisher->publish(in_centroids);
	}
}

void publishBoundingBoxArray(const ros::Publisher* in_publisher, const jsk_recognition_msgs::BoundingBoxArray& in_boundingbox_array, const std::string& in_target_frame, const std_msgs::Header& in_header)
{
	if (in_target_frame!=in_header.frame_id)
	{
		jsk_recognition_msgs::BoundingBoxArray boundingboxes_transformed;
		boundingboxes_transformed.header = in_header;
		boundingboxes_transformed.header.frame_id = in_target_frame;
		for (auto i=in_boundingbox_array.boxes.begin(); i!= in_boundingbox_array.boxes.end(); i++)
		{
			jsk_recognition_msgs::BoundingBox boundingbox_transformed;
			transformBoundingBox(*i, boundingbox_transformed, in_target_frame, in_header);
			boundingboxes_transformed.boxes.push_back(boundingbox_transformed);
		}
		in_publisher->publish(boundingboxes_transformed);
	}
	else
	{
		in_publisher->publish(in_boundingbox_array);
	}
}

void publishCloud(const ros::Publisher* in_publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	cloud_msg.header=_velodyne_header;
	in_publisher->publish(cloud_msg);
}

void publishColorCloud(const ros::Publisher* in_publisher, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	cloud_msg.header=_velodyne_header;
	in_publisher->publish(cloud_msg);
}

void keepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
					pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
					float in_left_lane_threshold = 1.5,
					float in_right_lane_threshold = 1.5)
{
	pcl::PointIndices::Ptr far_indices (new pcl::PointIndices);
	for(unsigned int i=0; i< in_cloud_ptr->points.size(); i++)
	{
		pcl::PointXYZ current_point;
		current_point.x=in_cloud_ptr->points[i].x;
		current_point.y=in_cloud_ptr->points[i].y;
		current_point.z=in_cloud_ptr->points[i].z;

		if (
				current_point.y > (in_left_lane_threshold) || current_point.y < -1.0*in_right_lane_threshold
			)
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

std::vector<ClusterPtr> clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
		jsk_recognition_msgs::BoundingBoxArray& in_out_boundingbox_array,
		lidar_tracker::centroids& in_out_centroids,
		double in_max_cluster_distance=0.5)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	if (in_cloud_ptr->points.size() > 0)
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
	unsigned int k = 0;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

	std::vector<ClusterPtr> clusters;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);//coord + color cluster
	for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		ClusterPtr cluster(new Cluster());
		cluster->SetCloud(in_cloud_ptr, it->indices, _velodyne_header, k, (int)_colors[k].val[0], (int)_colors[k].val[1], (int)_colors[k].val[2], "", _pose_estimation);
		clusters.push_back(cluster);

		k++;
	}
	//std::cout << "Clusters: " << k << std::endl;
	return clusters;

}

void segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
		jsk_recognition_msgs::BoundingBoxArray& in_out_boundingbox_array,
		lidar_tracker::centroids& in_out_centroids,
		lidar_tracker::CloudClusterArray& in_out_clusters)
{
	//cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
	//in this way, the points farther in the pc will also be clustered

	//0 => 0-15m d=0.5
	//1 => 15-30 d=1
	//2 => 30-45 d=1.6
	//3 => 45-60 d=2.1
	//4 => >60   d=2.6

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(5);

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

		float origin_distance = sqrt( pow(current_point.x,2) + pow(current_point.y,2) );

		if 		(origin_distance < _clustering_distances[0] )	{cloud_segments_array[0]->points.push_back (current_point);}
		else if(origin_distance < _clustering_distances[1])		{cloud_segments_array[1]->points.push_back (current_point);}
		else if(origin_distance < _clustering_distances[2])		{cloud_segments_array[2]->points.push_back (current_point);}
		else if(origin_distance < _clustering_distances[3])		{cloud_segments_array[3]->points.push_back (current_point);}
		else													{cloud_segments_array[4]->points.push_back (current_point);}
	}

	std::vector <ClusterPtr> all_clusters;
	for(unsigned int i=0; i<cloud_segments_array.size(); i++)
	{
		std::vector<ClusterPtr> local_clusters = clusterAndColor(cloud_segments_array[i], out_cloud_ptr, in_out_boundingbox_array, in_out_centroids, _clustering_thresholds[i]);

		all_clusters.insert(all_clusters.end(), local_clusters.begin(), local_clusters.end());
	}

	//Clusters can be merged or checked in here
	//....
	//Get final PointCloud to be published

	for(unsigned int i=0; i<all_clusters.size(); i++)
	{
		*out_cloud_ptr = *out_cloud_ptr + *(all_clusters[i]->GetCloud());

		jsk_recognition_msgs::BoundingBox bounding_box = all_clusters[i]->GetBoundingBox();
		pcl::PointXYZ min_point = all_clusters[i]->GetMinPoint();
		pcl::PointXYZ max_point = all_clusters[i]->GetMaxPoint();
		pcl::PointXYZ center_point = all_clusters[i]->GetCentroid();
		geometry_msgs::Point centroid;
		centroid.x = center_point.x; centroid.y = center_point.y; centroid.z = center_point.z;
		bounding_box.header = _velodyne_header;

		if (	//(fabs(bounding_box.pose.position.x) > 2.1 && fabs(bounding_box.pose.position.y) > 0.8 ) && //ignore points that belong to our car
				bounding_box.dimensions.x >0 && bounding_box.dimensions.y >0 && bounding_box.dimensions.z > 0 &&
				bounding_box.dimensions.x < _max_boundingbox_side && bounding_box.dimensions.y < _max_boundingbox_side
				&&max_point.z > -1.5 && min_point.z > -1.5 && min_point.z < 1.0
				)
		{
			in_out_boundingbox_array.boxes.push_back(bounding_box);
			in_out_centroids.points.push_back(centroid);
			_visualization_marker.points.push_back(centroid);

			lidar_tracker::CloudCluster cloud_cluster;
			all_clusters[i]->ToRosMessage(_velodyne_header, cloud_cluster);
			in_out_clusters.clusters.push_back(cloud_cluster);
		}
	}
}

void removeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr, float in_max_height=0.2, float in_floor_max_angle=0.35)
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

void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size=0.2)
{

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(in_cloud_ptr);
	sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
	sor.filter(*out_cloud_ptr);

}

void clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_min_height=-1.3, float in_max_height=0.5)
{
	out_cloud_ptr->points.clear();
	for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
	{
		if (in_cloud_ptr->points[i].z >= in_min_height &&
				in_cloud_ptr->points[i].z <= in_max_height)
		{
			out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
		}
	}
}

void differenceNormalsSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
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
		pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

		lidar_tracker::centroids centroids;
		lidar_tracker::CloudClusterArray cloud_clusters;
		jsk_recognition_msgs::BoundingBoxArray boundingbox_array;

		pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);

		_velodyne_header = in_sensor_cloud->header;

		if (_downsample_cloud)
			downsampleCloud(current_sensor_cloud_ptr, downsampled_cloud_ptr, _leaf_size);
		else
			downsampled_cloud_ptr=current_sensor_cloud_ptr;

		if(_keep_lanes)
			keepLanePoints(downsampled_cloud_ptr, inlanes_cloud_ptr, _keep_lane_left_distance, _keep_lane_right_distance);
		else
			inlanes_cloud_ptr = downsampled_cloud_ptr;

		if(_remove_ground)
		{
			removeFloor(inlanes_cloud_ptr, nofloor_cloud_ptr, onlyfloor_cloud_ptr);
			publishCloud(&_pub_ground_cloud, onlyfloor_cloud_ptr);
		}
		else
			nofloor_cloud_ptr = inlanes_cloud_ptr;

		clipCloud(nofloor_cloud_ptr, clipped_cloud_ptr, _clip_min_height, _clip_max_height);
		publishCloud(&_pub_points_lanes_cloud, clipped_cloud_ptr);

		if (_use_diffnormals)
			differenceNormalsSegmentation(clipped_cloud_ptr, diffnormals_cloud_ptr);
		else
			diffnormals_cloud_ptr = clipped_cloud_ptr;

		segmentByDistance(diffnormals_cloud_ptr, colored_clustered_cloud_ptr, boundingbox_array, centroids, cloud_clusters);
		publishColorCloud(&_pub_cluster_cloud, colored_clustered_cloud_ptr);
		// Publish BB
		boundingbox_array.header = _velodyne_header;
		publishBoundingBoxArray(&_pub_jsk_boundingboxes, boundingbox_array, _output_frame, _velodyne_header);
		centroids.header = _velodyne_header;
		publishCentroids(&_centroid_pub, centroids, _output_frame, _velodyne_header);

		_marker_pub.publish(_visualization_marker);
		_visualization_marker.points.clear();//transform? is it used?
		cloud_clusters.header = _velodyne_header;
		publishCloudClusters(&_pub_clusters_message, cloud_clusters, _output_frame, _velodyne_header);

		_using_sensor_cloud = false;
	}
}

/*
void vectormap_callback(const visualization_msgs::MarkerArray::Ptr in_vectormap_markers)
{
	float min_x=std::numeric_limits<float>::max();float max_x=-std::numeric_limits<float>::max();
	float min_y=std::numeric_limits<float>::max();float max_y=-std::numeric_limits<float>::max();
	pcl::PointXYZ min_point;
	pcl::PointXYZ max_point;
	std::vector<geometry_msgs::Point> vectormap_points;
	for(auto i=in_vectormap_markers->markers.begin(); i!= in_vectormap_markers->markers.end(); i++)
	{
		visualization_msgs::Marker current_marker = *i;
		if (current_marker.ns == "road_edge")
		{
			for (unsigned int j=0; j< current_marker.points.size(); j++)
			{
				geometry_msgs::Point p = current_marker.points[j];
				if(p.x<min_x)	min_x = p.x;
				if(p.y<min_y)	min_y = p.y;
				if(p.x>max_x)	max_x = p.x;
				if(p.y>max_y)	max_y = p.y;
				vectormap_points.push_back(p);
			}
		}
	}
	min_point.x = min_x;	min_point.y = min_y;
	max_point.x = max_x;	max_point.y = max_y;

	min_point.x*=-1.0;
	min_point.y*=-1.0;
	//translate the points to the minimum point
	for (auto i=vectormap_points.begin(); i!=vectormap_points.end(); i++)
	{
		(*i).x+=min_point.x;
		(*i).y+=min_point.y;
	}
	max_point.x+=min_point.x;
	max_point.y+=min_point.y;
	//get world tf
	_transform_listener->lookupTransform("/map", "/world",
					ros::Time(_velodyne_header.stamp), *_transform);

	tf::Vector3 map_origin_point;
	map_origin_point = _transform->getOrigin();

	cv::Mat map_image = cv::Mat::zeros(max_point.x, max_point.y, CV_8UC1);

	for (auto i=vectormap_points.begin(); i!=vectormap_points.end(); i++)
	{
		map_image.at<uchar>( (int)(i->x), (int)(i->y) ) = 255;
	}
	cv::imshow("vectormap", map_image);
	cv::waitKey(0);
}*/

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "euclidean_cluster");

	ros::NodeHandle h;
	ros::NodeHandle private_nh("~");

	tf::StampedTransform transform;
	tf::TransformListener listener;

	_transform = &transform;
	_transform_listener = &listener;

	cv::generateColors(_colors, 100);

	_pub_cluster_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_cluster",1);
	_pub_ground_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_ground",1);
	_centroid_pub = h.advertise<lidar_tracker::centroids>("/cluster_centroids",1);
	_marker_pub = h.advertise<visualization_msgs::Marker>("centroid_marker",1);

	_pub_points_lanes_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_lanes",1);
	_pub_jsk_boundingboxes = h.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes",1);
	_pub_clusters_message = h.advertise<lidar_tracker::CloudClusterArray>("/cloud_clusters",1);

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

	_use_diffnormals = false;
	if (private_nh.getParam("use_diffnormals", _use_diffnormals))
	{
		if (_use_diffnormals)
			ROS_INFO("Euclidean Clustering: Applying difference of normals on clustering pipeline");
		else
			ROS_INFO("Euclidean Clustering: Difference of Normals will not be used.");
	}

	/* Initialize tuning parameter */
	private_nh.param("downsample_cloud", _downsample_cloud, false);	ROS_INFO("downsample_cloud: %d", _downsample_cloud);
	private_nh.param("remove_ground", _remove_ground, true);		ROS_INFO("remove_ground: %d", _remove_ground);
	private_nh.param("leaf_size", _leaf_size, 0.1);					ROS_INFO("leaf_size: %f", _leaf_size);
	private_nh.param("cluster_size_min", _cluster_size_min, 20);	ROS_INFO("cluster_size_min %d", _cluster_size_min);
	private_nh.param("cluster_size_max", _cluster_size_max, 100000);ROS_INFO("cluster_size_max: %d", _cluster_size_max);
	private_nh.param("pose_estimation", _pose_estimation, false);	ROS_INFO("pose_estimation: %d", _pose_estimation);
	private_nh.param("clip_min_height", _clip_min_height, -1.3);	ROS_INFO("clip_min_height: %f", _clip_min_height);
	private_nh.param("clip_max_height", _clip_max_height, 0.5);		ROS_INFO("clip_max_height: %f", _clip_max_height);
	private_nh.param("keep_lanes", _keep_lanes, false);				ROS_INFO("keep_lanes: %d", _keep_lanes);
	private_nh.param("keep_lane_left_distance", _keep_lane_left_distance, 5.0);		ROS_INFO("keep_lane_left_distance: %f", _keep_lane_left_distance);
	private_nh.param("keep_lane_right_distance", _keep_lane_right_distance, 5.0);	ROS_INFO("keep_lane_right_distance: %f", _keep_lane_right_distance);
	private_nh.param("clustering_thresholds", _clustering_thresholds);
	private_nh.param("clustering_distances", _clustering_distances);
	private_nh.param("max_boundingbox_side", _max_boundingbox_side, 10.0);			ROS_INFO("_max_boundingbox_side: %f", _max_boundingbox_side);
	private_nh.param<std::string>("output_frame", _output_frame, "velodyne");			ROS_INFO("output_frame: %s", _output_frame.c_str());

	_velodyne_transform_available = false;

	if (_clustering_distances.size()!=4)
	{
		_clustering_distances = {15, 30, 45, 60};//maximum distance from sensor origin to separate segments
	}
	if (_clustering_thresholds.size()!=5)
	{
		_clustering_thresholds = {0.5, 1.1, 1.6, 2.1, 2.6};//Nearest neighbor distance threshold for each segment
	}

	std::cout << "_clustering_thresholds: "; for (auto i = _clustering_thresholds.begin(); i != _clustering_thresholds.end(); ++i)  std::cout << *i << ' '; std::cout << std::endl;
	std::cout << "_clustering_distances: ";for (auto i = _clustering_distances.begin(); i != _clustering_distances.end(); ++i)  std::cout << *i << ' '; std::cout <<std::endl;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = h.subscribe (points_topic, 1, velodyne_callback);
	//ros::Subscriber sub_vectormap = h.subscribe ("vector_map", 1, vectormap_callback);

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
