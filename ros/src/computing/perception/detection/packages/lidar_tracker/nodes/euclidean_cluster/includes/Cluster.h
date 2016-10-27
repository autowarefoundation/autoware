/*
 * Cluster.h
 *
 *  Created on: Oct 19, 2016
 *      Author: Ne0
 */
#ifndef CLUSTER_H_
#define CLUSTER_H_

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
#include <pcl/common/pca.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>

#include <jsk_recognition_msgs/BoundingBox.h>

#include <lidar_tracker/CloudCluster.h>

#include <limits>
#include <cmath>

class Cluster {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr	pointcloud_;
	pcl::PointXYZ 						min_point_;
	pcl::PointXYZ 						max_point_;
	pcl::PointXYZ 						average_point_;
	pcl::PointXYZ 						centroid_;
	double 								orientation_angle_;
	float 								length_, width_, height_;

	jsk_recognition_msgs::BoundingBox 	bounding_box_;

	std::string							label_;
	int									id_;
	int									r_, g_, b_;

	Eigen::Matrix3f 					eigen_vectors_;
	Eigen::Vector3f 					eigen_values_;
public:
	/* \brief Constructor. Creates a Cluster object using the specified points in a PointCloud
	 * \param[in] in_origin_cloud_ptr 	Origin PointCloud
	 * \param[in] in_cluster_indices 	Indices of the Origin Pointcloud to create the Cluster
	 * \param[in] in_id 				ID of the cluster
	 * \param[in] in_r 					Amount of Red [0-255]
	 * \param[in] in_g 					Amount of Green [0-255]
	 * \param[in] in_b 					Amount of Blue [0-255]
	 * \param[in] in_label 				Label to identify this cluster (optional)
	 * \param[in] in_estimate_pose		Flag to enable Pose Estimation of the Bounding Box
	 * */
	void SetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_origin_cloud_ptr, const std::vector<int>& in_cluster_indices, std_msgs::Header in_ros_header, int in_id, int in_r, int in_g, int in_b, std::string in_label, bool in_estimate_pose);

	/* \brief Returns the lidar_tracker::CloudCluster message associated to this Cluster */
	void ToRosMessage(std_msgs::Header in_ros_header, lidar_tracker::CloudCluster& out_cluster_message);

	Cluster();
	virtual ~Cluster();

	/* \brief Returns the pointer to the PointCloud containing the points in this Cluster */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr	GetCloud();
	/* \brief Returns the minimum point in the cluster */
	pcl::PointXYZ 						GetMinPoint();
	/* \brief Returns the maximum point in the cluster*/
	pcl::PointXYZ 						GetMaxPoint();
	/* \brief Returns the average point in the cluster*/
	pcl::PointXYZ 						GetAveragePoint();
	/* \brief Returns the centroid point in the cluster */
	pcl::PointXYZ 						GetCentroid();
	/* \brief Returns the calculated BoundingBox of the object */
	jsk_recognition_msgs::BoundingBox	GetBoundingBox();
	/* \brief Returns the angle in radians of the BoundingBox. 0 if pose estimation was not enabled. */
	double								GetOrientationAngle();
	/* \brief Returns the Length of the Cluster */
	float								GetLenght();
	/* \brief Returns the Width of the Cluster */
	float								GetWidth();
	/* \brief Returns the Height of the Cluster */
	float								GetHeight();
	/* \brief Returns the Id of the Cluster */
	int									GetId();
	/* \brief Returns the Label of the Cluster */
	std::string							GetLabel();
	/* \brief Returns the Eigen Vectors of the cluster */
	Eigen::Matrix3f						GetEigenVectors();
	/* \brief Returns the Eigen Values of the Cluster */
	Eigen::Vector3f						GetEigenValues();

	/* \brief Returns a pointer to a PointCloud object containing the merged points between current Cluster and the specified PointCloud
	 * \param[in] in_cloud_ptr 	Origin PointCloud
	 * */
	pcl::PointCloud<pcl::PointXYZ>::Ptr	JoinCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr);

};

typedef boost::shared_ptr<Cluster> ClusterPtr;

#endif /* CLUSTER_H_ */
