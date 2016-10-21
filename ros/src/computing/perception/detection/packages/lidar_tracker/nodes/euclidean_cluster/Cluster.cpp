/*
 * Cluster.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: Ne0
 */

#include "Cluster.h"

Cluster::Cluster()
{}


jsk_recognition_msgs::BoundingBox Cluster::GetBoundingBox()
{
	return bounding_box_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cluster::GetCloud()
{
	return pointcloud_;
}

pcl::PointXYZ Cluster::GetMinPoint()
{
	return min_point_;
}

pcl::PointXYZ Cluster::GetMaxPoint()
{
	return max_point_;
}

pcl::PointXYZ Cluster::GetCentroid()
{
	return centroid_;
}

void Cluster::SetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_origin_cloud_ptr, const std::vector<int>& in_cluster_indices, int in_id, int in_r, int in_g, int in_b, std::string in_label, bool in_estimate_pose)
{
	label_ 	= in_label;	id_		= in_id;
	r_		= in_r;	g_		= in_g;	b_		= in_b;
	//extract pointcloud using the indices
	//calculate min and max points
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	float min_x=std::numeric_limits<float>::max();float max_x=-std::numeric_limits<float>::max();
	float min_y=std::numeric_limits<float>::max();float max_y=-std::numeric_limits<float>::max();
	float min_z=std::numeric_limits<float>::max();float max_z=-std::numeric_limits<float>::max();
	float average_x = 0, average_y = 0, average_z = 0;

	for (auto pit = in_cluster_indices.begin(); pit != in_cluster_indices.end(); ++pit)
	{
		//fill new colored cluster point by point
		pcl::PointXYZRGB p;
		p.x = in_origin_cloud_ptr->points[*pit].x;
		p.y = in_origin_cloud_ptr->points[*pit].y;
		p.z = in_origin_cloud_ptr->points[*pit].z;
		p.r = in_r;
		p.g = in_g;
		p.b = in_b;

		average_x+=p.x;		average_y+=p.y;		average_z+=p.z;

		centroid_.x += p.x; centroid_.y += p.y;	centroid_.z += p.z;

		current_cluster->points.push_back(p);

		if(p.x<min_x)	min_x = p.x;
		if(p.y<min_y)	min_y = p.y;
		if(p.z<min_z)	min_z = p.z;
		if(p.x>max_x)	max_x = p.x;
		if(p.y>max_y)	max_y = p.y;
		if(p.z>max_z)	max_z = p.z;
	}
	//min, max points
	min_point_.x = min_x;	min_point_.y = min_y;	min_point_.z = min_z;
	max_point_.x = max_x;	max_point_.y = max_y;	max_point_.z = max_z;

	//calculate centroid, average
	if (in_cluster_indices.size() > 0)
	{
		centroid_.x /= in_cluster_indices.size();
		centroid_.y /= in_cluster_indices.size();
		centroid_.z /= in_cluster_indices.size();

		average_x /= in_cluster_indices.size();
		average_y /= in_cluster_indices.size();
		average_z /= in_cluster_indices.size();
	}

	average_point_.x = average_x; average_point_.y = average_y;	average_point_.z = average_z;

	//calculate bounding box
	length_ = max_point_.x - min_point_.x;
	width_ = max_point_.y - min_point_.y;
	height_ = max_point_.z - min_point_.z;

	//bounding_box_.header = _velodyne_header;

	bounding_box_.pose.position.x = min_point_.x + length_/2;
	bounding_box_.pose.position.y = min_point_.y + width_/2;
	bounding_box_.pose.position.z = min_point_.z + height_/2;

	bounding_box_.dimensions.x = ((length_<0)?-1*length_:length_);
	bounding_box_.dimensions.y = ((width_<0)?-1*width_:width_);
	bounding_box_.dimensions.z = ((height_<0)?-1*height_:height_);

	//pose estimation
	double rz = 0;

	if (in_estimate_pose)
	{
		//pose estimation for the cluster
		//test using linear regression
		//Slope(b) = (NΣXY - (ΣX)(ΣY)) / (NΣX2 - (ΣX)2)
		float sum_x=0, sum_y=0, sum_xy=0, sum_xx=0;
		for (unsigned int i=0; i<current_cluster->points.size(); i++)
		{
			sum_x+= current_cluster->points[i].x;
			sum_y+= current_cluster->points[i].y;
			sum_xy+= current_cluster->points[i].x*current_cluster->points[i].y;
			sum_xx+= current_cluster->points[i].x*current_cluster->points[i].x;
		}
		double slope= (current_cluster->points.size()*sum_xy - (sum_x*sum_y))/(current_cluster->points.size()*sum_xx - sum_x*sum_x);

		rz = atan(slope);
	}

	//set bounding box direction
	tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);
	tf::quaternionTFToMsg(quat, bounding_box_.pose.orientation);

	/*if (	//(fabs(bounding_box.pose.position.x) > 2.1 && fabs(bounding_box.pose.position.y) > 0.8 ) && //ignore points that belong to our car
			bounding_box.dimensions.x >0 && bounding_box.dimensions.y >0 && bounding_box.dimensions.z > 0 &&
			bounding_box.dimensions.x < 15 && bounding_box.dimensions.y >0 && bounding_box.dimensions.y < 15 &&
			max_point.z > -1.5 && min_point.z > -1.5 && min_point.z < 1.0 )
	{
		in_out_boundingbox_array.boxes.push_back(bounding_box);
		in_out_centroids.points.push_back(centroid);
		_visualization_marker.points.push_back(centroid);
	}*/

	current_cluster->width = current_cluster->points.size();
	current_cluster->height = 1;
	current_cluster->is_dense = true;

	pointcloud_ = current_cluster;
}

Cluster::~Cluster() {
	// TODO Auto-generated destructor stub
}

