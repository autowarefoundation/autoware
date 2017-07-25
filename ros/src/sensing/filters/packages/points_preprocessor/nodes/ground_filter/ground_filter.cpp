/*
 * ground_filter.cpp
 *
 * Created on	: May 19, 2017
 * Author	: Patiphon Narksri					
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
#include <opencv/cv.h>

enum Label
{
	GROUND = 0,
	VERTICAL = 1,
	UNKNOWN = 3
};

class GroundFilter
{
public:
	
	GroundFilter();

private:

	ros::NodeHandle node_handle_;
	ros::Subscriber points_node_sub_;
	ros::Publisher groundless_points_pub_;
	ros::Publisher ground_points_pub_;

	std::string point_topic_;
	int 		sensor_model_;
	double 		sensor_height_;
	double 		max_slope_;
	int 		min_point_;
	double 		clipping_thres_;
	double 		gap_thres_;
	double		point_distance_;
	bool		floor_removal_;

	int 		vertical_res_;
	int 		horizontal_res_;
	double 		limiting_ratio_;
	cv::Mat 	index_map_;
	Label 		class_label_[64];

	boost::chrono::high_resolution_clock::time_point t1_;
	boost::chrono::high_resolution_clock::time_point t2_;
	boost::chrono::nanoseconds elap_time_;

	const int 	DEFAULT_HOR_RES = 2000;

	void InitLabelArray(int in_model);
	void InitDepthMap(int in_width);
	void PublishPointCloud(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
				int in_indices[], int &in_out_index_size, 
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &in_cloud);


	void VelodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg);
	void FilterGround(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_groundless_points,
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_ground_points);

};

GroundFilter::GroundFilter() : node_handle_("~")
{
	ROS_INFO("Inititalizing Ground Filter...");
	node_handle_.param<std::string>("point_topic", point_topic_, "/points_raw");	ROS_INFO("Input Point Cloud: %s", point_topic_.c_str());
 	node_handle_.param("remove_floor",  floor_removal_,  true);						ROS_INFO("Floor Removal: %d", floor_removal_);
	node_handle_.param("sensor_model", sensor_model_, 64);							ROS_INFO("Sensor Model: %d", sensor_model_);
	node_handle_.param("sensor_height", sensor_height_, 1.72);						ROS_INFO("Sensor Height: %f", sensor_height_);
	node_handle_.param("max_slope", max_slope_, 20.0);								ROS_INFO("Max Slope: %f", max_slope_);
	node_handle_.param("point_distance", point_distance_, 0.05);					ROS_INFO("Point Distance: %f", point_distance_);

	node_handle_.param("min_point", min_point_, 3);									ROS_INFO("Min Points: %d", min_point_);
	node_handle_.param("clipping_thres", clipping_thres_, 0.5);						ROS_INFO("Lower Clipping Threshold: %f", clipping_thres_);
	node_handle_.param("gap_thres", gap_thres_, 0.5);								ROS_INFO("Point Gap Threshold: %f", gap_thres_);

	std::string no_ground_topic, ground_topic;
	node_handle_.param<std::string>("no_ground_point_topic", no_ground_topic, "/points_no_ground");	ROS_INFO("No Ground Output Point Cloud: %s", no_ground_topic.c_str());
	node_handle_.param<std::string>("ground_point_topic", ground_topic, "/points_ground");	ROS_INFO("Only Ground Output Point Cloud: %s", ground_topic.c_str());

	int default_horizontal_res = DEFAULT_HOR_RES;
	switch(sensor_model_)
	{
		case 64:
			default_horizontal_res = 2083;
			break;
		case 32:
			default_horizontal_res = 2250;
			break;
		case 16:
			default_horizontal_res = 1800;
			break;
	}
	node_handle_.param("horizontal_res", horizontal_res_, default_horizontal_res);

	points_node_sub_ = node_handle_.subscribe(point_topic_, 10, &GroundFilter::VelodyneCallback, this);
	groundless_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 10);
	ground_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ground_topic, 10);

	vertical_res_ = sensor_model_;
	InitLabelArray(sensor_model_);
	limiting_ratio_ = tan(max_slope_*M_PI/180);

}

void GroundFilter::InitLabelArray(int in_model)
{
	for(int a = 0; a < vertical_res_; a++)
	{
		class_label_[a] = UNKNOWN;
	}
}

void GroundFilter::InitDepthMap(int in_width)
{
	const int mOne = -1;
	index_map_ = cv::Mat_<int>(vertical_res_, in_width, mOne);
}

void GroundFilter::PublishPointCloud(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
				int in_indices[], int &in_out_index_size, 
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &in_cloud)
{
	velodyne_pointcloud::PointXYZIR point;
	for (int i = 0; i < in_out_index_size; i++)
	{
		point.x = in_cloud_msg->points[in_indices[i]].x;
		point.y = in_cloud_msg->points[in_indices[i]].y;
		point.z = in_cloud_msg->points[in_indices[i]].z;
		point.intensity = in_cloud_msg->points[in_indices[i]].intensity;
		point.ring = in_cloud_msg->points[in_indices[i]].ring;
		in_cloud.push_back(point);
	}
	in_out_index_size = 0;	
}

void GroundFilter::FilterGround(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
			pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_groundless_points,
			pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_ground_points)
{

	velodyne_pointcloud::PointXYZIR point;

	horizontal_res_ = int(in_cloud_msg->points.size() / vertical_res_);
	InitDepthMap(horizontal_res_);

	for (size_t i = 0; i < in_cloud_msg->points.size(); i++)
	{
		double u = atan2(in_cloud_msg->points[i].y,in_cloud_msg->points[i].x) * 180/M_PI;
		if (u < 0) u = 360 + u;
		int column = horizontal_res_ - (int)((double)horizontal_res_ * u / 360.0) - 1;
		int row = vertical_res_ - 1 - in_cloud_msg->points[i].ring;
		index_map_.at<int>(row, column) = i;
	}
	
	for (int i = 0; i < horizontal_res_; i++)
	{
		Label point_class[vertical_res_];
		int unknown_index[vertical_res_];
		int point_index[vertical_res_];
		int unknown_index_size = 0;
		int point_index_size = 0;
		double z_ref = 0;
		double r_ref = 0;
		std::copy(class_label_, class_label_ + vertical_res_, point_class); 

		for (int j = vertical_res_ - 1; j >= 0; j--)
		{
			if (index_map_.at<int>(j,i) > -1 && point_class[j] == UNKNOWN)
			{
				double x0 = in_cloud_msg->points[index_map_.at<int>(j, i)].x;
				double y0 = in_cloud_msg->points[index_map_.at<int>(j, i)].y;
				double z0 = in_cloud_msg->points[index_map_.at<int>(j, i)].z;
				double r0 = sqrt(x0*x0 + y0*y0);
				double r_diff = r0 - r_ref;
				double z_diff = fabs(z0 - z_ref);
				double pair_angle = z_diff/r_diff;
				if (((pair_angle > 0 && pair_angle < limiting_ratio_) && z_diff < gap_thres_ && z0 < clipping_thres_) || point_index_size == 0)
				{
					r_ref = r0;
					z_ref = z0;
					point_index[point_index_size] = j;
					point_index_size++;
				} else
				{
					if (point_index_size > min_point_)
					{
						for (int m = 0; m < point_index_size; m++)
						{
								
								int index = index_map_.at<int>(point_index[m],i);
								point.x = in_cloud_msg->points[index].x;
								point.y = in_cloud_msg->points[index].y;
								point.z = in_cloud_msg->points[index].z;
								point.intensity = in_cloud_msg->points[index].intensity;
								point.ring = in_cloud_msg->points[index].ring;
								out_ground_points.push_back(point);
								point_class[point_index[m]] = GROUND;
						}
						point_index_size = 0;
					}
					else
					{
						for (int m = 0; m < point_index_size; m++)
						{
							int index = index_map_.at<int>(point_index[m],i);
							point.z = in_cloud_msg->points[index].z;
							if (point.z > clipping_thres_ - sensor_height_)
							{
								point.x = in_cloud_msg->points[index].x;
								point.y = in_cloud_msg->points[index].y;
								point.intensity = in_cloud_msg->points[index].intensity;
								point.ring = in_cloud_msg->points[index].ring;
								out_groundless_points.push_back(point);
								point_class[point_index[m]] = VERTICAL;
							} else {
								unknown_index[unknown_index_size] = index;
								unknown_index_size++;
							}
						}
						point_index_size = 0;
					}
				}
			}
			if (j == 0)
			{
				if (point_index_size != 0)
				{
					if (point_index_size > min_point_)
					{
						for (int m = 0; m < point_index_size; m++)
						{
							int index = index_map_.at<int>(point_index[m],i);
							point.x = in_cloud_msg->points[index].x;
							point.y = in_cloud_msg->points[index].y;
							point.z = in_cloud_msg->points[index].z;
							point.intensity = in_cloud_msg->points[index].intensity;
							point.ring = in_cloud_msg->points[index].ring;
							out_ground_points.push_back(point);
							point_class[point_index[m]] = GROUND;
						}
						point_index_size = 0;
					}
					else
					{
						for (int m = 0; m < point_index_size; m++)
						{
							int index = index_map_.at<int>(point_index[m],i);
							point.z = in_cloud_msg->points[index].z;
							if (point.z > clipping_thres_ - sensor_height_)
							{
								point.x = in_cloud_msg->points[index].x;
								point.y = in_cloud_msg->points[index].y;
								point.intensity = in_cloud_msg->points[index].intensity;
								point.ring = in_cloud_msg->points[index].ring;
								out_groundless_points.push_back(point);
								point_class[point_index[m]] = VERTICAL;
							}
							else
							{
								unknown_index[unknown_index_size] = index;
								unknown_index_size++;
							}
						}
						point_index_size = 0;
					}//end else
				}//end if (point_index_size != 0)

				double centroid = 0;
				int cluster_index[vertical_res_];
				int cluster_index_size = 0;
				for (int m = unknown_index_size - 1; m >= 0; m--)
				{
					double x0 = in_cloud_msg->points[unknown_index[m]].x;
					double y0 = in_cloud_msg->points[unknown_index[m]].y;
					double r0 = sqrt(x0*x0 + y0*y0);
					double r_diff = fabs(r0 - centroid);
					if ((r_diff < point_distance_) || cluster_index_size == 0)
					{
						cluster_index[cluster_index_size] = unknown_index[m];
						cluster_index_size++;
						centroid = r0;
						if (m == 0)
						{
							if(cluster_index_size > 1)
							{
								PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_groundless_points);
							}
							else
							{
								PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_ground_points);
							}
						}
					}
					else
					{
						if(cluster_index_size > 1)
						{
							PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_groundless_points);
						}
						else
						{
							PublishPointCloud(in_cloud_msg, cluster_index, cluster_index_size, out_ground_points);
						}
					}
				}//end for (int m = unknown_index_size - 1; m >= 0; m--)
			}//end if (j == 0)
		}
	}

}

void GroundFilter::VelodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg)
{

	pcl::PointCloud<velodyne_pointcloud::PointXYZIR> vertical_points;
	pcl::PointCloud<velodyne_pointcloud::PointXYZIR> ground_points;
	vertical_points.header = in_cloud_msg->header;
	ground_points.header = in_cloud_msg->header;
	vertical_points.clear();
	ground_points.clear();

	FilterGround(in_cloud_msg, vertical_points, ground_points);

	if (!floor_removal_)
	{
		vertical_points = *in_cloud_msg;
	} 
	
	groundless_points_pub_.publish(vertical_points);
	ground_points_pub_.publish(ground_points);

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "ground_filter");
	GroundFilter node;
	ros::spin();

	return 0;

}
