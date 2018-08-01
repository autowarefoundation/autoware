/*
 * ring_ground_filter.cpp
 *
 * Created on	: June 5, 2018
 * Author	: Patiphon Narksri
 *
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
	GROUND,
	VERTICAL,
	UNKNOWN //Initial state, not classified
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
	std::string no_ground_topic, ground_topic;
	int 		sensor_model_;
	double 		sensor_height_;
	double 		max_slope_;
	double vertical_thres_;
	bool		floor_removal_;

	int 		vertical_res_;
	int 		horizontal_res_;
	cv::Mat 	index_map_;
	Label 		class_label_[64];
	double	radius_table_[64];

	//boost::chrono::high_resolution_clock::time_point t1_;
	//boost::chrono::high_resolution_clock::time_point t2_;
	//boost::chrono::nanoseconds elap_time_;
	ros::Time t1_;
	ros::Time t2_;
	ros::Duration elap_time_;

	const int 	DEFAULT_HOR_RES = 2000;

	void InitLabelArray(int in_model);
	void InitRadiusTable(int in_model);
	void InitDepthMap(int in_width);
	void VelodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg);
	void FilterGround(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_groundless_points,
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_ground_points);

};

GroundFilter::GroundFilter() : node_handle_("~")
{
	ROS_INFO("Inititalizing Ground Filter...");
	node_handle_.param<std::string>("point_topic", point_topic_, "/points_raw");
	ROS_INFO("Input Point Cloud: %s", point_topic_.c_str());
 	node_handle_.param("remove_floor",  floor_removal_,  true);
 	ROS_INFO("Floor Removal: %d", floor_removal_);
	node_handle_.param("sensor_model", sensor_model_, 64);
	ROS_INFO("Sensor Model: %d", sensor_model_);
	node_handle_.param("sensor_height", sensor_height_, 1.80);
	ROS_INFO("Sensor Height: %f", sensor_height_);
	node_handle_.param("max_slope", max_slope_, 10.0);
	ROS_INFO("Max Slope: %f", max_slope_);
	node_handle_.param("vertical_thres", vertical_thres_, 0.08);
	ROS_INFO("Vertical Threshold: %f", vertical_thres_);

	node_handle_.param<std::string>("no_ground_point_topic", no_ground_topic, "/points_no_ground");
	ROS_INFO("No Ground Output Point Cloud: %s", no_ground_topic.c_str());
	node_handle_.param<std::string>("ground_point_topic", ground_topic, "/points_ground");
	ROS_INFO("Only Ground Output Point Cloud: %s", ground_topic.c_str());


	int default_horizontal_res;
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
		default:
			default_horizontal_res = DEFAULT_HOR_RES;
			break;
	}
	node_handle_.param("horizontal_res", horizontal_res_, default_horizontal_res);

	points_node_sub_ = node_handle_.subscribe(point_topic_, 10000, &GroundFilter::VelodyneCallback, this);
	groundless_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 10000);
	ground_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ground_topic, 10000);

	vertical_res_ = sensor_model_;
	InitLabelArray(sensor_model_);
	InitRadiusTable(sensor_model_);
}

void GroundFilter::InitLabelArray(int in_model)
{
	for(int a = 0; a < vertical_res_; a++)
	{
		class_label_[a] = UNKNOWN;
	}
}

void GroundFilter::InitRadiusTable(int in_model)
{
	double a;
	double b;
	double theta;
	switch (in_model)
	{
		case 64:
			a = 1.0/3*M_PI/180;
			b = max_slope_*M_PI/180;
			for (int i = 0; i < 64; i++)
			{
				if (i <= 31)
				{
					if (i == 31) a = -a;
					theta = (1.0/3*i - 2.0)*M_PI/180;
					radius_table_[i] = fabs(sensor_height_*(1.0/(tan(theta)+tan(b)) - 1.0/(tan(a+theta)+tan(b))));
				}
				else
				{
					a = 0.5*M_PI/180;
					theta = (8.83 + (0.5)*(i - 32.0))*M_PI/180;
					radius_table_[i] = fabs(sensor_height_*(1.0/(tan(theta)+tan(b)) - 1.0/(tan(a+theta)+tan(b))));
				}
			}
			break;
		case 32:
			a = 4.0/3*M_PI/180;
			b = max_slope_*M_PI/180;
			for (int i = 0; i < 32; i++)
			{
				theta = (-31.0/3 + (4.0/3)*i)*180/M_PI;
				radius_table_[i] = fabs(sensor_height_*(1.0/(tan(theta)+tan(b)) - 1.0/(tan(a+theta)+tan(b))));
			}
			break;
		case 16:
			a = 2.0*M_PI/180;
			b = max_slope_*M_PI/180;
			for (int i = 0; i < 16; i++)
			{
				theta = (-30.0/2 + (2.0)*i)*180/M_PI;
				radius_table_[i] = fabs(sensor_height_*(1.0/(tan(theta)+tan(b)) - 1.0/(tan(a+theta)+tan(b))));
			}
			break;
		default:
			a = 1.0/3*M_PI/180;
			b = max_slope_*M_PI/180;
			for (int i = 0; i < 64; i++)
			{
				if (i <= 31)
				{
					if (i == 31) a = -a;
					theta = (1.0/3*i - 2.0)*M_PI/180;
					radius_table_[i] = fabs(sensor_height_*(1.0/(tan(theta)+tan(b)) - 1.0/(tan(a+theta)+tan(b))));
				}
				else
				{
					a = 0.5*M_PI/180;
					theta = (8.83 + (0.5)*(i - 32.0))*M_PI/180;
					radius_table_[i] = fabs(sensor_height_*(1.0/(tan(theta)+tan(b)) - 1.0/(tan(a+theta)+tan(b))));
				}
			}
			break;
	}
}

void GroundFilter::InitDepthMap(int in_width)
{
	const int mOne = -1;
	index_map_ = cv::Mat_<int>(vertical_res_, in_width, mOne);
}

void GroundFilter::FilterGround(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
			pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_groundless_points,
			pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_ground_points)
{

	velodyne_pointcloud::PointXYZIR point;
	InitDepthMap(horizontal_res_);

	for (size_t i = 0; i < in_cloud_msg->points.size(); i++)
	{
		double u = atan2(in_cloud_msg->points[i].y,in_cloud_msg->points[i].x) * 180/M_PI;
		if (u < 0) { u = 360 + u; }
		int column = horizontal_res_ - (int)((double)horizontal_res_ * u / 360.0) - 1;
		int row = vertical_res_ - 1 - in_cloud_msg->points[i].ring;
		index_map_.at<int>(row, column) = i;
	}

	for (int i = 0; i < horizontal_res_; i++)
	{
		Label point_class[vertical_res_];
		int point_index[vertical_res_];
		int point_index_size = 0;
		double z_max = 0;
		double z_min = 0;
		double r_ref = 0;
		std::copy(class_label_, class_label_ + vertical_res_, point_class);
		for (int j = 0; j < vertical_res_; j++)
		{
			if (index_map_.at<int>(j,i) > -1 && point_class[j] == UNKNOWN)
			{
				double x0 = in_cloud_msg->points[index_map_.at<int>(j, i)].x;
				double y0 = in_cloud_msg->points[index_map_.at<int>(j, i)].y;
				double z0 = in_cloud_msg->points[index_map_.at<int>(j, i)].z;
				double r0 = sqrt(x0*x0 + y0*y0);
				double r_diff = fabs(r0 - r_ref);
				if (r_diff < radius_table_[j] || r_ref == 0)
				{
					r_ref = r0;
					if (z0 > z_max || r_ref == 0) z_max = z0;
					if (z0 < z_min || r_ref == 0) z_min = z0;
					point_index[point_index_size] = j;
					point_index_size++;
				}
				else
				{
					if (point_index_size > 1 && (z_max - z_min) > vertical_thres_)
					{
						for (int m = 0; m < point_index_size; m++)
						{
							int index = index_map_.at<int>(point_index[m],i);
							point.x = in_cloud_msg->points[index].x;
							point.y = in_cloud_msg->points[index].y;
							point.z = in_cloud_msg->points[index].z;
							point.intensity = in_cloud_msg->points[index].intensity;
							point.ring = in_cloud_msg->points[index].ring;
							out_groundless_points.push_back(point);
							point_class[point_index[m]] = VERTICAL;
						}
						point_index_size = 0;
					}
					else
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
					r_ref = r0;
					z_max = z0;
					z_min = z0;
					point_index[point_index_size] = j;
					point_index_size++;
				}
			}
			if (j == vertical_res_ - 1 && point_index_size != 0)
			{
					if (point_index_size > 1 && (z_max - z_min) > vertical_thres_)
					{
						for (int m = 0; m < point_index_size; m++)
						{
							int index = index_map_.at<int>(point_index[m],i);
							point.x = in_cloud_msg->points[index].x;
							point.y = in_cloud_msg->points[index].y;
							point.z = in_cloud_msg->points[index].z;
							point.intensity = in_cloud_msg->points[index].intensity;
							point.ring = in_cloud_msg->points[index].ring;
							out_groundless_points.push_back(point);
							point_class[point_index[m]] = VERTICAL;
						}
						point_index_size = 0;
					}
					else
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
			}
		}
	}
}

void GroundFilter::VelodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg)
{

	//t1_ = ros::Time().now();
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
	//t2_ = boost::chrono::high_resolution_clock::now();
	//t2_ = ros::Time().now();
	//elap_time_ = t2_ - t1_;//boost::chrono::duration_cast<boost::chrono::nanoseconds>(t2_-t1_);
	//std::cout << "Computational Time for one frame: " << elap_time_ << '\n';
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "ring_ground_filter");
	GroundFilter node;
	ros::spin();

	return 0;

}
