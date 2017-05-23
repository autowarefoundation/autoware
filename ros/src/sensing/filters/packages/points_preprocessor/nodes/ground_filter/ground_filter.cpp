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

	ros::NodeHandle n;
        ros::Subscriber sub;
	ros::Publisher vertical_points_pub;
	ros::Publisher ground_points_pub;

        std::string     point_topic;
	int 		sensor_model;
	double 		sensor_height;
	double 		max_slope;
	int 		min_point;
	double 		clipping_thres;
	double 		gap_thres;
	double 		point_distance;
        bool            floor_removal;

	int 		vertical_res;
	int 		horizontal_res;
	double 		limiting_ratio;
	cv::Mat 	index_map;
	Label 		class_label[64];

	boost::chrono::high_resolution_clock::time_point t1;
	boost::chrono::high_resolution_clock::time_point t2;
	boost::chrono::nanoseconds elap_time;

	void initLabelArray(int model);
	void initDepthMap(int width);
	void publishPoint(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg,
				int index[], int &index_size, 
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &topic);


	void velodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg);
	void groundSeparate(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg, 
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &vertical_points, 
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &ground_points);

};

GroundFilter::GroundFilter() : n("~")
{

	n.param<std::string>("point_topic", point_topic, "/points_raw");
  	n.param("remove_floor",  floor_removal,  true);
        n.param("sensor_model", sensor_model, 64);
        n.param("sensor_height", sensor_height, 1.72);
        n.param("max_slope", max_slope, 20.0);
	n.param("point_distance", point_distance, 0.05);

        n.param("min_point", min_point, 3);
	n.param("clipping_thres", clipping_thres, 0.5);
	n.param("gap_thres", gap_thres, 0.5);

	vertical_res 	= 64;
	horizontal_res 	= 2000;
	limiting_ratio 	= tan(20.0*M_PI/180);
        
	sub = n.subscribe(point_topic, 10, &GroundFilter::velodyneCallback, this);
        vertical_points_pub = n.advertise<sensor_msgs::PointCloud2>("/points_lanes", 10);
        ground_points_pub = n.advertise<sensor_msgs::PointCloud2>("/points_ground", 10);

	vertical_res = sensor_model;
	initLabelArray(sensor_model);
	limiting_ratio = tan(max_slope*M_PI/180);

}

void GroundFilter::initLabelArray(int model)
{
	for(int a = 0; a < vertical_res; a++)
	{
		class_label[a] = UNKNOWN;
	}
}

void GroundFilter::initDepthMap(int width)
{
	const int mOne = -1;
	index_map = cv::Mat_<int>(vertical_res, width, mOne);
}

void GroundFilter::publishPoint(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg,
				int index[], int &index_size, 
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &topic)
{

	velodyne_pointcloud::PointXYZIR point;
	for (int i = 0; i < index_size; i++)
	{
		point.x = msg->points[index[i]].x;
		point.y = msg->points[index[i]].y;
		point.z = msg->points[index[i]].z;
		point.intensity = msg->points[index[i]].intensity;
		point.ring = msg->points[index[i]].ring;
		topic.push_back(point);
	}
	index_size = 0;	

}

void GroundFilter::groundSeparate(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg, 
			pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &vertical_points, 
			pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &ground_points)
{

        velodyne_pointcloud::PointXYZIR point;
	
        horizontal_res = int(msg->points.size() / vertical_res);
        initDepthMap(horizontal_res);

        for (int i = 0; i < msg->points.size(); i++)
        {
                double u = atan2(msg->points[i].y,msg->points[i].x) * 180/M_PI;
                if (u < 0) u = 360 + u;  
                int column = horizontal_res - (int)((double)horizontal_res * u / 360.0) - 1;   
                int row = vertical_res - 1 - msg->points[i].ring;
                index_map.at<int>(row, column) = i;
        }

	for (int i = 0; i < horizontal_res; i++)
        {
                Label point_class[vertical_res];
		int unknown_index[vertical_res];
		int point_index[vertical_res];
		int unknown_index_size = 0;
		int point_index_size = 0;
		double z_ref = 0;
		double r_ref = 0;
		std::copy(class_label, class_label + vertical_res, point_class); 

		for (int j = vertical_res - 1; j >= 0; j--)
                {
                        if (index_map.at<int>(j,i) > -1 && point_class[j] == UNKNOWN)
                        {
				double x0 = msg->points[index_map.at<int>(j, i)].x;
				double y0 = msg->points[index_map.at<int>(j, i)].y;
				double z0 = msg->points[index_map.at<int>(j, i)].z;
				double r0 = sqrt(x0*x0 + y0*y0);
				double r_diff = r0 - r_ref;
				double z_diff = fabs(z0 - z_ref);
				double pair_angle = z_diff/r_diff;
				if (((pair_angle > 0 && pair_angle < limiting_ratio) && z_diff < gap_thres && z0 < clipping_thres) || point_index_size == 0)
				{
					r_ref = r0;
					z_ref = z0;
					point_index[point_index_size] = j;
					point_index_size++;
				} else {
					if (point_index_size > min_point)
					{
						for (int m = 0; m < point_index_size; m++)
						{
								
								int index = index_map.at<int>(point_index[m],i);
								point.x = msg->points[index].x;
								point.y = msg->points[index].y;
								point.z = msg->points[index].z;
								point.intensity = msg->points[index].intensity;
								point.ring = msg->points[index].ring;
								ground_points.push_back(point);
								point_class[point_index[m]] = GROUND;
						}
						point_index_size = 0;
					} else {
						for (int m = 0; m < point_index_size; m++)
						{
							int index = index_map.at<int>(point_index[m],i);
							point.z = msg->points[index].z;
							if (point.z > clipping_thres - sensor_height)
							{
								point.x = msg->points[index].x;
								point.y = msg->points[index].y;
								point.intensity = msg->points[index].intensity;
								point.ring = msg->points[index].ring;
								vertical_points.push_back(point);
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
					if (point_index_size > min_point)
					{
						for (int m = 0; m < point_index_size; m++)
						{
								
								int index = index_map.at<int>(point_index[m],i);
								point.x = msg->points[index].x;
								point.y = msg->points[index].y;
								point.z = msg->points[index].z;
								point.intensity = msg->points[index].intensity;
								point.ring = msg->points[index].ring;
								ground_points.push_back(point);
								point_class[point_index[m]] = GROUND;
						}
						point_index_size = 0;
					} else {
						for (int m = 0; m < point_index_size; m++)
						{
							int index = index_map.at<int>(point_index[m],i);
							point.z = msg->points[index].z;
							if (point.z > clipping_thres - sensor_height)
							{
								point.x = msg->points[index].x;
								point.y = msg->points[index].y;
								point.intensity = msg->points[index].intensity;
								point.ring = msg->points[index].ring;
								vertical_points.push_back(point);
								point_class[point_index[m]] = VERTICAL;
							} else {
								unknown_index[unknown_index_size] = index;
								unknown_index_size++;
							}
						}
						point_index_size = 0;
					}
				}
				double centroid = 0;
				int cluster_index[vertical_res];
				int cluster_index_size = 0;
				for (int m = unknown_index_size - 1; m >= 0; m--)
				{
					double x0 = msg->points[unknown_index[m]].x;
					double y0 = msg->points[unknown_index[m]].y;
					double r0 = sqrt(x0*x0 + y0*y0);
					double r_diff = fabs(r0 - centroid);
					if ((r_diff < point_distance) || cluster_index_size == 0)
					{
						cluster_index[cluster_index_size] = unknown_index[m];
						cluster_index_size++;
						centroid = r0;
						if (m == 0)
						{
							if(cluster_index_size > 1)
							{
								publishPoint(msg, cluster_index, cluster_index_size, vertical_points);
							} else {
								publishPoint(msg, cluster_index, cluster_index_size, ground_points);
							}
						}
					} else {
						if(cluster_index_size > 1)
						{
							publishPoint(msg, cluster_index, cluster_index_size, vertical_points);
						} else {
							publishPoint(msg, cluster_index, cluster_index_size, ground_points);
						}
					}
				}
                        }
                }
	}

}

void GroundFilter::velodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg)
{

	pcl::PointCloud<velodyne_pointcloud::PointXYZIR> vertical_points;
	pcl::PointCloud<velodyne_pointcloud::PointXYZIR> ground_points;
	vertical_points.header = msg->header;
        ground_points.header = msg->header;
        vertical_points.clear();
        ground_points.clear();

	groundSeparate(msg, vertical_points, ground_points);

	if (!floor_removal)
	{
		vertical_points = *msg;
	} 
	
	vertical_points_pub.publish(vertical_points);
        ground_points_pub.publish(ground_points);

}

int main(int argc, char **argv)
{

        ros::init(argc, argv, "ground_filter");
	GroundFilter node;
        ros::spin();

	return 0;

}
