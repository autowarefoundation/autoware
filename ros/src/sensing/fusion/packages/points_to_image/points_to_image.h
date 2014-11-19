#ifndef POINTS_TO_IMAGE_H
#define POINTS_TO_IMAGE_H

#include<rosinterface.h>
#include<opencv2/opencv.hpp>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include<sensor_msgs/Image.h>
#include<sensor_msgs/PointCloud2.h>


class Points_to_image
{
public:
    Points_to_image();
public:
    ROSSub<sensor_msgs::PointCloud2ConstPtr> * velodynesub;=new ROSSub<sensor_msgs::PointCloud2ConstPtr>(velodyneTopic,velodyneQueueSize,velodyneInterval);
};

#endif // POINTS_TO_IMAGE_H
