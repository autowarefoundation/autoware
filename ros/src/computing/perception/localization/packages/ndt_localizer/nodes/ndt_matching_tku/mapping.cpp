
// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "velodyne_pointcloud/point_types.h"
#include "velodyne_pointcloud/rawdata.h"
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

pcl::PointCloud<velodyne_pointcloud::PointXYZIR> map;
tf::TransformListener *tf_listener; 
ros::Publisher velodyne_pub;

char dir_name[100];
int is_dir=0;
pcl::PointCloud<pcl::PointXYZI> prev_points;
ros::Time prev_time;
//void velodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr& msg)
void velodyneCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
  static int count=0;
  
  //  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> pcl_out;
  pcl::PointCloud<pcl::PointXYZI> pcl_out;
  std_msgs::Header header;
  pcl_conversions::fromPCL(msg->header,header);
  printf("%f %f\n",header.stamp.toSec(),ros::Time::now().toSec());
  count++;
  //usleep(50000);
  tf::StampedTransform transform;
  if(prev_points.points.size()>0){
    //  tf_listener->waitForTransform("/world", header.frame_id, header.stamp, ros::Duration(1));
    //pcl_ros::transformPointCloud("/world", *msg, pcl_out, *tf_listener);
    try{
      /*    tf_listener->waitForTransform("/japan_7", header.frame_id, header.stamp, ros::Duration(1));
	    pcl_ros::transformPointCloud("/japan_7", *msg, pcl_out, *tf_listener);
      */
      //    tf_listener->waitForTransform("/world", header.frame_id, header.stamp, ros::Duration(1));
      tf_listener->waitForTransform("world","ndt_frame", prev_time/*header.stamp*/, ros::Duration(1));
      tf_listener->lookupTransform("world","ndt_frame",prev_time,transform);
      /*pcl_ros::transformPointCloud("world", prev_time, prev_points, "ndt_frame", pcl_out, *tf_listener);
       */      
    }catch(tf::TransformException ex){
      printf("old\n");
      return;
    }
    
    for(int i=0;i<prev_points.points.size(); i++){
      tf::Point pt(prev_points[i].x,prev_points[i].y,prev_points[i].z);
      tf::Point pt_world = transform * pt;
      pcl::PointXYZI wp;
      double distance=pt.x()*pt.x()+pt.y()*pt.y()+pt.z()*pt.z();
      if(distance<3*3)continue;
      wp.x=pt_world.x();
      wp.y=pt_world.y();
      wp.z=pt_world.z();
      wp.intensity=prev_points[i].intensity;

      pcl_out.push_back(wp);
    }
    pcl_out.header=prev_points.header;
    pcl_out.header.frame_id="world";
    velodyne_pub.publish(pcl_out);    
    
    if(is_dir && count%5 ==0){
      char fname[100];
      sprintf(fname,"%s/scan%06d",dir_name,count);
      FILE* fp_points=fopen(fname,"w");
      
      for(int i=0;i<pcl_out.points.size(); i++){
	fprintf(fp_points,"%.3f %.3f %.3f %.1f\n",
		pcl_out.points[i].y,
		pcl_out.points[i].x,
		pcl_out.points[i].z,
		pcl_out.points[i].intensity
	      ); 
      }
      fclose(fp_points);
    }

  }
  prev_points=*msg;
  prev_time =header.stamp;
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapping");
  ros::NodeHandle n;
    
  ros::Subscriber sub = n.subscribe("velodyne_points", 10, velodyneCallback);
  // velodyne_pub = n.advertise<pcl::PointCloud<velodyne_pointcloud::PointXYZIR > >("velodyne_points/world", 1);
  velodyne_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI > >("velodyne_points/world", 1);

  tf_listener    = new tf::TransformListener();
  sleep(2);
  if(argc>1){
    sprintf(dir_name,"%s",argv[1]);
    is_dir=1;
  }
  
  ros::spin();


  return 0;
}
// %EndTag(FULLTEXT)%
