#ifndef POINTS_MAP_FILTER_H_INCLUDED
#define POINTS_MAP_FILTER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//headers in PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

//headers in boost
#include <boost/optional.hpp>

//headers in STL
#include <mutex>
#include <regex>

class points_map_filter
{
public:
    points_map_filter(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~points_map_filter();
    void init();
    void run();
private:
    pcl::PassThrough<pcl::PointXYZ> pass_;
    ros::Subscriber map_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher map_pub_;
    std::mutex mtx_;
    ros::NodeHandle nh_,pnh_;
    void map_callback_(const sensor_msgs::PointCloud2::ConstPtr msg);
    void current_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg);
    double load_grid_size_;
    double load_trigger_distance_;
    std::string map_frame_;
    boost::optional<geometry_msgs::PoseStamped> last_load_pose_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    volatile bool map_recieved_;
};

#endif  //POINTS_MAP_FILTER_H_INCLUDED