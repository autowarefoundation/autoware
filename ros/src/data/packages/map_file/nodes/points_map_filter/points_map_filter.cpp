#include <map_file/points_map_filter.h>

points_map_filter::points_map_filter(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    map_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    nh_ = nh;
    pnh_ = pnh;
}

points_map_filter::~points_map_filter()
{

}

void points_map_filter::init()
{
    std::lock_guard<std::mutex> lock(mtx_);
    pnh_.param("load_grid_size", load_grid_size_, 100.0);
    pnh_.param("load_trigger_distance", load_trigger_distance_, 20.0);
    pnh_.param("map_frame", map_frame_, std::string("map"));
    last_load_pose_ = boost::none;
    map_sub_.shutdown();
    pose_sub_.shutdown();
    map_recieved_ = false;
    return;
}

void points_map_filter::run()
{
    std::lock_guard<std::mutex> lock(mtx_);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_map/filtered",10);
    map_sub_ =  nh_.subscribe("/points_map",1,&points_map_filter::map_callback_,this);
    pose_sub_ = nh_.subscribe("/current_pose",1,&points_map_filter::current_pose_callback_,this);
}

void points_map_filter::map_callback_(const sensor_msgs::PointCloud2::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    ROS_INFO_STREAM("loading map started.");
    pcl::fromROSMsg(*msg, *map_cloud_);
    map_recieved_ = true;
    map_pub_.publish(*msg);
    ROS_INFO_STREAM("loading map finished");
    return;
}

void points_map_filter::current_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    ROS_INFO_STREAM("pose received");
    if(!last_load_pose_ && map_recieved_)
    {
        last_load_pose_ = *msg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pass_.setInputCloud(map_cloud_);
        pass_.setFilterFieldName("x");
        pass_.setFilterLimits(last_load_pose_.get().pose.position.x-(load_grid_size_/2), last_load_pose_.get().pose.position.x+(load_grid_size_/2));
        pass_.filter(*cloud_filtered);
        pass_.setInputCloud(cloud_filtered);
        pass_.setFilterFieldName("y");
        pass_.setFilterLimits(last_load_pose_.get().pose.position.y-(load_grid_size_/2), last_load_pose_.get().pose.position.y+(load_grid_size_/2));
        pass_.filter(*cloud_filtered);
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud_filtered, msg);
        ROS_INFO_STREAM("update map");
        map_pub_.publish(msg);
    }
    else if(last_load_pose_ && map_recieved_)
    {
        double dist = std::sqrt(std::pow(last_load_pose_.get().pose.position.x-msg->pose.position.x,2)+std::pow(last_load_pose_.get().pose.position.y-msg->pose.position.y,2));
        if(dist > load_trigger_distance_)
        {
            last_load_pose_ = *msg;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pass_.setInputCloud(map_cloud_);
            pass_.setFilterFieldName("x");
            pass_.setFilterLimits(last_load_pose_.get().pose.position.x-(load_grid_size_/2), last_load_pose_.get().pose.position.x+(load_grid_size_/2));
            pass_.filter(*cloud_filtered);
            pass_.setInputCloud(cloud_filtered);
            pass_.setFilterFieldName("y");
            pass_.setFilterLimits(last_load_pose_.get().pose.position.y-(load_grid_size_/2), last_load_pose_.get().pose.position.y+(load_grid_size_/2));
            pass_.filter(*cloud_filtered);
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*cloud_filtered, msg);
            map_pub_.publish(msg);
            ROS_INFO_STREAM("update map");
        }
    }
    return;
}