#include <map_file/points_map_filter.h>

points_map_filter::points_map_filter(ros::NodeHandle nh,ros::NodeHandle pnh) : tf_listener_(tf_buffer_)
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
    init();
    pcl::fromROSMsg(*msg, *map_cloud_);
    map_recieved_ = true;
    map_pub_.publish(*msg);
    return;
}

void points_map_filter::current_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    geometry_msgs::TransformStamped transform_stamped_;
    try
    {
        transform_stamped_ = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }
    geometry_msgs::PoseStamped transformed_current_pose;
    tf2::doTransform(*msg, transformed_current_pose, transform_stamped_);
    if(!last_load_pose_ && map_recieved_)
    {
        last_load_pose_ = transformed_current_pose;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pass_.setInputCloud(map_cloud_);

    }
    else if(last_load_pose_ && map_recieved_)
    {

    }
    return;
}