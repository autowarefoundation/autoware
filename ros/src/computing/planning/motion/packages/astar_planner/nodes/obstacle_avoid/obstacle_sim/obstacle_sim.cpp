#include "obstacle_sim.h"

namespace astar_planner
{

void convertToPointCloud2(const std::vector<geometry_msgs::Point>& points, sensor_msgs::PointCloud2* ros_pointcloud)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_points;

  for (const auto& p : points)
  {
    pcl::PointXYZ pcl_p;
    pcl_p.x = p.x;
    pcl_p.y = p.y;
    pcl_p.z = p.z;

    pcl_points.push_back(pcl_p);
  }

  //sensor_msgs::PointCloud2 ros_pointcloud;
  pcl::toROSMsg(pcl_points, *ros_pointcloud);
}

geometry_msgs::Point transformPoint(const geometry_msgs::Point& p, const tf::Transform& tf)
{
  // convert to tf msg
  tf::Point tf_point;
  tf::pointMsgToTF(p, tf_point);

  // apply transform
  tf_point = tf * tf_point;

  // convert to ros msg
  geometry_msgs::Point ros_point;
  tf::pointTFToMsg(tf_point, ros_point);

  return ros_point;
}

ObstacleSim::ObstacleSim()
    : private_nh_("~"),
      sub_navgoal(false)
{
  initForROS();
}

ObstacleSim::~ObstacleSim()
{
}

void ObstacleSim::initForROS()
{
  // ros parameter settings
  private_nh_.param<double>("obstacle_height", obstacle_height_, 1.0);
  private_nh_.param<double>("obstacle_width", obstacle_width_, 1.0);
  private_nh_.param<double>("points_interval", points_interval_, 0.1);
  private_nh_.param<std::string>("obstacle_frame", obstacle_frame_, "/velodyne");

  // setup subscriber
  nav_goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &ObstacleSim::callbackFromNavGoal, this);

  // setup publisher
  obstacle_sim_points_pub_ = nh_.advertise<visualization_msgs::Marker>("obstacle_sim_points", 1, true);
  obstacle_sim_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacle_sim_pointcloud", 1, true);
  obstacle_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("astar_sim_obstacle", 1, true);
}

void ObstacleSim::callbackFromNavGoal(const geometry_msgs::PoseStampedConstPtr& msg)
{
  sub_navgoal = true;

  // frame of using in RViz
  world_frame_ = msg->header.frame_id;

  // reset points
  obstacle_points_.clear();

  // points at each vertex of rectangle
  geometry_msgs::Point upper_left, upper_right, lower_left, lower_right;
  upper_left.x  = msg->pose.position.x - obstacle_width_ / 2.0;
  upper_left.y  = msg->pose.position.y + obstacle_height_ / 2.0;
  upper_right.x = upper_left.x + obstacle_width_;
  upper_right.y = upper_left.y;
  lower_left.x  = upper_left.x;
  lower_left.y  = upper_left.y - obstacle_height_;
  lower_right.x = upper_right.x;
  lower_right.y = lower_left.y;

  // points along each side
  for (double x = upper_left.x; x < upper_right.x; x += points_interval_)
  {
    geometry_msgs::Point up, down;
    up.x = x;
    up.y = upper_left.y;
    down.x = x;
    down.y = lower_left.y;

    obstacle_points_.emplace_back(up);
    obstacle_points_.emplace_back(down);
  }
  for (double y = upper_left.y; y > lower_left.y; y -= points_interval_)
  {
    geometry_msgs::Point left, right;
    left.x = upper_left.x;
    left.y = y;
    right.x = upper_right.x;
    right.y = y;

    obstacle_points_.emplace_back(left);
    obstacle_points_.emplace_back(right);
  }

  displayObstacleMarker(upper_left, upper_right, lower_left, lower_right, world_frame_);
  
  // debug
  //displayObstaclePoints();
}

// debug
void ObstacleSim::displayObstaclePoints()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time();
  marker.ns = "obstacle_sim";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  for (const auto &p : obstacle_points_)
  {
    marker.points.push_back(p);
  }

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  //marker.lifetime = ros::Duration(0.1);
  marker.frame_locked = true;


  obstacle_sim_points_pub_.publish(marker);
}

void ObstacleSim::displayObstacleMarker(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3, const geometry_msgs::Point& p4, const std::string& frame)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time();
  marker.ns = "obstacle_sim";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  // 1 -- 2
  // |    |
  // 3 -- 4
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.points.push_back(p4);
  marker.points.push_back(p3);
  marker.points.push_back(p1);
  marker.points.push_back(p4);
  marker.points.push_back(p3);
  marker.points.push_back(p2);

  marker.scale.x = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  //marker.lifetime = ros::Duration(0.1);
  marker.frame_locked = true;

  obstacle_marker_pub_.publish(marker);
}

void ObstacleSim::publishPoints()
{
  // transform points to sensor frame to be detected as an obstacle
  tf::StampedTransform world2sensor;
  try
  {
    tf_listener_.lookupTransform(world_frame_, obstacle_frame_, ros::Time(0), world2sensor);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  tf::Transform sensor2world = world2sensor.inverse();

  std::vector<geometry_msgs::Point> obstacle_points_tf;
  for (const auto& p : obstacle_points_)
  {
    geometry_msgs::Point tf_p = transformPoint(p, sensor2world);
    tf_p.z = 0.0;

    obstacle_points_tf.emplace_back(tf_p);
  }

  sensor_msgs::PointCloud2 ros_pointcloud;
  convertToPointCloud2(obstacle_points_tf, &ros_pointcloud);
  ros_pointcloud.header.frame_id = obstacle_frame_;
  ros_pointcloud.header.stamp = ros::Time();

  obstacle_sim_pointcloud_pub_.publish(ros_pointcloud);
}

void ObstacleSim::run()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();

    // not subscribed obstacle pose yet
    if (!sub_navgoal)
    {
      loop_rate.sleep();
      continue;
    }

    // publish obstacle points
    publishPoints();

    loop_rate.sleep();
  }
}

} // namespace astar_planner
