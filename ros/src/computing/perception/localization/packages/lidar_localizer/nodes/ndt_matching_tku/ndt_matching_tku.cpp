/*
 * Normal Distributions Transform program
 * 205/04/24 tku
 * ndt_matching for ROS
 */

// number of cells
#define G_MAP_X 2000
#define G_MAP_Y 2000
#define G_MAP_Z 200
#define G_MAP_CELLSIZE 1.0

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <GL/glut.h>
#include <math.h>
#include <stdio.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include "algebra.h"
#include "ndt.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "velodyne_pointcloud/point_types.h"
#include "velodyne_pointcloud/rawdata.h"

/*grobal variables*/
NDMapPtr NDmap;
NDPtr NDs;
int NDs_num;

Point scan_points[130000];
int scan_points_num;

int is_first_time = 1;
int is_map_exist = 0;

double scan_points_weight[130000];

Point map_points[130000];
double map_points_i[130000];

int layer_select = LAYER_NUM - 1;

Posture prev_pose, prev_pose2;

// params
double g_map_center_x, g_map_center_y, g_map_center_z;
double g_map_rotation;
char g_ndmap_name[500];
int g_use_gnss;
int g_map_update = 1;
double g_ini_x, g_ini_y, g_ini_z, g_ini_roll, g_ini_pitch, g_ini_yaw;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;  // tf between base_link and localizer
static tf::Quaternion q_local_to_global, q_global_to_local;
static Eigen::Matrix4f tf_local_to_global, tf_global_to_local;

static pcl::PointCloud<pcl::PointXYZ> map;
static int map_loaded = 0;

static std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;
static double exe_time = 0.0;

static ros::Publisher localizer_pose_pub, ndt_pose_pub;
static geometry_msgs::PoseStamped localizer_pose_msg, ndt_pose_msg;

// double pose_mod(Posture *pose){
void pose_mod(Posture *pose)
{
  while (pose->theta < -M_PI)
    pose->theta += 2 * M_PI;
  while (pose->theta > M_PI)
    pose->theta -= 2 * M_PI;
  while (pose->theta2 < -M_PI)
    pose->theta2 += 2 * M_PI;
  while (pose->theta2 > M_PI)
    pose->theta2 -= 2 * M_PI;
  while (pose->theta3 < -M_PI)
    pose->theta3 += 2 * M_PI;
  while (pose->theta3 > M_PI)
    pose->theta3 -= 2 * M_PI;
}

double nrand(double n)
{
  double r;
  r = n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX);
  return r;
}

static void map_callback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
  if (map_loaded == 0)
  {
    pcl::fromROSMsg(*input, map);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));

    Point p;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = map_ptr->begin(); item != map_ptr->end(); item++)
    {
      p.x = (item->x - g_map_center_x) * cos(-g_map_rotation) - (item->y - g_map_center_y) * sin(-g_map_rotation);
      p.y = (item->x - g_map_center_x) * sin(-g_map_rotation) + (item->y - g_map_center_y) * cos(-g_map_rotation);
      p.z = item->z - g_map_center_z;
      add_point_map(NDmap, &p);
    }
    std::cout << "Finished loading point cloud map." << std::endl;
    std::cout << "Map points num: " << map_ptr->size() << " points." << std::endl;

    is_map_exist = 1;
    map_loaded = 1;
  }
}

static void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::cout << "call 2D pose estimate" << std::endl;
  try
  {
    ros::Time now = ros::Time(0);
    listener.waitForTransform("/map", input->header.frame_id, now, ros::Duration(10.0));
    listener.lookupTransform("/map", input->header.frame_id, now, transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  tf::Quaternion q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z,
                   input->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  g_ini_x = input->pose.pose.position.x + transform.getOrigin().x();
  g_ini_y = input->pose.pose.position.y + transform.getOrigin().y();
  g_ini_z = input->pose.pose.position.z + transform.getOrigin().z();

  m.getRPY(g_ini_roll, g_ini_pitch, g_ini_yaw);

  // global(map) to local(map center)
  Eigen::Translation3f translation(g_ini_x, g_ini_y, g_ini_z);
  Eigen::AngleAxisf rotation_x(g_ini_roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rotation_y(g_ini_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rotation_z(g_ini_yaw, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f global_t = (translation * rotation_z * rotation_y * rotation_x).matrix();
  Eigen::Matrix4f local_t = tf_global_to_local * global_t;

  tf::Quaternion q_g_to_l;
  tf::Matrix3x3 mat_g;
  mat_g.setValue(
      static_cast<double>(local_t(0, 0)), static_cast<double>(local_t(0, 1)), static_cast<double>(local_t(0, 2)),
      static_cast<double>(local_t(1, 0)), static_cast<double>(local_t(1, 1)), static_cast<double>(local_t(1, 2)),
      static_cast<double>(local_t(2, 0)), static_cast<double>(local_t(2, 1)), static_cast<double>(local_t(2, 2)));

  mat_g.getRotation(q_g_to_l);

  prev_pose.x = local_t(0, 3);
  prev_pose.y = local_t(1, 3);
  prev_pose.z = local_t(2, 3);
  prev_pose.theta = q_g_to_l.x();
  prev_pose.theta2 = q_g_to_l.y();
  prev_pose.theta3 = q_g_to_l.z();

  prev_pose2 = prev_pose;
}

void points_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  matching_start = std::chrono::system_clock::now();
  static tf::TransformBroadcaster br;
  static FILE *log_fp;
  ros::Time time;
  static tf::TransformListener listener;
  static ros::Time current_scan_time;
  current_scan_time = msg->header.stamp;

  static int iteration;

  Posture pose, bpose;
  double e = 0;
  double x_offset, y_offset, z_offset, theta_offset;

  tf::Quaternion ndt_q, localizer_q;

  pcl::PointCloud<pcl::PointXYZ> filtered_scan;
  pcl::fromROSMsg(*msg, filtered_scan);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));

  if (!log_fp)
    log_fp = fopen("/tmp/ndt_log", "w");

  int j = 0;
  for (int i = 0; i < (int)filtered_scan_ptr->points.size(); i++)
  {
    scan_points[j].x = filtered_scan_ptr->points[i].x + nrand(0.01);
    scan_points[j].y = filtered_scan_ptr->points[i].y + nrand(0.01);
    scan_points[j].z = filtered_scan_ptr->points[i].z + nrand(0.01);
    double dist =
        scan_points[j].x * scan_points[j].x + scan_points[j].y * scan_points[j].y + scan_points[j].z * scan_points[j].z;
    if (dist < 3 * 3)
      continue;

    j++;
    if (j > 130000)
      break;
  }
  scan_points_num = j;

  /*--matching---*/
  // calc offset
  x_offset = prev_pose.x - prev_pose2.x;
  y_offset = prev_pose.y - prev_pose2.y;
  z_offset = prev_pose.z - prev_pose2.z;
  theta_offset = prev_pose.theta3 - prev_pose2.theta3;

  if (theta_offset < -M_PI)
    theta_offset += 2 * M_PI;
  if (theta_offset > M_PI)
    theta_offset -= 2 * M_PI;

  // calc estimated initial position
  pose.x = prev_pose.x + x_offset;
  pose.y = prev_pose.y + y_offset;
  pose.z = prev_pose.z + z_offset;
  pose.theta = prev_pose.theta;
  pose.theta2 = prev_pose.theta2;
  pose.theta3 = prev_pose.theta3 + theta_offset;

  // matching
  for (layer_select = 1; layer_select >= 1; layer_select -= 1)
  {
    for (j = 0; j < 100; j++)
    {
      if (layer_select != 1 && j > 2)
      {
        break;
      }
      bpose = pose;

      e = adjust3d(scan_points, scan_points_num, &pose, layer_select);

      pose_mod(&pose);

      if ((bpose.x - pose.x) * (bpose.x - pose.x) + (bpose.y - pose.y) * (bpose.y - pose.y) +
              (bpose.z - pose.z) * (bpose.z - pose.z) + 3 * (bpose.theta - pose.theta) * (bpose.theta - pose.theta) +
              3 * (bpose.theta2 - pose.theta2) * (bpose.theta2 - pose.theta2) +
              3 * (bpose.theta3 - pose.theta3) * (bpose.theta3 - pose.theta3) <
          0.00001)
      {
        break;
      }
    }
    iteration = j;

    /*gps resetting*/
    if (g_use_gnss)
    {
      static FILE *e_fp;
      if (!e_fp)
      {
        e_fp = fopen("/tmp/e_log", "w");
      }
      fprintf(e_fp, "%f\n", e);
      if (layer_select == 1 && e < 1000)
      {
        printf("reset\n");
        tf::StampedTransform gps_tf_on_world;
        try
        {
          listener.lookupTransform("world", "gps", ros::Time(0), gps_tf_on_world);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s", ex.what());
          return;
        }
        printf("set initial position by gps posiotion\n");
        pose.x = gps_tf_on_world.getOrigin().x();
        pose.y = gps_tf_on_world.getOrigin().y();
        pose.z = gps_tf_on_world.getOrigin().z() + nrand(5);
        tf::Quaternion q(gps_tf_on_world.getRotation().x(), gps_tf_on_world.getRotation().y(),
                         gps_tf_on_world.getRotation().z(), gps_tf_on_world.getRotation().w());
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        pose.theta = roll;
        pose.theta2 = pitch + 0.22;
        pose.theta3 = yaw + nrand(0.7);
        prev_pose2 = prev_pose = pose;
        printf("reset %f %f %f %f %f %f\n", pose.x, pose.y, pose.z, pose.theta, pose.theta2, pose.theta3);

        return;
      }
    }

    // unti-distotion
    if (layer_select == 2)
    {
      double rate, xrate, yrate, dx, dy, dtheta;
      double tempx, tempy;
      int i;

      tempx = (pose.x - prev_pose.x);
      tempy = (pose.y - prev_pose.y);
      dx = tempx * cos(-prev_pose.theta3) - tempy * sin(-prev_pose.theta3);
      dy = tempx * sin(-prev_pose.theta3) + tempy * cos(-prev_pose.theta3);
      dtheta = pose.theta3 - prev_pose.theta3;
      if (dtheta < -M_PI)
      {
        dtheta += 2 * M_PI;
      }
      if (dtheta > M_PI)
      {
        dtheta -= 2 * M_PI;
      }

      rate = dtheta / (double)scan_points_num;
      xrate = dx / (double)scan_points_num;
      yrate = dy / (double)scan_points_num;

      printf("untidist x %f y %f yaw %f\n", dx, dy, dtheta);

      dx = -dx;
      dy = -dy;
      dtheta = -dtheta;
      for (i = 0; i < scan_points_num; i++)
      {
        tempx = scan_points[i].x * cos(dtheta) - scan_points[i].y * sin(dtheta) + dx;
        tempy = scan_points[i].x * sin(dtheta) + scan_points[i].y * cos(dtheta) + dy;

        scan_points[i].x = tempx;
        scan_points[i].y = tempy;

        dtheta += rate;
        dx += xrate;
        dy += yrate;
      }
    }
  }

  // localizer
  Eigen::Translation3f translation(pose.x, pose.y, pose.z);
  Eigen::AngleAxisf rotation_x(pose.theta, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rotation_y(pose.theta2, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rotation_z(pose.theta3, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f local_t = (translation * rotation_z * rotation_y * rotation_x).matrix();
  Eigen::Matrix4f global_t = tf_local_to_global * local_t;

  tf::Matrix3x3 mat_l;
  mat_l.setValue(
      static_cast<double>(global_t(0, 0)), static_cast<double>(global_t(0, 1)), static_cast<double>(global_t(0, 2)),
      static_cast<double>(global_t(1, 0)), static_cast<double>(global_t(1, 1)), static_cast<double>(global_t(1, 2)),
      static_cast<double>(global_t(2, 0)), static_cast<double>(global_t(2, 1)), static_cast<double>(global_t(2, 2)));

  mat_l.getRotation(localizer_q);
  localizer_pose_msg.header.frame_id = "/map";
  localizer_pose_msg.header.stamp = current_scan_time;
  localizer_pose_msg.pose.position.x = global_t(0, 3);
  localizer_pose_msg.pose.position.y = global_t(1, 3);
  localizer_pose_msg.pose.position.z = global_t(2, 3);
  localizer_pose_msg.pose.orientation.x = localizer_q.x();
  localizer_pose_msg.pose.orientation.y = localizer_q.y();
  localizer_pose_msg.pose.orientation.z = localizer_q.z();
  localizer_pose_msg.pose.orientation.w = localizer_q.w();

  // base_link
  Eigen::Matrix4f global_t2 = global_t * tf_ltob;
  tf::Matrix3x3 mat_b;  // base_link
  mat_b.setValue(
      static_cast<double>(global_t2(0, 0)), static_cast<double>(global_t2(0, 1)), static_cast<double>(global_t2(0, 2)),
      static_cast<double>(global_t2(1, 0)), static_cast<double>(global_t2(1, 1)), static_cast<double>(global_t2(1, 2)),
      static_cast<double>(global_t2(2, 0)), static_cast<double>(global_t2(2, 1)), static_cast<double>(global_t2(2, 2)));
  mat_b.getRotation(ndt_q);

  ndt_pose_msg.header.frame_id = "/map";
  ndt_pose_msg.header.stamp = current_scan_time;
  ndt_pose_msg.pose.position.x = global_t2(0, 3);
  ndt_pose_msg.pose.position.y = global_t2(1, 3);
  ndt_pose_msg.pose.position.z = global_t2(2, 3);
  ndt_pose_msg.pose.orientation.x = ndt_q.x();
  ndt_pose_msg.pose.orientation.y = ndt_q.y();
  ndt_pose_msg.pose.orientation.z = ndt_q.z();
  ndt_pose_msg.pose.orientation.w = ndt_q.w();

  localizer_pose_pub.publish(localizer_pose_msg);
  ndt_pose_pub.publish(ndt_pose_msg);

  scan_transrate(scan_points, map_points, &pose, scan_points_num);

  prev_pose2 = prev_pose;
  prev_pose = pose;

  if (is_first_time)
  {
    prev_pose2 = prev_pose;
    is_first_time = 0;
  }

  fprintf(log_fp, "%f %f %f %f %f %f %f\n", current_scan_time.toSec(),
          pose.x * cos(g_map_rotation) - pose.y * sin(g_map_rotation) + g_map_center_x,
          pose.x * sin(g_map_rotation) + pose.y * cos(g_map_rotation) + g_map_center_y, pose.z + g_map_center_z,
          pose.theta, pose.theta2, pose.theta3 + g_map_rotation);

  fflush(log_fp);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(global_t2(0, 3), global_t2(1, 3), global_t2(2, 3)));
  transform.setRotation(ndt_q);

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  matching_end = std::chrono::system_clock::now();
  exe_time = std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() / 1000.0;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << msg->header.seq << std::endl;
  std::cout << "Number of filtered scan points: " << scan_points_num << " points." << std::endl;
  std::cout << "Number of iteration: " << iteration << std::endl;
  std::cout << "Execution time: " << exe_time << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << pose.x << ", " << pose.y << ", " << pose.z << ", " << pose.theta << ", " << pose.theta2 << ", "
            << pose.theta3 << ")" << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

int main(int argc, char *argv[])
{
  std::cout << "3D NDT scan matching" << std::endl;

  ros::init(argc, argv, "ndt_matching_tku");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("init_x", g_ini_x);
  private_nh.getParam("init_y", g_ini_y);
  private_nh.getParam("init_z", g_ini_z);
  private_nh.getParam("init_roll", g_ini_roll);
  private_nh.getParam("init_pitch", g_ini_pitch);
  private_nh.getParam("init_yaw", g_ini_yaw);
  private_nh.getParam("use_gnss", g_use_gnss);

  if (!nh.getParam("tf_x", _tf_x))
  {
    std::cout << "tf_x is not set." << std::endl;
    return 1;
  }
  if (!nh.getParam("tf_y", _tf_y))
  {
    std::cout << "tf_y is not set." << std::endl;
    return 1;
  }
  if (!nh.getParam("tf_z", _tf_z))
  {
    std::cout << "tf_z is not set." << std::endl;
    return 1;
  }
  if (!nh.getParam("tf_roll", _tf_roll))
  {
    std::cout << "tf_roll is not set." << std::endl;
    return 1;
  }
  if (!nh.getParam("tf_pitch", _tf_pitch))
  {
    std::cout << "tf_pitch is not set." << std::endl;
    return 1;
  }
  if (!nh.getParam("tf_yaw", _tf_yaw))
  {
    std::cout << "tf_yaw is not set." << std::endl;
    return 1;
  }

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  Eigen::Translation3f tl_ltob((-1.0) * _tf_x, (-1.0) * _tf_y, (-1.0) * _tf_z);  // tl: translation
  Eigen::AngleAxisf rot_x_ltob((-1.0) * _tf_roll, Eigen::Vector3f::UnitX());     // rot: rotation
  Eigen::AngleAxisf rot_y_ltob((-1.0) * _tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_ltob((-1.0) * _tf_yaw, Eigen::Vector3f::UnitZ());
  tf_ltob = (tl_ltob * rot_z_ltob * rot_y_ltob * rot_x_ltob).matrix();

  // map path
  sprintf(g_ndmap_name, "%s", "ndmap");

  // map size
  g_map_x = G_MAP_X;
  g_map_y = G_MAP_Y;
  g_map_z = G_MAP_Z;
  g_map_cellsize = G_MAP_CELLSIZE;
  // map center
  g_map_center_x = g_ini_x;
  g_map_center_y = g_ini_y;
  g_map_center_z = g_ini_z;
  g_map_rotation = 0.0;

  Eigen::Translation3f tl_local_to_global(g_map_center_x, g_map_center_y, g_map_center_z);  // tl: translation
  Eigen::AngleAxisf rot_x_local_to_global(0.0, Eigen::Vector3f::UnitX());                   // rot: rotation
  Eigen::AngleAxisf rot_y_local_to_global(0.0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_local_to_global(g_map_rotation, Eigen::Vector3f::UnitZ());
  q_local_to_global.setRPY(0.0, 0.0, g_map_rotation);
  tf_local_to_global =
      (tl_local_to_global * rot_z_local_to_global * rot_y_local_to_global * rot_x_local_to_global).matrix();

  Eigen::Translation3f tl_global_to_local((-1.0) * g_map_center_x, (-1.0) * g_map_center_y,
                                          (-1.0) * g_map_center_z);      // tl: translation
  Eigen::AngleAxisf rot_x_global_to_local(0, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_global_to_local(0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_global_to_local((-1.0) * g_map_rotation, Eigen::Vector3f::UnitZ());
  q_global_to_local.setRPY(0.0, 0.0, (-1.0) * g_map_rotation);
  tf_global_to_local =
      (tl_global_to_local * rot_z_global_to_local * rot_y_global_to_local * rot_x_global_to_local).matrix();

  /*initialize(clear) NDmap data*/
  NDmap = initialize_NDmap();

  // load map
  prev_pose.x = (g_ini_x - g_map_center_x) * cos(-g_map_rotation) - (g_ini_y - g_map_center_y) * sin(-g_map_rotation);
  prev_pose.y = (g_ini_x - g_map_center_x) * sin(-g_map_rotation) + (g_ini_y - g_map_center_y) * cos(-g_map_rotation);
  prev_pose.z = g_ini_z - g_map_center_z;
  prev_pose.theta = g_ini_roll;
  prev_pose.theta2 = g_ini_pitch;
  prev_pose.theta3 = g_ini_yaw - g_map_rotation;

  prev_pose2 = prev_pose;
  is_first_time = 1;

  ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1000);
  localizer_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 1000);

  ros::Subscriber map_sub = nh.subscribe("points_map", 10, map_callback);
  ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 1000, initialpose_callback);
  ros::Subscriber points_sub = nh.subscribe("filtered_points", 1000, points_callback);

  ros::spin();

  return 1;
}
