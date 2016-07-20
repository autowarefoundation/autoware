/*
 * signals.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: sujiwo
 */


#include <iostream>
#include <ros/ros.h>
#include "Rate.h"
#include "vector_map.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <signal.h>
#include <cstdio>
#include "Math.h"
#include <Eigen/Eigen>
#include "road_wizard/Signals.h"
#include <runtime_manager/adjust_xy.h>

static std::string camera_id_str;

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static int adjust_proj_x = 0;
static int adjust_proj_y = 0;

typedef struct {
  double thiX;
  double thiY;
  double thiZ;
} Angle;

static VectorMap vmap;
static Angle cameraOrientation; // camera orientation = car's orientation

static Eigen::Vector3f position;
static Eigen::Quaternionf orientation;
static  float fx,
  fy,
  imageWidth,
  imageHeight,
  cx,
  cy;
static tf::StampedTransform trf;

#define SignalLampRadius 0.3

/* Callback function to shift projection result */
void adjust_xyCallback (const runtime_manager::adjust_xy::ConstPtr& config_msg)
{
  adjust_proj_x = config_msg->x;
  adjust_proj_y = config_msg->y;
}

void cameraInfoCallback (const sensor_msgs::CameraInfo::ConstPtr camInfoMsg)
{
  fx = static_cast<float>(camInfoMsg->P[0]);
  fy = static_cast<float>(camInfoMsg->P[5]);
  imageWidth = camInfoMsg->width;
  imageHeight = camInfoMsg->height;
  cx = static_cast<float>(camInfoMsg->P[2]);
  cy = static_cast<float>(camInfoMsg->P[6]);
}


/* convert degree value into 0 to 360 range */
static double setDegree0to360(double val)
{
  if (val < 0.0f) {
    return (val + 360.0f);
  }
  else if (360.0f < val) {
    return (val - 360.0f);
  }

  return val;
}


static void get_cameraRollPitchYaw(double* roll,
                                   double* pitch,
                                   double* yaw)
{
  geometry_msgs::Pose cameraPose;
  cameraPose.position.x    = (double)(position.x());
  cameraPose.position.y    = (double)(position.y());
  cameraPose.position.z    = (double)(position.z());
  cameraPose.orientation.x = (double)(orientation.x());
  cameraPose.orientation.y = (double)(orientation.y());
  cameraPose.orientation.z = (double)(orientation.z());
  cameraPose.orientation.w = (double)(orientation.w());

  tf::Quaternion quat;

  tf::quaternionMsgToTF(cameraPose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(*roll, *pitch, *yaw);

  /* convert from radian to degree */
  *roll  = setDegree0to360(*roll  * 180.0f / M_PI);
  *pitch = setDegree0to360(*pitch * 180.0f / M_PI);
  *yaw   = setDegree0to360(*yaw   * 180.0f / M_PI);
}


/*
  check if lower < val < upper
  This function also considers circulation
*/
static bool isRange(const double lower, const double upper, const double val)
{
  if (lower <= upper) {
    if (lower < val && val < upper) {
      return true;
    }
  }
  else {
    if (val < upper || lower < val) {
      return true;
    }
  }

  return false;
}


void getTransform (Eigen::Quaternionf &ori, Point3 &pos)
{
  static tf::TransformListener listener;

  // target_frame    source_frame
  ros::Time now = ros::Time();
  listener.waitForTransform (camera_id_str, "map", now, ros::Duration(10.0));
  listener.lookupTransform (camera_id_str, "map", now, trf);

  tf::Vector3 &p = trf.getOrigin();
  tf::Quaternion o = trf.getRotation();
  pos.x()=p.x(); pos.y()=p.y(); pos.z()=p.z();
  ori.w()=o.w(); ori.x()=o.x(); ori.y()=o.y(); ori.z()=o.z();
}


Point3 transform (const Point3 &psrc, tf::StampedTransform &tfsource)
{
  tf::Vector3 pt3 (psrc.x(), psrc.y(), psrc.z());
  tf::Vector3 pt3s = tfsource * pt3;
  return Point3 (pt3s.x(), pt3s.y(), pt3s.z());
}


/*
 * Project a point from world coordinate to image plane
 */
bool project2 (const Point3 &pt, int &u, int &v, bool useOpenGLCoord=false)
{
  float nearPlane = 1.0;
  float farPlane = 200.0;
  Point3 _pt = transform (pt, trf);
  float _u = _pt.x()*fx/_pt.z() + cx;
  float _v = _pt.y()*fy/_pt.z() + cy;

  u = static_cast<int>(_u);
  v = static_cast<int>(_v);
  if ( u < 0 || imageWidth < u || v < 0 || imageHeight < v || _pt.z() < nearPlane || farPlane < _pt.z() ) {
    u = -1, v = -1;
    return false;
  }

  if (useOpenGLCoord) {
    v = imageHeight - v;
  }

  return true;
}


void echoSignals2 (ros::Publisher &pub, bool useOpenGLCoord=false)
{
  int countPoint = 0;
  road_wizard::Signals signalsInFrame;

  for (unsigned int i=1; i<=vmap.signals.size(); i++) {
    Signal signal = vmap.signals[i];
    int pid = vmap.vectors[signal.vid].pid;

    Point3 signalcenter = vmap.getPoint(pid);
    Point3 signalcenterx (signalcenter.x(), signalcenter.y(), signalcenter.z()+SignalLampRadius);

    int u, v;
    if (project2 (signalcenter, u, v, useOpenGLCoord) == true) {
      countPoint++;
      // std::cout << u << ", " << v << ", " << std::endl;

      int radius;
      int ux, vx;
      project2 (signalcenterx, ux, vx, useOpenGLCoord);
      radius = (int)distance (ux, vx, u, v);

      road_wizard::ExtractedPosition sign;
      sign.signalId = signal.id;

      sign.u = u + adjust_proj_x; // shift project position by configuration value from runtime manager
      sign.v = v + adjust_proj_y; // shift project position by configuration value from runtime manager

      sign.radius = radius;
      sign.x = signalcenter.x(), sign.y = signalcenter.y(), sign.z = signalcenter.z();
      sign.hang = vmap.vectors[signal.vid].hang; // hang is expressed in [0, 360] degree
      sign.type = signal.type, sign.linkId = signal.linkid;
      sign.plId = signal.plid;

      /* convert signal's horizontal angle to yaw */
      double reversed_signalYaw = setDegree0to360(sign.hang + 180.0f);

      get_cameraRollPitchYaw(&cameraOrientation.thiX,
                             &cameraOrientation.thiY,
                             &cameraOrientation.thiZ);

      // std::cout << "signal : " << reversed_signalYaw << ", car : " << cameraOrientation.thiZ << std::endl;

      /*
        check whether this signal is oriented to the camera
        interested signals have below condition orientation:
        (camera_orientation - 70deg) < (signal_orientation + 180deg) < (camera_orientatin + 70deg)
      */
      double conditionRange_lower = setDegree0to360(cameraOrientation.thiZ - 70);
      double conditionRange_upper = setDegree0to360(cameraOrientation.thiZ + 70);

      // std::cout << "lower: " << conditionRange_lower << ", upper: " << conditionRange_upper << std::endl;

      if (isRange(conditionRange_lower, conditionRange_upper, reversed_signalYaw)) {
        signalsInFrame.Signals.push_back (sign);
      }
    }
  }

  signalsInFrame.header.stamp = ros::Time::now();
  pub.publish (signalsInFrame);

  // printf ("There are %d out of %u signals in frame\n", countPoint, static_cast<unsigned int>(vmap.signals.size()));
}


void interrupt (int s)
{
  ros::shutdown();
  exit(1);
}


int main (int argc, char *argv[])
{

  ros::init(argc, argv, "feat_proj", ros::init_options::NoSigintHandler);
  ros::NodeHandle rosnode;
  ros::NodeHandle private_nh("~");
  std::string cameraInfo_topic_name;
  private_nh.param<std::string>("camera_info_topic", cameraInfo_topic_name, "/camera/camera_info");

  /* get camera ID */
  camera_id_str = cameraInfo_topic_name;
  camera_id_str.erase(camera_id_str.find("/camera/camera_info"));
  if (camera_id_str == "/") {
    camera_id_str = "camera";
  }
  
  /* load vector map */
  ros::Subscriber sub_point     = rosnode.subscribe("vector_map_info/point_class",
                                                    SUBSCRIBE_QUEUE_SIZE,
                                                    &VectorMap::load_points,
                                                    &vmap);
  ros::Subscriber sub_line      = rosnode.subscribe("vector_map_info/line_class",
                                                    SUBSCRIBE_QUEUE_SIZE,
                                                    &VectorMap::load_lines,
                                                    &vmap);
  ros::Subscriber sub_lane      = rosnode.subscribe("vector_map_info/lane",
                                                    SUBSCRIBE_QUEUE_SIZE,
                                                    &VectorMap::load_lanes,
                                                    &vmap);
  ros::Subscriber sub_vector    = rosnode.subscribe("vector_map_info/vector_class",
                                                    SUBSCRIBE_QUEUE_SIZE,
                                                    &VectorMap::load_vectors,
                                                    &vmap);
  ros::Subscriber sub_signal    = rosnode.subscribe("vector_map_info/signal",
                                                    SUBSCRIBE_QUEUE_SIZE,
                                                    &VectorMap::load_signals,
                                                    &vmap);
  ros::Subscriber sub_whiteline = rosnode.subscribe("vector_map_info/white_line",
                                                    SUBSCRIBE_QUEUE_SIZE,
                                                    &VectorMap::load_whitelines,
                                                    &vmap);
  ros::Subscriber sub_dtlane    = rosnode.subscribe("vector_map_info/dtlane",
                                                    SUBSCRIBE_QUEUE_SIZE,
                                                    &VectorMap::load_dtlanes,
                                                    &vmap);

  /* wait until loading all vector map is completed */
  ros::Rate wait_rate(1);
  while(vmap.points.empty() || vmap.lines.empty() || vmap.whitelines.empty() ||
        vmap.lanes.empty() || vmap.dtlanes.empty() || vmap.vectors.empty() || vmap.signals.empty())
    {
      ros::spinOnce();
      wait_rate.sleep();
    }

  vmap.loaded = true;
  std::cout << "all vector map loaded." << std::endl;

  ros::Subscriber cameraInfoSubscriber = rosnode.subscribe (cameraInfo_topic_name, 100, cameraInfoCallback);
  ros::Subscriber adjust_xySubscriber  = rosnode.subscribe("/config/adjust_xy", 100, adjust_xyCallback);
  //  ros::Subscriber ndtPoseSubscriber    = rosnode.subscribe("/current_pose", 10, ndtPoseCallback);
  ros::Publisher  signalPublisher      = rosnode.advertise <road_wizard::Signals> ("roi_signal", 100);
  signal (SIGINT, interrupt);

  Rate loop (25);
  while (true) {

    ros::spinOnce();

    try {
      getTransform (orientation, position);
    } catch (tf::TransformException &exc) {
    }

    echoSignals2 (signalPublisher, false);
    loop.sleep();
  }


}
