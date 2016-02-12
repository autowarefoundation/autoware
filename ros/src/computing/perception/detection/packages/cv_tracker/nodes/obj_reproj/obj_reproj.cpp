/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <std_msgs/String.h>
#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <boost/array.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <sys/time.h>
#include <bitset>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <scan2image/ScanImage.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_tracker/image_obj_tracked.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>
#include "structure.h"
#include "calcoordinates.h"
#include "axialMove.h"
#include "geo_pos_conv.hh"
#include "CalObjLoc.h"
#include "cv_tracker/obj_label.h"
#include "calibration_camera_lidar/projection_matrix.h"
#include <sensor_msgs/CameraInfo.h>
#include <mutex>

#define XSTR(x) #x
#define STR(x) XSTR(x)

using namespace std;

static constexpr double LOOP_RATE = 15.0;

typedef struct _OBJPOS{
  int x1;
  int y1;
  int x2;
  int y2;
  float distance;
}OBJPOS;

static objLocation ol;

//store subscribed value
static vector<OBJPOS> global_cp_vector;

//mutex to handle global-scope objects
static std::mutex mtx_cp_vector;
static std::mutex mtx_flag_obj_pos_xyz;
static std::mutex mtx_flag_ndt_pose;
#define LOCK(mtx) (mtx).lock()
#define UNLOCK(mtx) (mtx).unlock()

//flag for comfirming whether updating position or not
static bool gnssGetFlag;
static bool ndtGetFlag;
static bool ready_;

//store own position and direction now.updated by position_getter
static LOCATION gnss_loc;
static LOCATION ndt_loc;
static ANGLE gnss_angle;
static ANGLE ndt_angle;

//flag for comfirming whether multiple topics are received
static bool isReady_obj_pos_xyz;
static bool isReady_ndt_pose;

static double cameraMatrix[4][4] = {
  {-7.8577658642752374e-03, -6.2035361880992401e-02,9.9804301981022692e-01, 5.1542126095196206e-01},
  {-9.9821250329813849e-01, 5.9620033356180935e-02,-4.1532977104442731e-03, -2.9214878315161133e-02},
  {-5.9245706805522491e-02, -9.9629165684497312e-01,-6.2392954139163306e-02, -6.6728858508628075e-01},
  {0, 0, 0, 1}
};

static ros::Publisher pub;
static ros::Publisher marker_pub;

static std::string object_type;
static ros::Time image_obj_tracked_time;
static ros::Time current_pose_time;

//coordinate system conversion between camera coordinate and map coordinate
static tf::StampedTransform transformCam2Map;

static visualization_msgs::MarkerArray convert_marker_array(const cv_tracker::obj_label& src)
{
  visualization_msgs::MarkerArray ret;
  int index = 0;
  std_msgs::ColorRGBA color_red;
  color_red.r = 1.0f;
  color_red.g = 0.0f;
  color_red.b = 0.0f;
  color_red.a = 1.0f;

  std_msgs::ColorRGBA color_blue;
  color_blue.r = 0.0f;
  color_blue.g = 0.0f;
  color_blue.b = 1.0f;
  color_blue.a = 1.0f;

  std_msgs::ColorRGBA color_green;
  color_green.r = 0.0f;
  color_green.g = 1.0f;
  color_green.b = 0.0f;
  color_green.a = 1.0f;

  for (const auto& reproj_pos : src.reprojected_pos)
    {
      visualization_msgs::Marker marker;
      /* Set frame ID */
      marker.header.frame_id = "map";

      /* Set namespace adn id for this marker */
      marker.ns = object_type;
      marker.id = index;
      index++;

      /* Set marker shape */
      marker.type = visualization_msgs::Marker::SPHERE;

      /* set pose of marker  */
      marker.pose.position = reproj_pos;

      /* set scale of marker */
      marker.scale.x = (double)1.5;
      marker.scale.y = (double)1.5;
      marker.scale.z = (double)1.5;

      /* set color */
      if (object_type == "car") {
        marker.color = color_blue;
      }
      else if (object_type == "person") {
        marker.color = color_green;
      }
      else {
        marker.color = color_red;
      }

      marker.lifetime = ros::Duration(0.3);

      ret.markers.push_back(marker);
    }

  return ret;
}

static void projection_callback(const calibration_camera_lidar::projection_matrix& msg)
{
  for (int row=0; row<4; row++) {
    for (int col=0; col<4; col++) {
      cameraMatrix[row][col] = msg.projection_matrix[row * 4 + col];
    }
  }
  ready_ = true;
}

static void camera_info_callback(const sensor_msgs::CameraInfo& msg)
{
  double fkx = msg.K[0 * 3 + 0]; // get K[0][0]
  double fky = msg.K[1 * 3 + 1]; // get K[1][1]
  double Ox  = msg.K[0 * 3 + 2]; // get K[0][2]
  double Oy  = msg.K[1 * 3 + 2]; // get K[1][2]
  ol.setCameraParam(fkx,fky,Ox,Oy);
}

void GetRPY(const geometry_msgs::Pose &pose,
	    double &roll,
	    double &pitch,
	    double &yaw){
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation,q);
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);

  //reverse angle value
  roll  = -roll;
  pitch = -pitch;
  yaw   = -yaw;
}

void makeSendDataDetectedObj(vector<OBJPOS> car_position_vector,
                             vector<OBJPOS>::iterator cp_iterator,
                             LOCATION mloc,
                             ANGLE angle,
                             cv_tracker::obj_label& send_data)
{
  geometry_msgs::Point tmpPoint;

  for(uint i=0; i<car_position_vector.size() ; i++, cp_iterator++){

    //middle of right-lower and left-upper
    double U = cp_iterator->x1 + cp_iterator->x2/2;
    double V = cp_iterator->y1 + cp_iterator->y2/2;

    //convert from "image" coordinate system to "camera" coordinate system
    ol.setOriginalValue(U,V,cp_iterator->distance);
    LOCATION ress = ol.cal();

    /* convert from "camera" coordinate system to "map" coordinate system */
    tf::Vector3 pos_in_camera_coord(ress.X, ress.Y, ress.Z);
    tf::Vector3 converted = transformCam2Map * pos_in_camera_coord;

    tmpPoint.x = converted.x();
    tmpPoint.y = converted.y();
    tmpPoint.z = converted.z();

    send_data.reprojected_pos.push_back(tmpPoint);
  }
}

//wrap SendData class
void locatePublisher(void){

  vector<OBJPOS> car_position_vector;
  LOCK(mtx_cp_vector);
  copy(global_cp_vector.begin(), global_cp_vector.end(), back_inserter(car_position_vector));
  UNLOCK(mtx_cp_vector);

  //get values from sample_corner_point , convert latitude and longitude,
  //and send database server.

  cv_tracker::obj_label obj_label_msg;
  visualization_msgs::MarkerArray obj_label_marker_msgs;

  vector<OBJPOS>::iterator cp_iterator;
  LOCATION mloc;
  ANGLE mang;

  cp_iterator = car_position_vector.begin();

  //calculate own coordinate from own lati and longi value
  //get my position now
  if(ndtGetFlag){
    mloc = ndt_loc;
    mang = ndt_angle;
  }else{
    mloc = gnss_loc;
    mang = gnss_angle;
  }
  gnssGetFlag = false;
  ndtGetFlag = false;

  //If position is over range,skip loop
  if((!(mloc.X > 180.0 && mloc.X < -180.0 ) ||
      (mloc.Y > 180.0 && mloc.Y < -180.0 ) ||
      mloc.Z < 0.0) ){

    //get data of car and pedestrian recognizing
    if(!car_position_vector.empty()){
      makeSendDataDetectedObj(car_position_vector,cp_iterator,mloc,mang,obj_label_msg);
    }
  }
  //publish recognized car data
  obj_label_msg.type = object_type;
  obj_label_marker_msgs = convert_marker_array(obj_label_msg);
  /* Extraordinary correspondence because of wrong timestamp(current_pose)
   * if a timestamp of current_pose is modified, this comment out should be removed
   */
//  if(image_obj_tracked_time.sec == current_pose.sec && image_obj_tracked_time.nsec == current_pose.nsec) {
    obj_label_msg.header.stamp = image_obj_tracked_time;
//  }

  pub.publish(obj_label_msg);
  marker_pub.publish(obj_label_marker_msgs);
}

static void obj_pos_xyzCallback(const cv_tracker::image_obj_tracked& fused_objects)
{
  if (!ready_)
    return;
  image_obj_tracked_time = fused_objects.header.stamp;

  LOCK(mtx_cp_vector);
  global_cp_vector.clear();
  UNLOCK(mtx_cp_vector);

  OBJPOS cp;

  object_type = fused_objects.type;
  //If angle and position data is not updated from prevous data send,
  //data is not sent
  //  if(gnssGetFlag || ndtGetFlag) {
    LOCK(mtx_cp_vector);
    for (unsigned int i = 0; i < fused_objects.rect_ranged.size(); i++){

      //If distance is zero, we cannot calculate position of recognized object
      //so skip loop
      if(fused_objects.rect_ranged.at(i).range <= 0) continue;

      cp.x1 = fused_objects.rect_ranged.at(i).rect.x;      // x-axis of the upper left
      cp.y1 = fused_objects.rect_ranged.at(i).rect.y;      // y-axis of the upper left
      cp.x2 = fused_objects.rect_ranged.at(i).rect.width;  // width of detection rectangle
      cp.y2 = fused_objects.rect_ranged.at(i).rect.height; // height of detection rectangle

      /*
        As cameraMatrix[0][3] is offset from lidar to camera,
        this cp.distance is z-axis value of detected object in camera coordinate system.
        (As received distance is in [cm] unit, I convert unit from [cm] to [mm] here)
      */
      cp.distance = (fused_objects.rect_ranged.at(i).range - cameraMatrix[0][3]) * 10;

      global_cp_vector.push_back(cp);
    }
    UNLOCK(mtx_cp_vector);

    //Confirm that obj_pos_xyz is subscribed
    LOCK(mtx_flag_obj_pos_xyz);
    isReady_obj_pos_xyz = true;
    UNLOCK(mtx_flag_obj_pos_xyz);

    if (isReady_obj_pos_xyz && isReady_ndt_pose) {
      locatePublisher();

      LOCK(mtx_flag_obj_pos_xyz);
      isReady_obj_pos_xyz = false;
      UNLOCK(mtx_flag_obj_pos_xyz);

      LOCK(mtx_flag_ndt_pose);
      isReady_ndt_pose    = false;
      UNLOCK(mtx_flag_ndt_pose);
    }
    //  }
}

#ifdef NEVER // XXX No one calls this functions. caller is comment out
static void position_getter_gnss(const geometry_msgs::PoseStamped &pose){
  //In Autoware axel x and axel y is opposite
  //but once they is converted to calculate.
  gnss_loc.X = pose.pose.position.x;
  gnss_loc.Y = pose.pose.position.y;
  gnss_loc.Z = pose.pose.position.z;

  GetRPY(pose.pose,gnss_angle.thiX,gnss_angle.thiY,gnss_angle.thiZ);
  printf("quaternion angle : %f\n",gnss_angle.thiZ*180/M_PI);

  gnssGetFlag = true;
  //printf("my position : %f %f %f\n",my_loc.X,my_loc.Y,my_loc.Z);
}
#endif

static void position_getter_ndt(const geometry_msgs::PoseStamped &pose){
  //In Autoware axel x and axel y is opposite
  //but once they is converted to calculate.
  current_pose_time = pose.header.stamp;
  ndt_loc.X = pose.pose.position.x;
  ndt_loc.Y = pose.pose.position.y;
  ndt_loc.Z = pose.pose.position.z;

  GetRPY(pose.pose,ndt_angle.thiX,ndt_angle.thiY,ndt_angle.thiZ);
  printf("quaternion angle : %f\n",ndt_angle.thiZ*180/M_PI);
  printf("location : %f %f %f\n",ndt_loc.X,ndt_loc.Y,ndt_loc.Z);

  ndtGetFlag = true;

  //Confirm ndt_pose is subscribed
  LOCK(mtx_flag_ndt_pose);
  isReady_ndt_pose = true;
  UNLOCK(mtx_flag_ndt_pose);

    if (isReady_obj_pos_xyz && isReady_ndt_pose) {
      locatePublisher();

      LOCK(mtx_flag_obj_pos_xyz);
      isReady_obj_pos_xyz = false;
      UNLOCK(mtx_flag_obj_pos_xyz);

      LOCK(mtx_flag_ndt_pose);
      isReady_ndt_pose    = false;
      UNLOCK(mtx_flag_ndt_pose);
    }
}

int main(int argc, char **argv){

  ros::init(argc ,argv, "obj_reproj") ;
  cout << "obj_reproj" << endl;

  ready_ = false;

  isReady_obj_pos_xyz = false;
  isReady_ndt_pose    = false;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  ros::Subscriber obj_pos_xyz = n.subscribe("image_obj_tracked", 1, obj_pos_xyzCallback);

  ros::Subscriber ndt_pose = n.subscribe("/current_pose", 1, position_getter_ndt);
  pub = n.advertise<cv_tracker::obj_label>("obj_label",1);
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("obj_label_marker", 1);

  ros::Subscriber projection = n.subscribe("/projection_matrix", 1, projection_callback);
  ros::Subscriber camera_info = n.subscribe("/camera/camera_info", 1, camera_info_callback);

  //set angle and position flag : false at first
  gnssGetFlag = false;
  ndtGetFlag = false;

  tf::TransformListener listener;
  ros::Rate loop_rate(LOOP_RATE);  // Try to loop in "LOOP_RATE" [Hz]
  while(n.ok())
    {
      /* try to get coordinate system conversion from "camera" to "map" */
      try {
        listener.lookupTransform("map", "camera", ros::Time(0), transformCam2Map);
      }
      catch (tf::TransformException ex) {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.1).sleep();
      }

      ros::spinOnce();
      loop_rate.sleep();

    }
  return 0;
}
