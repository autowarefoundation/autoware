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

#define XSTR(x) #x
#define STR(x) XSTR(x)

using namespace std;

typedef struct _OBJPOS{
  int x1;
  int y1;
  int x2;
  int y2;
  float distance;
}OBJPOS;

//for timestamp
/*
struct my_tm {
  time_t tim; // yyyymmddhhmmss
  long msec;  // milli sec
};
*/

static objLocation ol;

//store subscribed value
static vector<OBJPOS> global_cp_vector;
//vector<OBJPOS> global_pp_vector;

//flag for comfirming whether updating position or not
static bool gnssGetFlag;
static bool ndtGetFlag;
static bool ready_;

//store own position and direction now.updated by position_getter
static LOCATION gnss_loc;
static LOCATION ndt_loc;
static ANGLE gnss_angle;
static ANGLE ndt_angle;

static double cameraMatrix[4][4] = {
  {-7.8577658642752374e-03, -6.2035361880992401e-02,9.9804301981022692e-01, 5.1542126095196206e-01},
  {-9.9821250329813849e-01, 5.9620033356180935e-02,-4.1532977104442731e-03, -2.9214878315161133e-02},
  {-5.9245706805522491e-02, -9.9629165684497312e-01,-6.2392954139163306e-02, -6.6728858508628075e-01},
  {0, 0, 0, 1}
};

static ros::Publisher pub;

static std::string object_type;

#ifdef NEVER // XXX No one calls this functions
static void printDiff(struct timeval begin, struct timeval end){
  long diff;
  diff = (end.tv_sec - begin.tv_sec)*1000*1000 + (end.tv_usec - begin.tv_usec);
  printf("Diff: %ld us (%ld ms)\n",diff,diff/1000);
}
#endif

static void projection_callback(const calibration_camera_lidar::projection_matrix& msg)
{
	for (int row=0; row<4; row++) {
		for (int col=0; col<4; col++) {
			cameraMatrix[row][col] = msg.projection_matrix[row * 4 + col];
		}
	}
	ready_ = true;
}

void GetRPY(const geometry_msgs::Pose &pose,
	    double &roll,
	    double &pitch,
	    double &yaw){
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation,q);
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);

  //reverse angle value
  roll = -roll;
  pitch = -pitch;
  yaw = -yaw;
}

void makeSendDataDetectedObj(vector<OBJPOS> car_position_vector,
                             vector<OBJPOS>::iterator cp_iterator,
                             LOCATION mloc,
                             ANGLE angle,
                             cv_tracker::obj_label& send_data)
{
  LOCATION rescoord;
  geometry_msgs::Point tmpPoint;

  for(uint i=0; i<car_position_vector.size() ; i++, cp_iterator++){

    //middle of right-lower and left-upper
    double U = cp_iterator->x1 + cp_iterator->x2/2;
    double V = cp_iterator->y1 + cp_iterator->y2/2;

    //convert
    ol.setOriginalValue(U,V,cp_iterator->distance);
    LOCATION ress = ol.cal();
    //printf("coordinate from own:%f,%f,%f\n",ress.X,ress.Y,ress.Z);

    axiMove am;
    //convert axes from camera to velodyne
    LOCATION velocoordinate = am.cal(ress,cameraMatrix);

    //convert axes to north direction 0 angle.
    LOCATION anglefixed = am.cal(velocoordinate,angle);

    /*
      rectangular coordinate is that axial x is the direction to left and right,
      axial y is the direction to front and backend and axial z is the direction to upper and lower.
      So convert them.
     */
    rescoord.X = anglefixed.X;
    rescoord.Y = anglefixed.Y;
    rescoord.Z = anglefixed.Z;

    //add plane rectangular coordinate to that of target object.
    rescoord.X += mloc.X;
    rescoord.Y += mloc.Y;
    rescoord.Z += mloc.Z;

    /* Set the position of this object */
    tmpPoint.x = rescoord.X;
    tmpPoint.y = rescoord.Y;
    tmpPoint.z = rescoord.Z;

    send_data.reprojected_pos.push_back(tmpPoint);
  }
}

//wrap SendData class
void locatePublisher(vector<OBJPOS> car_position_vector){
  //get values from sample_corner_point , convert latitude and longitude,
  //and send database server.
  
  //  geometry_msgs::PoseArray pose_msg;
  cv_tracker::obj_label obj_label_msg;

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
 //     if(pose_msg.poses.size() != 0){
        // pose_msg.header.stamp = ros::Time::now();
        // pose_msg.header.frame_id = "map";
  obj_label_msg.type = object_type;
  pub.publish(obj_label_msg);
   //   }
}

static void obj_pos_xyzCallback(const cv_tracker::image_obj_tracked& fused_objects)
{
	if (!ready_)
		return;

  vector<OBJPOS> cp_vector;
  OBJPOS cp;
  
  object_type = fused_objects.type;
  //If angle and position data is not updated from prevous data send,
  //data is not sent
  if(gnssGetFlag || ndtGetFlag) {
    for (unsigned int i = 0; i < fused_objects.rect_ranged.size(); i++){
      
      //If distance is zero, we cannot calculate position of recognized object
      //so skip loop
      if(fused_objects.rect_ranged.at(i).range <= 0) continue;
      
      cp.x1 = fused_objects.rect_ranged.at(i).rect.x;//x-axis of the upper left
      cp.y1 = fused_objects.rect_ranged.at(i).rect.y;//x-axis of the lower left
      cp.x2 = fused_objects.rect_ranged.at(i).rect.width;//x-axis of the upper right
      cp.y2 = fused_objects.rect_ranged.at(i).rect.height;//x-axis of the lower left
      
      cp.distance = fused_objects.rect_ranged.at(i).range;
      
      //printf("\ncar : %d,%d,%d,%d,%f\n",cp.x1,cp.y1,cp.x2,cp.y2,cp.distance);
      
      cp_vector.push_back(cp);      
    }
    
    locatePublisher(cp_vector);
    
  }
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
  ndt_loc.X = pose.pose.position.x;
  ndt_loc.Y = pose.pose.position.y;
  ndt_loc.Z = pose.pose.position.z;

  GetRPY(pose.pose,ndt_angle.thiX,ndt_angle.thiY,ndt_angle.thiZ);
  printf("quaternion angle : %f\n",ndt_angle.thiZ*180/M_PI);
  printf("location : %f %f %f\n",ndt_loc.X,ndt_loc.Y,ndt_loc.Z);

  ndtGetFlag = true;
  //printf("my position : %f %f %f\n",my_loc.X,my_loc.Y,my_loc.Z);
}

int main(int argc, char **argv){
  
  ros::init(argc ,argv, "obj_reproj") ;  
  cout << "obj_reproj" << endl;

  ready_ = false;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  ros::Subscriber obj_pos_xyz = n.subscribe("image_obj_tracked", 1, obj_pos_xyzCallback);
  //ros::Subscriber pedestrian_pos_xyz = n.subscribe("/pedestrian_pixel_xyz", 1, pedestrian_pos_xyzCallback);

  /*
  ros::Subscriber azm = n.subscribe("/vel", 1, azimuth_getter);
  ros::Subscriber my_pos = n.subscribe("/fix", 1, position_getter);
  ros::Subscriber ndt = n.subscribe("/current_pose", 1, position_getter_ndt);
  */
  //ros::Subscriber gnss_pose = n.subscribe("/gnss_pose", 1, position_getter_gnss);
  ros::Subscriber ndt_pose = n.subscribe("/current_pose", 1, position_getter_ndt);
  pub = n.advertise<cv_tracker::obj_label>("obj_label",1); 

  ros::Subscriber projection = n.subscribe("/projection_matrix", 1, projection_callback);

  /*
  //read calibration value
  //TO DO : subscribe from topic
  cv::Mat Cintrinsic;
  std::string camera_yaml;

  n.param<std::string>("/scan2image/camera_yaml", camera_yaml,STR(CAMERA_YAML));

  cv::FileStorage camera_file(camera_yaml.c_str(), cv::FileStorage::READ); 
  if(!camera_file.isOpened()){
    fprintf(stderr,"%s, : cannot open file\n",camera_yaml.c_str());
    exit(EXIT_FAILURE);
  }
  camera_file["intrinsic"] >> Cintrinsic; 
  camera_file.release(); 

  double fkx = Cintrinsic.at<float>(0,0);
  double fky = Cintrinsic.at<float>(1,1);
  double Ox = Cintrinsic.at<float>(0,2);
  double Oy = Cintrinsic.at<float>(1,2);
  */

  double fkx = 1360.260477;
  double fky = 1360.426247;
  double Ox = 440.017336;
  double Oy = 335.274106;

/*  std::string lidar_3d_yaml = "";

  if (private_nh.getParam("lidar_3d_yaml", lidar_3d_yaml) == false) {
      std::cerr << "error! usage : rosrun  cv_tracker obj_reproj _lidar_3d_yaml:=[file]" << std::endl;
      exit(-1);
  }

  cv::FileStorage lidar_3d_file(lidar_3d_yaml.c_str(), cv::FileStorage::READ); 
  if(!lidar_3d_file.isOpened()){
    fprintf(stderr,"%s, : cannot open file\n",lidar_3d_yaml.c_str());
    exit(EXIT_FAILURE);
  }
  lidar_3d_file["CameraExtrinsicMat"] >> Lintrinsic; 
  lidar_3d_file.release(); 
*/



  /*
  double fkx = 5.83199829e+02;
  double fky = 3.74826355e+02;
  double Ox =  5.83989319e+02;
  double Oy = 2.41745468e+02;
  */

  ol.setCameraParam(fkx,fky,Ox,Oy);

  //set angle and position flag : false at first
  gnssGetFlag = false;
  ndtGetFlag = false;

  ros::spin();

  return 0;
}
