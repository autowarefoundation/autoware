#include "std_msgs/String.h"
#include "ros/ros.h"

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include "car_detector/FusedObjects.h"
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

#include "opencv/cv.h" 
#include "opencv/highgui.h"
#include "opencv/cxcore.h" 
#include "std_msgs/Float64.h"
#include "scan_to_image/ScanImage.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/NavSatFix.h"
#include "structure.h"
#include "calcoordinates.h"
#include "axialMove.h"
#include "geo_pos_conv.hh"
#include "CalObjLoc.h"
//#include "car_detector/CarPose.h"

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

objLocation ol;

//flag for comfirming whether updating position or not
bool gnssGetFlag;
bool ndtGetFlag;

//store own position and direction now.updated by position_getter
LOCATION gnss_loc;
LOCATION ndt_loc;
ANGLE gnss_angle;
ANGLE ndt_angle;

double cameraMatrix[4][4] = {
  {-7.8577658642752374e-03, -6.2035361880992401e-02,9.9804301981022692e-01, 5.1542126095196206e-01},
  {-9.9821250329813849e-01, 5.9620033356180935e-02,-4.1532977104442731e-03, -2.9214878315161133e-02},
  {-5.9245706805522491e-02, -9.9629165684497312e-01,-6.2392954139163306e-02, -6.6728858508628075e-01},
  {0, 0, 0, 1}
 
};

ros::Publisher pub;

void printDiff(struct timeval begin, struct timeval end){
  long diff;
  diff = (end.tv_sec - begin.tv_sec)*1000*1000 + (end.tv_usec - begin.tv_usec);
  printf("Diff: %ld us (%ld ms)\n",diff,diff/1000);
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

/*
string getNowTime(){
  struct my_tm *qt;
  struct tm *tmp;
  struct timeval tv;
  char tm[25];
  string res;
  ostringstream oss;

  qt=(struct my_tm*)malloc(sizeof(struct my_tm));
  if(qt == NULL)return NULL;
  gettimeofday(&tv,NULL);
  tmp=localtime(&tv.tv_sec);
  qt->tim=mktime(tmp);
  qt->msec=tv.tv_usec/1000;
  sprintf(tm,"%04d-%02d-%02d %02d:%02d:%02d.%d",
    tmp->tm_year + 1900, tmp->tm_mon + 1,
    tmp->tm_mday, tmp->tm_hour,
    tmp->tm_min, tmp->tm_sec,
    tv.tv_usec/1000);
  res = tm;
  return res;
}
*/

void makeSendDataDetectedObj(vector<OBJPOS> pedestrian_position_vector,
			     vector<OBJPOS>::iterator pp_iterator,
			     LOCATION mloc,
			     ANGLE angle,
			     geometry_msgs::PoseArray &pose){

  LOCATION rescoord;
  geometry_msgs::Pose tmpPose;

  for(uint i=0; i<pedestrian_position_vector.size() ; i++, pp_iterator++){

    //middle of right-lower and left-upper
    double U = pp_iterator->x1 + pp_iterator->x2/2;
    double V = pp_iterator->y1 + pp_iterator->y2/2;

    //convert
    ol.setOriginalValue(U,V,pp_iterator->distance);
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

    //add plane rectangular coordinate to that of target car.
    rescoord.X += mloc.X;
    rescoord.Y += mloc.Y;
    rescoord.Z += mloc.Z;

    tmpPose.position.x = rescoord.X;
    tmpPose.position.y = rescoord.Y;
    tmpPose.position.z = rescoord.Z;
    pose.poses.push_back(tmpPose);
  }

}




//wrap SendData class
void locatePublisher(vector<OBJPOS> pedestrian_position_vector){

  //get values from sample_corner_point , convert latitude and longitude,
  //and send database server.
  
  geometry_msgs::PoseArray pose_msg;

  vector<OBJPOS>::iterator pp_iterator;
  LOCATION mloc;
  ANGLE mang;

  pp_iterator = pedestrian_position_vector.begin();

  //calculate own coordinate from own lati and longi value
  //get my position now

  if(ndtGetFlag){
    mloc = ndt_loc;
    mang = ndt_angle;
  }else{
    mloc = gnss_loc;
    mang = gnss_angle;
  }

  
  //If position is over range,skip loop
  if((!(mloc.X > 180.0 && mloc.X < -180.0 ) || 
      (mloc.Y > 180.0 && mloc.Y < -180.0 ) || 
      mloc.Z < 0.0) ){

    //get data of car and pedestrian recognizing
    if(pedestrian_position_vector.size() > 0 ){
      makeSendDataDetectedObj(pedestrian_position_vector,pp_iterator,mloc,mang,pose_msg);
    }

    printf("%f %f %f\n",mloc.X,mloc.Y,mloc.Z);

    //publish recognized object data
    if(pose_msg.poses.size() != 0){
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.frame_id = "map";
      pub.publish(pose_msg);
    }
  }

}


void pedestrian_pos_xyzCallback(const car_detector::FusedObjects& fused_objects)
{

  vector<OBJPOS> pp_vector;
  OBJPOS pp;
  
  //If angle and position data is not updated from prevous data send,
  //data is not sent
  if(gnssGetFlag || ndtGetFlag) {
    gnssGetFlag = false;
    ndtGetFlag = false;
    //認識した車の数だけ繰り返す
    for (int i = 0; i < fused_objects.car_num; i++){
      
      //If distance is zero, we cannot calculate position of recognized object
      //so skip loop
      if(fused_objects.distance.at(i) <= 0) continue;
      
      pp.x1 = fused_objects.corner_point[0+i*4];//x-axis of the upper left
      pp.y1 = fused_objects.corner_point[1+i*4];//x-axis of the lower left
      pp.x2 = fused_objects.corner_point[2+i*4];//x-axis of the upper right
      pp.y2 = fused_objects.corner_point[3+i*4];//x-axis of the lower left
      
      pp.distance = fused_objects.distance.at(i);
      
      //printf("\ncar : %d,%d,%d,%d,%f\n",cp.x1,cp.y1,cp.x2,cp.y2,cp.distance);
      
      pp_vector.push_back(pp);      
    }
    
    locatePublisher(pp_vector);
    
  }
}


/*
void position_getter_ndt(const geometry_msgs::PoseStamped &pose){

  my_loc.X = pose.pose.position.x;
  my_loc.Y = pose.pose.position.y;
  my_loc.Z = pose.pose.position.z;

  GetRPY(pose.pose,angle.thiX,angle.thiY,angle.thiZ);
  printf("quaternion angle : %f\n",angle.thiZ*180/M_PI);

  positionGetFlag = true;
  //printf("my position : %f %f %f\n",my_loc.X,my_loc.Y,my_loc.Z);
}
*/

void position_getter_gnss(const geometry_msgs::PoseStamped &pose){

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

void position_getter_ndt(const geometry_msgs::PoseStamped &pose){

  //In Autoware axel x and axel y is opposite
  //but once they is converted to calculate.
  ndt_loc.X = pose.pose.position.x;
  ndt_loc.Y = pose.pose.position.y;
  ndt_loc.Z = pose.pose.position.z;

  GetRPY(pose.pose,ndt_angle.thiX,ndt_angle.thiY,ndt_angle.thiZ);
  printf("quaternion angle : %f\n",ndt_angle.thiZ*180/M_PI);

  ndtGetFlag = true;
  //printf("my position : %f %f %f\n",my_loc.X,my_loc.Y,my_loc.Z);
}

int main(int argc, char **argv){
  
  ros::init(argc ,argv, "pedestrian_locate") ;  
  cout << "pedestrian_locate" << endl;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber pedestrian_pos_xyz = n.subscribe("/pedestrian_pixel_xyz", 1, pedestrian_pos_xyzCallback);

  ros::Subscriber gnss_pose = n.subscribe("/gnss_pose", 1, position_getter_gnss);
  ros::Subscriber ndt_pose = n.subscribe("/ndt_pose", 1, position_getter_ndt);

  pub = n.advertise<geometry_msgs::PoseArray>("pedestrian_pose",1); 

  //read calibration value
  //TO DO : subscribe from topic
  cv::Mat Cintrinsic;
  std::string camera_yaml;

  n.param<std::string>("/scan_to_image/camera_yaml", camera_yaml,STR(CAMERA_YAML));

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

  /*
  cv::Mat Lintrinsic;
  std::string lidar_3d_yaml = "/home/auto1/.ros/autoware/camera_lidar_3d.yaml";

  cv::FileStorage lidar_3d_file(lidar_3d_yaml.c_str(), cv::FileStorage::READ); 
  if(!lidar_3d_file.isOpened()){
    fprintf(stderr,"%s, : cannot open file\n",lidar_3d_yaml.c_str());
    exit(EXIT_FAILURE);
  }
  lidar_3d_file["CameraExtrinsicMat"] >> Lintrinsic; 
  lidar_3d_file.release(); 

  for(int i=0; i<4 ; i++){
    for(int j=0; j<4 ; j++){
      cameraMatrix[i][j] = Lintrinsic.at<double>(i,j);
    }
  }
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

}
