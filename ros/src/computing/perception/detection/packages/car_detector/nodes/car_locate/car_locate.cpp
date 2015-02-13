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
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/NavSatFix.h"
#include "structure.h"
#include "calcoordinates.h"
#include "axialMove.h"
#include "geo_pos_conv.hh"
#include "car_detector/CarPose.h"

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

selfLocation sl;

//store subscribed value
vector<OBJPOS> global_cp_vector;
//vector<OBJPOS> global_pp_vector;

//flag for comfirming whether updating position or not
bool positionGetFlag;

//store own position and direction now.updated by position_getter
LOCATION my_loc;
ANGLE angle;

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

void makeSendDataDetectedObj(vector<OBJPOS> car_position_vector,
			     vector<OBJPOS>::iterator cp_iterator,
			     LOCATION mloc,
			     vector<float> &x,
			     vector<float> &y,
			     vector<float> &z){

  LOCATION rescoord;

  for(uint i=0; i<car_position_vector.size() ; i++, cp_iterator++){

    //middle of right-lower and left-upper
    double U = cp_iterator->x1 + cp_iterator->x2/2;
    double V = cp_iterator->y1 + cp_iterator->y2/2;

    //convert
    sl.setOriginalValue(U,V,cp_iterator->distance);
    LOCATION ress = sl.cal();
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
    rescoord.Y = anglefixed.Z;
    rescoord.Z = anglefixed.Y;

    //add plane rectangular coordinate to that of target car.
    rescoord.X += mloc.X;
    rescoord.Y += mloc.Y;
    rescoord.Z += mloc.Z;

    x.push_back(rescoord.X);
    y.push_back(rescoord.Y);
    z.push_back(rescoord.Z);
  }

}


//wrap SendData class
void locatePublisher(vector<OBJPOS> car_position_vector){

  //get values from sample_corner_point , convert latitude and longitude,
  //and send database server.
  
  geometry_msgs::PoseStamped pose_msg;
  vector<float> x;
  vector<float> y;
  vector<float> z;
  vector<float>::iterator x_iterator;
  vector<float>::iterator y_iterator;
  vector<float>::iterator z_iterator;

  vector<OBJPOS>::iterator cp_iterator;
  LOCATION mloc;

  cp_iterator = car_position_vector.begin();

  //calculate own coordinate from own lati and longi value
  //get my position now
  mloc.X = my_loc.X;
  mloc.Y = my_loc.Y;
  mloc.Z = my_loc.Z;

  //get data of car and pedestrian recognizing
  if(car_position_vector.size() > 0 ){
    makeSendDataDetectedObj(car_position_vector,cp_iterator,mloc,x,y,z);
  }

  //publish recognized car data
  if(x.size() != 0 && y.size() != 0){
    x_iterator = x.begin();
    y_iterator = y.begin();
    z_iterator = z.begin();

    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    for(uint i=0; i<x.size()&&i<y.size()&&i<z.size(); i++,x_iterator++,y_iterator++,z_iterator++){
      pose_msg.pose.position.x = *y_iterator;
      pose_msg.pose.position.y = *x_iterator;
      pose_msg.pose.position.z = *z_iterator;
      pub.publish(pose_msg);
    }
  }

}


void car_pos_xyzCallback(const car_detector::FusedObjects& fused_objects)
{

  vector<OBJPOS> cp_vector;
  OBJPOS cp;
  
  //If angle and position data is not updated from prevous data send,
  //data is not sent
  if(positionGetFlag) {
    positionGetFlag = false;

  
    //If position is over range,skip loop
    if((!(my_loc.X > 180.0 && my_loc.X < -180.0 ) || 
       (my_loc.Y > 180.0 && my_loc.Y < -180.0 ) || 
	my_loc.Z != 0.0) ){

      //認識した車の数だけ繰り返す
      for (int i = 0; i < fused_objects.car_num; i++){
	
	//If distance is zero, we cannot calculate position of recognized object
	//so skip loop
	if(fused_objects.distance.at(i) <= 0) continue;

	cp.x1 = fused_objects.corner_point[0+i*4];//x-axis of the upper left
	cp.y1 = fused_objects.corner_point[1+i*4];//x-axis of the lower left
	cp.x2 = fused_objects.corner_point[2+i*4];//x-axis of the upper right
	cp.y2 = fused_objects.corner_point[3+i*4];//x-axis of the lower left

	cp.distance = fused_objects.distance.at(i);

	//printf("\ncar : %d,%d,%d,%d,%f\n",cp.x1,cp.y1,cp.x2,cp.y2,cp.distance);

	cp_vector.push_back(cp);      
      }

      locatePublisher(cp_vector);

    }
  }
  //  printf("car position get\n\n");

}

void position_getter_gnss(const geometry_msgs::PoseStamped &pose){

  my_loc.X = pose.pose.position.y;
  my_loc.Y = pose.pose.position.x;
  my_loc.Z = pose.pose.position.z;

  GetRPY(pose.pose,angle.thiX,angle.thiY,angle.thiZ);
  printf("quaternion angle : %f\n",angle.thiZ*180/M_PI);

  positionGetFlag = true;
  //printf("my position : %f %f %f\n",my_loc.X,my_loc.Y,my_loc.Z);
}

int main(int argc, char **argv){
  
  ros::init(argc ,argv, "car_locate") ;  
  cout << "car_locate" << endl;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber car_pos_xyz = n.subscribe("/car_pixel_xyz", 1, car_pos_xyzCallback);
  //ros::Subscriber pedestrian_pos_xyz = n.subscribe("/pedestrian_pixel_xyz", 1, pedestrian_pos_xyzCallback);

  /*
  ros::Subscriber azm = n.subscribe("/vel", 1, azimuth_getter);
  ros::Subscriber my_pos = n.subscribe("/fix", 1, position_getter);
  ros::Subscriber ndt = n.subscribe("/ndt_pose", 1, position_getter_ndt);
  */
  ros::Subscriber gnss_pose = n.subscribe("/gnss_pose", 1, position_getter_gnss);

  pub = n.advertise<geometry_msgs::PoseStamped>("car_pose",1); 

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

  sl.setCameraParam(fkx,fky,Ox,Oy);

  //set angle and position flag : false at first
  positionGetFlag = false;

  ros::spin();

}
