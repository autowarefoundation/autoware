/*
自分から認識した物体の種類と位置情報をデータベースに送信する

簡単な仕様：
１、pedestrian_pos_xyzとcar_pos_xyzから画面上のxy座標とdistanceを取得する
２、取得したxyとdistanceから上から見たxy座標を求める
３、このxy座標はカメラから見た座標なのでvelodyneからみ見た座標に変換する
４、これでvelodyneから座標が得られるのでこれを東西南北を軸とする直交座標に変換する
５、直交座標を緯度・経度に変換する
６、データベースサーバに対して1秒ごとに送信する

送信データのフォーマットは
緯度、経度、物体の種類、自動車のid

データは認識した物体ごとに送る

*/


#include "std_msgs/String.h"
#include "ros/ros.h"

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include "car_detector/FusedObjects.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
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
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/NavSatFix.h"
#include "../SendData.h"

/*
#include "structure.h"
#include "calcoordinates.h"
#include "axialMove.h"
#include "geo_pos_conv.hh"
*/

#define XSTR(x) #x
#define STR(x) XSTR(x)

using namespace std;

//for timestamp
struct my_tm {
  time_t tim; // yyyymmddhhmmss
  long msec;  // milli sec
};

pthread_mutex_t mutex;
pthread_mutex_t ped_mutex;
pthread_mutex_t pos_mutex;

//store subscribed value
vector<geometry_msgs::PoseStamped> global_cp_vector;
vector<geometry_msgs::PoseStamped> global_pp_vector;

//default server name and port to send data
const string defaultServerName = "db1.ertl.jp";
const int PORT = 5678;
//magic that I am C++
const char MAGIC[5] = "MPWC";

//flag for comfirming whether updating position or not
bool positionGetFlag;

//send to server class
SendData sd;

//store own position and direction now.updated by position_getter
geometry_msgs::PoseStamped my_loc;

double cameraMatrix[4][4];
/*
 = {
  {-7.8577658642752374e-03, -6.2035361880992401e-02,9.9804301981022692e-01, 5.1542126095196206e-01},
  {-9.9821250329813849e-01, 5.9620033356180935e-02,-4.1532977104442731e-03, -2.9214878315161133e-02},
  {-5.9245706805522491e-02, -9.9629165684497312e-01,-6.2392954139163306e-02, -6.6728858508628075e-01},
  {0, 0, 0, 1}
 
};
*/

void printDiff(struct timeval begin, struct timeval end){
  long diff;
  diff = (end.tv_sec - begin.tv_sec)*1000*1000 + (end.tv_usec - begin.tv_usec);
  printf("Diff: %ld us (%ld ms)\n",diff,diff/1000);
}

/*
void GetRPY(const geometry_msgs::Pose &pose,
	    double &roll,
	    double &pitch,
	    double &yaw){
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation,q);
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
}
*/

string getTimeStamp(long sec,long nsec){
  struct tm *tmp;
  struct timeval tv;
  char temp[30];
  string res;

  tv.tv_sec = sec;
  tv.tv_usec = nsec/1000;

  tmp=localtime(&tv.tv_sec);
  sprintf(temp,"%04d-%02d-%02d %02d:%02d:%02d.%d",
	  tmp->tm_year + 1900, tmp->tm_mon + 1,
	  tmp->tm_mday, tmp->tm_hour,
	  tmp->tm_min, tmp->tm_sec,
	  static_cast<int>(tv.tv_usec/1000));
  res = temp;
  return res;
}


string makeSendDataDetectedObj(vector<geometry_msgs::PoseStamped> car_position_vector){

  ostringstream oss;
  vector<geometry_msgs::PoseStamped>::iterator cp_iterator;
  cp_iterator = car_position_vector.begin();

  for(int i=0; i<car_position_vector.size() ; i++, cp_iterator++){

    //create sql
    //In Autoware, x and y is oppsite.So reverse these when sending.
    oss << "INSERT INTO POS_NOUNIQUE(id,x,y,area,type,self,tm) ";
    oss << "values(0," << fixed << setprecision(6) << cp_iterator->pose.position.y << "," << fixed << setprecision(6) << cp_iterator->pose.position.x << ",0,0,1,'" << getTimeStamp(cp_iterator->header.stamp.sec,cp_iterator->header.stamp.nsec) << "');\n";

    /*
    oss << "INSERT INTO POS_NOUNIQUE(id,sender_id,x,y,area,type,self,tm) ";
    oss << "values(0,0," << fixed << setprecision(6) << cp_iterator->pose.position.y << "," << fixed << setprecision(6) << cp_iterator->pose.position.x << ",0,0,1,'" << getTimeStamp(cp_iterator->header.stamp.sec,cp_iterator->header.stamp.nsec) << "');\n";
    */

  }

  return oss.str();

}


//wrap SendData class
void* wrapSender(void *tsd){

  //get values from sample_corner_point , convert latitude and longitude,
  //and send database server.
  

  vector<geometry_msgs::PoseStamped> car_position_vector(global_cp_vector.size());
  vector<geometry_msgs::PoseStamped> pedestrian_position_vector(global_pp_vector.size());
  string value = "";
  ostringstream oss;

  //thread safe process for vector
  pthread_mutex_lock( &mutex );
  std::copy(global_cp_vector.begin(), global_cp_vector.end(), car_position_vector.begin());
  global_cp_vector.clear();
  vector<geometry_msgs::PoseStamped>(global_cp_vector).swap(global_cp_vector);
  pthread_mutex_unlock( &mutex );

  pthread_mutex_lock( &ped_mutex );
  std::copy(global_pp_vector.begin(), global_pp_vector.end(), pedestrian_position_vector.begin());
  global_pp_vector.clear();
  vector<geometry_msgs::PoseStamped>(global_pp_vector).swap(global_pp_vector);
  pthread_mutex_unlock( &ped_mutex );

  //create header
  char magic[5] = "MPWC";
  u_int16_t major = htons(1);
  u_int16_t minor = htons(0);
  u_int32_t sqlinst = htonl(2);
  u_int32_t sqlnum = htonl(car_position_vector.size()+pedestrian_position_vector.size()+1);
  char header[16];
  memcpy(header,magic,4);
  memcpy(&header[4],&major,2);
  memcpy(&header[6],&minor,2);
  memcpy(&header[8],&sqlinst,4);
  memcpy(&header[12],&sqlnum,4);
  value.append(header,16);

  cout << "sqlnum : " << car_position_vector.size() + pedestrian_position_vector.size() + 1 << endl;

  /*
  geo.set_plane(7);
  geo.set_llh(my_xloc,my_yloc,my_zloc);
  */

  //get data of car and pedestrian recognizing
  if(car_position_vector.size() > 0 ){
    value += makeSendDataDetectedObj(car_position_vector);
  }

  if(pedestrian_position_vector.size() > 0){
    value += makeSendDataDetectedObj(pedestrian_position_vector);
  }

  oss << "INSERT INTO POS_NOUNIQUE(id,x,y,area,type,self,tm) ";
  oss << "values(0," <<  fixed << setprecision(6) << my_loc.pose.position.y << "," << fixed << setprecision(6) << my_loc.pose.position.x << ",0,0,1,'" << getTimeStamp(my_loc.header.stamp.sec,my_loc.header.stamp.nsec) << "');\n";

  /*
    oss << "INSERT INTO POS_NOUNIQUE(id,sender_id,x,y,area,type,self,tm) ";
    oss << "values(0,0," <<  fixed << setprecision(6) << my_loc.pose.position.y << "," << fixed << setprecision(6) << my_loc.pose.position.x << ",0,0,1,'" << getTimeStamp(my_loc.header.stamp.sec,my_loc.header.stamp.nsec) << "');\n";
  */

  value += oss.str();
  //cout << value;

  string res = sd.Sender(value);
  cout << "retrun message from DBserver : " << res << endl;

}


void* intervalCall(void *a){

  pthread_t th;

  while(1){
    //If angle and position data is not updated from prevous data send,
    //data is not sent
    //if(1){
    if(!positionGetFlag) {
      sleep(1);
      continue;
    }
    positionGetFlag = false;

    //create new thread for socket communication.
    if(pthread_create(&th, NULL, wrapSender, NULL)){
      printf("thread create error\n");
    }
    sleep(1);
    if(pthread_join(th,NULL)){
      printf("thread join error.\n");
    }
    
  }
}


void car_locateCallback(const geometry_msgs::PoseStamped car_locate)
{
  if(global_cp_vector.size() == 0 || 
     (car_locate.header.stamp.sec == global_cp_vector.back().header.stamp.sec&&
      car_locate.header.stamp.nsec == global_cp_vector.back().header.stamp.nsec)){
    global_cp_vector.push_back(car_locate);
  }else{
    global_cp_vector.clear();
    vector<geometry_msgs::PoseStamped>(global_cp_vector).swap(global_cp_vector);
    global_cp_vector.push_back(car_locate);
  }

  printf("car ok\n");

}

void pedestrian_locateCallback(const geometry_msgs::PoseStamped pedestrian_locate)
{

  if(global_pp_vector.size() == 0 || 
     (pedestrian_locate.header.stamp.sec == global_pp_vector.back().header.stamp.sec&&
      pedestrian_locate.header.stamp.nsec == global_pp_vector.back().header.stamp.nsec)
){ 
    global_pp_vector.push_back(pedestrian_locate);
  }else{
    global_pp_vector.clear();
    vector<geometry_msgs::PoseStamped>(global_pp_vector).swap(global_pp_vector);
    global_pp_vector.push_back(pedestrian_locate);
  }

  printf("pedestrian ok\n");

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

  my_loc = pose;
  positionGetFlag = true;

  printf("gnss ok\n");
  
}

int main(int argc, char **argv){
  
  ros::init(argc ,argv, "obj_uploader") ;  
  cout << "obj_uploader" << endl;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber car_locate = n.subscribe("/car_pose", 5, car_locateCallback);
  ros::Subscriber pedestrian_locate = n.subscribe("/pedestrian_pose", 5, pedestrian_locateCallback);
 ros::Subscriber gnss_pose = n.subscribe("/gnss_pose", 1, position_getter_gnss);

  /*
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
  sl.setCameraParam(fkx,fky,Ox,Oy);
  */

  //set server name and port
  string serverName = defaultServerName;
  int portNum = PORT;
  if(argc == 3){
    serverName = argv[1];
    portNum = atoi(argv[2]);
  }

  sd = SendData(serverName,portNum);

  //set angle and position flag : false at first
  positionGetFlag = false;

  pthread_t th;
  if(pthread_create(&th, NULL, intervalCall, NULL)){
    printf("thread create error\n");
  }
  pthread_detach(th);

  ros::spin();

}
