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
#include "sensor_msgs/NavSatFix.h"
#include "structure.h"
#include "SendData.h"
#include "calcoordinates.h"
#include "axialMove.h"
#include "geo_pos_conv.hh"

#define XSTR(x) #x
#define STR(x) XSTR(x)

using namespace std;

typedef struct _OBJPOS{
  int x1;
  int y1;
  int x2;
  int y2;
  float distance;
  string tm;
}OBJPOS;

//for timestamp
struct my_tm {
  time_t tim; // yyyymmddhhmmss
  long msec;  // milli sec
};

pthread_mutex_t mutex;
pthread_mutex_t ped_mutex;
pthread_mutex_t pos_mutex;

selfLocation sl;

//store subscribed value
vector<OBJPOS> global_cp_vector;
vector<OBJPOS> global_pp_vector;

//default server name to send data
char defaultServerName[20] = "db1.ertl.jp";
//magic that I am C++
const char MAGIC[5] = "MPWC";

//flag for comfirming whether updating angle and position or not
bool angleGetFlag;
bool positionGetFlag;

//send to server class
SendData sd;

//store own position and direction now.updated by position_getter
LOCATION my_loc;
ANGLE angle;


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


string makeSendDataDetectedObj(vector<OBJPOS> car_position_vector,vector<OBJPOS>::iterator cp_iterator,geo_pos_conv geo){

  ostringstream oss;
  LOCATION rescoord;

  for(int i=0; i<car_position_vector.size() ; i++, cp_iterator++){

    //middle of right-lower and left-upper
    double U = cp_iterator->x1 + cp_iterator->x2/2;
    double V = cp_iterator->y1 + cp_iterator->y2/2;

    //convert
    sl.setOriginalValue(U,V,cp_iterator->distance);
    LOCATION ress = sl.cal();
    printf("coordinate from own:%f,%f,%f\n",ress.X,ress.Y,ress.Z);

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
    rescoord.X += geo.x();
    rescoord.Y += geo.y();
    rescoord.Z += 0;

    //create sql
    oss << "INSERT INTO POS_NOUNIQUE(id,x,y,area,type,self,tm) ";
    oss << "values(0," << fixed << setprecision(6) << rescoord.X << "," << fixed << setprecision(6) << rescoord.Y << ",0,0,1,'" << cp_iterator->tm << "');\n";

    //oss << "0 " << fixed << setprecision(6) << rescoord.X << " " << fixed << setprecision(6) << rescoord.Y << " 0,";

  
  }

  //  cout << "recognized obj position : " << endl << oss.str() << endl;
  

  return oss.str();

}




//wrap SendData class
void* wrapSender(void *tsd){

  //get values from sample_corner_point , convert latitude and longitude,
  //and send database server.
  
  vector<OBJPOS> car_position_vector(global_cp_vector.size());
  vector<OBJPOS>::iterator cp_iterator;
  vector<OBJPOS> pedestrian_position_vector(global_pp_vector.size());
  vector<OBJPOS>::iterator pp_iterator;
  string value = "";
  geo_pos_conv geo;
  ostringstream oss;

  //thread safe process for vector
  pthread_mutex_lock( &mutex );
  std::copy(global_cp_vector.begin(), global_cp_vector.end(), car_position_vector.begin());
  global_cp_vector.clear();
  vector<OBJPOS>(global_cp_vector).swap(global_cp_vector);
  pthread_mutex_unlock( &mutex );

  pthread_mutex_lock( &ped_mutex );
  std::copy(global_pp_vector.begin(), global_pp_vector.end(), pedestrian_position_vector.begin());
  global_pp_vector.clear();
  vector<OBJPOS>(global_pp_vector).swap(global_pp_vector);
  pthread_mutex_unlock( &ped_mutex );

  cp_iterator = car_position_vector.begin();
  pp_iterator = pedestrian_position_vector.begin();

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


  //calculate own coordinate from own lati and longi value
  //get my position now
  pthread_mutex_lock( &pos_mutex );
  double my_xloc = my_loc.X;
  double my_yloc = my_loc.Y;
  double my_zloc = my_loc.Z;
  pthread_mutex_unlock( &pos_mutex );

  geo.set_plane(7);
  geo.set_llh(my_xloc,my_yloc,my_zloc);


  //get data of car and pedestrian recognizing
  if(car_position_vector.size() > 0 ){
    value += makeSendDataDetectedObj(car_position_vector,cp_iterator,geo);
  }

  if(pedestrian_position_vector.size() > 0){
    value += makeSendDataDetectedObj(pedestrian_position_vector,pp_iterator,geo);
  }

  oss << "INSERT INTO POS_NOUNIQUE(id,x,y,area,type,self,tm) ";
  oss << "values(0," <<  fixed << setprecision(6) << geo.x() << "," << fixed << setprecision(6) << geo.y() << ",0,0,1,'" << getNowTime() << "');\n";

  //oss << "0 " << fixed << setprecision(6) << geo.x() << " " << fixed << setprecision(6) << geo.y() << " 0,";

  //  printf("geo : %f\t%f\n",geo.x(),geo.y());

  value += oss.str();
  //printf("%s\n",oss.str().c_str());
  //printf("%d\n",value.size());
  

  sd.setValue(value);
  string res = sd.Sender();
  cout << "retrun message from DBserver : " << res << endl;

}


void* intervalCall(void *a){

  pthread_t th;

  while(1){
    //If angle and position data is not updated from prevous data send,
    //data is not sent
    if(!(angleGetFlag && positionGetFlag)) {
      sleep(1);
      continue;
    }
    angleGetFlag = false;
    positionGetFlag = false;

    //If position is over range,skip loop
    if((my_loc.X > 180.0 && my_loc.X < -180.0 ) || 
       (my_loc.Y > 180.0 && my_loc.Y < -180.0 ) || 
       my_loc.Z != 0.0 ){
      fprintf(stderr,"GNSS value is invalid.\n");
      sleep(1);
      continue;
    }

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


void car_pos_xyzCallback(const car_detector::FusedObjects& fused_objects)
{
  OBJPOS cp;
  
  pthread_mutex_lock( &mutex );  

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
    cp.tm = getNowTime();

    //printf("\ncar : %d,%d,%d,%d,%f\n",cp.x1,cp.y1,cp.x2,cp.y2,cp.distance);

    global_cp_vector.push_back(cp);
      
  }

  pthread_mutex_unlock( &mutex );  
    
  //  printf("car position get\n\n");

}

void pedestrian_pos_xyzCallback(const car_detector::FusedObjects& fused_objects)
{
  OBJPOS cp;

  pthread_mutex_lock( &ped_mutex );  

  for(int i = 0; i < fused_objects.car_num; i++) { //fused_objects.car_num は障害物の個数
    //If distance is zero, skip loop
    if(fused_objects.distance.at(i) <= 0) continue;

    cp.x1 = fused_objects.corner_point[0+i*4];//x-axis of the upper left
    cp.y1 = fused_objects.corner_point[1+i*4];//x-axis of the lower left
    cp.x2 = fused_objects.corner_point[2+i*4];//x-axis of the upper right
    cp.y2 = fused_objects.corner_point[3+i*4];//x-axis of the lower left

    cp.distance = fused_objects.distance.at(i);
    cp.tm = getNowTime();

    printf("\npedestrian : %d,%d,%d,%d,%f\n",cp.x1,cp.y1,cp.x2,cp.y2,cp.distance);

    global_pp_vector.push_back(cp);

  }

  pthread_mutex_unlock( &ped_mutex );  
    
  //  printf("pedestrian position get\n\n");
}



void azimuth_getter(const geometry_msgs::TwistStamped& azi)
{

  angle.thiX = 0;
  angle.thiY = azi.twist.angular.z*180/M_PI;
  angle.thiZ = 0;
  angleGetFlag = true;
  //printf("azimuth : %f\n",angle.thiY);
  //printf("ok\n");

}

void position_getter(const sensor_msgs::NavSatFix& pos)
{
  pthread_mutex_lock( &pos_mutex );
  my_loc.X = pos.latitude;
  my_loc.Y = pos.longitude;
  my_loc.Z = 0;
  positionGetFlag = true;
  pthread_mutex_unlock( &pos_mutex );
  //printf("my position : %f %f %f\n",my_loc.X,my_loc.Y,my_loc.Z);

}


//test
void set_car_position_xyz()
{
  int i;
  OBJPOS cp;

  global_cp_vector.clear();

  //認識した車の数だけ繰り返す
  //    for (i = 0; i < car_position_xyz.car_num; i++){
  for (i = 0; i < 3; i++){
    //各認識した車へのデータのアクセス例
    cp.x1 = 10*i;
    cp.y1 = 20*i;
    cp.x2 = 30*i;
    cp.y2 = 40*i;
    cp.distance = 20;
    global_cp_vector.push_back(cp);
      
  }

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

  ros::Subscriber car_pos_xyz = n.subscribe("/car_pixel_xyz", 1, car_pos_xyzCallback);
  ros::Subscriber pedestrian_pos_xyz = n.subscribe("/pedestrian_pixel_xyz", 1, pedestrian_pos_xyzCallback);

  ros::Subscriber azm = n.subscribe("/vel", 1, azimuth_getter);
  ros::Subscriber my_pos = n.subscribe("/fix", 1, position_getter);

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
  /*
  double fkx = 5.83199829e+02;
  double fky = 3.74826355e+02;
  double Ox =  5.83989319e+02;
  double Oy = 2.41745468e+02;
  */

  sl.setCameraParam(fkx,fky,Ox,Oy);

  //set_car_position_xyz();

  //set server name and port
  char *serverName;
  int portNum = 5678;
  if(argc == 3){
    serverName = argv[1];
    portNum = atoi(argv[2]);
  }else{
    serverName = defaultServerName;
  }
  sd = SendData(serverName,portNum);

  //set angle and position flag : false at first
  angleGetFlag = false;
  positionGetFlag = false;

  pthread_t th;
  if(pthread_create(&th, NULL, intervalCall, NULL)){
    printf("thread create error\n");
  }

  pthread_detach(th);

  ros::spin();

}
