
#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include "ui_socket/mode_cmd.h"
#include "ui_socket/gear_cmd.h"
#include "runtime_manager/accel_cmd.h"
#include "runtime_manager/brake_cmd.h"
#include "runtime_manager/steer_cmd.h"
#include <std_msgs/Int32.h>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <string>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>



int PORT = 10001;

using namespace std;

//cmd data
/*
typedef struct _CMDDATA{
  geometry_msgs::Twist twistValue;
  int modeValue;
  int gearValue;
  int accellValue;
  int steerValue;
  int brakeValue;
}CMDDATA;
*/

typedef struct _CMDDATA{
  double linear_x;
  double angular_z;
  int modeValue;
  int gearValue;
  int accellValue;
  int brakeValue;
  int steerValue;

}CMDDATA;

CMDDATA cd;

//gear is limited by modeFlag,
//accell, steer and brake is limited by modeFlag and gearFlag.
bool modeFlag;
bool gearFlag;

void initCMDDATA(){
  cd.linear_x = 0;
  cd.angular_z = 0;
  cd.modeValue = 0;
  cd.gearValue = 0;
  cd.accellValue = 0;
  cd.brakeValue = 0;
  cd.steerValue = 0;

}


/*
void CMDCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
 ostringstream oss;
 double linear_x = msg->twist.linear.x;
 double angular_z = msg->twist.angular.z;
 oss << linear_x << ",";
 oss << angular_z;
 value = oss.str();
 updateFlag = true;

}
*/

void CMDCallback(const geometry_msgs::TwistStamped &msg)
{
  cd.linear_x = msg.twist.linear.x;
  cd.angular_z = msg.twist.angular.z;
}


void modeCMDCallback(const ui_socket::mode_cmd &mode)
{

  /*
  if(mode.mode == 1){//auto mobile mode
    modeFlag = true;
  }else{ //manual mode
    modeFlag = false;
    gearFlag = false;
    initCMDDATA();
  }
  */
  if(mode.mode == -1 || mode.mode == 0){
    initCMDDATA();
  }
  cd.modeValue = mode.mode;
}

void gearCMDCallback(const ui_socket::gear_cmd &gear)
{
  cd.gearValue = gear.gear;
  /*
  if(modeFlag){
    cd.gearValue = gear.gear;
    gearFlag = true;
  }
  */
}


void accellCMDCallback(const runtime_manager::accel_cmd &accell)
{

  cd.accellValue = accell.accel;
  /*
    if(modeFlag && gearFlag){
    cd.accellValue = accell.accel;
    }
  */
}


void steerCMDCallback(const runtime_manager::steer_cmd &steer)
{
  cd.steerValue = steer.steer;
  /*
  if(modeFlag && gearFlag){
    cd.steerValue = steer.steer;
  }
  */
}


void brakeCMDCallback(const runtime_manager::brake_cmd &brake)
{
  cd.brakeValue = brake.brake;
  /*
  if(modeFlag && gearFlag){
    cd.brakeValue = brake.brake;
  }
  */
}


void* returnCMDValue(void *arg){

  int *fd = static_cast<int *>(arg);
  int conn_fd = *fd;
  delete fd;
  string value;
  int n;

  //string version
  ostringstream oss;
  oss << cd.linear_x << ",";
  oss << cd.angular_z << ",";
  oss << cd.modeValue << ",";
  oss << cd.gearValue << ",";
  oss << cd.accellValue << ",";
  oss << cd.brakeValue << ",";
  oss << cd.steerValue;

  value = oss.str();

  //struct version
  /*
  char *tmpv;
  tmpv = (char*)malloc(sizeof(CMDDATA));
  memcpy(tmpv,&cd,sizeof(CMDDATA));
  value.copy(tmpv,sizeof(CMDDATA),0);
  */

  //n = write(conn_fd, tmpv, sizeof(CMDDATA));
  n = write(conn_fd, value.c_str(), value.size());
  if(n < 0){
    fprintf(stderr,"data return error\nmiss to send cmd data\n");
    return nullptr;
  }
  
  if(close(conn_fd)<0){
    fprintf(stderr,"socket close failed in pthread.\n");
  }

  printf("%s %lu\n",value.c_str(),value.size());
  //printf("%u %u\n",sizeof(cd),sizeof(CMDDATA));
  
  return nullptr;

}


void* receiverCaller(void *a){
  int sock0;
  struct sockaddr_in addr;
  struct sockaddr_in client;
  socklen_t len;
  //  int yes = 1;
  pthread_t th;

  sock0 = socket(AF_INET, SOCK_STREAM, 0);
  addr.sin_family = PF_INET;
  addr.sin_port = htons(PORT);
  addr.sin_addr.s_addr = INADDR_ANY;
  //make it available immediately to connect
  //setsockopt(sock0,SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes));
  bind(sock0, (struct sockaddr *)&addr, sizeof(addr));
  listen(sock0, 20);
  len = sizeof(client);

  while(true){
    //get connect to android
    printf("Waiting access...\n");
    int *conn_fd = new int();
    *conn_fd = accept(sock0, (struct sockaddr *)&client, &len);
    if(*conn_fd == -1){
      printf("ERROR: cannot accept\n");
      break;
    }

    if(pthread_create(&th, NULL, returnCMDValue, (void *)conn_fd)){
      printf("thread create error\n");
    }
    pthread_detach(th);

    printf("get connect.\n");

  }
  close(sock0);

  return nullptr;
}

int main(int argc, char **argv){

  ros::init(argc ,argv, "vehicle_sender") ;
  ros::NodeHandle nh;

  std::cout << "vehicle sender" << std::endl;
  ros::Subscriber sub[6];
  sub[0] = nh.subscribe("/twist_cmd", 1,CMDCallback);
  sub[1] = nh.subscribe("/mode_cmd", 1,modeCMDCallback);
  sub[2] = nh.subscribe("/gear_cmd", 1,gearCMDCallback);
  sub[3] = nh.subscribe("/accel_cmd", 1,accellCMDCallback);
  sub[4] = nh.subscribe("/steer_cmd", 1,steerCMDCallback);
  sub[5] = nh.subscribe("/brake_cmd", 1,brakeCMDCallback);

  //default message
  modeFlag = false;
  gearFlag = false;

  initCMDDATA();

  pthread_t th;
  if(pthread_create(&th, NULL, receiverCaller, NULL)){
    printf("thread create error\n");
  }
  pthread_detach(th);

  ros::spin();

  return 0;
}
