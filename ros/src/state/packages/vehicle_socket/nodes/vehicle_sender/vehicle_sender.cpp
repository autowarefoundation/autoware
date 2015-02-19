
#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

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

//default message
#define DEFAULT_MESSAGE "no command data"
//#define DEFAULT_MESSAGE "0,100"

int PORT = 10001;

using namespace std;

string value;//cmd value
bool updateFlag;

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

void CMDCallback(const geometry_msgs::Twist &msg)
{
 ostringstream oss;
 double linear_x = msg.linear.x;
 double angular_z = msg.angular.z;
 oss << linear_x << ",";
 oss << angular_z;
 value = oss.str();
 updateFlag = true;

}

void* returnCMDValue(void *arg){

  int *fd = static_cast<int *>(arg);
  int conn_fd = *fd;
  delete fd;
  // string result = "";
  int n;

  /*
  while(true){
    n = recv(conn_fd, recvdata, sizeof(recvdata), 0);

    if(n<0){
      printf("ERROR: can not recieve message\n");
      return nullptr;
    }else if(n == 0){
      break;
    }

    result.append(recvdata,n);

    //if receive data is bigger than 12 byte, exit loop
    if(result.size() == 12){
      break;
    }else if(result.size() > 12){
      fprintf(stderr,"recv size is too big\n");
      return nullptr;
    }
  }

  if(result.compare("cmd request")){
    n = write(conn_fd, value.c_str(), value.size());
    if(n < 0){
      fprintf(stderr,"data return error\nmiss to send cmd data\n");
      return nullptr;
    }
  }
  */

  if(!updateFlag){
    value = DEFAULT_MESSAGE;
  }

  n = write(conn_fd, value.c_str(), value.size());
  if(n < 0){
    fprintf(stderr,"data return error\nmiss to send cmd data\n");
    return nullptr;
  }
  
  if(close(conn_fd)<0){
    fprintf(stderr,"socket close failed in pthread.\n");
  }

  printf("%s\n",value.c_str());
  updateFlag = false;

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
  ros::Subscriber sub[1];
  sub[0] = nh.subscribe("/cmd_vel", 1,CMDCallback);
  //sub[0] = nh.subscribe("twist_cmd", 100,CMDCallback);
  //sub[1] = nh.subscribe("",100,ModeCallback);
  //sub[1] = nh.subscribe("gear_cmd", 100,GearCallback);

  //default message
  value = DEFAULT_MESSAGE;
  updateFlag = false;

  pthread_t th;
  if(pthread_create(&th, NULL, receiverCaller, NULL)){
    printf("thread create error\n");
  }
  pthread_detach(th);

  ros::spin();

  return 0;
}
