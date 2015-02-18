
#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>

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

#include <vehicle_socket/CanInfo.h>

#include "SendData.h"



void CMDCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
 double linear_x = msg->twist.linear.x;
 double angular_z = msg->twist.angular.z;


}

//void ModeCallback(){}

/*void GearCallback(const ui_socket::gear_cmdConstPtr &gear)
{
}
*/

/*
void* returnCMDValue(void *arg){

  int *fd = static_cast<int *>(arg);
  int conn_fd = *fd;
  delete fd;
  char recvdata[1024];
  string result = "";
  int n;
  vehicle_socket::CanInfo msg;

  while(true){
    n = recv(conn_fd, recvdata, sizeof(recvdata), 0);

    if(n<0){
      printf("ERROR: can not recieve message\n");
      result = "";
      break;
    }else if(n == 0){
      break;
    }
    result.append(recvdata,n);

    //recv data is bigger than 1M,return error
    if(result.size() > 1024 * 1024){
      fprintf(stderr,"recv data is too big.\n");
      result = "";
      break;
    }
  }

  if(close(conn_fd)<0){
    fprintf(stderr,"socket close failed in pthread.\n");
  }

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
  listen(sock0, 5);
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

    printf("get connect.\n");
    //printf("count: %d\n", count);

    if(pthread_create(&th, NULL, returnCMDValue, (void *)conn_fd)){
      printf("thread create error\n");
    }
    pthread_detach(th);
  }
  close(sock0);

  return nullptr;
}
*/


int main(int argc, char **argv){

  ros::init(argc ,argv, "vehicle_sender") ;
  ros::NodeHandle nh;

  std::cout << "vehicle sender" << std::endl;
  ros::Subscriber sub[1];
  sub[0] = nh.subscribe("twist_cmd", 100,CMDCallback);
//sub[1] = nh.subscribe("",100,ModeCallback);
  //sub[1] = nh.subscribe("gear_cmd", 100,GearCallback);

  pthread_t th;
  if(pthread_create(&th, NULL, receiverCaller, NULL)){
    printf("thread create error\n");
  }
  pthread_detach(th);

  ros::spin();

  return 0;
}
