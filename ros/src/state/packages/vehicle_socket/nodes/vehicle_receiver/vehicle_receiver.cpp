#include "ros/ros.h"

#include <stdio.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <string>
#include <sys/types.h>
#include <unistd.h>
#include "std_msgs/String.h"
#include <pthread.h>
#include <sys/time.h>

using namespace std;

ros::Publisher pub;


void* getCanValue(void *fd){

  int conn_fd =*static_cast<int*>(fd);
  delete fd;
  char recvdata[1024];
  string result = "";
  int n;
  std_msgs::String msg;

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

  if(result.compare("")!=0){
    msg.data = result.c_str();
    ROS_INFO("test\t%s",msg.data.c_str());
    pub.publish(msg);
  }

  if(close(conn_fd)<0){
    fprintf(stderr,"socket close failed in pthread.\n");
  }

}

/*
int sendSignal(){
  int signal = 0;
  if(send(sock_num, &signal, 4, 0) == -1){
    printf("ERROR: can not send signal\n");
    return -1;
  }
  return 0;
}
*/

void* receiverCaller(void *a){
  int conn_fd;
  int sock0;
  struct sockaddr_in addr;
  struct sockaddr_in client;
  socklen_t len;
  int size;
  //  int yes = 1;
  pthread_t th;

  sock0 = socket(AF_INET, SOCK_STREAM, 0);
  addr.sin_family = PF_INET;
  addr.sin_port = htons(11111);
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

    if(pthread_create(&th, NULL, getCanValue, (void *)conn_fd)){
      printf("thread create error\n");
    }
    pthread_detach(th);
  }
  close(sock0);


}


int main(int argc, char **argv){
  
  ros::init(argc ,argv, "vehicle_receiver") ;
  ros::NodeHandle nh;
  
  std::cout << "vehicle receiver" << std::endl;

  pub = nh.advertise<std_msgs::String>("can_info",100);

  pthread_t th;
  if(pthread_create(&th, NULL, receiverCaller, NULL)){
    printf("thread create error\n");
  }
  pthread_detach(th);

  ros::spin();

  return 0;
}
