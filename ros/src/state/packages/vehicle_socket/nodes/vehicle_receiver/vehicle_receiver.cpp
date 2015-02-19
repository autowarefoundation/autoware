#include "ros/ros.h"

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

#define CAN_DATA_NUM 52

using namespace std;

ros::Publisher pub;

int mode;

int PORT = 10000;

static bool parseCanValue(const string& value, vehicle_socket::CanInfo& msg){
  istringstream ss(value);
  vector<string> columns;

  string column;
  while(getline(ss, column, ',')){
    columns.push_back(column);
  }

  if(columns.size() == CAN_DATA_NUM+1){
    msg.tm = columns[0].substr(1, columns[0].length() - 2);
    msg.devmode = stoi(columns[1]);
    msg.drvcontmode = stoi(columns[2]);
    msg.drvoverridemode = stoi(columns[3]);
    msg.drvservo = stoi(columns[4]);
    msg.drivepedal = stoi(columns[5]);
    msg.targetpedalstr = stoi(columns[6]);
    msg.inputpedalstr = stoi(columns[7]);
    msg.targetveloc = stod(columns[8]);
    msg.speed = stod(columns[9]);
    msg.driveshift = stoi(columns[10]);
    msg.targetshift = stoi(columns[11]);
    msg.inputshift = stoi(columns[12]);
    msg.strmode = stoi(columns[13]);
    msg.strcontmode = stoi(columns[14]);
    msg.stroverridemode = stoi(columns[15]);
    msg.strservo = stoi(columns[16]);
    msg.targettorque = stoi(columns[17]);
    msg.torque = stoi(columns[18]);
    msg.angle = stod(columns[19]);
    msg.targetangle = stod(columns[20]);
    msg.bbrakepress = stoi(columns[21]);
    msg.brakepedal = stoi(columns[22]);
    msg.brtargetpedalstr = stoi(columns[23]);
    msg.brinputpedalstr = stoi(columns[24]);
    msg.battery = stod(columns[25]);
    msg.voltage = stoi(columns[26]);
    msg.anp = stod(columns[27]);
    msg.battmaxtemparature = stoi(columns[28]);
    msg.battmintemparature = stoi(columns[29]);
    msg.maxchgcurrent = stod(columns[30]);
    msg.maxdischgcurrent = stod(columns[31]);
    msg.sideacc = stod(columns[32]);
    msg.accellfromp = stod(columns[33]);
    msg.anglefromp = stod(columns[34]);
    msg.brakepedalfromp = stod(columns[35]);
    msg.speedfr = stod(columns[36]);
    msg.speedfl = stod(columns[37]);
    msg.speedrr = stod(columns[38]);
    msg.speedrl = stod(columns[39]);
    msg.velocfromp2 = stod(columns[40]);
    msg.drvmode = stoi(columns[41]);
    msg.devpedalstrfromp = stoi(columns[42]);
    msg.rpm = stoi(columns[43]);
    msg.velocflfromp = stod(columns[44]);
    msg.ev_mode = stoi(columns[45]);
    msg.temp = stoi(columns[46]);
    msg.shiftfrmprius = stoi(columns[47]);
    msg.light = stoi(columns[48]);
    msg.gaslevel = stoi(columns[49]);
    msg.door = stoi(columns[50]);
    msg.cluise = stoi(columns[51]);
    mode = stoi(columns[52]);
    return true;
  }
  return false;
}

void* getCanValue(void *arg){

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

  if(result.compare("")!=0){
    if(parseCanValue(result, msg)){
       msg.header.frame_id = "/can";
      msg.header.stamp = ros::Time::now();
      pub.publish(msg);
    }
  }

  if(close(conn_fd)<0){
    fprintf(stderr,"socket close failed in pthread.\n");
  }

  return nullptr;
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

    if(pthread_create(&th, NULL, getCanValue, (void *)conn_fd)){
      printf("thread create error\n");
    }
    pthread_detach(th);
  }
  close(sock0);

  return nullptr;
}


int main(int argc, char **argv){
  
  ros::init(argc ,argv, "vehicle_receiver") ;
  ros::NodeHandle nh;
  
  std::cout << "vehicle receiver" << std::endl;

  pub = nh.advertise<vehicle_socket::CanInfo>("can_info", 100);

  pthread_t th;
  if(pthread_create(&th, NULL, receiverCaller, NULL)){
    printf("thread create error\n");
  }
  pthread_detach(th);

  ros::spin();

  return 0;
}
