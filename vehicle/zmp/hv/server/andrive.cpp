//Andrive
/* ---------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/time.h>
#include "timeval_ops.h"

#define DEBUG_PRINT 0
#define ANDRV_MODE_MANUAL 0
#define ANDRV_MODE_PROGRAM 1

void __andrvConnect(void);
void __andrvChangeMode(void*, int);
int __andrvGetSensorValue(void);
int __andrvSendSignal(void);
double __andrvControlSteering(double setpoint, double val, double dt);
//void* andrvThreadEntry(void* arg);

int sock_num;
int sock0;

int pitch;
int accelerator;
int brake;
int steering;
int gearNum;

pthread_t __andrvThread;

void MainWindow::andrvActivate(void)
{
  struct timeval time;
  int pre_pitch = 0;

  printf("Andrive activated\n");
  pthread_create(&__andrvThread, NULL, andrvThreadEntry, hev);

  __andrvConnect();
  printf("Andrive connected\n");

  __andrvChangeMode(hev, ANDRV_MODE_PROGRAM);
  printf("Andrive controlled\n");

  pitch = 0;
  accelerator = 0;
  brake = 0;
  steering = 0;
  while(1) {
    if(__andrvGetSensorValue() == -1) {
      printf("Sensing failed\n");
      __andrvChangeMode(hev, ANDRV_MODE_MANUAL);
      printf("Andrive fallbacked\n");
      break;
    }
    if(__andrvSendSignal() == -1) {
      printf("Signal failed\n");
      break;
    }

    gettimeofday(&time, NULL);

    pitch = -pitch;

    brake = brake * 1980.0 / 100.0;
    accelerator = accelerator * 990.0 / 100.0;

    if(pitch >= 0) {
      //steering = 13300 * pitch / 120;
      steering = 13300 * pitch / 100;
    }
    else if(pitch < 0) {
      //steering = 13300 * pitch / 120;
      steering = 13300 * pitch / 100;
    }

#if DEBUG_PRINT
    printf("\n\n\n\n\n\n\nsteering = %d", steering);
#endif
    pre_pitch = pitch;
  }
}

void* MainWindow::andrvThreadEntry(void* arg)
{
  HevCnt* hev = (HevCnt*)arg;
  struct timeval tv_c, tv_p, tv_dt;
  int dt; // microsecond
  int local_steering;

  printf("Andrive Worker Thread launched!\n");
  gettimeofday(&tv_c, NULL);

  while (1) {
    // current time.
    gettimeofday(&tv_c, NULL);
    timeval_sub(&tv_dt, &tv_c, &tv_p);
    dt = timeval_to_us(&tv_dt);

    // steering
    local_steering = __andrvControlSteering(steering, local_steering, dt);
    hev->SetStrAngle(steering);
    
    // brake
    hev->SetBrakeStroke(brake);
    
    // accel
    hev->SetDrvStroke(accelerator);

#if DEBUG_PRINT
    if (local_steering != 0)
      printf("local_steering = %d\n", local_steering);
#endif

    // record current time.
    timeval_copy(&tv_p, &tv_c);

    // suspend.
    //usleep(1000);
  }

  return NULL;
}

double __andrvControlSteering(double setpoint, double val, double dt)
{
#define KP 0.0001
#define KI 0.0001
#define KD 0.0005
  static double e_p = 0, i_p = 0, setpoint_p = 0;
  double e = setpoint - val;
  double i = i_p + e;
  double d = e - e_p;
  double delta;
  
  if (setpoint != setpoint_p)
    i = e;

  delta = KP * e + KI * i + KD * d;
#if DEBUG_PRINT
  if (delta != 0) {
    printf("error = %d\n", e);
    printf("delta = %d\n", delta);
  }
#endif

  e_p = e;
  i_p = i;
  setpoint_p = setpoint;

  return val + delta;
}

void __andrvConnect(void)
{
  struct sockaddr_in addr;
  struct sockaddr_in client;
  socklen_t len;
  int sock;
  int size;
  int yes = 1;
  
  sock0 = socket(AF_INET, SOCK_STREAM, 0);
  
  addr.sin_family = AF_INET;
  //addr.sin_port = htons(12345); addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(12335); addr.sin_addr.s_addr = INADDR_ANY;
  //make it available immediately to connect
  setsockopt(sock0,SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes));
  bind(sock0, (struct sockaddr *)&addr, sizeof(addr));
  listen(sock0, 5);
  len = sizeof(client);
  sock = accept(sock0, (struct sockaddr *)&client, &len);
  if(sock == -1){
    printf("ERROR: cannot accept\n");
    return ;
  }
  sock_num = sock;
}

void __andrvChangeMode(void* p, int mode)
{
  HevCnt* hev = (HevCnt*)p;  

  switch (mode) {
  case ANDRV_MODE_PROGRAM:
    hev->SetDrvMode(MODE_PROGRAM);
    hev->SetStrMode(MODE_PROGRAM);
    hev->SetDrvServo(0x10);
    hev->SetStrServo(0x10);
    break;
  case ANDRV_MODE_MANUAL:
    usleep(1000);
    hev->SetDrvServo(0x00);
    hev->SetStrServo(0x00);
    hev->SetDrvMode(MODE_MANUAL);
    hev->SetStrMode(MODE_MANUAL);
    break;
  default:
    printf("Unknown mode\n");
  }
}

int __andrvGetSensorValue(void)
{
  int sensorInfo[4];
  //float sensorInfo[4];
  int recvsize;
  if(recv(sock_num, &sensorInfo, 16, 0) == -1){
    printf("ERROR: can not recieve message\n");
    return -1;
  }
  
  //        printf("pitch: %d, accelerator: %d, brake %d, gearNum: %d\n", sensorInfo[0], sensorInfo[1], sensorInfo[2], sensorInfo[3]);
  //printf("pitch: %f, accelerator: %f, brake %f, gearNum: %d\n", sensorInfo[0], sensorInfo[1], sensorInfo[2], (int)sensorInfo[3]);
  
  pitch = sensorInfo[0];
  accelerator = sensorInfo[1];
  brake = sensorInfo[2];
  gearNum = (int)sensorInfo[3];
  
  /*
   * ギアボックス
   * B:0演じブレーキ強
   * R:1後退
   * N:2ニュートラル
   * D:3ドライブ
   */
  
  
  if(sensorInfo[1] == 999 && sensorInfo[2] == 999){
    printf("STOP Andorive!!!\n");
    
    //delete vdegima;
    exit(0);
    
    if(close(sock0)<0){
      printf("ERROR: can not close sock0\n");
      return -1;
    }
    if(close(sock_num)<0){
      printf("ERROR: can not close sock_num\n");
      return -1;
    }
    return -1;
  }
  
  return 0;
}

int __andrvSendSignal(void)
{
  int signal = 0;
  
  if(send(sock_num, &signal, 4, 0) == -1){
    printf("ERROR: can not send signal\n");
    return -1;
  }
  return 0;
}
