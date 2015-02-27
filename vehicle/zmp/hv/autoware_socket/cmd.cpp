#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <queue>

#include "mainwindow.h"
#include "data.h"

double cycle_time = 0.0;
double estimate_accel = 0.0;
int ndrv = 0;
int nbrk = 0;
double nstr = 0.0;

std::vector<std::string> split(const std::string& input, char delimiter)
{
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

//get current time
long long int getTime() {
  //returns time in milliseconds 
  struct timeval current_time;
  struct timezone ttz;
  double t;
  // lock();
  gettimeofday(&current_time, &ttz);
  t = ((current_time.tv_sec * 1000.0) +
       (current_time.tv_usec) / 1000.0);
  // unlock();
  return static_cast<long long int>(t);
}

//convert km/h to m/s
double KmhToMs(double v) {
  return (v*1000.0/(60.0*60.0));
}

void Getter(CMDDATA &cmddata){
  std::string request;
  std::string cmdRes;
  char recvdata[32];

  struct sockaddr_in server;
  int sock;
  //  char deststr[80] = serverIP.c_str();
  unsigned int **addrptr;

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    perror("socket");
    return;
  }

  server.sin_family = AF_INET;
  server.sin_port = htons(10001); /* HTTPのポートは80番です */

  server.sin_addr.s_addr = inet_addr(rosServerIP.c_str());
  if (server.sin_addr.s_addr == 0xffffffff) {
    struct hostent *host;

    host = gethostbyname(rosServerIP.c_str());
    if (host == NULL) {
      if (h_errno == HOST_NOT_FOUND) {
        /* h_errnoはexternで宣言されています */
        //        fprintf(stderr,"cmd : host not found : %s\n", rosServerIP.c_str());
        fprintf(stdout,"cmd : host not found : %s\n", rosServerIP.c_str());
      } else {
        /*
          HOST_NOT_FOUNDだけ特別扱いする必要はないですが、
          とりあえず例として分けてみました
        */
        fprintf(stdout,"cmd : %s : %s\n", hstrerror(h_errno), rosServerIP.c_str());
        //fprintf(stderr,"cmd : %s : %s\n", hstrerror(h_errno), rosServerIP.c_str());
      }
      return;
    }

    addrptr = (unsigned int **)host->h_addr_list;

    while (*addrptr != NULL) {
      server.sin_addr.s_addr = *(*addrptr);

      /* connect()が成功したらloopを抜けます */
      if (connect(sock,
		  (struct sockaddr *)&server,
		  sizeof(server)) == 0) {
	break;
      }

      addrptr++;
      /* connectが失敗したら次のアドレスで試します */
    }
   
    /* connectが全て失敗した場合 */
    if (*addrptr == NULL) {
      perror("cmd : connect");
      return;
    }
  } else {
    if (connect(sock,
		(struct sockaddr *)&server, sizeof(server)) != 0) {
      perror("cmd : connect");
      return;
    }
  }
  int n;

  while (true) {
    memset(recvdata, 0, sizeof(recvdata));
    n = recv(sock, recvdata, sizeof(recvdata),0);
    if (n < 0) {
      perror("cmd : read erro");
      return;
    }else if(n == 0){
      break;
    }
    cmdRes.append(recvdata,n);
  }

  //string version
  std::vector<std::string> cmdVector;
  cmdVector = split(cmdRes,',');
  if(cmdVector.size() == 7){
    cmddata.vel.tv = atof(cmdVector[0].c_str());
    cmddata.vel.sv = atof(cmdVector[1].c_str());
    cmddata.mode = atoi(cmdVector[2].c_str());
    cmddata.gear = atoi(cmdVector[3].c_str());
    cmddata.accell = atoi(cmdVector[4].c_str());
    cmddata.steer = atoi(cmdVector[5].c_str());
    cmddata.brake = atoi(cmdVector[6].c_str());
    
    printf("cmd : linear:%f angular:%f\n",cmddata.vel.tv,cmddata.vel.sv);
  }else{
    fprintf(stderr,"cmd : Recv data is invalid\n");
  }
  printf("cmd : return data : %s\n",cmdRes.c_str());

  //struct version
  /*
  if(cmdRes.size() == sizeof(CMDDATA)){
    memcpy(&cmddata,cmdRes.data(),sizeof(CMDDATA));
    printf("cmd : linear:%f angular:%f\n",cmddata.linear_x,cmddata.angular_z);
  }else{
    fprintf(stderr,"cmd : Recv data is invalid %u\n",sizeof(CMDDATA));
  }
  */

  close(sock);

  return;
}

/*
void CMDGetter(){
  pthread_t _getter;
  while(1){
    if(pthread_create(&_getter, NULL, Getter, NULL)){
      fprintf(stderr,"cmd : pthread create error");
      return;
    }
    pthread_detach(_getter);
    usleep(10*1000);
  }
}
*/

bool Control(vel_data_t vel,vel_data_t &current,void* p) 
{
  //こいつグローバル変数だったけど何の意味があるか？
   bool IsAccelerated = false;

  //calculate current time
  vel.tstamp = getTime();
  cycle_time = vel.tstamp - current.tstamp;

  MainWindow* Main = (MainWindow*)p;

  queue<double> vel_buffer;
  static uint vel_buffer_size = 10; 
 

  double old_velocity = 0.0;
  //vel_data_t current = _shared_memory.readCurVel();

  //currentは前回更新時の値でよいのか？
  double current_velocity = current.tv*3.6;
  double current_steering_angle = current.sv;
 
  double cmd_velocity = vel.tv*3.6;
  double cmd_steering_angle = vel.sv;
 
  // estimate current acceleration
  vel_buffer.push(fabs(current_velocity));
  
  if (vel_buffer.size() > vel_buffer_size) {
    old_velocity = vel_buffer.front();
    vel_buffer.pop();
    estimate_accel = (fabs(current_velocity)-old_velocity)/(cycle_time*vel_buffer_size);
    
  }
  cout << endl << "Current " << "tv : " << current_velocity << " sv : "<< current_steering_angle << endl; 
  cout << endl << "Command " << "tv : " << cmd_velocity << " sv : "<< cmd_steering_angle << endl; 
  cout << "Estimate Accel : " << estimate_accel << endl; 

  // TRY TO INCREASE STEERING
  //  sv +=0.1*sv;

  // if tv non zero then check if in drive gear first
  //--------------------------------------------------------------
  // if in neutral and get +'ve cmd vel
  //    - put brake on
  //    - change shift to drive
  // if in neutral and vel cmd 0
  //    - release brake if pressed
  
  //ギアがNの場合DかRに
  Main->ChangeShiftMode(cmd_velocity);
   
  //------------------------------------------------------
  // if shift in drive
  //     - if cmd vel > 0 && brake pressed
  //              - release brake
  //     - if cmd vel == 0
  //              - if cur_vel < 0.1, nearly stopped
  //                     - full brake too stop, wait, and shift to neutral
  //              - else - brake to HEV_MED_BRAKE
  //                       - decellerate until nearly stopped
  // now motion if in drift mode and shift is in drive
  
 
  if (true
  //if ( _drv_state.drvInf.actualShift == SHIFT_POS_D 
       //|| _hev_state.drvInf.actualShift == SHIFT_POS_R
       ) {
    fprintf(stdout,"In Drive or Reverse\n");
    //fprintf(stderr,"In Drive or Reverse\n");
         
    // if accell 
    if (fabs(cmd_velocity) >= fabs(current_velocity) 
        && fabs(cmd_velocity) > 0.0 
        && fabs(current_velocity) <= KmhToMs(51.0) ) {

      //accelerate !!!!!!!!!!!!!!!!
      cout << "Acceleration" << endl;
      Main->AccelerateControl(current_velocity,cmd_velocity);
      IsAccelerated = true;
    }else if (fabs(cmd_velocity) < fabs(current_velocity) 
              && fabs(cmd_velocity) > 0.0) 
      {
        //decelerate!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        cout << "Deceleration" << endl;
        Main->DecelerateControl(current_velocity,cmd_velocity);
        IsAccelerated = false;
      }
    
    else if (cmd_velocity == 0.0) {
	
      //Stopping!!!!!!!!!!!
       
      Main->StoppingControl(current_velocity,cmd_velocity);
      IsAccelerated = false;
    } // if cmdvel < curvel
    
  } //if shift in drive
  else if( _drv_state.drvInf.actualShift == SHIFT_POS_N ){
    cout << "Shift value unknown or Neutral" << endl;
  }
  else {
    fprintf(stderr,"Shift value unknown.\nGetting shift value is error\n");
  }
  
  // set steering angle
  Main->SteeringControl(cmd_steering_angle);

  current.tv = vel.tv;
  current.sv = vel.sv; 
  current.tstamp = vel.tstamp;
  return true;
  
}

void MainWindow::TestPrint(){
#ifdef DEBUG
  printf("test print,%d,%d,%f(/10)\n",ndrv,nbrk,nstr);
#endif
}

void initPrintValue(){
  ndrv = 0;
  nbrk = 0;
  nstr = 0;
}


void *MainWindow::CMDGetterEntry(void *a){
  MainWindow* main = (MainWindow*)a;
  CMDDATA cmddata;
  vel_data_t current;
  //main->CMDGetter();
  memset(&current,0,sizeof(current));
  while(1){
    Getter(cmddata);    
    main->SetDrvMode(cmddata.mode);
    if(cmddata.mode == DRVMODE_PROGRAM){
      Control(cmddata.vel,current,main);
    }
    main->TestPrint();
    initPrintValue();
    
    usleep(main->cmdduration*1000);
  }
  return NULL;
}
