#include "mainwindow.h"
#include "autoware_socket.h"

#include <sstream>

std::string candata;
int drvmode;

int CheckDrvMode()
{  
  int drv_mode = _hev_state.drvInf.mode; // 0x00 : manual ; 0x10 : program
  int drv_servo = _hev_state.drvInf.servo; // 0x00 : OFF 0x10 :ON
  int str_mode = _hev_state.strInf.mode; // 0x00 : manual ; 0x10 : program
  int str_servo = _hev_state.strInf.servo; // 0x00 : OFF 0x10 :ON

  // cout <<  "===============CheckHevMode===============" << endl;
  if(drv_mode == 0x10 && drv_servo == 0x10 && str_mode == 0x10 && str_servo == 0x10){
    return CMD_MODE_PROGRAM;
  }else{
    return CMD_MODE_MANUAL;
  }
}

void *CANSenderEntry(void *a)
{
  struct sockaddr_in server;
  int sock;
  //  char deststr[80] = serverIP.c_str();
  std::string senddata; 
  std::ostringstream oss;
  oss << candata << "," << drvmode; //candata and drvmode are global.
  senddata = oss.str();
  unsigned int **addrptr;

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    perror("socket");
    return NULL;
  }

  server.sin_family = AF_INET;
  server.sin_port = htons(10000); /* HTTPのポートは80番です */

  server.sin_addr.s_addr = inet_addr(ros_ip_address.c_str());
  if (server.sin_addr.s_addr == 0xffffffff) {
    struct hostent *host;

    host = gethostbyname(ros_ip_address.c_str());
    if (host == NULL) {
      if (h_errno == HOST_NOT_FOUND) {
        /* h_errnoはexternで宣言されています */
        fprintf(stderr,"info : host not found : %s\n", ros_ip_address.c_str());
      } else {
        /*
          HOST_NOT_FOUNDだけ特別扱いする必要はないですが、
          とりあえず例として分けてみました
        */
        fprintf(stderr,"info : %s : %s\n", hstrerror(h_errno), ros_ip_address.c_str());
      }
      return NULL;
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
      perror("info : connect");
      return NULL;
    }
  } else {
    if (connect(sock,
		(struct sockaddr *)&server, sizeof(server)) != 0) {
      perror("info : connect");
      return NULL;
    }
  }

  int n;

  printf("info : %s\n",senddata.c_str());
    
  n = send(sock, senddata.c_str(), senddata.size()+1,0);
  if (n < 0) {
    perror("write");
    return NULL;
  }
    
  //while (n > 0) {
  /*
  memset(recvdata, 0, sizeof(recvdata));
  n = recv(sock, recvdata, sizeof(recvdata),0);
  if (n < 0) {
    perror("read");
    return;
  }
  */
  close(sock);

  return NULL;
}

void wrapSender(void)
{
  pthread_t _cansender;
  
  if(pthread_create(&_cansender, NULL, CANSenderEntry, NULL)){
    fprintf(stderr,"info : pthread create error");
    return;
  }

  usleep(can_tx_interval*1000);
  pthread_join(_cansender, NULL);
}

void MainWindow::SendCAN(void)
{
  char tmp[300] = "";
  string can = "";

  // add time when sent out.
  sprintf(tmp,"'%d/%02d/%02d %02d:%02d:%02d.%ld',",
          _s_time->tm_year+1900, _s_time->tm_mon+1, _s_time->tm_mday,
          _s_time->tm_hour + 9, _s_time->tm_min, _s_time->tm_sec, 
          _getTime.tv_usec);

  can += tmp;
    
  if(_selectLog.drvInf == true){
    sprintf(tmp,"%d,%d,%d,%d,%d,%d,%d,%3.2f,%3.2f,%d,%d,%d,",
            _drvInf.mode, _drvInf.contMode, _drvInf.overrideMode, 
            _drvInf.servo, _drvInf.actualPedalStr, _drvInf.targetPedalStr, 
            _drvInf.inputPedalStr,
            _drvInf.targetVeloc, _drvInf.veloc,
            _drvInf.actualShift, _drvInf.targetShift, _drvInf.inputShift);
    can += tmp;
  } else {
    sprintf(tmp,"0,0,0,0,0,0,0,0,0,0,0,0,");
    can += tmp;
  }
  if(_selectLog.strInf == true){
    sprintf(tmp,"%d,%d,%d,%d,%d,%d,%3.2f,%3.2f,",
            _strInf.mode, _strInf.cont_mode, _strInf.overrideMode, 
            _strInf.servo,
            _strInf.targetTorque, _strInf.torque,
            _strInf.angle, _strInf.targetAngle);
    can += tmp;
  } else {
    sprintf(tmp,"0,0,0,0,0,0,0,0,");
    can += tmp;
  }
  if(_selectLog.brkInf == true){
    sprintf(tmp,"%d,%d,%d,%d,",
            _brakeInf.pressed, _brakeInf.actualPedalStr, 
            _brakeInf.targetPedalStr, _brakeInf.inputPedalStr);
    can += tmp;
  } else {
    sprintf(tmp,"0,0,0,0,");
    can += tmp;
  }
  if(_selectLog.battInf == true){
    sprintf(tmp,"%3.2f,%d,%3.2f,%d,%d,%3.2f,%3.2f,",
            _battInf.soc, _battInf.voltage, _battInf.current,
            _battInf.max_temp, _battInf.min_temp,
            _battInf.max_chg_current, _battInf.max_dischg_current);
    can += tmp;
  } else {
    sprintf(tmp,"0,0,0,0,0,0,0,");
    can += tmp;
  }
  if(_selectLog.otherInf == true){
    sprintf(tmp,"%3.3f,%3.5f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%d,%d,%d,%3.1f,%d,%d,%d,%d,%d,%d,%d",
            _otherInf.sideAcc, _otherInf.acc, _otherInf.angleFromP, 
            _otherInf.brkPedalStrFromP, _otherInf.velocFrFromP, 
            _otherInf.velocFlFromP, _otherInf.velocRrFromP, 
            _otherInf.velocRlFromP,
            _otherInf.velocFromP2,
            _otherInf.drv_mode, _otherInf.drvPedalStrFromP, _otherInf.rpm,
            _otherInf.velocFlFromP,
            _otherInf.ev_mode, _otherInf.temp, _otherInf.shiftFromPrius, 
            _otherInf.light,
            _otherInf.level, _otherInf.door, _otherInf.cluise);
    
    can += tmp;
  } else {
    sprintf(tmp,"0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
    can += tmp;
  }
  
  //sprintf(tmp,"\n");
  candata = can;
  
  UpdateState();
  drvmode = CheckDrvMode();
  
  wrapSender();
}






