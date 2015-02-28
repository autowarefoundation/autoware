#include "mainwindow.h"
#include "data.h"

#include <sstream>

std::string rosServerIP;
std::string candata;
int drvmode;

DrvState _drv_state;

int CheckDrvMode()
{  
  int drv_mode = _drv_state.drvInf.mode; // 0x00 : manual ; 0x10 : program
  int drv_servo = _drv_state.drvInf.servo; // 0x00 : OFF 0x10 :ON
  int str_mode = _drv_state.strInf.mode; // 0x00 : manual ; 0x10 : program
  int str_servo = _drv_state.strInf.servo; // 0x00 : OFF 0x10 :ON

  // cout <<  "===============CheckHevMode===============" << endl;
  if(drv_mode == 0x10 && drv_servo == 0x10 && str_mode == 0x10 && str_servo == 0x10){
    return DRVMODE_PROGRAM;
  }else{
    return DRVMODE_MANUAL;
  }
}

void *SendData(void *a){
//void SendData(){

  struct sockaddr_in server;
  int sock;
  //  char deststr[80] = serverIP.c_str();
  std::string senddata; 
  std::ostringstream oss;
  oss << candata << "," << drvmode;//candata and drvmode is global value.
  senddata = oss.str();
  unsigned int **addrptr;

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    perror("socket");
    return NULL;
  }

  server.sin_family = AF_INET;
  server.sin_port = htons(10000); /* HTTPのポートは80番です */

  server.sin_addr.s_addr = inet_addr(rosServerIP.c_str());
  if (server.sin_addr.s_addr == 0xffffffff) {
    struct hostent *host;

    host = gethostbyname(rosServerIP.c_str());
    if (host == NULL) {
      if (h_errno == HOST_NOT_FOUND) {
        /* h_errnoはexternで宣言されています */
        fprintf(stderr,"info : host not found : %s\n", rosServerIP.c_str());
      } else {
        /*
          HOST_NOT_FOUNDだけ特別扱いする必要はないですが、
          とりあえず例として分けてみました
        */
        fprintf(stderr,"info : %s : %s\n", hstrerror(h_errno), rosServerIP.c_str());
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

void MainWindow::wrapSender(void){

  pthread_t _sendData;
  
  if(pthread_create(&_sendData, NULL, SendData, NULL)){
    fprintf(stderr,"info : pthread create error");
    return;
  }

  usleep(canduration*1000);
  pthread_join(_sendData,NULL);
}

void MainWindow::sendDataGetAndSend()
{

  char temp[300] = "";
  string canData = "";

  //時間は送信時に付け足すことにする
   
  sprintf(temp,"'%d/%02d/%02d %02d:%02d:%02d.%ld',",_s_time->tm_year+1900, _s_time->tm_mon+1, _s_time->tm_mday,
          _s_time->tm_hour + 9, _s_time->tm_min, _s_time->tm_sec, _getTime.tv_usec);
  canData += temp;
    
    if(_selectLog.drvInf == true){

        /*ここ改造*/
        sprintf(temp,"%d,%d,%d,%d,%d,%d,%d,%3.2f,%3.2f,%d,%d,%d,",
                _drvInf.mode, _drvInf.contMode, _drvInf.overrideMode, _drvInf.servo,
                _drvInf.actualPedalStr, _drvInf.targetPedalStr, _drvInf.inputPedalStr,
                _drvInf.targetVeloc, _drvInf.veloc,
                _drvInf.actualShift, _drvInf.targetShift, _drvInf.inputShift);
        canData += temp;
        

    } else {
        /*ここ改造*/
        sprintf(temp,"0,0,0,0,0,0,0,0,0,0,0,0,");
        canData += temp;
    }
    if(_selectLog.strInf == true){
        /*ここ改造*/
        sprintf(temp,"%d,%d,%d,%d,%d,%d,%3.2f,%3.2f,",
                _strInf.mode, _strInf.cont_mode, _strInf.overrideMode, _strInf.servo,
                _strInf.targetTorque, _strInf.torque,
                _strInf.angle, _strInf.targetAngle);
        canData += temp;

    } else {
        /*ここ改造*/
        sprintf(temp,"0,0,0,0,0,0,0,0,");
        canData += temp;

    }
    if(_selectLog.brkInf == true){
        /*ここ改造*/
        sprintf(temp,"%d,%d,%d,%d,",
                _brakeInf.pressed, _brakeInf.actualPedalStr, _brakeInf.targetPedalStr, _brakeInf.inputPedalStr);
        canData += temp;
        
    } else {
        /*ここ改造*/
        sprintf(temp,"0,0,0,0,");
        canData += temp;

    }
    if(_selectLog.battInf == true){

        /*ここ改造*/
        sprintf(temp,"%3.2f,%d,%3.2f,%d,%d,%3.2f,%3.2f,",
                       _battInf.soc, _battInf.voltage, _battInf.current,
                       _battInf.max_temp, _battInf.min_temp,
                       _battInf.max_chg_current, _battInf.max_dischg_current);
        canData += temp;

    } else {
        /*ここ改造*/
        sprintf(temp,"0,0,0,0,0,0,0,");
        canData += temp;

    }
    if(_selectLog.otherInf == true){
        /*ここ改造*/
        sprintf(temp,"%3.3f,%3.5f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%d,%d,%d,%3.1f,%d,%d,%d,%d,%d,%d,%d",
                _otherInf.sideAcc, _otherInf.acc, _otherInf.angleFromP, _otherInf.brkPedalStrFromP,
                _otherInf.velocFrFromP, _otherInf.velocFlFromP, _otherInf.velocRrFromP, _otherInf.velocRlFromP,
                _otherInf.velocFromP2,
                _otherInf.drv_mode, _otherInf.drvPedalStrFromP, _otherInf.rpm,
                _otherInf.velocFlFromP,
                _otherInf.ev_mode, _otherInf.temp, _otherInf.shiftFromPrius, _otherInf.light,
                _otherInf.level, _otherInf.door, _otherInf.cluise);

        canData += temp;

    } else {
      /*ここ改造*/
      sprintf(temp,"0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
      canData += temp;
    }

    //sprintf(temp,"\n");
    candata = canData;
    hev->GetDrvInf(&_drv_state.drvInf);
    hev->GetStrInf(&_drv_state.strInf);
    hev->GetBrakeInf(&_drv_state.brkInf);
    drvmode = CheckDrvMode();
    wrapSender();

}

bool MainWindow::setConfig(){
  
  canduration = 10; //ログをとる間隔はデフォルトで10ms
  cmdduration = 100;
  rosServerIP = "192.168.1.101";

  std::ifstream ifs("./config");
  std::string str;
  if(ifs.fail()){
    return false;
  }

  if(getline(ifs,str)){
    canduration = atoi(str.c_str());
  }else{
    return false;
  }

  if(getline(ifs,str)){
    rosServerIP = str;
  }else{
    return false;
  }

  return true;
}




