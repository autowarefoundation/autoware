/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "mainwindow.h"
#include "autoware_socket.h"

// CAN + Drive Mode
std::string candata;
int drvmode;

void *CANSenderEntry(void *a)
{
  struct sockaddr_in server;
  int sock;
  std::string senddata; 
  std::ostringstream oss;
  oss << CAN_KEY_MODE << "," << drvmode << "," << candata; // global variables.
  senddata = oss.str();
  unsigned int **addrptr;

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    perror("socket");
    return NULL;
  }

  server.sin_family = AF_INET;
  server.sin_port = htons(10000);

  server.sin_addr.s_addr = inet_addr(ros_ip_address.c_str());
  if (server.sin_addr.s_addr == 0xffffffff) {
    struct hostent *host;

    host = gethostbyname(ros_ip_address.c_str());
    if (host == NULL) {
      if (h_errno == HOST_NOT_FOUND) {
        fprintf(stderr,"info : host not found : %s\n", ros_ip_address.c_str());
      } else {
        fprintf(stderr,"info : %s : %s\n", hstrerror(h_errno), ros_ip_address.c_str());
      }
      return NULL;
    }

    addrptr = (unsigned int **)host->h_addr_list;

    while (*addrptr != NULL) {
      server.sin_addr.s_addr = *(*addrptr);
      
      // if connected, break out this loop.
      if (connect(sock,
                  (struct sockaddr *)&server,
                  sizeof(server)) == 0) {
        break;
      }

      addrptr++;
      // if failed to connect, try another address.
    }

    // if totally failed to connect, return NULL.
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
  sprintf(tmp, "%d,'%d/%02d/%02d %02d:%02d:%02d.%ld'",
          CAN_KEY_TIME,
          _s_time->tm_year+1900, _s_time->tm_mon+1, _s_time->tm_mday,
          _s_time->tm_hour + 9, _s_time->tm_min, _s_time->tm_sec, 
          _getTime.tv_usec);
  can += tmp;

  if (_selectLog.drvInf == true) {
    sprintf(tmp, ",%d,%3.2f,%d,%d,%d,%d",
            CAN_KEY_VELOC, _drvInf.veloc,
            CAN_KEY_ACCEL, _drvInf.actualPedalStr,
            CAN_KEY_SHIFT, _drvInf.actualShift);
    can += tmp;
  }
  if (_selectLog.strInf == true) {
    sprintf(tmp, ",%d,%3.2f,%d,%d",
            CAN_KEY_ANGLE, _strInf.angle,
            CAN_KEY_TORQUE, _strInf.torque);
    can += tmp;
  }
  if (_selectLog.brkInf == true) {
    sprintf(tmp, ",%d,%d", CAN_KEY_BRAKE, _brakeInf.actualPedalStr);
    can += tmp;
  }
  
  candata = can;

  // send drive mode in addition to CAN.
  UpdateState();
  drvmode = ZMP_STR_CONTROLLED() ? CAN_MODE_STR:0;
  drvmode |= ZMP_DRV_CONTROLLED() ? CAN_MODE_DRV:0;
  
  wrapSender();
}
