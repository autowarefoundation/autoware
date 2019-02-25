/*
 * Copyright 2015 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
