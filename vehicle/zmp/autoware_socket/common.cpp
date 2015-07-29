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

int can_tx_interval = 10; // ms
int cmd_rx_interval = 100; // ms
std::string ros_ip_address = "192.168.1.101";

HevState _hev_state;

bool MainWindow::ConfigSocket(void)
{
  std::ifstream ifs("./config");
  std::string str;
  if (ifs.fail()) {
    return false;
  }

  if (getline(ifs,str)) {
    can_tx_interval = atoi(str.c_str());
    cout << "CAN Interval = " << can_tx_interval << " ms" << endl;
  } else {
    return false;
  }

  if (getline(ifs, str)) {
    cmd_rx_interval = atoi(str.c_str());
    cout << "CMD Interval = " << cmd_rx_interval << " ms" << endl;
  } else {
    return false;
  }

  if (getline(ifs,str)) {
    ros_ip_address = str;
    cout << "ROS IP Address = " << ros_ip_address << endl;
  } else {
    return false;
  }

  return true;
}

void MainWindow::UpdateState(void)
{
  _hev_state.tstamp = (long long int) (getTime());
  hev->GetDrvInf(&_hev_state.drvInf);
  hev->GetStrInf(&_hev_state.strInf);
  hev->GetBrakeInf(&_hev_state.brkInf);
}

void MainWindow::ClearCntDiag(void)
{
  hev->ClearCntDiag();
}
