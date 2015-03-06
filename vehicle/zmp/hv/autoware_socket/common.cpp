#include "mainwindow.h"
#include "autoware_socket.h"

#include <sstream>

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
    cout << "CMD Interval = " << can_tx_interval << " ms" << endl;
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
