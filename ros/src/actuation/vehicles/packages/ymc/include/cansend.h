#ifndef CANSEND_H
#define CANSEND_H

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

class CanSender
{
private:
  int s_;
  struct ifreq ifr_;

public:
  CanSender();
  ~CanSender();

  void init(const std::string& device);
  void send(char* can_frame);

  static std::string makeCmd(const unsigned char data[], const size_t& size, const std::string& id);
  static std::vector<std::string> splitString(const std::string& str);
};

#endif  // CANSEND_H
