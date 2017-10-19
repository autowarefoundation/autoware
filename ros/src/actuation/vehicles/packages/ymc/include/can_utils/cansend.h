#ifndef MY_CANSEND_H
#define MY_CANSEND_H

#include <iostream>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace mycansend
{

class CanSender
{
private:
 int s_;
 struct ifreq ifr_;

 bool initialized_;

public:
 CanSender(const std::string& device);
 CanSender();
 ~CanSender();

 void init(const std::string& device);
 void send(char* can_frame);
};
  
}


#endif /* ifndef CANSEND_H */
