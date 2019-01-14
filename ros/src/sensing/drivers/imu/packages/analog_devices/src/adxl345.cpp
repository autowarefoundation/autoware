// Copyright (c) 2017, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>
#include "ros/ros.h"
#include "adi_driver/adxl345.h"

adxl345::Imu::Imu()
  : fd_(-1)
{
}

int adxl345::Imu::open_device(const std::string device)
{
  struct termios config;

  fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0)
  {
    return -1;
  }
  if (tcgetattr(fd_, &defaults_) < 0)
  {
    perror("tcgetattr");
    return -1;
  }
  cfmakeraw(&config);
  if (tcsetattr(fd_, TCSANOW, &config) < 0)
  {
    perror("tcsetattr config");
    return -1;
  }
  // set SPI mode
  unsigned char buff[20];
  buff[0] = 0x5A;
  buff[1] = 0x02;  // Set mode command
  buff[2] = 0x93;  // Set SPI mode 3
  // buff[3] = 5;  // 1MHz clock speed
  buff[3] = 11;  // 500kHz clock speed

  if (write(fd_, buff, 4) < 0)
  {
    ROS_ERROR("write error");
  }
  if (tcdrain(fd_) < 0)
  {
    ROS_ERROR("set_spi_mode tcdrain");
  }
  if (read(fd_, buff, 4) < 0)
  {
    ROS_ERROR("set_spi_mode read");
  }
  // Read back error byte
  if (buff[0] != 0xFF)
  {
    ROS_ERROR("**set_spi_mode: Error setting spi mode!**\n\n");
  }
  write_address(0x2d, 0x08);

  return 0;
}

void adxl345::Imu::close_device()
{
  if (tcsetattr(fd_, TCSANOW, &defaults_) < 0)
  {
    perror("tcsetattr defaults");
  }
  close(fd_);
}

/**
 * @return 0: Success, 1: Failed
 */
int adxl345::Imu::get_product_id(unsigned char& data)
{
  data = read_address(0x00);
  return 0;
}

char adxl345::Imu::read_address(char address)
{
  char buff[3];
  buff[0] = 0x61;
  buff[1] = address | (0x01 << 7);
  buff[2] = 0x00;
  if (write(fd_, buff, 3) < 0)
  {
    perror("write");
  }
  if (tcdrain(fd_) < 0)
  {
    ROS_ERROR("tcdrain");
  }
  int size = read(fd_, buff, 3);
  if (size !=3)
  {
    perror("read");
  }
  return buff[2];
}

int16_t adxl345::Imu::read_short(char address)
{
  char buff[4];
  buff[0] = 0x61;
  buff[1] = address | (0x03 << 6);
  buff[2] = 0x00;
  buff[3] = 0x00;
  if (write(fd_, buff, 4) < 0)
  {
    perror("write");
  }
  if (tcdrain(fd_) < 0)
  {
    ROS_ERROR("tcdrain");
  }
  int size = read(fd_, buff, 4);
  if (size !=4)
  {
    perror("read");
  }
  // printf("%2x %02x\n", (unsigned char)buff[2], (unsigned char)buff[3]);
  return *reinterpret_cast<uint16_t*>(&buff[2]);
}


char adxl345::Imu::write_address(char address, char data)
{
  char buff[3];
  buff[0] = 0x61;
  buff[1] = address;
  buff[2] = data;
  if (write(fd_, buff, 3) < 0)
  {
    perror("write");
  }
  if (tcdrain(fd_) < 0)
  {
    ROS_ERROR("tcdrain");
  }
  int size = read(fd_, buff, 3);
  if (size !=3)
  {
    perror("read");
  }
  return buff[2];
}

int adxl345::Imu::update(void)
{
  accl[0] = read_short(0x32) * 2.0 * 9.8 / 511.0;
  accl[1] = read_short(0x34) * 2.0 * 9.8 / 511.0;
  accl[2] = read_short(0x36) * 2.0 * 9.8 / 511.0;
  return 0;
}
