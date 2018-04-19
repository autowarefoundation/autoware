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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <string>
#include "adi_driver/adis16470.h"

/**
 * @brief change big endian 2 byte into short
 * @param data Head pointer to the data
 * @retrun converted value
 */
int16_t big_endian_to_short(unsigned char *data)
{
  unsigned char buff[2] = {data[1], data[0]};
  return *reinterpret_cast<int16_t*>(buff);
}

/**
 * @brief Constructor
 */
Adis16470::Adis16470()
  : fd_(-1)
{
}

/**
 * @brief Open device
 * @param device Device file name (/dev/ttyACM*)
 * @retval 0 Success
 * @retval -1 Failure
 */
int Adis16470::openPort(const std::string device)
{
  fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0)
  {
    perror("openPort");
    return -1;
  }
  if (tcgetattr(fd_, &defaults_) < 0)
  {
    perror("openPort");
    return -1;
  }
  struct termios config;
  cfmakeraw(&config);
  if (tcsetattr(fd_, TCSANOW, &config) < 0)
  {
    perror("openPort");
    return -1;
  }
  // Set SPI mode
  unsigned char buff[20] = {0};
  buff[0] = 0x5A;
  buff[1] = 0x02;  // Set mode command
  buff[2] = 0x93;  // Set SPI mode
  buff[3] = 5;  // 1MHz clock speed

  int size = write(fd_, buff, 4);
  if (size != 4)
  {
    perror("openPort");
  }
  if (tcdrain(fd_) < 0)
  {
    perror("openPort");
  }
  size = read(fd_, buff, 2);
  if (size != 2)
  {
    perror("openPort");
    return -1;
  }
  // Check first byte
  if (buff[0] != 0xff)
  {
    perror("openPort");
    return -1;
  }
  return 0;
}

/**
 * @brief Close device
 */
void Adis16470::closePort()
{
  if (tcsetattr(fd_, TCSANOW, &defaults_) < 0)
  {
    perror("closePort");
  }
  close(fd_);
}

/**
 * @param data Product ID (0x4056)
 * @retval 0 Success
 * @retval -1 Failed
 */
int Adis16470::get_product_id(int16_t& pid)
{
  // get product ID
  int r;
  unsigned char buff[20];

  // Sending data
  buff[0] = 0x61;
  buff[1] = 0x72;
  buff[2] = 0x00;
  int size = write(fd_, buff, 3);
  if (size != 3)
  {
    perror("get_product_id");
    return -1;
  }
  if (tcdrain(fd_) < 0)
  {
    perror("get_product_id");
    return -1;
  }
  size = read(fd_, buff, 3);
  if (size != 3)
  {
    perror("get_product_id");
    return -1;
  }
  // Receiving data
  buff[0] = 0x61;
  buff[1] = 0x00;
  buff[2] = 0x00;
  size = write(fd_, buff, 3);
  if (size != 3)
  {
    perror("get_product_id");
    return -1;
  }
  if (tcdrain(fd_) < 0)
  {
    perror("get_product_id");
    return -1;
  }
  size = read(fd_, buff, 3);
  if (size != 3)
  {
    perror("get_product_id");
    return -1;
  }
  // Convert to short
  pid = big_endian_to_short(&buff[1]);
  return 0;
}

/**
 * @brief Read data from the register
 * @param address Register address
 * @retval 0 Success
 * @retval -1 Failed
 * 
 * - Adress is the first byte of actual address
 * - Actual data at the adress will be returned by next call.
 */
int Adis16470::read_register(char address, int16_t& data)
{
  unsigned char buff[3] = {0x61, address, 0x00};
  int size = write(fd_, buff, 3);
  if (size != 3)
  {
    perror("read_register");
    return -1;
  }
  if (tcdrain(fd_) < 0)
  {
    perror("read_register");
    return -1;
  }
  size = read(fd_, buff, 3);
  if (size !=3)
  {
    perror("read");
  }
  data = big_endian_to_short(&buff[1]);
}

/**
 * @brief Update all information by bust read
 * @retval 0 Success
 * @retval -1 Failed
 * 
 * - See burst read function at pp.14 
 * - Data resolution is 16 bit
 */
int Adis16470::update_burst(void)
{
  unsigned char buff[64] = {0};
  // 0x6800: Burst read function
  buff[0] = 0x61;
  buff[1] = 0x68;
  buff[2] = 0x00;
  int size = write(fd_, buff, 24);
  if (size != 24)
  {
    perror("update_burst");
    return -1;
  }
  if (tcdrain(fd_) < 0)
  {
    perror("update_burst");
    return -1;
  }
  size = read(fd_, buff, 30);
  if (size != 30)
  {
    perror("update_burst");
    return -1;
  }
  int16_t diag_stat = big_endian_to_short(&buff[3]);
  if (diag_stat != 0)
  {
    fprintf(stderr, "diag_stat error: %04x\n", (uint16_t)diag_stat);
    return -1;
  }
  // X_GYRO_OUT
  gyro[0] = big_endian_to_short(&buff[5]) * M_PI / 180 / 10.0;
  // Y_GYRO_OUT
  gyro[1] = big_endian_to_short(&buff[7]) * M_PI / 180 / 10.0;
  // Z_GYRO_OUT
  gyro[2] = big_endian_to_short(&buff[9]) * M_PI / 180 / 10.0;
  // X_ACCL_OUT
  accl[0] = big_endian_to_short(&buff[11]) * M_PI / 180 / 10.0;
  // Y_ACCL_OUT
  accl[1] = big_endian_to_short(&buff[13]) * M_PI / 180 / 10.0;
  // Z_ACCL_OUT
  accl[2] = big_endian_to_short(&buff[15]) * M_PI / 180 / 10.0;

  return 0;
}

/**
 * @brief update gyro and accel in high-precision read
 */
int Adis16470::update(void)
{
  int16_t gyro_out[3], gyro_low[3], accl_out[3], accl_low[3];

  read_register(0x04, gyro_low[0]);
  read_register(0x06, gyro_low[0]);
  read_register(0x08, gyro_out[0]);
  read_register(0x0a, gyro_low[1]);
  read_register(0x0c, gyro_out[1]);
  read_register(0x0e, gyro_low[2]);
  read_register(0x10, gyro_out[2]);
  read_register(0x12, accl_low[0]);
  read_register(0x14, accl_out[0]);
  read_register(0x16, accl_low[1]);
  read_register(0x18, accl_out[1]);
  read_register(0x1a, accl_low[2]);
  read_register(0x00, accl_out[2]);

  // 32bit convert
  for (int i=0; i < 3; i++)
  {
    gyro[i] = ((int32_t(gyro_out[i]) << 16) + int32_t(gyro_low[i])) * M_PI / 180.0 / 655360.0;
    accl[i] = ((int32_t(accl_out[i]) << 16) + int32_t(accl_low[i])) * 9.8 / 52428800.0;
  }
  return 0;
}
