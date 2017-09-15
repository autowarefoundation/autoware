/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2015 Denis Dillenberger, Rich Mattes
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */


#ifndef IMUGPS_NODE_H
#define IMUGPS_NODE_H

#include <ros/ros.h>

#include <string>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <array>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
namespace velodyne_packet_structs
{
  #pragma pack(1)

struct GyroTempAccelBlockRaw
{
    uint16_t gyro;
    uint16_t temp;
    uint16_t accel_x;
    uint16_t accel_y;
};

struct VelodynePositioningPacketRaw
{
    char _not_used1[14];
    GyroTempAccelBlockRaw gyro_temp_accel[3];
    char _not_used2[160];
    uint32_t gps_timestamp;
    char _not_used3[4];
    char nmea_sentence[72];
    char _not_used4[234];
};

struct GyroTempAccelBlock
{
    float gyro;
    float temp;
    float accel_x;
    float accel_y;
};


struct VelodynePositioningPacket
{
    GyroTempAccelBlock gyro_temp_accel[3];
    GyroTempAccelBlock gyro_temp_accel_xyz[3];
    double gps_timestamp;
    char nmea_sentence[72];
};

#pragma pack()
}

class GpsImuDriver
{
public:
  //! Open UDP port 8308 and listen on it
  GpsImuDriver();

  //! Disconnect cleanly
  ~GpsImuDriver();

  //! Bind socket to UDP Port, init IO thread
  //! @return true on success, false otherwise
  bool bind(const int udp_port);

  //! Return connection status
  bool isConnected() const { return is_connected_; }

  //! Disconnect and cleanup
  void disconnect();

private:
  //! Private NodeHandle
  ros::NodeHandle nh_;

  //! NMEA sentence publisher
  ros::Publisher nmea_pub;

  //! IMU data publisher
  ros::Publisher imu_pub;

  //! GPS time publisher
  ros::Publisher gpstime_pub;

  //! GPS temperature publisher
  ros::Publisher temperature_pub;

  //! IP of LIDAR, only packets from that address are accepted
  std::string devip_;

  //! IP of LIDAR, only packets from that address are accepted
  int udpport_;

  //! Asynchronous callback function, called if data has been reveived by the UDP socket
  void handleSocketRead(const boost::system::error_code& error, std::size_t bytes_transferred);

  //! Start asynchronous receiving
  void asyncReceiveFrom();

  //! Try to read and parse next packet from the buffer
  //! @returns True if a packet has been parsed, false otherwise
  bool handlePacket(velodyne_packet_structs::VelodynePositioningPacketRaw &vppr);

  //! Event handler thread
  boost::thread io_service_thread_;
  boost::asio::io_service io_service_;

  //! Receiving socket
  boost::asio::ip::udp::socket* udp_socket_;

  //! Endpoint in case of UDP receiver
  boost::asio::ip::udp::endpoint udp_endpoint_;

  //! Internal connection state
  bool is_connected_;

  //! Buffer of UDP receiver
  std::array< char, 65536 > udp_buffer_;

  //! time in seconds since epoch, when last data was received from sensor
  //! can be used as sensor data watchdog
  double last_data_time_;

  //! time of the last gps timestamp in the raw packet message.
  //! Used to detect hour rollover
  uint32_t last_gps_timestamp_;

  //! time in seconds since epoch of the top of the current hour
  //! derived from NMEA string in raw packets, used to offset
  //! time since hour gps_timestamp in raw packets.
  time_t hour_time_;
};

#endif // IMUGPS_NODE_H
