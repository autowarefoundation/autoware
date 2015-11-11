/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  Velodyne 3D LIDAR data input classes
 *
 *    These classes provide raw Velodyne LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     velodyne::Input -- pure virtual base class to access the data
 *                      independently of its source
 *
 *     velodyne::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     velodyne::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 */

#ifndef __VELODYNE_INPUT_H
#define __VELODYNE_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>

#include <ros/ros.h>
#include <velodyne_msgs/VelodynePacket.h>

namespace velodyne_driver
{
  static uint16_t UDP_PORT_NUMBER = 2368;

  /** @brief Pure virtual Velodyne input base class */
  class Input
  {
  public:
    Input() {}

    /** @brief Read one Velodyne packet.
     *
     * @param pkt points to VelodynePacket message
     *
     * @returns 0 if successful,
     *          -1 if end of file
     *          > 0 if incomplete packet (is this possible?)
     */
    virtual int getPacket(velodyne_msgs::VelodynePacket *pkt) = 0;


    /** @brief Set source IP, from where packets are accepted
     *
     * @param ip IP of a Velodyne LIDAR e.g. 192.168.51.70
     */
    virtual void setDeviceIP( const std::string& ip ) { devip_str_ = ip; }
  protected:
    std::string devip_str_;
  };

  /** @brief Live Velodyne input from socket. */
  class InputSocket: public Input
  {
  public:
    InputSocket(ros::NodeHandle private_nh,
                uint16_t udp_port = UDP_PORT_NUMBER);
    ~InputSocket();

    virtual int getPacket(velodyne_msgs::VelodynePacket *pkt);
    void setDeviceIP( const std::string& ip );
  private:

    int sockfd_;
    in_addr devip_;
  };


  /** @brief Velodyne input from PCAP dump file.
   *
   * Dump files can be grabbed by libpcap, Velodyne's DSR software,
   * ethereal, wireshark, tcpdump, or the \ref vdump_command.
   */
  class InputPCAP: public Input
  {
  public:
    InputPCAP(ros::NodeHandle private_nh,
              double packet_rate,
              std::string filename="",
              bool read_once=false,
              bool read_fast=false,
              double repeat_delay=0.0);
    ~InputPCAP();

    virtual int getPacket(velodyne_msgs::VelodynePacket *pkt);
    void setDeviceIP( const std::string& ip );

  private:

    std::string filename_;
    FILE *fp_;
    pcap_t *pcap_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    bool read_fast_;
    double repeat_delay_;
    ros::Rate packet_rate_;
    bpf_program velodyne_pointdata_filter_;
  };

} // velodyne_driver namespace

#endif // __VELODYNE_INPUT_H
