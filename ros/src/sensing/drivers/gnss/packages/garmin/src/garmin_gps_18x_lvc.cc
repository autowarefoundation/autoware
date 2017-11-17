/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2015 Denis Dillenberger, Rich Mattes
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include "garmin_gps_18x_lvc.h"

#include <sensor_msgs/Imu.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Temperature.h>

const double microseconds_to_seconds = 1.0E-6;

GpsImuDriver::GpsImuDriver():nh_("~")
{
  // Initialize
  udp_socket_ = 0;
  is_connected_ = false;
  last_gps_timestamp_ = 0;
  hour_time_ = 0;

  // Advertise topics, read parameters
//  std::cout << "advertising topics" << std::endl;
  nmea_pub = nh_.advertise<nmea_msgs::Sentence>("/nmea_sentence",10);
//  imu_pub = nh_.advertise<sensor_msgs::Imu>("/imu_data",100);
  gpstime_pub = nh_.advertise<sensor_msgs::TimeReference>("/gpstime",10);
//  temperature_pub = nh_.advertise<sensor_msgs::Temperature>("/temperature",10);
  nh_.param("device_ip",devip_,std::string(""));
  nh_.param("udpport",udpport_,8308);

  // Start listening at UDP port
  bind(udpport_);
}

//-----------------------------------------------------------------------------
bool GpsImuDriver::bind(const int udp_port)
{
  if( isConnected() )
    disconnect();

  try
  {
    // Bind socket
      udp_socket_ = new boost::asio::ip::udp::socket(io_service_, boost::asio::ip::udp::v4());
      udp_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 8308));
      // Start async reading
      asyncReceiveFrom();
      io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
      is_connected_ = true;
  }
  catch (std::exception& e)
  {
      std::cerr << "Exception: " <<  e.what() << std::endl;
      return false;
  }
//  std::cout << "Receiving Velodyne IMU data at local UDP port " << udp_port << "." << std::endl;

  return true;
}

//-----------------------------------------------------------------------------
void GpsImuDriver::asyncReceiveFrom()
{
  udp_socket_->async_receive_from(boost::asio::buffer(&udp_buffer_[0],udp_buffer_.size()), udp_endpoint_,
                                  boost::bind(&GpsImuDriver::handleSocketRead, this,
                                              boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

//-----------------------------------------------------------------------------
void GpsImuDriver::handleSocketRead(const boost::system::error_code &error, std::size_t bytes_transferred)
{
  if (!error )
  {
    if( !devip_.empty() && udp_endpoint_.address().to_string() != devip_ )
    {
      asyncReceiveFrom();
      return;
    }

    // Handle packet
    velodyne_packet_structs::VelodynePositioningPacketRaw vppr;
    std::memcpy(&vppr,&udp_buffer_[0],sizeof(vppr));
    handlePacket(vppr);
    asyncReceiveFrom();
  }
  else
  {
      if( error.value() != 995 )
      {
        std::cerr << "ERROR: " << "data connection error: " << error.message() << "(" << error.value() << ")" << std::endl;
        std::cout << "Trying to rebind socket" << std::endl;
        if( bind(udpport_) )
          std::cout << "Rebind sucessful" << std::endl;
        else
          std::cerr << "" << std::endl;
       }
  }
  last_data_time_ = std::time(0);
}

int16_t convert12bit2int16(uint16_t v)
{
  v = v & 0x0fff;
  int16_t r=v;
  if(r>2047)
      r=-((~(r--)) & 0x0fff);
  return r;
}

std::vector<std::string> explode(const std::string& text, const std::string& separators)
{
  std::vector<std::string> words;
  size_t n     = text.length ();
  size_t start = 0;
  size_t stop = text.find_first_of (separators);
  if (stop > n) stop = n;

  while (start <= n)
  {
    words.push_back (text.substr (start, stop-start));
    start = stop+1;
    stop = text.find_first_of (separators, start);
    if (stop > n) stop = n;
  }
  return words;
}

uint32_t nmeaHourToUnixTime(const std::string& nmea_sentence)
{
  std::vector< std::string > words = explode(nmea_sentence,",");
  if(words.size() < 10 )
      return -1;
  uint32_t hour = atoi(words[1].substr(0,2).c_str());

  uint32_t day = atoi(words[9].substr(0,2).c_str());
  uint32_t mon = atoi(words[9].substr(2,2).c_str());
  uint32_t year = atoi(words[9].substr(4,2).c_str());

  time_t hour_time;
  tm time_info;
  memset(&time_info, 0, sizeof(time_info));
  // Assumes year is between 2000 and 2099
  time_info.tm_year = year + 100;
  time_info.tm_mon = mon - 1;
  time_info.tm_mday = day;
  time_info.tm_hour = hour;
  hour_time = timegm(&time_info);

  return hour_time;
}

//-----------------------------------------------------------------------------
bool GpsImuDriver::handlePacket(velodyne_packet_structs::VelodynePositioningPacketRaw &vppr)
{

  velodyne_packet_structs::VelodynePositioningPacket vpp;

  for( int i=0; i<3; i++ )
  {
      vpp.gyro_temp_accel[i].gyro = convert12bit2int16(vppr.gyro_temp_accel[i].gyro) * 0.09766 * M_PI / 180.0;
      vpp.gyro_temp_accel[i].temp = convert12bit2int16(vppr.gyro_temp_accel[i].temp) * 0.1453+25;
      vpp.gyro_temp_accel[i].accel_x = convert12bit2int16(vppr.gyro_temp_accel[i].accel_x) * 0.001221;
      vpp.gyro_temp_accel[i].accel_y = convert12bit2int16(vppr.gyro_temp_accel[i].accel_y) * 0.001221;
  }

  const float earth_gravity = 9.80665;

  vpp.gps_timestamp = vppr.gps_timestamp * microseconds_to_seconds;
  vpp.gyro_temp_accel_xyz[0].gyro = vpp.gyro_temp_accel[1].gyro;
  vpp.gyro_temp_accel_xyz[0].temp = vpp.gyro_temp_accel[1].temp;
  vpp.gyro_temp_accel_xyz[0].accel_x = -vpp.gyro_temp_accel[0].accel_y * earth_gravity;
  vpp.gyro_temp_accel_xyz[0].accel_y = vpp.gyro_temp_accel[2].accel_x * earth_gravity;

  vpp.gyro_temp_accel_xyz[1].gyro = -vpp.gyro_temp_accel[0].gyro;
  vpp.gyro_temp_accel_xyz[1].temp = vpp.gyro_temp_accel[0].temp;
  vpp.gyro_temp_accel_xyz[1].accel_x = -vpp.gyro_temp_accel[1].accel_y * earth_gravity;
  vpp.gyro_temp_accel_xyz[1].accel_y = -vpp.gyro_temp_accel[2].accel_y * earth_gravity;

  vpp.gyro_temp_accel_xyz[2].gyro = -vpp.gyro_temp_accel[2].gyro;
  vpp.gyro_temp_accel_xyz[2].temp = vpp.gyro_temp_accel[2].temp;
  vpp.gyro_temp_accel_xyz[2].accel_x = -vpp.gyro_temp_accel[0].accel_x * earth_gravity;
  vpp.gyro_temp_accel_xyz[2].accel_y = -vpp.gyro_temp_accel[1].accel_x * earth_gravity;

  nmea_msgs::Sentence nmea_sentence_msg;
  std::string nmea_sentence(vppr.nmea_sentence);

  /* This section re-constructs the timestamp of the packet based on the
   * GPS NMEA time and the packet's microsecond counter.  The microsecond counter
   * contains the time from the beginning of the GPS hour, so to reconstruct the
   * packet's scan time, the counter value needs to be added to the time of the 
   * beginning of the hour.
   */
  // Find the current hour using the NMEA sentence from the Velodyne sensor message
  double nmeaHourUnixTime = nmeaHourToUnixTime(nmea_sentence);
  // If the time is un-initialized, or if the seconds counter in the counter is 
  // greater than 2, set the hour time (the 2 second delay allows for the NMEA
  // string to be updated as it is delayed by several hundred milliseconds
  // from the GPS PPS event)
  if (hour_time_ == 0 || vpp.gps_timestamp > 2.0) {
      hour_time_ = nmeaHourUnixTime;
  }
  // The seconds counter has rolled over - increment the hour. 
  else if (vppr.gps_timestamp < last_gps_timestamp_) {
      hour_time_ += 3600;
  }
  last_gps_timestamp_ = vppr.gps_timestamp;

  // Publish all topics with the same ROS time stamp.
  ros::Time topic_publish_time = ros::Time::now();
  // === Time Reference Message ===
  // Set the TimeReference time_ref with the re-constructed sensor time
  double sensor_scan_time = (double)hour_time_ + vpp.gps_timestamp ;
  sensor_msgs::TimeReference gpstime_msg;
  gpstime_msg.header.stamp = topic_publish_time;
  gpstime_msg.time_ref = ros::Time(sensor_scan_time);
  gpstime_msg.source = std::string("Sensor On-Board Clock");
  gpstime_pub.publish(gpstime_msg);

  // === NMEA Sentence ===
  nmea_sentence = nmea_sentence.substr(0,nmea_sentence.length()-2);
  nmea_sentence_msg.sentence = nmea_sentence;
  nmea_sentence_msg.header.stamp = topic_publish_time;
  nmea_sentence_msg.header.frame_id = "/gps";
  nmea_pub.publish(nmea_sentence_msg);

  // === IMU Message ===
  sensor_msgs::Imu imumsg;
  imumsg.header.frame_id="/velodyne:"+devip_;
  imumsg.header.stamp = topic_publish_time;

  imumsg.linear_acceleration.x = (vpp.gyro_temp_accel_xyz[0].accel_x+vpp.gyro_temp_accel_xyz[0].accel_y)/2.0;
  imumsg.linear_acceleration.y = (vpp.gyro_temp_accel_xyz[1].accel_x+vpp.gyro_temp_accel_xyz[1].accel_y)/2.0;
  imumsg.linear_acceleration.z = (vpp.gyro_temp_accel_xyz[2].accel_x+vpp.gyro_temp_accel_xyz[2].accel_y)/2.0;
  imumsg.angular_velocity.x = vpp.gyro_temp_accel_xyz[0].gyro;
  imumsg.angular_velocity.y = vpp.gyro_temp_accel_xyz[1].gyro;
  imumsg.angular_velocity.z = vpp.gyro_temp_accel_xyz[2].gyro;
  imumsg.orientation_covariance[0] = -1;
  imu_pub.publish(imumsg);

  // === Temperature Messages ===
  for( std::size_t i=0; i<3; i++ )
  {
    sensor_msgs::Temperature tmpmsg;
    tmpmsg.header.stamp = topic_publish_time;
    std::stringstream ss;
    ss << "/velodyne:" << devip_ << i;
    tmpmsg.header.frame_id = ss.str();
    tmpmsg.header.stamp = ros::Time::now();
    tmpmsg.temperature = vpp.gyro_temp_accel_xyz[i].temp;
    temperature_pub.publish(tmpmsg);
  }
}

//-----------------------------------------------------------------------------
void GpsImuDriver::disconnect()
{
    is_connected_ = false;
    try
    {
        if( udp_socket_ )
            udp_socket_->close();
        io_service_.stop();
        if( boost::this_thread::get_id() != io_service_thread_.get_id() )
            io_service_thread_.join();
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
    }
}

//-----------------------------------------------------------------------------
GpsImuDriver::~GpsImuDriver()
{
  disconnect();
  delete udp_socket_;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "garmin_gps_18x_lvc", ros::init_options::AnonymousName);
    GpsImuDriver* gid = new GpsImuDriver();
    ros::spin();
    delete gid;
    return 0;
}
