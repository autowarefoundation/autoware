#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <fstream>
#include <iomanip>
#include <boost/thread.hpp>
#include <sys/time.h>
#include <boost/lambda/lambda.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <unistd.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
using namespace boost::lambda;
using namespace std;

class mmtimer{
public:
  mmtimer(void) {
    restart();
  }
  virtual ~mmtimer(void) {}
  double elapsed() {
    timeval tv2;
    gettimeofday(&tv2, NULL);
    double time2 = tv2.tv_sec + (double)tv2.tv_usec*1e-6;
    return time2-_time;
  }
  void restart() {
    timeval tv;
    gettimeofday(&tv, NULL);
    _time = tv.tv_sec + (double)tv.tv_usec*1e-6;
  }
private:
  double _time;
};


const int MAXQUEUE = 500;
struct QUEUE_TYPE
{
  int count;
  int front;
  int rear;
  char entry[MAXQUEUE];
};
struct XBOW_PACKET
{
  unsigned short packet_type;
  char length;
  unsigned short crc;
  char data[256];
};
QUEUE_TYPE circ_buf;

int process_xbow_packet(QUEUE_TYPE *queue_ptr, XBOW_PACKET *result);
unsigned short calcCRC(QUEUE_TYPE *queue_ptr, unsigned int startIndex, unsigned int num);
void Initialize(QUEUE_TYPE *queue_ptr);
int AddQueue(char item, QUEUE_TYPE *queue_ptr);
int DeleteQueue(char *item, QUEUE_TYPE *queue_ptr);
char peekByte(QUEUE_TYPE *queue_ptr, unsigned int index);
unsigned short peekWord(QUEUE_TYPE *queue_ptr, unsigned int index);
int Pop(QUEUE_TYPE *queue_ptr, int numToPop);
int Size(QUEUE_TYPE *queue_ptr);
int Empty(QUEUE_TYPE *queue_ptr);
int Full(QUEUE_TYPE *queue_ptr);

/*******************************************************************************
* FUNCTION: process_xbow_packet looks for packets in a queue
* ARGUMENTS: queue_ptr: is pointer to queue to process
* result: will contain the parsed info when return value is 1
* RETURNS: 0 when failed.
* 1 when successful
*******************************************************************************/
int process_xbow_packet(QUEUE_TYPE *queue_ptr, XBOW_PACKET *result)
{
  unsigned short myCRC = 0, packetCRC = 0, numToPop=0, counter=0;
//	unsigned short packet_type = 0;
//	char packet[100], tempchar, dataLength;
  char dataLength;
  if(Empty(queue_ptr))
  {
    return 0; /* empty buffer */
  }
  /* find header */
  for(numToPop=0; numToPop+1<Size(queue_ptr) ;numToPop+=1)
  {
    if(0x5555==peekWord(queue_ptr, numToPop)) break;
  }
  Pop(queue_ptr, numToPop);
  if(Size(queue_ptr) <= 0)
  {
  /* header was not found */
    return 0;
  }
  /* make sure we can read through minimum length packet */
  if(Size(queue_ptr)<7)
  {
    return 0;
  }
  /* get data length (5th byte of packet) */
  dataLength = peekByte(queue_ptr, 4);
  /* make sure we can read through entire packet */
  if(Size(queue_ptr) < 7+dataLength)
  {
    return 0;
  }
  /* check CRC */
  myCRC = calcCRC(queue_ptr, 2,dataLength+3);
  packetCRC = peekWord(queue_ptr, dataLength+5);
  if(myCRC != packetCRC)
  {
    Pop(queue_ptr, dataLength+7);
    return 0;
  }
  /* fill out result of parsing in structure */
  result->packet_type = peekWord(queue_ptr, 2);
  result->length = peekByte(queue_ptr, 4);
  result->crc = packetCRC;
  for(counter=0; counter < result->length; counter++)
  {
    result->data[counter] = peekByte(queue_ptr, 5+counter);
  }
  Pop(queue_ptr, dataLength+7);
  return 1;
}

/*******************************************************************************
* FUNCTION: calcCRC calculates a 2-byte CRC on serial data using
* CRC-CCITT 16-bit standard maintained by the ITU
* (International Telecommunications Union).
* ARGUMENTS: queue_ptr is pointer to queue holding area to be CRCed
* startIndex is offset into buffer where to begin CRC calculation
* num is offset into buffer where to stop CRC calculation
* RETURNS: 2-byte CRC
440 Series User’s Manual
Doc# 7430-0131-01 Rev. D Page 73
*******************************************************************************/
unsigned short calcCRC(QUEUE_TYPE *queue_ptr, unsigned int startIndex, unsigned int num) {
  unsigned int i=0, j=0;
  unsigned short crc=0x1D0F; //non-augmented inital value equivalent to augmented initial value 0xFFFF
  for (i=0; i<num; i+=1) {
    crc ^= peekByte(queue_ptr, startIndex+i) << 8;
    for(j=0;j<8;j+=1) {
      if(crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc = crc << 1;
    }
  }
  return crc;
}
// for user send packets
unsigned short calcCRC(unsigned char *byte_ptr, unsigned int num) {
  unsigned int i=0, j=0;
  unsigned short crc=0x1D0F; //non-augmented inital value equivalent to augmented initial value 0xFFFF
  for (i=0; i<num; i+=1) {
    crc ^= byte_ptr[i] << 8;
    for(j=0;j<8;j+=1) {
      if(crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc = crc << 1;
    }
  }
  return crc;
}
/*******************************************************************************
* FUNCTION: Initialize - initialize the queue
* ARGUMENTS: queue_ptr is pointer to the queue
*******************************************************************************/
void Initialize(QUEUE_TYPE *queue_ptr)
{
  queue_ptr->count = 0;
  queue_ptr->front = 0;
  queue_ptr->rear = -1;
}
/*******************************************************************************
* FUNCTION: AddQueue - add item in front of queue
* ARGUMENTS: item holds item to be added to queue
* queue_ptr is pointer to the queue
* RETURNS: returns 0 if queue is full. 1 if successful
*******************************************************************************/
int AddQueue(char item, QUEUE_TYPE *queue_ptr)
{
  int retval = 0;
  if(queue_ptr->count >= MAXQUEUE)
  {
    retval = 0; /* queue is full */
  }
  else
  {
    queue_ptr->count++;
    queue_ptr->rear = (queue_ptr->rear + 1) % MAXQUEUE;
    queue_ptr->entry[queue_ptr->rear] = item;
    retval = 1;
  }
  return retval;
}
/*******************************************************************************
* FUNCTION: DeleteQeue - return an item from the queue
* ARGUMENTS: item will hold item popped from queue
* queue_ptr is pointer to the queue
* RETURNS: returns 0 if queue is empty. 1 if successful
440 Series User’s Manual
Doc# 7430-0131-01 Rev. D Page 74
*******************************************************************************/
int DeleteQueue(char *item, QUEUE_TYPE *queue_ptr)
{
  int retval = 0;
  if(queue_ptr->count <= 0)
  {
    retval = 0; /* queue is empty */
  }
  else
  {
    queue_ptr -> count--;
    *item = queue_ptr->entry[queue_ptr->front];
    queue_ptr->front = (queue_ptr->front+1) % MAXQUEUE;
    retval=1;
  }
  return retval;
}
/*******************************************************************************
* FUNCTION: peekByte returns 1 byte from buffer without popping
* ARGUMENTS: queue_ptr is pointer to the queue to return byte from
* index is offset into buffer to which byte to return
* RETURNS: 1 byte
* REMARKS: does not do boundary checking. please do this first
*******************************************************************************/
char peekByte(QUEUE_TYPE *queue_ptr, unsigned int index) {
  char byte;
  int firstIndex;
  firstIndex = (queue_ptr->front + index) % MAXQUEUE;
  byte = queue_ptr->entry[firstIndex];
  return byte;
}
/*******************************************************************************
* FUNCTION: peekWord returns 2-byte word from buffer without popping
* ARGUMENTS: queue_ptr is pointer to the queue to return word from
* index is offset into buffer to which word to return
* RETURNS: 2-byte word
* REMARKS: does not do boundary checking. please do this first
*******************************************************************************/
unsigned short peekWord(QUEUE_TYPE *queue_ptr, unsigned int index) {
  unsigned short word, firstIndex, secondIndex;
  firstIndex = (queue_ptr->front + index) % MAXQUEUE;
  secondIndex = (queue_ptr->front + index + 1) % MAXQUEUE;
  word = (queue_ptr->entry[firstIndex] << 8) & 0xFF00;
  word |= (0x00FF & queue_ptr->entry[secondIndex]);
  return word;
}
/*******************************************************************************
* FUNCTION: Pop - discard item(s) from queue
440 Series User’s Manual
Doc# 7430-0131-01 Rev. D Page 75
* ARGUMENTS: queue_ptr is pointer to the queue
* numToPop is number of items to discard
* RETURNS: return the number of items discarded
*******************************************************************************/
int Pop(QUEUE_TYPE *queue_ptr, int numToPop)
{
  int i=0;
  char tempchar;
  for(i=0; i<numToPop; i++)
  {
    if(!DeleteQueue(&tempchar, queue_ptr))
    {
      break;
    }
  }
  return i;
}
/*******************************************************************************
* FUNCTION: Size
* ARGUMENTS: queue_ptr is pointer to the queue
* RETURNS: return the number of items in the queue
*******************************************************************************/
int Size(QUEUE_TYPE *queue_ptr)
{
  return queue_ptr->count;
}
/*******************************************************************************
* FUNCTION: Empty
* ARGUMENTS: queue_ptr is pointer to the queue
* RETURNS: return 1 if empty, 0 if not
*******************************************************************************/
int Empty(QUEUE_TYPE *queue_ptr)
{
  return queue_ptr->count <= 0;
}
/*******************************************************************************
* FUNCTION: Full
* ARGUMENTS: queue_ptr is pointer to the queue
* RETURNS: return 1 if full, 0 if not full
*******************************************************************************/
int Full(QUEUE_TYPE *queue_ptr)
{
  return queue_ptr->count >= MAXQUEUE;
}

inline short MKShort(const unsigned char* data) {
  return (short)data[1] + (short)data[0]*256;
}
inline int MKInt(const unsigned char* data) {
  return (int)data[3] + (int)data[2]*256 + (int)data[1]*256*256 + (int)data[0]*256*256*256;
}

struct SNAV1Msg{

  double dRollAngle; //[rad]
  double dPitchAngle;
  double dYawAngle;
  double dRollRate; //[rad/s]
  double dPitchRate;
  double dYawRate;
  double dXAccel; //[m/s^2]
  double dYAccel;
  double dZAccel;
  double dNVel;   //[m/s]
  double dEVel;
  double dDVel;
  double dLongitude;
  double dLatitude;
  double dAltitude;
  short xRateTemp;
  unsigned int timeITOW;
  unsigned short BITStatus;
};

bool MsgToNav1(const unsigned char* data, SNAV1Msg &sMsg) {
  sMsg.dRollAngle  = MKShort(data+0)*2.0*M_PI/(256*256);
  sMsg.dPitchAngle = MKShort(data+2)*2.0*M_PI/(256*256);
  sMsg.dYawAngle   = MKShort(data+4)*2.0*M_PI/(256*256);
  sMsg.dRollRate = MKShort(data+6)*7.0*M_PI/(256*256);
  sMsg.dPitchRate = MKShort(data+8)*7.0*M_PI/(256*256);
  sMsg.dYawRate = MKShort(data+10)*7.0*M_PI/(256*256);
  sMsg.dXAccel = MKShort(data+12)*20.0/(256*256);
  sMsg.dYAccel = MKShort(data+14)*20.0/(256*256);
  sMsg.dZAccel = MKShort(data+16)*20.0/(256*256);
  sMsg.dNVel = MKShort(data+18)*512.0/(256*256);
  sMsg.dEVel = MKShort(data+20)*512.0/(256*256);
  sMsg.dDVel = MKShort(data+22)*512.0/(256*256);
  sMsg.dLongitude = MKInt(data+24)*2.0*M_PI/(256*256);
  sMsg.dLatitude = MKInt(data+28)*2.0*M_PI/(256*256);
  sMsg.dAltitude = MKShort(data+32)/4.0;
  sMsg.xRateTemp = MKShort(data+34)*200.0/(256*256);
  sMsg.timeITOW			= (unsigned int)MKInt(data+36);
  sMsg.BITStatus     = (unsigned short)MKShort(data+40);
}

void Nav1ToRosImu(const SNAV1Msg &rNav1, sensor_msgs::Imu &ImuData) {
  ImuData.angular_velocity.x = rNav1.dRollRate;
  ImuData.angular_velocity.y = rNav1.dPitchRate;
  ImuData.angular_velocity.z = rNav1.dYawRate;
  ImuData.linear_acceleration.x = rNav1.dXAccel;
  ImuData.linear_acceleration.y = rNav1.dYAccel;
  ImuData.linear_acceleration.z = rNav1.dZAccel;
  ImuData.orientation = tf::createQuaternionMsgFromRollPitchYaw(rNav1.dRollAngle, rNav1.dPitchAngle, rNav1.dYawAngle);
}

void RunVG440(const std::string &rsPort, int nBaudRate, const std::string &rsTopic, const std::string &rsFrame)
{
  using namespace boost::asio;

  try {

    QUEUE_TYPE *_pQueue = new QUEUE_TYPE;
    Initialize(_pQueue);
    XBOW_PACKET packet;

    ros::NodeHandle n;
    ros::Publisher Publisher = n.advertise<sensor_msgs::Imu>(rsTopic, 1000);

    io_service io;
    serial_port port( io, rsPort );
    port.set_option(serial_port_base::baud_rate(nBaudRate));
    port.set_option(serial_port_base::character_size(8));
    port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    port.set_option(serial_port_base::parity(serial_port_base::parity::none));
    port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

    boost::array<unsigned char, 5000> DataBuf;
    int nCnt = 0;
    mmtimer mt;
    while (ros::ok())  {
      try {
        size_t nRet = boost::asio::read(port, boost::asio::buffer(DataBuf), boost::asio::transfer_at_least(1));
        for (int n=0; n<nRet; ++n) {
          AddQueue(DataBuf[n], _pQueue);
        }
        if (process_xbow_packet(_pQueue, &packet)) {
          /*
            Packet Type
            0x4e31:N1 NAV440の初期値
          */
          if (packet.packet_type == 0x4e31) { 
            sensor_msgs::Imu ImuData;
            ImuData.header.stamp = ros::Time::now();
            ImuData.header.frame_id = rsFrame;
            ImuData.header.seq = nCnt;
            SNAV1Msg NAV1;
            MsgToNav1((unsigned char*)packet.data, NAV1);
            Nav1ToRosImu(NAV1, ImuData);
            Publisher.publish(ImuData);
            ++nCnt;
            if (nCnt % 100 == 0) {
              cout << "FPS: " << nCnt/mt.elapsed() << endl;
            }
          }
          else {
            cout << "other msg:" << packet.packet_type << endl;
          }
        }
      }
      catch (boost::system::system_error &e) {
        if (e.code().message() == "Interrupted system call") { //ctrl+c
          ros::shutdown();
        }
        else {
          cout << "ReadData Error:" << e.what() << endl;
        }
      }
      ros::spinOnce();
    }
  }
  catch (std::exception &e) {
    cout << "RunVG400 Error: " << e.what() << endl;
  }
}

int main(int argc, char **argv) {

  string sTopic;
  string sPort;
  string sFrame;
  int nBaudRate;

  string sFrameDefault = "/vg440";
  string sTopicDefault = "/imu_data";
  string sPortDefault  = "/dev/ttyUSB0";
  int nBaudRateDefault = 57600;
  ros::init(argc, argv, "vg440_node");
  ros::NodeHandle n_private_("~");
  n_private_.param("frame", sFrame, sFrameDefault);
  n_private_.param("topic", sTopic, sTopicDefault);
  n_private_.param("port",  sPort,  sPortDefault);
  n_private_.param("baudrate",  nBaudRate,  nBaudRateDefault);

  RunVG440(sPort, nBaudRate, sTopic, sFrame);

  return 0;
}
