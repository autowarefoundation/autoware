#include <ros/ros.h>
#include "velodyne_pointcloud/rawdata.h"
#include "ros/package.h"
#include "driver.h"
#include <string>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>  
#include <string>  
#include <fstream>  
#include <iostream>  
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>
#include "velodyne_msgs/VelodyneScan.h"
#include <unistd.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "driver.h"
#include "std_msgs/UInt16MultiArray.h"
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include <compression_msgs/CompressedPacket.h>
//#include <image_transport/image_transport.h>



#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace velodyne_rawdata;
using namespace cv;

typedef unsigned short uint16;

int recon_frame;
int rec=0;
//int too_fast;

uint16* rota;
uint16 row;
ros::Publisher *pubp;
uint16 column;
//ros::NodeHandle *node2;
//std_msgs::UInt16MultiArray rotation;
//uchar* ptr_distance;
//uchar* ptr_inten;
//Mat img_distance(64, 2088, CV_16U);
//Mat img_inten(64, 2088, CV_8U);

namespace velodyne_driver
{

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  if (column==2088)
  {
  private_nh.param("model", config_.model, std::string("64E_S2"));
  }
  else if (column==6*260)
    {
    private_nh.param("model", config_.model, std::string("64E"));
    }
  else if (column==12*181)
    {
    private_nh.param("model", config_.model, std::string("32E"));
    }
  else if (column==6*580)
    {
    private_nh.param("model", config_.model, std::string("64E_S3"));
    }
  else if (column==12*79)
    {
    private_nh.param("model", config_.model, std::string("VLP16"));
    }


  double packet_rate;                   // packet frequency (Hz)
   std::string model_full_name;
   if ((config_.model == "64E_S2") ||
       (config_.model == "64E_S2.1"))    // generates 1333312 points per second
     {                                   // 1 packet holds 384 points
       packet_rate = 3472.17;            // 1333312 / 384
       model_full_name = std::string("HDL-") + config_.model;
     }
   else if (config_.model == "64E_S3")
     {
       packet_rate = 5800.0; // experimental
       model_full_name = std::string("HDL-") + config_.model;
     }
   else if (config_.model == "64E")
     {
       packet_rate = 2600.0;
       model_full_name = std::string("HDL-") + config_.model;
     }
   else if (config_.model == "32E")
     {
       packet_rate = 1808.0;
       model_full_name = std::string("HDL-") + config_.model;
     }
   else if (config_.model == "VLP16")
     {
       packet_rate = 781.25;             // 300000 / 384
       model_full_name = "VLP-16";
     }
   else
     {
       ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
       packet_rate = 2600.0;
 }
  std::string deviceName("Velodyne HDL-" + config_.model);

  private_nh.param("rpm", config_.rpm, 600.0);
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int) ceil(packet_rate / frequency);
  private_nh.getParam("npackets", config_.npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate/config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));

  // open Velodyne input device or file
  // raw data output topic
  output_ = node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
}





/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(Mat img_distance,Mat img_inten)
{
  uint16_t rot_angle;
  long i, j, ID_block,ID_packet;
  union two_bytes tmp;
  float dis;
  int intensity;
  uint8_t packet[1206];
  uint8_t head_p[6] = { 30, 30, 30, 30, 30, 30 };
  
 //Mat img_distance(row, column, CV_16U,ptr_distance);
 //cout<<img_distance<<endl;
 //Mat img_inten(row, column, CV_8U,ptr_inten);

  //char rot_route[100];
  //char image_route[100];
  recon_frame++;
  //cout << config_.npackets;
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
  scan->packets.resize(config_.npackets);

  //int wa;
  //cin>>wa;
  //usleep(too_fast);

  //sprintf(rot_route,"%sRotation%06d.txt",fold,recon_frame);
  //cout<<rot_route<<endl;
  //sprintf(image_route,"%simage%06d.png",fold,recon_frame);


  //img_distance = imread(image_route, CV_LOAD_IMAGE_UNCHANGED);
  //img_distance=cv::imdecode(dist, CV_LOAD_IMAGE_UNCHANGED);
  //ifstream rot(rot_route);


//  if (!rot.is_open())
//  {
//   cout << "End"; return false;
//  }
//  else
//  {
	cout << "Frame " << recon_frame << " is reconstructing"<<endl;
//  }


  if (row==64)
  {

  ID_block = 0;
  ID_packet=0;
  for (i = 0; i < column; i++)
  {
        //cout<<i<<endl;
    rot_angle=*(rota+i);
	//cout << rot_angle << endl;
	tmp.uint = uint16_t(0xeeff);
	packet[ID_block * 100] = tmp.bytes[0];
	packet[ID_block * 100 + 1] = tmp.bytes[1];
	//cout << tmp.uint << endl;
	//cout << uint16_t(0xeeff) << endl;

	//cout << (tmp.uint == uint16_t(0xeeff))<<(1==1)<<endl;
	tmp.uint = rot_angle;
	packet[ID_block * 100 + 2] = tmp.bytes[0];
	packet[ID_block * 100 + 3] = tmp.bytes[1];

	//cout << rot_angle <<" "<<tmp.uint << endl;
	for (j = 0; j < 32; j++)
	{
		//cout << i << "." << id_block << "." << j << endl;

  intensity=img_inten.at<uchar>(j,i);  
  dis= img_distance.at<ushort>(j, i);
 //cout<<"666"<<endl;

  //cout<<intensity<<endl;
	// cout << dis << endl;
  dis = (dis+0.5) / 100.0 / 0.002f;
	tmp.uint = dis;
	int loca = ID_block * 100 + 4 + j * 3;
	// cout << "loca" << loca << endl;
	packet[loca] = tmp.bytes[0];
	packet[loca + 1] = tmp.bytes[1];


	/*tmp.bytes[0] = block[loca];
	tmp.bytes[1] = block[loca + 1];;
	float distance = tmp.uint*0.002f;
	int d = distance*100.0;
	cout << "recon" << d << endl;*/

  packet[loca + 2] = intensity;  //intensity
	}

	ID_block++;

	tmp.uint = uint16_t(0xddff);
	packet[ID_block * 100] = tmp.bytes[0];
	packet[ID_block * 100 + 1] = tmp.bytes[1];
	//cout << tmp.uint << endl;
	//cout << uint16_t(0xeeff) << endl;

	//cout << (tmp.uint == uint16_t(0xeeff))<<(1==1)<<endl;
	tmp.uint = rot_angle;
	packet[ID_block * 100 + 2] = tmp.bytes[0];
	packet[ID_block * 100 + 3] = tmp.bytes[1];

	for (j = 32; j < 64; j++)
	{
		//cout << i << "." << id_block << "." << j << endl;


    dis = img_distance.at<ushort>(j, i);
    intensity=img_inten.at<uchar>(j,i);
		// cout << dis << endl;
		dis = (dis+0.5) / 100.0 / 0.002f;
		tmp.uint = dis;
		int loca = ID_block * 100 + 4 + (j-32) * 3;
		// cout << "loca" << loca << endl;
		packet[loca] = tmp.bytes[0];
		packet[loca + 1] = tmp.bytes[1];


		/*tmp.bytes[0] = block[loca];
		tmp.bytes[1] = block[loca + 1];;
		float distance = tmp.uint*0.002f;
		int d = distance*100.0;
		cout << "recon" << d << endl;*/

    packet[loca + 2] = intensity;  //intensity
	}

	ID_block++;






	if (ID_block == 12) 
        { 
        
        memcpy(packet+1200, head_p, 6); 
        velodyne_msgs::VelodynePacket *pkt=&scan->packets[ID_packet];
        memcpy(&pkt->data[0],packet,1206);
        pkt->stamp=ros::Time::now();
        ID_block = 0; 
        ID_packet++;
        } //status	
  }
  }




  if (row==32)
  {
    ID_block = 0;
    ID_packet=0;
    for (i = 0; i < column; i++)
    {
          //cout<<i<<endl;
      rot_angle=*(rota+i);
    //cout << rot_angle << endl;
    tmp.uint = uint16_t(0xeeff);
    packet[ID_block * 100] = tmp.bytes[0];
    packet[ID_block * 100 + 1] = tmp.bytes[1];
    //cout << tmp.uint << endl;
    //cout << uint16_t(0xeeff) << endl;

    //cout << (tmp.uint == uint16_t(0xeeff))<<(1==1)<<endl;
    tmp.uint = rot_angle;
    packet[ID_block * 100 + 2] = tmp.bytes[0];
    packet[ID_block * 100 + 3] = tmp.bytes[1];
    //cout << rot_angle <<" "<<tmp.uint << endl;
    for (j = 0; j < 32; j++)
    {
      //cout << i << "." << id_block << "." << j << endl;


    dis= img_distance.at<ushort>(j, i);
    intensity=img_inten.at<uchar>(j,i);
    // cout << dis << endl;
    dis = (dis+0.5) / 100.0 / 0.002f;
    tmp.uint = dis;
    int loca = ID_block * 100 + 4 + j * 3;
    // cout << "loca" << loca << endl;
    packet[loca] = tmp.bytes[0];
    packet[loca + 1] = tmp.bytes[1];


    /*tmp.bytes[0] = block[loca];
    tmp.bytes[1] = block[loca + 1];;
    float distance = tmp.uint*0.002f;
    int d = distance*100.0;
    cout << "recon" << d << endl;*/

    packet[loca + 2] = intensity;  //intensity
    }
    ID_block++;
    if (ID_block == 12)
          {

          memcpy(packet+1200, head_p, 6);
          velodyne_msgs::VelodynePacket *pkt=&scan->packets[ID_packet];
          memcpy(&pkt->data[0],packet,1206);
          pkt->stamp=ros::Time::now();
          ID_block = 0;
          ID_packet++;
          }

  }


}


  //Mat img_distance(64, 2088, CV_16U);
  //img_distance = imread("H:\\alpha.png", CV_LOAD_IMAGE_UNCHANGED);
  //ifstream rot("H:\\Rotation_angle.txt");





  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.


  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Velodyne scan.");
  scan->header.stamp = ros::Time(scan->packets[config_.npackets - 1].stamp);
  scan->header.frame_id = config_.frame_id;
  //cout<<scan->packets[1]<<endl;
  pubp->publish(scan);
  //output_.publish(scan);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();
  cout <<rec<<endl;
  rec=0;
  //delete[] ptr_distance;
  delete[] rota;
  return true;
}

} // namespace velodyne_driver


void CompressedPacketCallback(const compression_msgs::CompressedPacket::ConstPtr& CompressedData)
 {
   row=CompressedData->scan_row;
   column=CompressedData->scan_column;


   rota=new uint16[column];
   Mat img_distance(row, column, CV_16U);
   Mat img_inten(row, column, CV_8U);

    for(int i=0; i<column;i++ )
    //or (int *it=msg->data.begin(); it!=msg->data.end();++it)
    {
        *(rota+i)=CompressedData->rotation[i];
        //cout<<rota[i]<<endl;
    }

    rec=1;


    img_distance=cv::imdecode(CompressedData->distance, CV_LOAD_IMAGE_UNCHANGED);


    img_inten=cv::imdecode(CompressedData->intensity, CV_LOAD_IMAGE_UNCHANGED);

   // ptr_distance=img_distance.data;
    //ptr_inten=img_inten.data;

    ros::NodeHandle node2;
    ros::NodeHandle private_nh("~");
    velodyne_driver::VelodyneDriver dvr(node2, private_nh);
    dvr.poll(img_distance,img_inten);
         //cout<<static_cast<const void*>(ptr_distance)<<endl;
    //Mat img_distance(row, column, CV_16U,ptr_distance);
   // cout<<img_distance<<endl;

    return;
 }



int main(int argc, char** argv)
{
  ros::init(argc, argv, "velodyne_decompression");
  ros::NodeHandle node;
  ros::NodeHandle np;

  //node2=&node;



  printf("start\n");
//  fold=argv[1];
//  too_fast=atoi(argv[2]);
//  too_fast=too_fast*1000;
//  cout << too_fast;
  recon_frame=0;
  ros::Publisher pub=np.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
  pubp=&pub;
//  ros::NodeHandle n;
//  ros::NodeHandle nh;

  ros::Subscriber sub = node.subscribe("CompressedPacket", 1000, CompressedPacketCallback);


  //ros::spin();

  // start the driver

  // loop until shut down or end of file
  //while(ros::ok() && dvr.poll())
  ros::spin();

   // ros::spinOnce();




  return 0;
}
