#include "ros/ros.h"
#include <ros/time.h>
#include "velodyne_pointcloud/rawdata.h"
#include "ros/package.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>  
#include <fstream>  
#include <iostream>  
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include "std_msgs/UInt16MultiArray.h"
#include <velodyne_compression/CompressedPacket.h>
#include <cmath>



#include <vector>
#include <sstream>

using namespace std;
using namespace velodyne_rawdata;
using namespace cv;

velodyne_compression::CompressedPacket CompressedData;
ros::Publisher *pubp;

int i=1;
int row_type;
void velodyneCallback(const velodyne_msgs::VelodyneScan::ConstPtr &msg)
{
static int count=0;
count++;

if (msg->packets.size()==348) //64ES2
{

  cout<<"Senor version:HDL64ES2/HDL64ES2.1"<<endl;
  CompressedData.scan_row=64;
  CompressedData.scan_column=2088;
}
else if (msg->packets.size()==580)//64ES3
{
  CompressedData.scan_row=64;
  CompressedData.scan_column=6*580;;
  cout<<"Senor version:HDL64ES3"<<endl;
}
else if (msg->packets.size()==260)//64E 260
{
  CompressedData.scan_row=64;
  CompressedData.scan_column=6*260;
  cout<<"Senor version:HDL64E"<<endl;
}
else if (msg->packets.size()==181)//32E 181
{
  CompressedData.scan_row=32;
  CompressedData.scan_column=12*181;;
  cout<<"Senor version:HDL32E"<<endl;
}

else if (msg->packets.size()==79)//VLP16
{
  CompressedData.scan_row=32;
  CompressedData.scan_column=12*79;;
  cout<<"Senor version:VLP16"<<endl;
}
else
{
   cout<<"Unknown device"<<endl;
}
Mat img(CompressedData.scan_row, CompressedData.scan_column, CV_16U);
Mat img2(CompressedData.scan_row, CompressedData.scan_column, CV_8U);


vector<int>compression_params;
compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
compression_params.push_back(9);
CompressedData.rotation.clear();
  
 
for (int next=0; next<msg->packets.size();++next)
{
 const raw_packet_t *raw=(const raw_packet_t *) &msg->packets[next].data[0];

for (int i = 0; i < BLOCKS_PER_PACKET; i++) {

      // upper bank lasers are numbered [0..31]
      // NOTE: this is a change from the old velodyne_common implementation
      int bank_origin = 0;
      if (raw->blocks[i].header == LOWER_BANK) {
        // lower bank lasers are [32..63]
        bank_origin = 32;
      }
for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
        
        uint8_t laser_number;       ///< hardware laser number

        laser_number=j+bank_origin;
        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[i].data[k];
        tmp.bytes[1] = raw->blocks[i].data[k+1];
        float distance = tmp.uint*DISTANCE_RESOLUTION;
        int d=distance*100.0;
        //printf("get %d\n",d);
        int col;
        if (CompressedData.scan_row==32)
        {col=next*12+i;}
        else
        {col=floor(next*6+i/2);}

        img.at<ushort>(laser_number, col) = d;
        img2.at<uchar>(laser_number, col) = raw->blocks[i].data[k+2];

        if (j==0)
        {  
          if (CompressedData.scan_row==32)
          {
            CompressedData.rotation.push_back(raw->blocks[i].rotation);
          }
          else
            if (i%2==0)
            {
               CompressedData.rotation.push_back(raw->blocks[i].rotation);
            }
        }

}
}

} 


CompressedData.header=msg->header;
if (cv::imencode(".png", img, CompressedData.distance, compression_params))
{

    cv::imencode(".png", img2, CompressedData.intensity, compression_params);
    cout<<"Frame "<<i<<" is Compressing"<<endl;
    i++;
}

pubp->publish(CompressedData);

}

int main(int argc, char **argv)
{
  printf("Compression start \n");


  ros::init(argc, argv, "velodyne_compression");

  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Publisher pub=nh.advertise<velodyne_compression::CompressedPacket>("velodyne_packets_compressed",1000);


  pubp=&pub;

  ros::Subscriber sub = n.subscribe("/velodyne_packets", 500000, velodyneCallback);

  ros::spin();

  return 0;
}


