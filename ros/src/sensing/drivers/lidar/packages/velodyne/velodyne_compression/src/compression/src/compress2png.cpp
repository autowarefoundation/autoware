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
#include <compression_msgs/CompressedPacket.h>
#include <cmath>

//#include "std_msgs/Int32MultiArray.h"
//#include "compressed_image_transport/compressed_publisher.h"
//#include "compressed_image_transport/compression_common.h"

#include <vector>
#include <sstream>

using namespace std;
using namespace velodyne_rawdata;
using namespace cv;

compression_msgs::CompressedPacket CompressedData;

//char *fold;
ros::Publisher *pubp;

int i=1;
int row_type;
void velodyneCallback(const velodyne_msgs::VelodyneScan::ConstPtr &msg)
{
//cout<<msg->packets.size()<<endl;
static int count=0;
count++;

  //char name[100];
  //char route[100];
  
  //sprintf(name,"%sRotation%06d.txt",fold,count);
  //sprintf(name,"Rotation%06d.txt",count);
  //cout<<name;
  //sprintf(route,"%simage%06d.png",fold,count);
  //sprintf(route,"mage%06d.png",count);
//  FILE *fp;
//  fp=fopen(name,"w");
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
  
 



//cout<<msg->header<<endl;
for (int next=0; next<msg->packets.size();++next)
{
//printf("get %d\n",msg->packets.size());

 const raw_packet_t *raw=(const raw_packet_t *) &msg->packets[next].data[0];

  //cout<<raw<<endl; 
for (int i = 0; i < BLOCKS_PER_PACKET; i++) {

      // upper bank lasers are numbered [0..31]
      // NOTE: this is a change from the old velodyne_common implementation
      int bank_origin = 0;
//cout<<raw->blocks[i].header<<endl;
      if (raw->blocks[i].header == LOWER_BANK) {
        // lower bank lasers are [32..63]
        bank_origin = 32;
      }
     // cout<<i<<endl;
     // cout<<raw->blocks[i].rotation<<endl;
for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
        
        uint8_t laser_number;       ///< hardware laser number

        laser_number=j+bank_origin;
        //cout<<j<<endl;
        //cout<<bank_origin<<endl;
       // printf("get %d\n",laser_number);

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
        // printf("%d %d %d\n",laser_number, floor,d);
        //fprintf(fp,"%d %d %l\n ",laser_number,raw->blocks[i].rotation,d);

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
        //fprintf(fp,"%d ",raw->blocks[i].rotation);




}
}

} 

//imwrite(route,img,compression_params);


//ros::init(1,"no","compressed_velodyne_publisher");
//ros::NodeHandle nh;
//image_transport::ImageTransport it(nh);
//image_transport::Publisher pub=it.advertise("image_raw/compressed",1);



//sensor_msgs::CompressedImage compressed;

CompressedData.header=msg->header;

//cout<<img.rows<<endl;
//cout<<img2<<endl;
if (cv::imencode(".png", img, CompressedData.distance, compression_params))
{
//pub.publish(compressed);
//    ros::Publisher pub=nh.advertise<sensor_msgs::CompressedImage>("Compressed",1);
//    pub.publish(compressed);

//cout<<img2.rows<<" "<<img2.cols<<endl;
    cv::imencode(".png", img2, CompressedData.intensity, compression_params);
    cout<<"Frame "<<i<<" is Compressing"<<endl;
    i++;
}
//else
//{
//    ROS_ERROR("cv::imencode (png) failed on input image");
//}



pubp->publish(CompressedData);



//fclose(fp);
}

int main(int argc, char **argv)
{
  printf("Compression start \n");


  ros::init(argc, argv, "velodyne_compression");

  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Publisher pub=nh.advertise<compression_msgs::CompressedPacket>("CompressedPacket",1000);


  pubp=&pub;

  ros::Subscriber sub = n.subscribe("/velodyne_packets", 500000, velodyneCallback);

  ros::spin();

  return 0;
}


