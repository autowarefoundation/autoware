#include <ros/ros.h>
#include "autoware_msgs/CANPacket.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


ros::Publisher pub;
double g_steer,g_speed;

void chatterCallback(const autoware_msgs::CANPacket::ConstPtr& msg)
{
  unsigned short w;
  static int enc_sum;
  short diff;
  static short steer,shift,speed2,speed3,brake;
  static char  speed;
  static short enc,enc_p,enc_diff;
  static short wheel1,wheel2,wheel3,wheel4;
  static short gyro,accy,accz;
  static char accx; 
  static FILE* log_fp;


  int changed=0;
  if(msg->id==0x24){
    w=msg->dat[0]*256+msg->dat[1];
    gyro=w;
    w=msg->dat[2]*256+msg->dat[3];
    accy=w;
    w=msg->dat[4]*256+msg->dat[5];
    accz=w;
    w=msg->dat[7];
    accx=w;
    changed=1;
  }
  if(msg->id==0x25){
    w=msg->dat[0]*4096+msg->dat[1]*16;
    steer=w;
    steer=steer/16;
    changed=1;
  }

  if(msg->id==0xaa){
    w=msg->dat[0]*256+msg->dat[1];
    wheel1=w;
    w=msg->dat[2]*256+msg->dat[3];
    wheel2=w;
    w=msg->dat[4]*256+msg->dat[5];
    wheel3=w;
    w=msg->dat[6]*256+msg->dat[7];
    wheel4=w;
    changed=1;
  }
  if(msg->id==182){
    w=msg->dat[0]*256+msg->dat[1];
    speed3=w;
    changed=1;
  }
  if(msg->id==0x127){
    shift=msg->dat[3];
    speed=msg->dat[4];
    changed=1;
  }
  if(msg->id==0x230){
    w=msg->dat[0]*256+msg->dat[1];
    enc_p=enc;
    enc=w;
    diff=enc-enc_p;
    enc_diff=diff;
    enc_sum+=diff;
    changed=1;
  }

  if(changed){
    if(!log_fp)log_fp=fopen("/tmp/can_log","w");
    fprintf(log_fp,"%f %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
	    msg->header.stamp.toSec(),msg->time,steer,shift,speed,speed2,speed3,enc_sum,enc_diff,brake,
	    wheel1,wheel2,wheel3,wheel4,
	    accx,accy,accz,gyro);
    g_steer=(double)steer/100.;
    g_speed=(double)speed3/360.;
  }  

}


int main (int argc, char *argv[]){
  autoware_msgs::CANPacket candat;

  ros::init(argc, argv, "can_draw");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("can_raw", 1000, chatterCallback);

  pub = n.advertise<visualization_msgs::Marker>("can_status", 10);  


  //visualization_msgs::MarkerArray markers;


  while(ros::ok()){  
    visualization_msgs::Marker line_list;
    geometry_msgs::Point p;
    
    line_list.header.frame_id = "velodyne";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "steer_speed";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    //  line_list.lifetime=ros::Duration(0.2);
    
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.2;
    line_list.color.r = 1.0;
    line_list.color.g = 0.0;
    line_list.color.b = 0.0;
    line_list.color.a = 1;

    p.x=-1;
    p.y=0;
    p.z=-1.5;

    double angle=0;
    printf("%f %f\n",g_speed,g_steer);
    for(double l=0; l<g_speed*g_speed/(2.0 * 1.0)+4; l+=0.2){
      line_list.points.push_back(p);      
      angle+=g_steer/100;
      p.x += 0.2*cos(angle);
      p.y += 0.2*sin(angle);
      line_list.points.push_back(p);      
    }
    pub.publish(line_list);

    printf("p\n");
    usleep(50000);
    ros::spinOnce();
  }
}
