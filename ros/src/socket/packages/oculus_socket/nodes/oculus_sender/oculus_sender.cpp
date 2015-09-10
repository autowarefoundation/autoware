#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <time.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define BUFSIZE 1024
#define NDT_PORT 59630
#define PI 3.141592

double roll;
double pitch;
double yaw;

int sock1;
struct sockaddr_in addr1;
char buf1[BUFSIZE];
char buf2[BUFSIZE];
static pcl::PointCloud<pcl::PointXYZ> _vscan;
geometry_msgs::Point prev;

struct timeval t;

static void current_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{

  tf::Quaternion q(input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z, input->pose.orientation.w);
  tf::Matrix3x3 m(q);

  m.getEulerYPR(yaw, pitch, roll);
  
  ROS_INFO("x=[%f] y=[%f] z=[%f]", input->pose.position.x, input->pose.position.y, input->pose.position.z);
  ROS_INFO("roll=[%f] pitch=[%f] yaw=[%f], PI=[%f]", roll/PI*180, pitch/PI*180, yaw/PI*180, PI);
  gettimeofday(&t,NULL);
  sprintf(buf1,"%ld%ld CAR %d %f %f %f %f %f %f", t.tv_sec, t.tv_usec, 0 , input->pose.position.x, input->pose.position.y, input->pose.position.z, roll/PI*180, pitch/PI*180, yaw/PI*180);

  sendto(sock1, buf1, sizeof(buf1), 0, (struct sockaddr *)&addr1, sizeof(addr1));
}

static void obj_pose_Callback(const visualization_msgs::MarkerArray::ConstPtr& input)
{
  for (std::vector<visualization_msgs::Marker>::const_iterator item = input->markers.begin(); item != input->markers.end(); item++){

    tf::Quaternion q(item->pose.orientation.x, item->pose.orientation.y, item->pose.orientation.z, item->pose.orientation.w);
    tf::Matrix3x3 m(q);

    m.getEulerYPR(yaw, pitch, roll);
  
    ROS_INFO("x=[%.6f] y=[%.6f] z=[%.6f]", item->pose.position.x, item->pose.position.y, item->pose.position.z);
    ROS_INFO("roll=[%f] pitch=[%f] yaw=[%f], PI=[%f]", roll/PI*180, pitch/PI*180, yaw/PI*180, PI);
    gettimeofday(&t,NULL);
    if(item->color.b == 1.0f){
      sprintf(buf1,"%ld%ld OTH %d %f %f %f %f %f %f", t.tv_sec, t.tv_usec , 0, item->pose.position.x, item->pose.position.y, item->pose.position.z, roll/PI*180, pitch/PI*180, yaw/PI*180);
    }
    else if(item->color.g == 1.0f){
      sprintf(buf1,"%ld%ld PED %d %f %f %f %f %f %f", t.tv_sec, t.tv_usec , 0, item->pose.position.x, item->pose.position.y, item->pose.position.z, roll/PI*180, pitch/PI*180, yaw/PI*180);
    }
    else {
      sprintf(buf1,"%ld%ld OBJ %d %f %f %f %f %f %f", t.tv_sec, t.tv_usec , 0, item->pose.position.x, item->pose.position.y, item->pose.position.z, roll/PI*180, pitch/PI*180, yaw/PI*180);
    }

    sendto(sock1, buf1, sizeof(buf1), 0, (struct sockaddr *)&addr1, sizeof(addr1));
  }
}

static void vscan_Callback(const sensor_msgs::PointCloud2ConstPtr& msg){

  pcl::fromROSMsg(*msg, _vscan);
  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = _vscan.begin(); item != _vscan.end(); item++){
    if (item->x == 0 && item->y == 0)continue;
    if(prev.x==0&&prev.y==0&&prev.y==0){	
      prev.x = item->x;
      prev.y = item->y;
      prev.z = item->z;
    }else{
      ROS_INFO("bottom_x=[%f] bottom_y=[%f] bottom_z=[%f]", prev.x, prev.y, prev.z);
      ROS_INFO("top_x=[%f] top_y=[%f] top_z=[%f]", item->x, item->y, item->z);
      gettimeofday(&t,NULL);
      sprintf(buf2,"%ld%ld VSC %d %f %f %f %f %f %f",t.tv_sec, t.tv_usec, 0, prev.x, prev.y, prev.z, item->x, item->y, item->z);

      sendto(sock1, buf2, sizeof(buf1), 0, (struct sockaddr *)&addr1, sizeof(addr1));

      prev.x=0; prev.y=0; prev.z=0; 
    }
  }
}


int main (int argc, char **argv)
{
  ros::init(argc,argv,"udp_sender");
  ros::NodeHandle n;

  ros::Subscriber current_pose_sub = n.subscribe("current_pose",1000,current_pose_Callback);
  //  ros::Subscriber vscan_sub = n.subscribe("vscan_points", 100, vscan_Callback);
  ros::Subscriber obj_car_sub = n.subscribe("obj_car/obj_pose",1000,obj_pose_Callback);
  ros::Subscriber obj_person_sub = n.subscribe("obj_person/obj_pose",1000,obj_pose_Callback);


  sock1 = socket(AF_INET, SOCK_DGRAM, 0);
  addr1.sin_family = AF_INET;
  addr1.sin_port = htons(NDT_PORT);
  std::cout << "\n... Connecting " << argv[1] << std::endl;
  addr1.sin_addr.s_addr = inet_addr(argv[1]);

  ros::spin();

  return 0;
}

