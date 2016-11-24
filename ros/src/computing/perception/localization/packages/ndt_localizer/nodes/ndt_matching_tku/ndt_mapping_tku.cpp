/*
  Normal Distributions Transform test program.

  2005/4/24 tku
*/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <math.h>
#include <GL/glut.h>
#include <chrono>
#include "ndt.h"
#include "algebra.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "velodyne_pointcloud/point_types.h"
#include "velodyne_pointcloud/rawdata.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#define DEFAULT_NDMAP_FILE "../data/nd_dat"

/*grobal variables*/
NDMapPtr NDmap;/*���ֲ�����*/
NDPtr NDs;
int NDs_num;

char scan_file_base_name[100];

Point scan_points[130000];
double scan_points_i[130000];
double scan_points_e[130000];
int scan_points_num;

int is_first_time=1;
int is_map_exist=0;

double scan_points_weight[130000];
double scan_points_totalweight;

Point map_points[130000];
double map_points_i[130000];
int mapping_points_num;

int scan_num;
int layer_select=LAYER_NUM-1;


Posture prev_pose,prev_pose2;

//params
double g_map_center_x,g_map_center_y,g_map_center_z;
double g_map_rotation;
int g_map_x,g_map_y,g_map_z;
double g_map_cellsize;
char g_ndmap_name[500];
int g_use_gnss;
int g_map_update=1;
double g_ini_x,g_ini_y,g_ini_z,g_ini_roll,g_ini_pitch,g_ini_yaw;

/**/
void print_matrix3d(double mat[3][3]);
void matrix_test(void);
void print_matrix6d(double mat[6][6]);

void save_nd_map(char* name);
void publish_nd_map();

static pcl::PointCloud<pcl::PointXYZ> map;
static int map_loaded = 0;

static ros::Publisher ndmap_pub;

static std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;
static double exe_time = 0.0;

static std::string _downsampler = "voxel_grid";
int _downsampler_num = 1;

//double pose_mod(Posture *pose){
void pose_mod(Posture *pose){
  while(pose->theta  < -M_PI)pose->theta  += 2*M_PI;
  while(pose->theta  >  M_PI)pose->theta  -= 2*M_PI;
  while(pose->theta2 < -M_PI)pose->theta2 += 2*M_PI;
  while(pose->theta2 >  M_PI)pose->theta2 -= 2*M_PI;
  while(pose->theta3 < -M_PI)pose->theta3 += 2*M_PI;
  while(pose->theta3 >  M_PI)pose->theta3 -= 2*M_PI;
}

double nrand(double n){
  double r;
  r = n*sqrt(-2.0*log((double)rand()/RAND_MAX))*cos(2.0*M_PI*rand()/RAND_MAX);
  return r;
}

/*
static void map_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  if (map_loaded == 0)
  {
    // Convert the data type(from sensor_msgs to pcl).
    pcl::fromROSMsg(*input, map);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));

    Point p;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = map_ptr->begin(); item != map_ptr->end(); item++){

    	p.x=(item->x-g_map_center_x)*cos(-g_map_rotation)-(item->y-g_map_center_y)*sin(-g_map_rotation);
        p.y=(item->x-g_map_center_x)*sin(-g_map_rotation)+(item->y-g_map_center_y)*cos(-g_map_rotation);
        p.z=item->z-g_map_center_z;

        add_point_map(NDmap, &p);
    }
    std::cout << "Finished loading point cloud map." << std::endl;
    std::cout << "Map points num: " << map_ptr->size() << " points." << std::endl;

    save_nd_map(g_ndmap_name);
    is_map_exist=1;

    map_loaded = 1;
  }
}
*/

void points_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
  matching_start = std::chrono::system_clock::now();
  static int count=0;
  static tf::TransformBroadcaster br;
  static FILE *log_fp;
  ros::Time time;
  static tf::TransformListener listener;
  std_msgs::Header header;

  static int iteration;
  int j = 0;

  Posture pose, bpose,initial_pose;
  static Posture key_pose;
//  double e=0,theta,x2,y2;
  double e=0;
  double x_offset,y_offset,z_offset,theta_offset;
//  double distance,diff;
  double distance;

  //  pcl_conversions::fromPCL(msg->header.stamp,time);
  pcl_conversions::fromPCL(msg->header,header);
  
//  ROS_INFO("%s",msg->header.frame_id.c_str());
  //  ROS_INFO("%f",msg->header.stamp.toSec());// + (double)msg->header.stamp.nsec/1000000000.);
  
  /*  ros::Time stamp;
  stamp = msg->header.stamp;
  double now = ros::Time::now().toSec();
  */
  if(!log_fp)log_fp=fopen("/tmp/ndt_log","w");

  //  ROS_INFO("get data %d",msg->points.size());
  count++;
  scan_points_totalweight=0;

  if(_downsampler == "voxel_grid"){
    pcl::PointCloud<pcl::PointXYZ> scan;

    for(int i = 0; i < (int)msg->points.size(); i++){
	scan.points.push_back (pcl::PointXYZ(msg->points[i].x, msg->points[i].y,msg->points[i].z));
    }
    scan.header=msg->header;

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(0.5,0.5,0.5);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);
    j=0;
    for(int i = 0; i < (int)filtered_scan_ptr->points.size(); i++){
    	scan_points[j].x = filtered_scan_ptr->points[i].x+nrand(0.01);
    	scan_points[j].y = filtered_scan_ptr->points[i].y+nrand(0.01);
    	scan_points[j].z = filtered_scan_ptr->points[i].z+nrand(0.01);
    	scan_points_i[j] = 100;//filterd_scan_ptr->points[i].intensity;

    	double dist=scan_points[j].x*scan_points[j].x + scan_points[j].y*scan_points[j].y + scan_points[j].z*scan_points[j].z;
    	if(dist<3*3)continue;
    	if(scan_points[j].z>-1&&scan_points[j].z<0 && scan_points[j].y>-20&&scan_points[j].y<20)continue;

    	scan_points_weight[j]=1;
    	scan_points_totalweight+=scan_points_weight[j];
    	j++;
    	if(j>130000)break;
    }
    scan_points_num = j;
    mapping_points_num= j;
//    printf("points_num = %d \n",scan_points_num);
  }

  if(_downsampler == "distance"){

    /*----------����ǡ����ɤ߼��---------*/  
    j=0;
    for(int i = 0; i < (int)msg->points.size(); i++){
      scan_points[j].x = msg->points[i].x+nrand(0.01);
      scan_points[j].y = msg->points[i].y+nrand(0.01);
      scan_points[j].z = msg->points[i].z+nrand(0.01);
      scan_points_i[j] = msg->points[i].intensity;
 
      double dist=scan_points[j].x*scan_points[j].x+scan_points[j].y*scan_points[j].y+scan_points[j].z*scan_points[j].z;
      if(dist<3*3)continue;
      scan_points_weight[j]=(dist)* (1.2-exp(-1*(-0.5 - scan_points[j].z)*(-0.5 - scan_points[j].z)/2.0));
      //scan_points_weight[j]= (1.2-exp(-1*(-0.5 - scan_points[j].z)*(-0.5 - scan_points[j].z)/2.0));
      scan_points_totalweight+=scan_points_weight[j];

      j++;
      if(j>130000)break;
    }
    scan_points_num = j;
    mapping_points_num= j;
//    printf("points_num = %d \n",scan_points_num);
  }

    /*--matching---*/
    /*
    Posture pose, bpose,initial_pose;
    static Posture key_pose;
    double e=0,theta,x2,y2;
    double x_offset,y_offset,z_offset,theta_offset;
    double distance,diff; 
*/
    //calc offset
    x_offset = prev_pose.x -prev_pose2.x;
    y_offset = prev_pose.y -prev_pose2.y;
    z_offset = prev_pose.z -prev_pose2.z;
    theta_offset= prev_pose.theta3-prev_pose2.theta3;

    if(theta_offset < -M_PI)theta_offset+=2*M_PI;
    if(theta_offset >  M_PI)theta_offset-=2*M_PI;	  

    
    //calc estimated initial position
    pose.x = prev_pose.x + x_offset;
    pose.y = prev_pose.y + y_offset;
    pose.z = prev_pose.z + z_offset;    
    pose.theta  = prev_pose.theta;
    pose.theta2 = prev_pose.theta2;
    pose.theta3 = prev_pose.theta3+theta_offset;
   
       
    initial_pose = pose;
    
    //matching
    for(layer_select = 1; layer_select>=1; layer_select-=1){
//    	printf("layer=%d\n",layer_select);
    	for(j = 0;j < 100; j++){
    		if(layer_select!=1 && j>2){
    			break;
    		}
    		bpose = pose;
	
    		e = adjust3d(scan_points, scan_points_num, &pose, layer_select);
    		//	printf("%f\n",e);

    		pose_mod(&pose);
	
    		if((bpose.x-pose.x)*(bpose.x-pose.x)+(bpose.y-pose.y)*(bpose.y-pose.y)+(bpose.z-pose.z)*(bpose.z-pose.z)
    				+3*(bpose.theta-pose.theta)*(bpose.theta-pose.theta)
					+3*(bpose.theta2-pose.theta2)*(bpose.theta2-pose.theta2)
					+3*(bpose.theta3-pose.theta3)*(bpose.theta3-pose.theta3) < 0.00001){
    			break;
    		}
    	}
    	iteration = j;

    	/*gps resetting*/
    	if(g_use_gnss){
    		static FILE *e_fp;
    		if(!e_fp){
    			e_fp=fopen("/tmp/e_log","w");
    		}
    		fprintf(e_fp,"%f\n",e);
    		if(layer_select==1&&e<1000){
    			printf("reset\n");
    			tf::StampedTransform gps_tf_on_world;
    			try{
    				listener.lookupTransform("world","gps",ros::Time(0),gps_tf_on_world);
    			}
    			catch(tf::TransformException ex){
    				ROS_ERROR("%s",ex.what());
    				return;
    			}
    			printf("set initial position by gps posiotion\n");
    			pose.x=gps_tf_on_world.getOrigin().x();
    			pose.y=gps_tf_on_world.getOrigin().y();
    			pose.z=gps_tf_on_world.getOrigin().z()+nrand(5);
    			tf::Quaternion q(gps_tf_on_world.getRotation().x(), gps_tf_on_world.getRotation().y(), gps_tf_on_world.getRotation().z(), gps_tf_on_world.getRotation().w());
    			double roll,pitch,yaw;
    			tf::Matrix3x3 m(q);
    			m.getRPY(roll,pitch,yaw);
    			pose.theta=roll;
    			pose.theta2=pitch+0.22;
    			pose.theta3=yaw+nrand(0.7);
    			prev_pose2=prev_pose=pose;
    			printf("reset %f %f %f %f %f %f\n", pose.x, pose.y, pose.z,
    					pose.theta, pose.theta2, pose.theta3);
    			//reset
	
    			/*	tf::Transform transform;
		//tf::Quaternion q;
	transform.setOrigin( tf::Vector3(pose.x, pose.y, pose.z) );
	q.setRPY(pose.theta,pose.theta2,pose.theta3);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),  "world", "ndt_frame"));
	*/
    			return;
    		}
    	}
      //unti-distotion

    	if(layer_select==2 && 1){
//    		double rate,angle,xrate,yrate,dx,dy,dtheta;
    		double rate,xrate,yrate,dx,dy,dtheta;
    		double tempx,tempy;
    		int i;
	
    		tempx=(pose.x - prev_pose.x);
    		tempy=(pose.y - prev_pose.y);
    		dx = tempx*cos(-prev_pose.theta3) - tempy*sin(-prev_pose.theta3);
    		dy = tempx*sin(-prev_pose.theta3) + tempy*cos(-prev_pose.theta3);
    		dtheta= pose.theta3-prev_pose.theta3;
    		if(dtheta < -M_PI){
    			dtheta+=2*M_PI;
    		}
    		if(dtheta >  M_PI){
    			dtheta-=2*M_PI;
    		}
	
    		rate = dtheta/(double)scan_points_num;
    		xrate=dx/(double)scan_points_num;
    		yrate=dy/(double)scan_points_num;
	
    		printf("untidist x %f y %f yaw %f\n",dx,dy,dtheta);
	
    		dx=-dx;
    		dy=-dy;
    		dtheta =-dtheta;
    		for(i=0;i<scan_points_num;i++){
    			tempx = scan_points[i].x * cos(dtheta) - scan_points[i].y * sin(dtheta)+dx;
    			tempy = scan_points[i].x * sin(dtheta) + scan_points[i].y * cos(dtheta)+dy;
	  
    			scan_points[i].x =tempx;
    			scan_points[i].y =tempy;
	  
    			dtheta += rate;
    			dx += xrate;
    			dy += yrate;
    		}
    	}
    }
    
    scan_transrate(scan_points, map_points, &pose, scan_points_num);

    for(int i = 0;i < scan_points_num;i++){
      map_points_i[i]=scan_points_i[i];
    }
    
    // update ND map
    distance = 
      (key_pose.x-pose.x)* (key_pose.x-pose.x)+
      (key_pose.y-pose.y)* (key_pose.y-pose.y)+
      (key_pose.z-pose.z)* (key_pose.z-pose.z);
   
    if(g_map_update&&(!is_map_exist || (distance>0.1*0.1 && scan_points_num > 100))){
    	int i;
    	for(i = 0;i < scan_points_num;i++){
    		add_point_map(NDmap,&map_points[i]);
    	}
    	key_pose = pose;
    	is_map_exist=1;
//    	publish_nd_map();
    }
    
    prev_pose2=prev_pose;
    prev_pose=pose;

    if(is_first_time){
      prev_pose2=prev_pose;
      is_first_time=0;
    }
    /*
    printf("%f %f %f %f %f %f %f %d \n",0,//(double)msg->header.stamp.toSec(),
	   pose.x,pose.y,pose.z,pose.theta,pose.theta2,pose.theta3,NDs_num) ; 
*/
    fprintf(log_fp,"%f %f %f %f %f %f %f\n",
	    (double)header.stamp.toSec(),
	    pose.x*cos(g_map_rotation)-pose.y*sin(g_map_rotation)+g_map_center_x,
	    pose.x*sin(g_map_rotation)+pose.y*cos(g_map_rotation)+g_map_center_y,
	    pose.z+g_map_center_z,pose.theta,pose.theta2,pose.theta3+g_map_rotation) ; 

    fflush(log_fp);
    tf::Transform transform;    
    tf::Quaternion q;
    transform.setOrigin( tf::Vector3(pose.x, pose.y, pose.z) );
    q.setRPY(pose.theta,pose.theta2,pose.theta3);
    transform.setRotation(q);

//    br.sendTransform(tf::StampedTransform(transform, header.stamp,  "world", "ndt_frame"));
    br.sendTransform(tf::StampedTransform(transform, header.stamp,  "map", "velodyne"));

    matching_end = std::chrono::system_clock::now();
    exe_time = std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() / 1000.0;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << msg->header.seq << std::endl;
  std::cout << "Number of scan points: " << msg->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << scan_points_num << " points." << std::endl;
  std::cout << "Number of iteration: " << iteration << std::endl;
  std::cout << "Execution time: " << exe_time << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << pose.x << ", " << pose.y << ", " << pose.z << ", " << pose.theta << ", " << pose.theta2 << ", " << pose.theta3 << ")" << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
  
//  ROS_INFO("get data %d",msg->points.size());

}

/*add point to ndcell */
int add_point_covariance(NDPtr nd,PointPtr p){

  /*add data num*/
  nd->num ++;
  nd->flag = 0; /*need to update*/
  //printf("%d \n",nd->num);
  
  /*calcurate means*/
  nd->m_x += p->x;
  nd->m_y += p->y;
  nd->m_z += p->z;

  /*calcurate covariances*/
  nd->c_xx += p->x*p->x;
  nd->c_yy += p->y*p->y;
  nd->c_zz += p->z*p->z;

  nd->c_xy += p->x*p->y;
  nd->c_yz += p->y*p->z;
  nd->c_zx += p->z*p->x;	

  return 1;
}


int inv_check(double inv[3][3]){
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      if(isnan(inv[i][j]))return 0;
      if(fabs(inv[i][j])>1000)return 0;
    }
  }
  return 1;
}

/*calcurate covariances*/
int update_covariance(NDPtr nd){
  double a,b,c;/*for calcurate*/
  if(!nd->flag){/*need calcurate?*/    
		/*means*/
    nd->mean.x =a= nd->m_x / nd->num; 
    nd->mean.y =b= nd->m_y / nd->num; 
    nd->mean.z =c= nd->m_z / nd->num; 
    
    /*covariances*/
    nd->covariance[0][0] = (nd->c_xx -2*a*nd->m_x )/ nd->num + a*a;
    nd->covariance[1][1] = (nd->c_yy -2*b*nd->m_y )/ nd->num + b*b;
    nd->covariance[2][2] = (nd->c_zz -2*c*nd->m_z )/ nd->num + c*c;
    nd->covariance[0][1]=nd->covariance[1][0] = (nd->c_xy - nd->m_x*b - nd->m_y*a)/ nd->num+ a*b;
    nd->covariance[1][2]=nd->covariance[2][1] = (nd->c_yz - nd->m_y*c - nd->m_z*b)/ nd->num+ b*c;
    nd->covariance[2][0]=nd->covariance[0][2] = (nd->c_zx - nd->m_z*a - nd->m_x*c)/ nd->num+ c*a;
    nd->sign=0;	
    nd->flag = 1;  /*this ND updated*/
    if(nd->num >= 5){    
      if(1||round_covariance(nd)==1){
	if(ginverse_matrix3d(nd->covariance, nd->inv_covariance))
	  if(inv_check(nd->inv_covariance))
	    nd->sign=1;
      }
    }
  }

  return 1;
}


/*add point to ndmap*/
int add_point_map(NDMapPtr ndmap, PointPtr point){
  int x,y,z,i;
  NDPtr  *ndp[8];  

  /*
    
  +---+---+
  |   |   |
  +---+---+
  |   |###|
  +---+---+
  
  */
  
  /*mapping*/
  x = (point->x / ndmap->size) + ndmap->x/2;
  y = (point->y / ndmap->size) + ndmap->y/2;
  z = (point->z / ndmap->size) + ndmap->z/2;
    
  /*clipping*/
  if(x < 1 || x >=  ndmap->x )return 0;
  if(y < 1 || y >=  ndmap->y )return 0;
  if(z < 1 || z >=  ndmap->z )return 0;
  
  /*select root ND*/
  ndp[0] = ndmap->nd + x*ndmap->to_x + y*ndmap->to_y +z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1;
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;
 
  /*add  point to map */
  for(i = 0;i < 8;i++){
    if((*ndp[i])==0)*ndp[i]=add_ND();
    if((*ndp[i])!=0)add_point_covariance(*ndp[i], point);
  }
  
  if(ndmap->next){
    add_point_map(ndmap->next , point);  
  }
    
  return 0;		
}

/*get nd cell at point*/
int get_ND(NDMapPtr ndmap,PointPtr point, NDPtr *nd,int  ndmode ){
  int x,y,z;
  int i;
  NDPtr *ndp[8];

  /*
    
  +---+---+
  |   |   |
  +---+---+
  |   |###|
  +---+---+
  
  *//*
  layer = layer_select;
  while(layer > 0){
    if(ndmap->next)ndmap = ndmap->next;
    layer--;
  }
    */
  /*mapping*/
  if(ndmode <3 ){
    x = (double)((point->x / ndmap->size) + ndmap->x/2 -0.5);
    y = (double)((point->y / ndmap->size) + ndmap->y/2 -0.5);
    z = (double)((point->z / ndmap->size) + ndmap->z/2 -0.5);  
  }else{
    x = (point->x / ndmap->size) + ndmap->x/2 ;
    y = (point->y / ndmap->size) + ndmap->y/2 ;
    z = (point->z / ndmap->size) + ndmap->z/2 ; 
  }
  
  /*clipping*/
  if(x < 1 || x >=  ndmap->x )return 0;
  if(y < 1 || y >=  ndmap->y )return 0;
  if(z < 1 || z >=  ndmap->z )return 0;
  

  /*select root ND*/
  ndp[0] = ndmap->nd + x*ndmap->to_x + y*ndmap->to_y +z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1; 
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;
  
  for(i = 0;i < 8;i ++){
    if(*ndp[i] != 0){
      if(!(*ndp[i])->flag)update_covariance(*ndp[i]);
      nd[i]=*ndp[i];
    }else {
      nd[i]=NDs;
      //return 0;
    }
  }

   
  return 1;		
}


NDPtr add_ND(void){
  NDPtr ndp;
  //int m;

  if(NDs_num>=MAX_ND_NUM){
    printf("over flow\n");
    return 0;
  }
  
  ndp= NDs+NDs_num;
  NDs_num++;
  
  ndp->flag = 0;
  ndp->sign = 0;
  ndp->num  = 0;
  ndp->m_x  = 0;
  ndp->m_y  = 0;
  ndp->m_z  = 0;
  ndp->c_xx = 0;
  ndp->c_yy = 0;
  ndp->c_zz = 0;
  ndp->c_xy = 0;
  ndp->c_yz = 0;
  ndp->c_zx = 0;
  ndp->w = 1;
  ndp->is_source = 0;
  
  return ndp;
}

NDMapPtr initialize_NDmap_layer(int layer, NDMapPtr child){
//int i,j,k,i2,i3,m;
  int i,j,k;
  int x,y,z;
  NDPtr    *nd,*ndp;
  NDMapPtr ndmap;
  
//  i2 = i3 = 0;
//  printf("Initializing...layer %d\n",layer);
  
  
  x = (g_map_x >> layer)+1;
  y = (g_map_y >> layer)+1; 
  z = (g_map_z >> layer)+1;
  
  /*����γ��ݡ�*/
  nd = (NDPtr*)malloc(x*y*z*sizeof(NDPtr));
  ndmap= (NDMapPtr)malloc(sizeof(NDMap));
  
  ndmap->x = x;
  ndmap->y = y;
  ndmap->z = z;
  ndmap->to_x = y*z;
  ndmap->to_y = z;
  ndmap->layer = layer;
  ndmap->nd = nd;
  ndmap->next = child;
  ndmap->size = g_map_cellsize * ((int)1<<layer);
//  printf("size %f\n",ndmap->size);
  
  ndp = nd;

  /*�쥤�䡼�ν��*/
  for(i = 0; i < x; i++){
    for(j = 0; j < y; j++){
      for(k = 0; k < z; k++){
	*ndp=0;
	ndp++;
      }
    }
  }

  /*�쥤�䡼�֤�Ϣ�롩*/
  return ndmap;
}

/*ND�ܥ�����ν��*/
NDMapPtr initialize_NDmap(void){
  int i;
  NDMapPtr ndmap;
  NDPtr null_nd;

  printf("Initialize NDmap\n");
  ndmap = 0;

  //init NDs
  NDs=(NDPtr)malloc(sizeof(NormalDistribution)*MAX_ND_NUM);
  NDs_num=0;
  
  null_nd= add_ND();

  for(i = LAYER_NUM-1;i >= 0;i--){
    ndmap = initialize_NDmap_layer(i,ndmap);

    /*progress dots*/
//    printf("layer %d\n",i);
  }

//  printf("done\n");

  return ndmap;/*���ֲ����ؤΥݥ��󥿤��֤�*/
}


int round_covariance(NDPtr nd){
  double v[3][3],a;

  eigenvecter_matrix3d(nd->covariance,v,nd->l);
  //  print_matrix3d(v);
  if(fabs(v[0][0]*v[0][0]+v[1][0]*v[1][0]+v[2][0]*v[2][0]-1) >0.1)printf("!1");
  if(fabs(v[0][0]*v[0][1]+v[1][0]*v[1][1]+v[2][0]*v[2][1]) >0.01)printf("!01");
  if(fabs(v[0][1]*v[0][2]+v[1][1]*v[1][2]+v[2][1]*v[2][2]) >0.01)printf("!02");
  if(fabs(v[0][2]*v[0][0]+v[1][2]*v[1][0]+v[2][2]*v[2][0]) >0.01)printf("!03");

  a = fabs(nd->l[1]/nd->l[0]);
  if(a <0.001){
    return 0;
    if(nd->l[1] > 0)
      nd->l[1] = fabs(nd->l[0])/10.0;
    else 
      nd->l[1] = -fabs(nd->l[0])/10.0;

  
    a = fabs(nd->l[2]/nd->l[0]);
    if(a <0.01){
      if(nd->l[2] > 0)
	nd->l[2] = fabs(nd->l[0])/10.0;
      else 
	nd->l[2] =-fabs(nd->l[0])/10.0;
     
     
    }
    //    printf("r");
    matrix3d_eigen(v,nd->l[0],nd->l[1],nd->l[2],nd->covariance);
  }
  return 1;
}


/*����ND�ܥ�������ǤΤ�����֤Ǥγ�Ψ*/
double probability_on_ND(NDPtr nd,double xp,double yp,double zp){
  //  double xp,yp,zp;
  double e;
  
  if(nd->num < 5)return 0;
  /*
  xp = x - nd->mean.x;
  yp = y - nd->mean.y;
  zp = z - nd->mean.z;
  */
  e = exp((xp*xp*nd->inv_covariance[0][0] +
	   yp*yp*nd->inv_covariance[1][1] + 
	   zp*zp*nd->inv_covariance[2][2] +
	   2.0* xp*yp*nd->inv_covariance[0][1] +
	   2.0* yp*zp*nd->inv_covariance[1][2] +
	   2.0* zp*xp*nd->inv_covariance[2][0] )
	  /-2.0);

  if(e > 1)return 1;
  if(e < 0)return 0;  
  return(e);
}

void load (char *name){
  FILE *fp;
  double x,y,z,q;
  Point p;

  fp=fopen(name,"r");
  int i = 0;
  while(fscanf(fp,"%lf %lf %lf %lf",&y,&x,&z,&q)!=EOF){//x,y swaped
    p.x=(x-g_map_center_x)*cos(-g_map_rotation)-(y-g_map_center_y)*sin(-g_map_rotation);
    p.y=(x-g_map_center_x)*sin(-g_map_rotation)+(y-g_map_center_y)*cos(-g_map_rotation);
    p.z=z-g_map_center_z;
    add_point_map(NDmap, &p);
    std::cout << i << std::endl;
    i++;
  }
  std::cout << "Finished loading " << name << std::endl;
  fclose(fp);
}

void save_nd_map(char* name){
  int i,j,k,layer;
  NDData nddat;
  NDMapPtr ndmap;
  NDPtr *ndp;
  FILE *ofp;
  
  //for pcd 
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ p;
  
  //cloud.is_dense = false;
  //cloud.points.resize (cloud.width * cloud.height);
  //ros::Time stamp;
  //stamp = msg->header.stamp;
  //double now = ros::Time::now().toSec();
  
  ndmap=NDmap;
  ofp=fopen(name,"w");
  
  for(layer=0;layer<2;layer++){
    ndp = ndmap->nd;
    /*�쥤�䡼�ν��*/
    for(i = 0; i < ndmap->x; i++){
      for(j = 0; j < ndmap->y; j++){
	for(k = 0; k < ndmap->z; k++){
	  if(*ndp){
	    update_covariance(*ndp); 
	    nddat.nd = **ndp;
	    nddat.x =i;
	    nddat.y =j;
	    nddat.z =k;
	    nddat.layer= layer;
	    
	    fwrite(&nddat,sizeof(NDData),1,ofp);

	    //regist the point to pcd data;
	    p.x = (*ndp)->mean.x;
	    p.y = (*ndp)->mean.y;
	    p.z = (*ndp)->mean.z;
	    cloud.points.push_back(p);
    
	  }
	  ndp++;
	}
      }
//      printf("a\n");
    }
    ndmap=ndmap->next;
    
  }
//  printf("done\n");
  fclose(ofp);


  //save pcd

  cloud.header.frame_id = "/map";
  cloud.width=cloud.points.size();
  cloud.height=1;
  pcl::io::savePCDFileASCII ("/tmp/ndmap.pcd", cloud);
  printf("NDMap points num: %d points.\n", (int)cloud.points.size());

  sensor_msgs::PointCloud2::Ptr ndmap_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(cloud, *ndmap_ptr);
  ndmap_pub.publish(*ndmap_ptr);

}





void publish_nd_map(){
  int i,j,k,layer;
  NDData nddat;
  NDMapPtr ndmap;
  NDPtr *ndp;

  //for pcd
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ p;

  //cloud.is_dense = false;
  //cloud.points.resize (cloud.width * cloud.height);
  //ros::Time stamp;
  //stamp = msg->header.stamp;
  //double now = ros::Time::now().toSec();

  ndmap=NDmap;

  for(layer=0;layer<2;layer++){
    ndp = ndmap->nd;
    /*�쥤�䡼�ν��*/
    for(i = 0; i < ndmap->x; i++){
      for(j = 0; j < ndmap->y; j++){
	for(k = 0; k < ndmap->z; k++){
	  if(*ndp){
	    update_covariance(*ndp);
	    nddat.nd = **ndp;
	    nddat.x =i;
	    nddat.y =j;
	    nddat.z =k;
	    nddat.layer= layer;

	    //regist the point to pcd data;
	    p.x = (*ndp)->mean.x;
	    p.y = (*ndp)->mean.y;
	    p.z = (*ndp)->mean.z;
	    cloud.points.push_back(p);

	  }
	  ndp++;
	}
      }
//      printf("a\n");
    }
    ndmap=ndmap->next;

  }

  //save pcd

  cloud.header.frame_id = "/map";
  cloud.width=cloud.points.size();
  cloud.height=1;
//  pcl::io::savePCDFileASCII ("/tmp/ndmap.pcd", cloud);
//  printf("NDMap points num: %d points.\n", (int)cloud.points.size());

  sensor_msgs::PointCloud2::Ptr ndmap_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(cloud, *ndmap_ptr);
  ndmap_pub.publish(*ndmap_ptr);

}










//load ndt setting file
int load_ndt_ini(char* name){
  FILE *ifp;

  ifp=fopen(name,"r");
  if(!ifp)return 0;

  //map path
  if(fscanf(ifp,"%s",g_ndmap_name)==EOF)return 0;
  //map size
  if(fscanf(ifp,"%d %d %d %lf",
	    &g_map_x,&g_map_y,&g_map_z,&g_map_cellsize)==EOF)return 0;
  //map center
  if(fscanf(ifp,"%lf %lf %lf %lf",
	    &g_map_center_x,&g_map_center_y,&g_map_center_z,
	    &g_map_rotation)==EOF)return 0;
  //use gnss
  if(fscanf(ifp,"%d", &g_use_gnss)==EOF)return 0;
  if(!g_use_gnss){
    if(fscanf(ifp,"%lf %lf %lf %lf %lf %lf",
	      &g_ini_x,&g_ini_y,&g_ini_z,
	      &g_ini_roll,&g_ini_pitch,&g_ini_yaw)==EOF)return 0;
  }

  //
  return 1;
}

int load_nd_map(char* name){
//  int i,j,k,layer;
  NDData nddat;
  NDMapPtr ndmap[2];
  NDPtr ndp;
  FILE *ifp;
  //  FILE *logfp;


  ndmap[0]=NDmap;
  ndmap[1]=NDmap->next;

  ifp=fopen(name,"r");
  if(!ifp)return 0;

  while(fread(&nddat,sizeof(NDData),1,ifp)>0){  
    ndp=add_ND();
    *ndp = nddat.nd;
    *(ndmap[nddat.layer]->nd + nddat.x*ndmap[nddat.layer]->to_x + nddat.y*ndmap[nddat.layer]->to_y + nddat.z)
      = ndp;
    ndp->flag=0;
    update_covariance(ndp); 
    //fprintf(logfp,"%f %f %f \n",ndp->mean.x, ndp->mean.y, ndp->mean.z);
  }

  printf("%d NDVoxels are loaded\n",NDs_num);
  fclose(ifp);
  return 1;
}

/*�ᥤ��ؿ�*/
int main(int argc, char* argv[])
{
	printf("----------------------\n");
	printf(" 3D NDT scan matching \n");
	printf("----------------------\n");

  ros::init(argc, argv, "ndt_matching_tku");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("downsampler", _downsampler);
  std::cout << "Downsampler: " << _downsampler << std::endl;
  if(_downsampler == "voxel_grid"){
	  _downsampler_num = 1;
  }
  if(_downsampler == "distance"){
	  _downsampler_num = 0;
  }

  ndmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndmap", 1000);

//  ros::Subscriber map_sub = nh.subscribe("points_map", 10, map_callback);
  ros::Subscriber points_sub = nh.subscribe("points_raw", 1000, points_callback);
  //ros::Subscriber sub = n.subscribe("hokuyo3d/slice", 1000, velodyneCallback);
//  char nd_filename[500];

  //load setting files
  /*
  if(argc>1){
    if(!load_ndt_ini(argv[1])){
      printf("setting file error\n");
      return 0;
    }
  }else{
    printf("can not find setting file\n");
    return 0;
  }
*/

  //map path
  sprintf(g_ndmap_name, "%s", "ndmap");

  //map size
  g_map_x = 2000;
  g_map_y = 2000;
  g_map_z = 200;
  g_map_cellsize = 1.0;
  //map center
  g_map_center_x = 0.0;
  g_map_center_y = 0.0;
  g_map_center_z = 0.0;
  g_map_rotation = 0.0;
  //use gnss
  g_use_gnss = 0;
  //initial pose
  g_ini_x = 0.0;
  g_ini_y = 0.0;
  g_ini_z = 0.0;
  g_ini_roll = 0.0;
  g_ini_pitch = 0.0;
  g_ini_yaw = 0.0;
/*
  printf("Map size\nx = %d(%5.2fm) \ny= %d(%5.2fm) \nz = %d(%5.2fm)\nmemory size = %dMB(Map%dMB)\n",
	 g_map_x, g_map_x*g_map_cellsize,
	 g_map_y, g_map_y*g_map_cellsize,
	 g_map_z, g_map_z*g_map_cellsize,
	 (g_map_x*g_map_y*g_map_z*(1+1/64.) * sizeof(NDPtr)+MAX_ND_NUM*sizeof(NormalDistribution))/(1024*1024),
	 (g_map_x*g_map_y*g_map_z*(1+1/64.) * sizeof(NDPtr)+MAX_ND_NUM*sizeof(NormalDistribution))/(1024*1024));
	 */
//  sprintf(scan_file_base_name,"%s",argv[1]);
  	
  /*initialize(clear) NDmap data*/
  NDmap = initialize_NDmap();

  //load map
  prev_pose.x = (g_ini_x-g_map_center_x)*cos(-g_map_rotation) - (g_ini_y-g_map_center_y)*sin(-g_map_rotation);
  prev_pose.y = (g_ini_x-g_map_center_x)*sin(-g_map_rotation) + (g_ini_y-g_map_center_y)*cos(-g_map_rotation);
  prev_pose.z = g_ini_z - g_map_center_z;
  prev_pose.theta  = g_ini_roll;
  prev_pose.theta2 = g_ini_pitch;
  prev_pose.theta3 = g_ini_yaw - g_map_rotation;
  
  prev_pose2=prev_pose;
  is_first_time=1;

  /*
  if(argc>2){
    printf("load point cloud map\n");    
    for(int i=2;i<argc;i++){
      printf("load(%d/%d) %s\n",i-2,argc-2,argv[i]);
      load(argv[i]);  
    }
    save_nd_map(g_ndmap_name);
    is_map_exist=1;
  }else if(argc>1){
    printf("load \n");
    if(load_nd_map(g_ndmap_name))  
      is_map_exist=1;
  }
*/
/*
  while (ros::ok()){
    ros::spinOnce();
  }
*/
  ros::spin();

  save_nd_map((char*)"/tmp/ndmap");
  return 1;
}
