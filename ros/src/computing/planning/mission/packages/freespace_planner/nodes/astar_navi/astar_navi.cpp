
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include "waypoint_follower/lane.h"
#include <nav_msgs/GetMap.h>

#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <queue>

#define LOOP_RATE 10
#define THETA (M_PI/12) //15 or 30 degree ?
#define ARC 1.3        //0.5 or 1.0 or 1.3 ?
#define ANGLE_RANGE (M_PI/24) //goal angle 15*2 (degree)
#define SIZE_T 24 // 360/THETA
#define WF_COST 0.1 //WaveFront cost
#define W_GC 1.0 //weight of gc
#define W_STRAIGHT 1.0 //weight of straight
#define W_HC 1.0 //weight of hc
#define W_CURVE 1.08 //weight of curve
#define ACTUAL_CAR_LENGTH 4.5
#define ACTUAL_CAR_WIDTH 1.75
#define CAR_LENGTH 5.0
#define CAR_WIDTH 2.0
#define WHEELBASE 2.7
#define LASER 0.0 //laser offset from the center of car
#define SEARCH_LIMIT 50000

struct node_t{
  double x, y, theta;
  int status;     // NONE, OPEN, CLOSED or OBS
  double gc, hc;  // Actual cost, Heuristic cost
  double wf;      // Wavefront heuristic cost
  node_t *parent; // parent node
  bool goal;      // whether goal or not
  bool visited;   // for Wavefront
  int steering;
};

// for printing car path
struct path_t{
  double x, y, theta;
};

// for WaveFront
struct point_t{
  int x, y;
};

// overload of "<"
bool operator< (const node_t &node1, const node_t &node2){
  return node1.gc+node1.wf < node2.gc+node2.wf;
}
// overload of ">"
bool operator> (const node_t &node1, const node_t &node2){
  return node1.gc+node1.wf > node2.gc+node2.wf;
}

/* for Astar search */
enum STATUS {NONE, OPEN, CLOSED, OBS};

std::string PATH_FRAME = "/map"; //planning frame id
static std::vector< std::vector< std::vector<node_t> > > map;
static std::priority_queue< node_t, std::vector<node_t>, std::greater<node_t> > openlist;
static std::vector<node_t> closelist;
static bool _map_set = false;
static bool _goal_set = false;
static bool _start_set = false;
static int sx, sy, stheta, gx, gy, gtheta;//for map array
static double dsx, dsy, dstheta;// DOUBLE START POINT
static double dgx, dgy, dgtheta;// DOUBLE GOAL POINT
static double current_x, current_y, current_theta;      // FOR CURRENT POINT
static double origin_yaw;       // for transform
static int SIZE_X, SIZE_Y;      // map size
static int Astar_count = 0;

static waypoint_follower::lane ruled_waypoint; //lane_path -> 
static nav_msgs::Path plan_path;// for visualization
static geometry_msgs::PoseArray plan_poses;//for visualization
static tf::Transform _transform;

void CalcCenter(double *center_x1, double *center_y1, double *center_x2, double *center_y2);
bool Collide(const double &car_x, const double &car_y, const double &car_theta);
int WaveFront(std::queue<point_t> &qu);
void WaveFrontInit();



tf::Vector3 TransformPoint(const geometry_msgs::Pose &pose){

  tf::Vector3 point(pose.position.x, pose.position.y, pose.position.z);
  tf::Vector3 tfpoint = _transform * point;

  return tfpoint;
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  int i, j, k;
  double roll, pitch, yaw;

  /* for tf transform */
  tf::Transform inverse;
  tf::poseMsgToTF(msg->info.origin, inverse);
  _transform = inverse.inverse();
  tf::Quaternion quat(msg->info.origin.orientation.x, msg->info.origin.orientation.y,
		      msg->info.origin.orientation.z, msg->info.origin.orientation.w);
  tf::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw); // origin yaw (-PI ~ PI)
  origin_yaw = yaw;

  std::cout << "\norigin_yaw: " << origin_yaw << std::endl;
  printf("w:%d  h:%d  w*h:%d\n", msg->info.width, msg->info.height, msg->info.width*msg->info.height);

  SIZE_X = msg->info.width;
  SIZE_Y = msg->info.height;

  /* resize node according to map size */
  map.resize(SIZE_Y);
  for (i = 0; i < SIZE_Y; i++)
    map[i].resize(SIZE_X);
  for (i = 0; i < SIZE_Y; i++)
    for (j = 0; j < SIZE_X; j++)
      map[i][j].resize(SIZE_T);
  

  /* gridmap information to array */
  for (i = 0; i < SIZE_Y; i++){
    for (j = 0; j < SIZE_X; j++){
      if (msg->data[SIZE_Y*i+j] == 100){	//obstacle
	map[i][j][0].status = OBS;
      } else if (msg->data[SIZE_Y*i+j] == -1){	//unknown
	map[i][j][0].status = OBS;
      } else {                              	//free space
	for (k = 0; k < SIZE_T; k++){
	  map[i][j][k].status = NONE;
	  //map[i][j][k].x = j; map[i][j][k].y = i;
	  map[i][j][k].gc = 0.0; map[i][j][k].hc = 0.0;
	  map[i][j][k].wf = 0.0;
	  map[i][j][k].goal = false;
	  map[i][j][k].visited = false;
	  map[i][j][k].parent = NULL;
	  map[i][j][k].steering = -1;
	}
      }
    }
  }
  

  _map_set = true;
}



void mapReset(const nav_msgs::OccupancyGrid &msg)
{
  int i, j, k;
  double roll, pitch, yaw;

  /* for tf transform */
  tf::Transform inverse;
  tf::poseMsgToTF(msg.info.origin, inverse);
  _transform = inverse.inverse();
  tf::Quaternion quat(msg.info.origin.orientation.x, msg.info.origin.orientation.y,
		      msg.info.origin.orientation.z, msg.info.origin.orientation.w);
  tf::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw); // origin yaw (-PI ~ PI)
  origin_yaw = yaw;

  std::cout << "\norigin_yaw: " << origin_yaw << std::endl;
  printf("w:%d  h:%d  w*h:%d\n", msg.info.width, msg.info.height, msg.info.width*msg.info.height);

  SIZE_X = msg.info.width;
  SIZE_Y = msg.info.height;

  /* resize node according to map size */
  map.resize(SIZE_Y);
  for (i = 0; i < SIZE_Y; i++)
    map[i].resize(SIZE_X);
  for (i = 0; i < SIZE_Y; i++)
    for (j = 0; j < SIZE_X; j++)
      map[i][j].resize(SIZE_T);
  

  /* gridmap information to array */
  for (i = 0; i < SIZE_Y; i++){
    for (j = 0; j < SIZE_X; j++){
      if (msg.data[SIZE_Y*i+j] == 100){//obstacle
	map[i][j][0].status = OBS;
	//map[i][j][0].x = j; map[i][j][0].y = i;
	/*
	map[i][j][0].gc = 0.0; map[i][j][0].hc = 0.0;
	map[i][j][0].wf = 0.0;
	map[i][j][0].goal = false;
	map[i][j][0].visited = false;
	map[i][j][0].steering = -1;
	*/
      } else if (msg.data[SIZE_Y*i+j] == -1){//unknown
	map[i][j][0].status = OBS;
	//map[i][j][0].x = j; map[i][j][0].y = i;
	/*
	map[i][j][k].gc = 0.0; map[i][j][k].hc = 0.0;
	map[i][j][k].wf = 0.0;
	map[i][j][k].goal = false;
	map[i][j][k].visited = false;
	map[i][j][k].steering = -1;
	*/
      } else {                              //free space
	for (k = 0; k < SIZE_T; k++){
	  map[i][j][k].status = NONE;
	  //map[i][j][k].x = j; map[i][j][k].y = i;
	  map[i][j][k].gc = 0.0; map[i][j][k].hc = 0.0;
	  map[i][j][k].wf = 0.0;
	  map[i][j][k].goal = false;
	  map[i][j][k].visited = false;
	  map[i][j][k].parent = NULL;
	  map[i][j][k].steering = -1;
	}
      }
    }
  }
  
  //////

  _map_set = true;
}




void GoalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  int i, j, k;
  double roll, pitch, yaw, tfyaw;

  tf::Vector3 tfpoint = TransformPoint(msg->pose);

  gx = (int)(tfpoint.getX()*10);
  gy = (int)(tfpoint.getY()*10);
  dgx = msg->pose.position.x;
  dgy = msg->pose.position.y;

  tf::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y,
		      msg->pose.orientation.z, msg->pose.orientation.w);
  tf::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);
  dgtheta = yaw; // in real world

  tf::Quaternion tfquat = _transform * quat;
  tf::Matrix3x3 tfm(tfquat);
  tfm.getRPY(roll, pitch, tfyaw);
  if (tfyaw < 0) {tfyaw+=2*M_PI;}
  gtheta = (int)(tfyaw/THETA); // in grid cell

  printf("\n(dgx, dgy, dgtheta) = (%lf, %lf, %lf)\n", dgx, dgy, dgtheta);
  printf("(gx, gy, gtheta) = (%d, %d, %d)\n", gx, gy, gtheta);

  /////////for debug/////////
  //gx = 96; gy = 25;
  //dgx = 9.62; dgy = 2.557;
  //dgtheta = 1.512;
  //gtheta = (int)(dgtheta/THETA);
  ///////////////////////////


  /* goal range is about 1m square */
  for (i = gy-5; i < gy+5; i++){
    for (j = gx-5; j < gx+5; j++){
      for (k = 0; k < SIZE_T; k++){
	map[i][j][k].goal = true;
      }
    }
  }

  if (Collide(tfpoint.getX(), tfpoint.getY(), tfyaw)){
    std::cout << "INVALID GOAL POINT!" << std::endl;
    return;
  }

  /* Set WaveFront heurisic */
  WaveFrontInit();

  _goal_set = true;
}



void StartCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  ros::NodeHandle n;
  geometry_msgs::PoseStamped posestamped;
  double roll, pitch, yaw, tfyaw;

  tf::Vector3 tfpoint = TransformPoint(msg->pose.pose);

  sx = (int)(tfpoint.getX()*10);
  sy = (int)(tfpoint.getY()*10);
  dsx = msg->pose.pose.position.x;
  dsy = msg->pose.pose.position.y;

  tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
		      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);
  dstheta = yaw; // in real world

  tf::Quaternion tfquat = _transform * quat;
  tf::Matrix3x3 tfm(tfquat);
  tfm.getRPY(roll, pitch, tfyaw);
  if (tfyaw < 0) {tfyaw+=2*M_PI;} //make yaw 0~2PI
  stheta = (int)(tfyaw/THETA);  //0~23 (THETA=PI/12) in grid cell

  printf("\n(dsx, dsy, dstheta) = (%lf, %lf, %lf)\n", dsx, dsy, dstheta);
  printf("(sx, sy, stheta) = (%d, %d, %d)\n", sx, sy, stheta);

  ros::Publisher start_pub = n.advertise<geometry_msgs::PoseStamped>("start_pose", 1000, true);

  /* for start pose visualization */
  posestamped.header.frame_id = PATH_FRAME;
  posestamped.pose.position.x = dsx;
  posestamped.pose.position.y = dsy;
  posestamped.pose.position.z = 0.0;
  posestamped.pose.orientation.x = msg->pose.pose.orientation.x;
  posestamped.pose.orientation.y = msg->pose.pose.orientation.y;
  posestamped.pose.orientation.z = msg->pose.pose.orientation.z;
  posestamped.pose.orientation.w = msg->pose.pose.orientation.w;

  start_pub.publish(posestamped);
  /********************************/

  /*Collision detection of start point */
  if (Collide(tfpoint.getX(), tfpoint.getY(), tfyaw)){
    std::cout << "INVALID START POINT!" << std::endl;
    return;
  }


  /* set start point */
  map[sy][sx][stheta].status = OPEN;
  map[sy][sx][stheta].x = dsx;  map[sy][sx][stheta].y = dsy;
  map[sy][sx][stheta].theta = dstheta;
  map[sy][sx][stheta].gc = 0.0;
  map[sy][sx][stheta].parent = NULL;

  /* push start point to openlist */
  openlist.push(map[sy][sx][stheta]);

  /* set start point to current point */
  current_x = map[sy][sx][stheta].x;  current_y = map[sy][sx][stheta].y;
  current_theta = map[sy][sx][stheta].theta;


  _start_set = true;
}



/********************************/
/* for wavefront heuristic cost */
/********************************/
bool WaveFrontCollide(const int &grid_x, const int &grid_y){
  int i, j;
  int size;

  size = (int)(CAR_WIDTH*10/2);

  for (i = grid_y-size; i <= grid_y+size; i++){
    for (j = grid_x-size; j <= grid_x+size; j++){
      if (i < 0 || i >= SIZE_Y || j < 0 || j >= SIZE_X) {return true;}
      if (map[i][j][0].status == OBS) {return true;}
    }
  }

  return false;
}

int WaveFront(std::queue<point_t> &qu){
  int i, j;
  int current_x, current_y;
  int next_x, next_y;
  double cost;
  struct point_t temp;

  //wave front loop
  while(1){
    current_x = qu.front().x;
    current_y = qu.front().y;
    qu.pop();

    for (i = 0; i < 8; i++){
      switch(i){
      case 0: //right
	next_x = current_x+1;
	next_y = current_y;
	cost = WF_COST;
	break;
      case 1: //up
	next_x = current_x;
	next_y = current_y+1; 
	cost = WF_COST;
	break;
      case 2: //left
	next_x = current_x-1;
	next_y = current_y; 
	cost = WF_COST;
	break;
      case 3: //down
	next_x = current_x;
	next_y = current_y-1; 
	cost = WF_COST;
	break;
      case 4: //upper left
	next_x = current_x-1;
	next_y = current_y+1;
	cost = WF_COST*sqrt(2);
	break;
      case 5: //upper right
	next_x = current_x+1;
	next_y = current_y+1;
	cost = WF_COST*sqrt(2);
	break;
      case 6: //lower left
	next_x = current_x-1;
	next_y = current_y-1;
	cost = WF_COST*sqrt(2);
	break;
      case 7: //lower right
	next_x = current_x+1;
	next_y = current_y-1;
	cost = WF_COST*sqrt(2);
	break;
      }

      if (next_x < 0 || next_x >= SIZE_X || next_y < 0 || next_y >= SIZE_Y){//out of range
	continue;
      } else if (map[next_y][next_x][0].visited || map[next_y][next_x][0].status == OBS ||
		 WaveFrontCollide(next_x, next_y)){//collide obstacles
	continue;
      } else {
	for (j = 0; j < SIZE_T; j++){
	  map[next_y][next_x][j].wf = map[current_y][current_x][0].wf + cost;
	}
	map[next_y][next_x][0].visited = true;
	temp.x = next_x;
	temp.y = next_y;
	qu.push(temp);
      }
    }

    //END
    if (qu.empty()){
      return 1;
    }
  }

}

void WaveFrontInit(){
  int i;
  std::queue<point_t> qu;
  struct point_t temp;

  // goal grid
  for (i = 0; i < SIZE_T; i++){
    map[gy][gx][i].visited = true;
    map[gy][gx][i].wf = 0.0;
  }
  temp.x = gx;
  temp.y = gy;
  qu.push(temp);

  if (WaveFront(qu)){
    std::cout << "WaveFront!" << std::endl;
  }


}
/*************/
///////////////
/*************/


/* Collision detection */
bool Collide(const double &car_x, const double &car_y, const double &car_theta){
  int i, j;
  int grid_x, grid_y;
  int size_x, size_y;
  double x2, y2;
  int grid_x2, grid_y2;
  double x_offset, y_offset;

  x_offset = (WHEELBASE/2) * cos(car_theta);
  y_offset = (WHEELBASE/2) * sin(car_theta);

  size_x = (int)(CAR_LENGTH*10/2);
  size_y = (int)(CAR_WIDTH*10/2);
  grid_x = (int)(car_x*10) - (int)(LASER*10);
  grid_y = (int)(car_y*10);

  //std::cout << "(" << grid_x << ", " << grid_y << ")" << std::endl;
  //std::cout << "theta: " << car_theta << std::endl;

  for (i = grid_y-size_y; i < grid_y+size_y; i++){
    for (j = grid_x-size_x; j < grid_x+size_x; j++){
      // calcurate coordinates
      x2 = (double)(j-grid_x)*cos(car_theta) - (double)(i-grid_y)*sin(car_theta);
      y2 = (double)(j-grid_x)*sin(car_theta) + (double)(i-grid_y)*cos(car_theta);
      x2 += (double)grid_x+x_offset;  y2 += (double)grid_y+y_offset;
      grid_x2 = (int)x2;  grid_y2 = (int)y2;
      //std::cout << "(" << x2 << ", " << y2 << ")" << std::endl;
      if (grid_x2 < 0 || grid_y2 < 0 || 
	  grid_x2 >= SIZE_X || grid_y2 >= SIZE_Y){return true;}
      if (map[grid_y2][grid_x2][0].status == OBS){return true;}
    }
  }

  return false;
}



/******************************/
/* for printing car footprint */
/******************************/
void CalcCarsize(const double &car_x, const double &car_y, const double &car_theta){
  int i, j;
  int grid_x, grid_y;
  int size_x, size_y;
  double x2, y2;
  int grid_x2, grid_y2;
  ros::NodeHandle n;
  geometry_msgs::PolygonStamped polygon;
  geometry_msgs::Point32 temp;
  double x_offset, y_offset;

  x_offset = (WHEELBASE/2) * cos(car_theta);
  y_offset = (WHEELBASE/2) * sin(car_theta);

  ros::Publisher footprint_pub = n.advertise<geometry_msgs::PolygonStamped>("car_footprint", 1000, true);

  size_x = (int)(ACTUAL_CAR_LENGTH*10/2);
  size_y = (int)(ACTUAL_CAR_WIDTH*10/2);
  grid_x = (int)(car_x*10);///
  grid_y = (int)(car_y*10);///

  //std::cout << "(" << grid_x << ", " << grid_y << ")" << std::endl;
  //std::cout << "theta: " << car_theta << std::endl;

  polygon.header.frame_id = PATH_FRAME;
  for (i = grid_y-size_y; i < grid_y+size_y; i++){
    for (j = grid_x-size_x; j < grid_x+size_x; j++){
      if ((i==grid_y-size_y && j==grid_x-size_x) ||
	  (i==grid_y-size_y && j==grid_x+size_x-1) ||
	  (i==grid_y+size_y-1 && j==grid_x-size_x) ||
	  (i==grid_y+size_y-1 && j==grid_x+size_x-1) ||
	  (i==grid_y-size_y && j==grid_x+size_x-5) ||
	  (i==grid_y+size_y-1 && j==grid_x+size_x-5)){
	//
	x2 = (double)(j-grid_x)*cos(car_theta) - (double)(i-grid_y)*sin(car_theta);
	y2 = (double)(j-grid_x)*sin(car_theta) + (double)(i-grid_y)*cos(car_theta);
	x2 += (double)grid_x+x_offset;  y2 += (double)grid_y+y_offset;
	grid_x2 = (int)x2;  grid_y2 = (int)y2;
	//publish polygon
	temp.x = (float)grid_x2/10;
	temp.y = (float)grid_y2/10;
	temp.z = 0.0;
	polygon.polygon.points.push_back(temp);
      } else {
	continue;
      }
    }
  }
  //swap [3] and [5]
  temp = polygon.polygon.points[3];
  polygon.polygon.points[3] = polygon.polygon.points[5];
  polygon.polygon.points[5] = temp;


  polygon.polygon.points.push_back(polygon.polygon.points[4]);
  polygon.polygon.points.push_back(polygon.polygon.points[1]);
  polygon.polygon.points.push_back(polygon.polygon.points[4]);
  polygon.polygon.points.push_back(polygon.polygon.points[5]);
 
  footprint_pub.publish(polygon);

}

/******************************/
/* for printing car footprint */
/******************************/
void Carfootprint(){
  int i;
  std::vector<path_t> car_path;
  struct path_t temppath;
  struct node_t *tempnode;

  // gy,gx,gtheta was set in AStar()
  tempnode = &map[gy][gx][gtheta];
  while (tempnode->parent != NULL){
    temppath.x = tempnode->x;
    temppath.y = tempnode->y;
    temppath.theta = tempnode->theta;
    car_path.push_back(temppath);

    tempnode = tempnode->parent;
  }
  temppath.x = tempnode->x;
  temppath.y = tempnode->y;
  temppath.theta = tempnode->theta;
  car_path.push_back(temppath);

  // print from start point
  for (i = car_path.size()-1; i >= 0; i--){
    CalcCarsize(car_path[i].x, car_path[i].y, car_path[i].theta);
    usleep(100000);//delay 0.05s
  }

}


/* goal angle detection */
bool GoalAngle(const double &current_theta){
   double new_theta;

   // make theta -PI ~ PI
   if (current_theta <= -M_PI){
     new_theta = current_theta + 2*M_PI;
   } else if (current_theta > M_PI){
     new_theta = current_theta - 2*M_PI;
   } else {
     new_theta = current_theta;
   }

   // goal or not
   if (new_theta > dgtheta-ANGLE_RANGE && new_theta < dgtheta+ANGLE_RANGE){
     return true;
   } else {
     return false;
   }

}


double ChangeTheta(const double &current_theta){

  // make theta 0 ~ 2PI
  if (current_theta < 0){
    return current_theta + 2*M_PI;
  } else if (current_theta >= 2*M_PI){
    return current_theta - 2*M_PI;
  } else {
    return current_theta;
  }

}


int AStar(){
  int i;
  double tftheta, cost;
  double center_x1, center_y1, center_x2, center_y2;
  int grid_x, grid_y, grid_theta;
  int norm_x, norm_y, norm_theta;
  struct node_t temp;
  geometry_msgs::Pose pose, tfpose;
  
  // x1,y1: right circle
  CalcCenter(&center_x1, &center_y1, &center_x2, &center_y2);


  Astar_count++;
  //std::cout << Astar_count << std::endl;
  //std::cout << "openlist size: " << openlist.size() << std::endl;
  

  /* calculate tf point */
  tftheta = current_theta - origin_yaw;
  tftheta = ChangeTheta(tftheta);
  current_theta = ChangeTheta(current_theta);
  //if (tftheta < 0 || tftheta >= 2*M_PI) {std::cout << "ERROR!!" << std::endl;}
  tfpose.position.x = current_x; tfpose.position.y = current_y; tfpose.position.z = 0.0;
  tf::Vector3 tfpoint = TransformPoint(tfpose);

  /* current grid cell */
  norm_x = (int)(tfpoint.getX()*10); norm_y = (int)(tfpoint.getY()*10);
  norm_theta = (int)(tftheta/THETA);

  /// skip closed?
  //if (map[norm_y][norm_x].status == CLOSED){
  //  openlist.pop();
  //  if(openlist.empty()) {return 1;}
  //  temp = openlist.top();
  //  x = temp.x;  y = temp.y;  theta = temp.theta;
  //  return -1;
  //}
  ///

  map[norm_y][norm_x][norm_theta].status = CLOSED;
  //closelist.push_back(openlist.top()); //is this necessary?
  openlist.pop();


  /* for visualization */
  //posestamped.header.frame_id = PATH_FRAME;
  tf::Quaternion quat;
  quat.setEuler(0.0,0.0,map[norm_y][norm_x][norm_theta].theta);//setEulerZYX?
  pose.orientation.x = quat.getX();
  pose.orientation.y = quat.getY();
  pose.orientation.z = quat.getZ();
  pose.orientation.w = quat.getW();
  pose.position.x = map[norm_y][norm_x][norm_theta].x;
  pose.position.y = map[norm_y][norm_x][norm_theta].y;
  pose.position.z = 0.0;
  //posestamped.orientation.w = 1.0;
  plan_poses.poses.push_back(pose);
  ///////////


  // search for 6 states //
  for (i = 0; i < 6; i++){
    switch (i){
    case 0:// forward //
      //continue;
      if (map[norm_y][norm_x][norm_theta].steering == 1) {continue;}
      temp.x = current_x + ARC*cos(current_theta);
      temp.y = current_y + ARC*sin(current_theta);
      temp.theta = current_theta;
      cost = ARC*W_STRAIGHT;
      break;
    case 1:// backward //
      //continue;
      if (map[norm_y][norm_x][norm_theta].steering == 0) {continue;}
      temp.x = current_x + ARC*cos(current_theta+M_PI);
      temp.y = current_y + ARC*sin(current_theta+M_PI);
      temp.theta = current_theta;
      cost = ARC*W_STRAIGHT;
      break;
    case 2:// left forward //
      //continue;
      if (map[norm_y][norm_x][norm_theta].steering == 3 ||
	  map[norm_y][norm_x][norm_theta].steering == 4) {continue;}
      temp.x = (ARC/THETA)*cos(current_theta + 3*M_PI/2 + THETA) + center_x2;
      temp.y = (ARC/THETA)*sin(current_theta + 3*M_PI/2 + THETA) + center_y2;
      temp.theta = current_theta + THETA;
      cost = ARC*W_CURVE;
      break;
    case 3:// left backward //
      //continue;
      if (map[norm_y][norm_x][norm_theta].steering == 2 ||
	  map[norm_y][norm_x][norm_theta].steering == 5) {continue;}
      temp.x = (ARC/THETA)*cos(current_theta + 3*M_PI/2 - THETA) + center_x2;
      temp.y = (ARC/THETA)*sin(current_theta + 3*M_PI/2 - THETA) + center_y2;
      temp.theta = current_theta - THETA;
      cost = ARC*W_CURVE;
      break;
    case 4:// right forward //
      //continue;
      if (map[norm_y][norm_x][norm_theta].steering == 5 ||
	  map[norm_y][norm_x][norm_theta].steering == 2) {continue;}
      temp.x = (ARC/THETA)*cos(current_theta + M_PI/2 - THETA) + center_x1;
      temp.y = (ARC/THETA)*sin(current_theta + M_PI/2 - THETA) + center_y1;
      temp.theta = current_theta - THETA;
      cost = ARC*W_CURVE;
      break;
    case 5:// right backward //
      //continue;
      if (map[norm_y][norm_x][norm_theta].steering == 4 ||
	  map[norm_y][norm_x][norm_theta].steering == 3) {continue;}
      temp.x = (ARC/THETA)*cos(current_theta + M_PI/2 + THETA) + center_x1;
      temp.y = (ARC/THETA)*sin(current_theta + M_PI/2 + THETA) + center_y1;
      temp.theta = current_theta + THETA;
      cost = ARC*W_CURVE;
      break;
    }


    // calculate grid cell in each state
    tftheta = temp.theta - origin_yaw;
    tftheta = ChangeTheta(tftheta);
    if (tftheta < 0 || tftheta >= 2*M_PI) {std::cout << "ERROR!!" << std::endl;}///
    tfpose.position.x = temp.x; tfpose.position.y = temp.y; tfpose.position.z = 0.0;
    tfpoint = TransformPoint(tfpose);

    grid_x = (int)(tfpoint.getX()*10); grid_y = (int)(tfpoint.getY()*10);
    grid_theta = (int)(tftheta/THETA);

    /* process for Astar search */
    if (grid_x < 0 || grid_x >= SIZE_X ||
	grid_y < 0 || grid_y >= SIZE_Y){
      continue;
    } else if(Collide(tfpoint.getX(), tfpoint.getY(), tftheta)){
      continue;
    } else if (map[grid_y][grid_x][grid_theta].goal &&
	       GoalAngle(temp.theta)){ //current node is GOAL
      std::cout << Astar_count << std::endl;
      map[grid_y][grid_x][grid_theta].x = temp.x;
      map[grid_y][grid_x][grid_theta].y = temp.y;
      map[grid_y][grid_x][grid_theta].theta = temp.theta;
      map[grid_y][grid_x][grid_theta].parent = &map[norm_y][norm_x][norm_theta];
      map[grid_y][grid_x][grid_theta].steering = i;
      gx = grid_x;  gy = grid_y;
      gtheta = grid_theta;
      return 0;
    } else if (map[grid_y][grid_x][grid_theta].status == OPEN){
      if (map[norm_y][norm_x][norm_theta].gc+cost < 
	  map[grid_y][grid_x][grid_theta].gc){
	map[grid_y][grid_x][grid_theta].x = temp.x;
	map[grid_y][grid_x][grid_theta].y = temp.y;
	map[grid_y][grid_x][grid_theta].theta = temp.theta;
	map[grid_y][grid_x][grid_theta].gc = map[norm_y][norm_x][norm_theta].gc+cost;
	//map[grid_y][grid_x][grid_theta].hc = W_HC * hypot(dgx-temp.x, dgy-temp.y);
	map[grid_y][grid_x][grid_theta].parent = &map[norm_y][norm_x][norm_theta];
	map[grid_y][grid_x][grid_theta].steering = i;
	openlist.push(map[grid_y][grid_x][grid_theta]); // can't erase old node?
      }
    } else if (map[grid_y][grid_x][grid_theta].status == CLOSED){
      if (map[norm_y][norm_x][norm_theta].gc+cost < 
	  map[grid_y][grid_x][grid_theta].gc){
	map[grid_y][grid_x][grid_theta].x = temp.x;
	map[grid_y][grid_x][grid_theta].y = temp.y;
	map[grid_y][grid_x][grid_theta].theta = temp.theta;
	map[grid_y][grid_x][grid_theta].gc = map[norm_y][norm_x][norm_theta].gc+cost;
	//map[grid_y][grid_x][grid_theta].hc = W_HC * hypot(dgx-temp.x, dgy-temp.y);
	map[grid_y][grid_x][grid_theta].parent = &map[norm_y][norm_x][norm_theta];
	map[grid_y][grid_x][grid_theta].steering = i;
	openlist.push(map[grid_y][grid_x][grid_theta]);
      }
    } else {
      map[grid_y][grid_x][grid_theta].x = temp.x; 
      map[grid_y][grid_x][grid_theta].y = temp.y;
      map[grid_y][grid_x][grid_theta].theta = temp.theta;
      map[grid_y][grid_x][grid_theta].status = OPEN;
      map[grid_y][grid_x][grid_theta].gc = map[norm_y][norm_x][norm_theta].gc+cost;
      //map[grid_y][grid_x][grid_theta].hc = W_HC * hypot(dgx-temp.x, dgy-temp.y);
      map[grid_y][grid_x][grid_theta].parent = &map[norm_y][norm_x][norm_theta];
      map[grid_y][grid_x][grid_theta].steering = i;
      openlist.push(map[grid_y][grid_x][grid_theta]);
    }

    //std::cout << "openlist size: " << openlist.size() << std::endl;
  }


  // if opnelist is empty, failed
  if (openlist.empty()){
    std::cout << "EMPTY!!" << std::endl;
    return 1;
  }


  // set next start point
  temp = openlist.top();
  current_x = temp.x;  current_y = temp.y;
  current_theta = temp.theta;

  // terminate search
  if (Astar_count == SEARCH_LIMIT)
    return 1;

  return -1;//for loop

}



void PrintPath(){
  int i;
  struct node_t *temp;
  std::vector<path_t> path_list;
  struct path_t temp_path;
  geometry_msgs::PoseStamped posestamped;
  waypoint_follower::waypoint waypoint;

  plan_path.header.frame_id = PATH_FRAME;//
  ruled_waypoint.header.frame_id = PATH_FRAME;///
  ruled_waypoint.header.stamp = ros::Time::now();//?

  temp = &map[gy][gx][gtheta];

  // search from a parent node
  
  while (temp->parent != NULL){
    std::cout << "" << temp->x << " " << temp->y << "" << std::endl;
    //Path
    posestamped.header = plan_path.header;
    posestamped.pose.position.x = temp->x;
    posestamped.pose.position.y = temp->y;
    posestamped.pose.position.z = 0.0;//
    posestamped.pose.orientation.w = 1.0;//??
    plan_path.poses.push_back(posestamped);


    temp_path.x = temp->x;
    temp_path.y = temp->y;
    temp_path.theta = temp->theta;
    path_list.push_back(temp_path);

    temp = temp->parent;
  }
  std::cout << "" << temp->x << " " << temp->y << "" << std::endl;
  posestamped.header = plan_path.header;
  posestamped.pose.position.x = temp->x;
  posestamped.pose.position.y = temp->y;
  posestamped.pose.position.z = 0.0;//
  posestamped.pose.orientation.w = 1.0;//
  plan_path.poses.push_back(posestamped);

  temp_path.x = temp->x;
  temp_path.y = temp->y;
  temp_path.theta = temp->theta;
  path_list.push_back(temp_path);

  //from start point
  std::cout << path_list.size() << std::endl;
  for (i = path_list.size()-1; i >= 0; i--){
    waypoint.pose.header = ruled_waypoint.header;
    waypoint.twist.header = ruled_waypoint.header;
    waypoint.pose.pose.position.x = path_list[i].x;
    waypoint.pose.pose.position.y = path_list[i].y;
    waypoint.pose.pose.position.z = 0.0;//
    waypoint.pose.pose.orientation.w = 1.0;
    waypoint.twist.twist.linear.x = 1.0;
    ruled_waypoint.waypoints.push_back(waypoint);
  }
  

}



void CalcCenter(double *center_x1, double *center_y1, double *center_x2, double *center_y2){

  *center_x1 = current_x - cos(current_theta + M_PI/2)*(ARC/THETA);
  *center_y1 = current_y - sin(current_theta + M_PI/2)*(ARC/THETA);
  *center_x2 = current_x - cos(current_theta + 3*M_PI/2)*(ARC/THETA);
  *center_y2 = current_y - sin(current_theta + 3*M_PI/2)*(ARC/THETA);

}

//init Data
void InitData(){

  ruled_waypoint.waypoints.clear();
  plan_path.poses.clear();
  plan_poses.poses.clear();
  closelist.clear();
  map.clear();
  Astar_count = 0;

  while (!openlist.empty()){
    openlist.pop();
  }

}


int main(int argc, char **argv)
{
  int i = 0, j;
  int end_flag = -1;
  static int count = 0;
  geometry_msgs::PoseStamped posestamped;
  nav_msgs::GetMap getmap;

  ros::init(argc, argv, "astar_navi");

  ros::NodeHandle n;


  ros::Subscriber map_sub = n.subscribe("/map", 1000, mapCallback);
  ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 1000, GoalCallback);
  ros::Subscriber start_sub = n.subscribe("/initialpose", 1000, StartCallback);

  ros::Publisher lane_pub = n.advertise<nav_msgs::Path>("lane_waypoint", 1000, true);
  ros::Publisher ruled_pub = n.advertise<waypoint_follower::lane>("traffic_waypoint", 1000, true);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseArray>("poses", 1000, true);
  ros::Publisher start_pub = n.advertise<geometry_msgs::PoseStamped>("start_pose", 1000, true);
  ros::Publisher footprint_pub = n.advertise<geometry_msgs::PolygonStamped>("car_footprint", 1000, true);

  ros::ServiceClient getmap_srv_ = n.serviceClient<nav_msgs::GetMap>("/static_map");

  plan_poses.header.frame_id = PATH_FRAME;//for debug



  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok()){
    ros::spinOnce();

    if (_map_set == false || _goal_set == false || _start_set == false) {
      std::cout << "\rtopic waiting          \rtopic waiting";
      for (j = 0; j < i; j++) {std::cout << ".";}
      i++;
      i = i%10;
      std::cout << std::flush;
      loop_rate.sleep();
      continue;
    }

    /*******for debug***********/
    /*
    sx = 58; sy = 128;
    dstheta = 0.000;
    stheta = 0;
    map[sy][sx][stheta].x = 5.899;  map[sy][sx][stheta].y = 12.898;
    map[sy][sx][stheta].theta = 0.00;
    */
    /***************************/
    

    /* Atar search loop */
    while(1){
      count++;
      end_flag = AStar();
      if (!(count%1000) || count < 1000){
	pose_pub.publish(plan_poses);//for visualization
      }
      if (!end_flag){
	std::cout << "GOAL!!" << std::endl;
	PrintPath();
	lane_pub.publish(plan_path);
	ruled_pub.publish(ruled_waypoint);
	pose_pub.publish(plan_poses);
	Carfootprint();
	_start_set = false;
	_goal_set = false;
	_map_set = false;

	/* reset map data */
	InitData();
	getmap_srv_.call(getmap);
	mapReset(getmap.response.map);

	break;
      } else if (end_flag == 1){
	std::cout << "FAILED..." << std::endl;

	_start_set = false;
	_goal_set = false;
	_map_set = false;

	/* reset map data */
	InitData();
	getmap_srv_.call(getmap);
	mapReset(getmap.response.map);

	break;
      } else {
	continue;
      }
    }

  }

  return 0;
}
