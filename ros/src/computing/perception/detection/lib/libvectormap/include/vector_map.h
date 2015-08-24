#ifndef __VECTOR_MAP__
#define __VECTOR_MAP__

//#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include "Math.h"

#include <map_file/PointClassArray.h>
#include <map_file/LineClassArray.h>
#include <map_file/LaneArray.h>
#include <map_file/VectorClassArray.h>
#include <map_file/SignalArray.h>
#include <map_file/WhiteLineArray.h>
#include <map_file/DTLaneArray.h>


typedef struct{
  int pid;
  double bx;
  double ly;
  double h;
  double b;
  double l;
  int ref;
  int mcode1;
  int mcode2;
  int mcode3;
}Point;

typedef struct{
  int lid;
  int bpid;
  int fpid;
  int blid;
  int flid;
}Line;

typedef struct{
  int vid;
  int pid;
  double hang;
  double vang;
}Vector;
  

typedef struct{
  int id;
  int vid;
  int plid;
  int type;
  int linkid;
}Signal;

typedef struct{
  int id;
  int lid;
  double width;
  char color;
  int type;
  int linkid;
}WhiteLine;

typedef struct{
  int lid;
}Mark;

typedef struct{
  int did;
  double dist;
  int pid;
  double dir;
  double apara;
  double r;
  double slope;
  double cant;
  double lw;
  double rw;
}DTLane;

typedef struct{
  int lnid;
  int did;
  int blid;
  int flid;
  int bnid;
  int fnid;
  int jct;
  int blid2;
  int blid3;
  int blid4;
  int flid2;
  int flid3;
  int flid4;
  int clossid;
  double span;
  int lcnt;
  int lno;
}Lane;


class VectorMap
{
 public:
  bool loaded;
  std::map<int, Point > points;
  std::map<int, Line > lines;
  std::map<int, WhiteLine > whitelines;
  std::map<int, Lane > lanes;
  std::map<int, DTLane > dtlanes;
  std::map<int, Vector > vectors;
  std::map<int, Signal > signals;

  void load_points(const map_file::PointClassArray& msg);
  void load_lines(const map_file::LineClassArray& msg);
  void load_lanes(const map_file::LaneArray& msg);
  void load_vectors(const map_file::VectorClassArray& msg);
  void load_signals(const map_file::SignalArray& msg);
  void load_whitelines(const map_file::WhiteLineArray& msg);
  void load_dtlanes(const map_file::DTLaneArray& msg);


  VectorMap () :
	  loaded(false) {}

  inline Point3 getPoint (const int idx)
  {
	  Point3 p;
	  Point psrc = points[idx];
	  p.x() = psrc.bx; p.y() = psrc.ly, p.z() = psrc.h;
	  return p;
  }

};

#endif
