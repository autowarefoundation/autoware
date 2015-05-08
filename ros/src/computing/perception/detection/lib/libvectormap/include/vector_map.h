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
}CLine;

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
  std::map<int, CLine > clines;
  std::map<int, Lane > lanes;
  std::map<int, DTLane > dtlanes;
  std::map<int, Vector > vectors;
  std::map<int, Signal > signals;

  int load_points(char *name);
  int load_lines(char *name);
  int load_lanes(char *name);
  int load_vectors(char *name);
  int load_signals(char *name);
  int load_clines(char *name);
  int load_dtlanes(char *name );


  VectorMap () :
	  loaded(false) {}


  void loadAll (const std::string &dirname);

  inline Point3 getPoint (const int idx)
  {
	  Point3 p;
	  Point psrc = points[idx];
	  p.x() = psrc.bx; p.y() = psrc.ly, p.z() = psrc.h;
	  return p;
  }

};

#endif
