/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "pack_into.hpp"
#include <map_db.h>
#include <libgen.h>
#include <fstream>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <sys/stat.h>

int swap_x_y = 1;
std::vector<std::string> vmap_file_list;
std::vector<std::string> vmap_csv_list{
	"area.csv",
	"crosswalk.csv",
	"curb.csv",
	"dtlane.csv",
	"gutter.csv",
	"idx.csv",
	"lane.csv",
	"line.csv",
	"node.csv",
	"point.csv",
	"pole.csv",
	"poledata.csv",
	"road_surface_mark.csv",
	"roadedge.csv",
	"roadsign.csv",
	"signaldata.csv",
	"stopline.csv",
	"streetlight.csv",
	"utilitypole.csv",
	"vector.csv",
	"whiteline.csv",
	"zebrazone.csv"
};

static GetFile gf;

typedef std::vector<std::vector<std::string>> Tbl;

Tbl read_csv(const char* filename, int* max_id)
{
  std::ifstream ifs(filename);
  std::string line;
  std::getline(ifs, line); // Remove first line

  Tbl tbl;

  *max_id = -1;
  while (std::getline(ifs, line)) {
    std::istringstream ss(line);

    std::vector<std::string> columns;
    std::string column;
    while (std::getline(ss, column, ',')) {
      columns.push_back(column);
    }
    tbl.push_back(columns);

    int id = std::stoi(columns[0]);
    if (id > *max_id) *max_id = id;
  }
  return tbl;
}

std::vector<RoadEdge> read_roadedge(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<RoadEdge> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].lid = std::stoi(tbl[i][1]);
    ret[id].linkid = std::stoi(tbl[i][2]);
  }
  return ret;
}

std::vector<Gutter> read_gutter(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<Gutter> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].aid = std::stoi(tbl[i][1]);
    ret[id].type = std::stoi(tbl[i][2]);
    ret[id].linkid = std::stoi(tbl[i][3]);
  }
  return ret;
}

std::vector<Curb> read_curb(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<Curb> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].lid = std::stoi(tbl[i][1]);
    ret[id].height = std::stod(tbl[i][2]);
    ret[id].width = std::stod(tbl[i][3]);
    ret[id].dir = std::stoi(tbl[i][4]);
    ret[id].linkid = std::stoi(tbl[i][5]);
  }
  return ret;
}

std::vector<WhiteLine> read_whiteline(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<WhiteLine> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].lid = std::stoi(tbl[i][1]);
    ret[id].width = std::stod(tbl[i][2]);
    ret[id].color = tbl[i][3].c_str()[0];
    ret[id].type = std::stoi(tbl[i][4]);
    ret[id].linkid = std::stoi(tbl[i][5]);
  }
  return ret;
}

std::vector<StopLine> read_stopline(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<StopLine> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].lid = std::stoi(tbl[i][1]);
    ret[id].tlid = std::stoi(tbl[i][2]);
    ret[id].signid = std::stoi(tbl[i][3]);
    ret[id].linkid = std::stoi(tbl[i][4]);
  }
  return ret;
}

std::vector<ZebraZone> read_zebrazone(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<ZebraZone> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].aid = std::stoi(tbl[i][1]);
    ret[id].linkid = std::stoi(tbl[i][2]);
  }
  return ret;
}

std::vector<CrossWalk> read_crosswalk(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<CrossWalk> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].aid = std::stoi(tbl[i][1]);
    ret[id].type = std::stoi(tbl[i][2]);
    ret[id].bdid = std::stoi(tbl[i][3]);
    ret[id].linkid = std::stoi(tbl[i][4]);
  }
  return ret;
}

std::vector<RoadMark> read_roadmark(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<RoadMark> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].aid = std::stoi(tbl[i][1]);
    ret[id].type = std::stoi(tbl[i][2]); // Don't use wide character
    ret[id].linkid = std::stoi(tbl[i][3]);
  }
  return ret;
}

std::vector<Pole> read_pole(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<Pole> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].plid = std::stoi(tbl[i][1]);
    ret[id].linkid = std::stoi(tbl[i][2]);
  }
  return ret;
}

std::vector<RoadSign> read_roadsign(const char *filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<RoadSign> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].vid = std::stoi(tbl[i][1]);
    ret[id].plid = std::stoi(tbl[i][2]);
    ret[id].type = std::stoi(tbl[i][3]); // Don't use wide character
    ret[id].linkid = std::stoi(tbl[i][4]);
  }
  return ret;
}

std::vector<Signal> read_signal(const char *filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<Signal> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].vid = std::stoi(tbl[i][1]);
    ret[id].plid = std::stoi(tbl[i][2]);
    ret[id].type = std::stoi(tbl[i][3]);
    ret[id].linkid = std::stoi(tbl[i][4]);
  }
  return ret;
}

std::vector<StreetLight> read_streetlight(const char *filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<StreetLight> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].lid = std::stoi(tbl[i][1]);
    ret[id].plid = std::stoi(tbl[i][2]);
    ret[id].linkid = std::stoi(tbl[i][3]);
  }
  return ret;
}

std::vector<UtilityPole> read_utilitypole(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<UtilityPole> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].plid = std::stoi(tbl[i][1]);
    ret[id].linkid = std::stoi(tbl[i][2]);
  }
  return ret;
}

std::vector<GuardRail> read_guardrail(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<GuardRail> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].aid = std::stoi(tbl[i][1]);
    ret[id].type = std::stoi(tbl[i][2]);
    ret[id].linkid = std::stoi(tbl[i][3]);
  }
  return ret;
}

std::vector<SideWalk> read_sidewalk(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<SideWalk> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].aid = std::stoi(tbl[i][1]);
    ret[id].linkid = std::stoi(tbl[i][2]);
  }
  return ret;
}

std::vector<CrossRoad> read_crossroad(const char* filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<CrossRoad> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].id = id;
    ret[id].aid = std::stoi(tbl[i][1]);
    ret[id].linkid = std::stoi(tbl[i][2]);
  }
  return ret;
}

std::vector<PointClass> read_pointclass(const char *filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<PointClass> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].pid = id;
    ret[id].b = std::stod(tbl[i][1]);
    ret[id].l = std::stod(tbl[i][2]);
    ret[id].h = std::stod(tbl[i][3]);
    ret[id].bx = std::stod(tbl[i][4]);
    ret[id].ly = std::stod(tbl[i][5]);
    ret[id].ref = std::stoi(tbl[i][6]);
    ret[id].mcode1 = std::stoi(tbl[i][7]);
    ret[id].mcode2 = std::stoi(tbl[i][8]);
    ret[id].mcode3 = std::stoi(tbl[i][9]);
  }
  return ret;
}

std::vector<VectorClass> read_vectorclass(const char *filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<VectorClass> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].vid = id;
    ret[id].pid = std::stoi(tbl[i][1]);
    ret[id].hang = std::stod(tbl[i][2]);
    ret[id].vang = std::stod(tbl[i][3]);
  }
  return ret;
}

std::vector<LineClass> read_lineclass(const char *filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<LineClass> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].lid = id;
    ret[id].bpid = std::stoi(tbl[i][1]);
    ret[id].fpid = std::stoi(tbl[i][2]);
    ret[id].blid = std::stoi(tbl[i][3]);
    ret[id].flid = std::stoi(tbl[i][4]);
  }
  return ret;
}

std::vector<AreaClass> read_areaclass(const char *filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<AreaClass> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].aid = id;
    ret[id].slid = std::stoi(tbl[i][1]);
    ret[id].elid = std::stoi(tbl[i][2]);
  }
  return ret;
}

std::vector<PoleClass> read_poleclass(const char *filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<PoleClass> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].plid = id;
    ret[id].vid = std::stoi(tbl[i][1]);
    ret[id].length = std::stod(tbl[i][2]);
    ret[id].dim = std::stod(tbl[i][3]);
  }
  return ret;
}

std::vector<BoxClass> read_boxclass(const char *filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<BoxClass> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].bid = id;
    ret[id].pid1 = std::stoi(tbl[i][1]);
    ret[id].pid2 = std::stoi(tbl[i][2]);
    ret[id].pid3 = std::stoi(tbl[i][3]);
    ret[id].pid4 = std::stoi(tbl[i][4]);
    ret[id].height = std::stod(tbl[i][5]);
  }
  return ret;
}

std::vector<DTLane> read_dtlane(const char *filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<DTLane> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].did = id;
    ret[id].dist = std::stod(tbl[i][1]);
    ret[id].pid = std::stoi(tbl[i][2]);
    ret[id].dir = std::stod(tbl[i][3]);
    ret[id].apara = std::stod(tbl[i][4]);
    ret[id].r = std::stod(tbl[i][5]);
    ret[id].slope = std::stod(tbl[i][6]);
    ret[id].cant = std::stod(tbl[i][7]);
    ret[id].lw = std::stod(tbl[i][8]);
    ret[id].rw = std::stod(tbl[i][9]);
  }
  return ret;
}

std::vector<Node> read_node(const char *filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<Node> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].nid = id;
    ret[id].pid = std::stoi(tbl[i][1]);
  }
  return ret;
}

std::vector<Lane> read_lane(const char *filename)
{
  int max_id;
  Tbl tbl = read_csv(filename, &max_id);
  size_t i, n = tbl.size();
  std::vector<Lane> ret(max_id + 1, {-1});
  for (i=0; i<n; i++) {
    int id = std::stoi(tbl[i][0]);
    ret[id].lnid = id;
    ret[id].did = std::stoi(tbl[i][1]);
    ret[id].blid = std::stoi(tbl[i][2]);
    ret[id].flid = std::stoi(tbl[i][3]);
    ret[id].bnid = std::stoi(tbl[i][4]);
    ret[id].fnid = std::stoi(tbl[i][5]);
    ret[id].jct = std::stoi(tbl[i][6]);
    ret[id].blid2 = std::stoi(tbl[i][7]);
    ret[id].blid3 = std::stoi(tbl[i][8]);
    ret[id].blid4 = std::stoi(tbl[i][9]);
    ret[id].flid2 = std::stoi(tbl[i][10]);
    ret[id].flid3 = std::stoi(tbl[i][11]);
    ret[id].flid4 = std::stoi(tbl[i][12]);
    ret[id].clossid = std::stoi(tbl[i][13]);
    ret[id].span = std::stod(tbl[i][14]);
    ret[id].lcnt = std::stoi(tbl[i][15]);
    ret[id].lno = std::stoi(tbl[i][16]);
  }
  return ret;
}


void calc_ang_to_xyzw(double vang, double hang, double* x, double* y, double* z, double *w)
{
  *x = *y = *z = 0;
  *w = 1;

  if (vang == 90) {
    double rad = M_PI * (-hang) / 180;
    *x = 0;
    *y = 0;
    *z = sin(rad/2);
    *w = cos(rad/2);
  }
}

void set_marker_data(visualization_msgs::Marker* marker,
		    double px, double py, double pz, double ox, double oy, double oz, double ow,
		    double sx, double sy, double sz, double r, double g, double b, double a)
{
  if(swap_x_y) {
    marker->pose.position.x = py;
    marker->pose.position.y = px;
    marker->pose.orientation.x = oy;
    marker->pose.orientation.y = ox;
  } else {
    marker->pose.position.x = px;
    marker->pose.position.y = py;
    marker->pose.orientation.x = ox;
    marker->pose.orientation.y = oy;
  }
  marker->pose.position.z = pz;

  marker->pose.orientation.z = oz;
  marker->pose.orientation.w = ow;

  marker->scale.x = sx;
  marker->scale.y = sy;
  marker->scale.z = sz;

  marker->color.r = r;
  marker->color.g = g;
  marker->color.b = b;
  marker->color.a = a;

  //  marker->id++;
}

void add_marker_points(visualization_msgs::Marker* marker, double x, double y, double z)
{
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  marker->points.push_back(p);
}

void add_marker_points_pointclass(visualization_msgs::Marker* marker, PointClass& pclass)
{
  if(swap_x_y) {
    add_marker_points(marker,
		      pclass.ly,
		      pclass.bx,
		      pclass.h);
  } else {
    add_marker_points(marker,
		      pclass.bx,
		      pclass.ly,
		      pclass.h);
  }
}

void push_marker(visualization_msgs::Marker* marker,
		   visualization_msgs::MarkerArray* marker_array)
{
    marker_array->markers.push_back(*marker);
    marker->id++;
}

void set_poleclass_data(PoleClass& poleclass,
			double r, double g, double b, double a,
			std::vector<VectorClass> vectorclasses,
			std::vector<PointClass> pointclasses,
			visualization_msgs::Marker* marker,
			visualization_msgs::MarkerArray* marker_array)
{
  int vid = poleclass.vid;
  int pid = vectorclasses[vid].pid;

  marker->type = visualization_msgs::Marker::CYLINDER;

  double ox, oy, oz, ow;
  calc_ang_to_xyzw(vectorclasses[vid].vang, vectorclasses[vid].hang, 
		   &ox, &oy, &oz, &ow);

  set_marker_data(marker,
		  pointclasses[pid].bx, 
		  pointclasses[pid].ly, 
		  pointclasses[pid].h + poleclass.length / 2,
		  ox, oy, oz, ow,
		  poleclass.dim, 
		  poleclass.dim, 
		  poleclass.length,
		  r, g, b, a);

  push_marker(marker, marker_array);
}		  

template <typename T>
void set_areaclass_data(T zones,
		       double w, 
		       double r, double g, double b, double a,
		       std::vector<AreaClass> areaclasses,
		       std::vector<LineClass> lines,
		       std::vector<PointClass> pointclasses,
		       visualization_msgs::Marker* marker,
		       visualization_msgs::MarkerArray* marker_array)
{
  marker->type = visualization_msgs::Marker::LINE_STRIP;

  for (int i=0; i<static_cast<int>(zones.size()); i++) {
    if (zones[i].id <= 0) {
      continue;
    }
    int aid = zones[i].aid;
    int slid = areaclasses[aid].slid;
    int elid = areaclasses[aid].elid;

    set_marker_data(marker,
		    0, 0, 0,
		    0, 0, 0, 1,
		    w, 0, 0,
		    r, g, b, a);
    marker->points.clear();
    for(int lid = slid; ; lid = lines[lid].flid) {
      int pid = lines[lid].bpid;
      add_marker_points_pointclass(marker, pointclasses[pid]);
      if (lid == elid) {
	pid = lines[lid].fpid;
	add_marker_points_pointclass(marker, pointclasses[pid]);
	break;
      }
    }
    push_marker(marker, marker_array);
  }
}


int main(int argc, char **argv)
{

/*

#!/bin/sh
rosrun map_file vector_map_loader <csv files>

# EOF

*/

  ros::init(argc, argv, "sample_vector_map");
  ros::NodeHandle n;
  bool publish_lane;
  n.param<bool>("/sample_vector_map/publish_lane", publish_lane, false);
  ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("/vector_map", 1000, true);
  ros::Publisher stat_publisher = n.advertise<std_msgs::Bool>("/vmap_stat", 100);;
  std_msgs::Bool vmap_stat_msg;

  ros::Publisher pub_point_class = n.advertise<map_file::PointClassArray>(
	  "/vector_map_info/point_class", 1, true);
  ros::Publisher pub_vector_class = n.advertise<map_file::VectorClassArray>(
	  "/vector_map_info/vector_class", 1, true);
  ros::Publisher pub_line_class = n.advertise<map_file::LineClassArray>(
	  "/vector_map_info/line_class", 1, true);
  ros::Publisher pub_area_class = n.advertise<map_file::AreaClassArray>(
	  "/vector_map_info/area_class", 1, true);
  ros::Publisher pub_pole_class = n.advertise<map_file::PoleClassArray>(
	  "/vector_map_info/pole_class", 1, true);
  ros::Publisher pub_box_class = n.advertise<map_file::BoxClassArray>(
	  "/vector_map_info/box_class", 1, true);

  ros::Publisher pub_dtlane = n.advertise<map_file::DTLaneArray>(
	  "/vector_map_info/dtlane", 1, true);
  ros::Publisher pub_node = n.advertise<map_file::NodeArray>(
	  "/vector_map_info/node", 1, true);
  ros::Publisher pub_lane = n.advertise<map_file::LaneArray>(
	  "/vector_map_info/lane", 1, true);

  ros::Publisher pub_road_edge = n.advertise<map_file::RoadEdgeArray>(
	  "/vector_map_info/road_edge", 1, true);
  ros::Publisher pub_gutter = n.advertise<map_file::GutterArray>(
	  "/vector_map_info/gutter", 1, true);
  ros::Publisher pub_curb = n.advertise<map_file::CurbArray>(
	  "/vector_map_info/curb", 1, true);
  ros::Publisher pub_white_line = n.advertise<map_file::WhiteLineArray>(
	  "/vector_map_info/white_line", 1, true);
  ros::Publisher pub_stop_line = n.advertise<map_file::StopLineArray>(
	  "/vector_map_info/stop_line", 1, true);
  ros::Publisher pub_zebra_zone = n.advertise<map_file::ZebraZoneArray>(
	  "/vector_map_info/zebra_zone", 1, true);
  ros::Publisher pub_cross_walk = n.advertise<map_file::CrossWalkArray>(
	  "/vector_map_info/cross_walk", 1, true);
  ros::Publisher pub_road_mark = n.advertise<map_file::RoadMarkArray>(
	  "/vector_map_info/road_mark", 1, true);
  ros::Publisher pub_pole = n.advertise<map_file::PoleArray>(
	  "/vector_map_info/pole", 1, true);
  ros::Publisher pub_road_sign = n.advertise<map_file::RoadSignArray>(
	  "/vector_map_info/road_sign", 1, true);
  ros::Publisher pub_signal = n.advertise<map_file::SignalArray>(
	  "/vector_map_info/signal", 1, true);
  ros::Publisher pub_street_light = n.advertise<map_file::StreetLightArray>(
	  "/vector_map_info/street_light", 1, true);
  ros::Publisher pub_utility_pole = n.advertise<map_file::UtilityPoleArray>(
	  "/vector_map_info/utility_pole", 1, true);
  ros::Publisher pub_guard_rail = n.advertise<map_file::GuardRailArray>(
	  "/vector_map_info/guard_rail", 1, true);
  ros::Publisher pub_side_walk = n.advertise<map_file::SideWalkArray>(
	  "/vector_map_info/side_walk", 1, true);
  ros::Publisher pub_cross_road = n.advertise<map_file::CrossRoadArray>(
	  "/vector_map_info/cross_road", 1, true);

  std::vector<PointClass> pointclasses;
  std::vector<PoleClass> poleclasses;
  std::vector<VectorClass> vectorclasses;
  std::vector<AreaClass> areaclasses;
  std::vector<LineClass> lines;

  std::vector<Pole> poles;
  std::vector<Signal> signals;
  std::vector<RoadSign> roadsigns;
  std::vector<DTLane> dtlanes;
  std::vector<Node> nodes;
  std::vector<Lane> lanes;
  std::vector<WhiteLine> whitelines;
  std::vector<ZebraZone> zebrazones;
  std::vector<CrossWalk> crosswalks;
  std::vector<RoadEdge> roadedges;
  std::vector<RoadMark> roadmarks;
  std::vector<StopLine> stoplines;
  std::vector<CrossRoad> crossroads;
  std::vector<SideWalk> sidewalks;
  std::vector<Gutter> gutters;
  std::vector<Curb> curbs;
  std::vector<StreetLight> streetlights;
  std::vector<UtilityPole> utilitypoles;

  std::cerr << "Load csv files" << std::endl;

  argc--;
  argv++;
  if(argc < 1){
    std::cerr << "Usage:\tvector_map_loader csv_file\n";
    std::cerr << "\tvector_map_loader download <x> <y>\n";
    std::exit(1);
  }
  std::string name(argv[0]);
  if(name == "download") {
    std::cerr << "vmap_loader download mode\n";
    std::string dirname;
    if(argc < 3){
      std::cerr << "Usage: vector_map_loader download <x> <y>\n";
      std::exit(1);
    } else {
      dirname = "/data/map/";
      int i = std::atoi(argv[1]);
      i -= i % 1000 + 1000;
      dirname += std::to_string(i);
      dirname += "/";
      i = std::atoi(argv[2]);
      i -= i % 1000 + 1000;
      dirname += std::to_string(i);
      dirname += "/vector";
    }

    if(argc >= 5){
      std::string host_name = argv[3];
      int port = std::atoi(argv[4]);
      gf = GetFile(host_name, port);
    } else {
      gf = GetFile();
    }

    struct stat st;
    std::string tmp_dir = "/tmp" + dirname;
    if(stat(tmp_dir.c_str(), &st) != 0) {
      std::istringstream ss(dirname);
      std::string column;
      std::getline(ss, column, '/');
      tmp_dir = "/tmp";
      while (std::getline(ss, column, '/')) {
	tmp_dir += "/" + column;
	errno = 0;
	int ret = mkdir(tmp_dir.c_str(), 0755);
	if(ret < 0 && errno != EEXIST) perror("mkdir");
      }
      std::cerr << "mkdir " << tmp_dir << std::endl;
    }

    for(auto x: vmap_csv_list) {
      std::cerr << "start get file = " <<dirname << "/" << x << " ... ";
      if(gf.GetHTTPFile(dirname+"/"+x) == 0) {
	vmap_file_list.push_back("/tmp"+dirname+"/"+x);
	std::cerr << " download done" << std::endl;
      } else {
	std::cerr << " download failed" << std::endl;
      }
    }
  } else {
    std::cerr << "vmap_loader local file mode\n";
  while(argc > 0) {
      std::cerr << "file=" << argv[0] << std::endl;
      vmap_file_list.push_back(argv[0]);
      argc--;
      argv++;
    }
  }

  for(auto x: vmap_file_list) {
    std::string name(basename((char *)x.c_str()));
    //    std::string name(basename(argv[0]));

    if(name == "point.csv") {
      pointclasses = read_pointclass(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", pointclasses.size()=" <<  pointclasses.size() << std::endl;
      pub_point_class.publish(pack_point_class_array(pointclasses));
    } else if(name == "pole.csv") {
      poleclasses = read_poleclass(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", poleclasses.size()=" <<  poleclasses.size() << std::endl;
      pub_pole_class.publish(pack_pole_class_array(poleclasses));
    } else if(name == "vector.csv") {
      vectorclasses = read_vectorclass(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", vectorclasses.size()" << vectorclasses.size() << std::endl;
      pub_vector_class.publish(pack_vector_class_array(vectorclasses));
    } else if(name == "area.csv") {
      areaclasses = read_areaclass(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", areaclasses.size()=" << areaclasses.size() << std::endl;
      pub_area_class.publish(pack_area_class_array(areaclasses));
    } else if(name == "line.csv") {
      lines = read_lineclass(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", lines.size()=" << lines.size() << std::endl;
      pub_line_class.publish(pack_line_class_array(lines));
    } else if(name == "poledata.csv") {
      poles = read_pole(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", poles.size()=" << poles.size() << std::endl;
      pub_pole.publish(pack_pole_array(poles));
    } else if(name == "signaldata.csv") {
      signals = read_signal(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", signals.size()=" << signals.size() << std::endl;
      pub_signal.publish(pack_signal_array(signals));
    } else if(name == "roadsign.csv") {
      roadsigns = read_roadsign(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", roadsigns.size()=" << roadsigns.size() << std::endl;
      pub_road_sign.publish(pack_roadsign_array(roadsigns));
    } else if(name == "dtlane.csv") {
      dtlanes = read_dtlane(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", dtlanes.size()=" << dtlanes.size() << std::endl;
      pub_dtlane.publish(pack_dtlane_array(dtlanes));
    } else if(name == "node.csv") {
      nodes = read_node(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", nodes.size()=" << nodes.size() << std::endl;
      pub_node.publish(pack_node_array(nodes));
    } else if(name == "lane.csv") {
      lanes = read_lane(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", lanes.size()=" << lanes.size() << std::endl;
      pub_lane.publish(pack_lane_array(lanes));
    } else if(name == "whiteline.csv") {
      whitelines = read_whiteline(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", whitelines.size()=" << whitelines.size() << std::endl;
      pub_white_line.publish(pack_whiteline_array(whitelines));
    } else if(name == "zebrazone.csv") {
      zebrazones = read_zebrazone(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", zebrazones.size()=" << zebrazones.size() << std::endl;
      pub_zebra_zone.publish(pack_zebrazone_array(zebrazones));
    } else if(name == "crosswalk.csv") {
      crosswalks = read_crosswalk(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", crosswalks.size()=" << crosswalks.size() << std::endl;
      pub_cross_walk.publish(pack_crosswalk_array(crosswalks));
    } else if(name == "roadedge.csv") {
      roadedges = read_roadedge(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", roadedges.size()=" << roadedges.size() << std::endl;
      pub_road_edge.publish(pack_roadedge_array(roadedges));
    } else if(name == "road_surface_mark.csv") {
      roadmarks = read_roadmark(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", roadmarks.size()=" << roadmarks.size() << std::endl;
      pub_road_mark.publish(pack_roadmark_array(roadmarks));
    } else if(name == "stopline.csv") {
      stoplines = read_stopline(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", stoplines.size()=" << stoplines.size() << std::endl;
      pub_stop_line.publish(pack_stopline_array(stoplines));
    } else if(name == "crossroads.csv") {
      crossroads = read_crossroad(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", crossroads.size()=" << crossroads.size() << std::endl;
      pub_cross_road.publish(pack_crossroad_array(crossroads));
    } else if(name == "sidewalk.csv") {
      sidewalks = read_sidewalk(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", sidewalks.size()=" << sidewalks.size() << std::endl;
      pub_side_walk.publish(pack_sidewalk_array(sidewalks));
    } else if(name == "gutter.csv") {
      gutters = read_gutter(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", gutters.size()=" << gutters.size() << std::endl;
      pub_gutter.publish(pack_gutter_array(gutters));
    } else if(name == "curb.csv") {
      curbs = read_curb(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", curb.size()=" << curbs.size() << std::endl;
      pub_curb.publish(pack_curb_array(curbs));
    } else if(name == "streetlight.csv") {
      streetlights = read_streetlight(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", streetlights.size()=" << streetlights.size() << std::endl;
      pub_street_light.publish(pack_streetlight_array(streetlights));
    } else if(name == "utilitypole.csv") {
      utilitypoles = read_utilitypole(x.c_str());
      std::cerr << "  load " << x.c_str()
		<< ", utilitypoles.size()=" << utilitypoles.size() << std::endl;
      pub_utility_pole.publish(pack_utilitypole_array(utilitypoles));
    }

    argc--;
    argv++;
  }

  std::cerr << "finished load files" << std::endl;

  if(pointclasses.size() <= 0) {
    std::cerr << "Usage: vector_map_loader csv_file\n"
	      << "\tpoint.csv is not loaded"
              << std::endl;
    std::exit(1);
  }


  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "vector_map";
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.frame_locked = true;
  marker.type = visualization_msgs::Marker::CYLINDER;

  size_t i;

  std::cerr << "start publish vector map" << std::endl;

  marker.id = 0;

  // road data
  if (publish_lane) {
    if(lanes.size() > 0 && dtlanes.size() <= 0) {
      std::cerr << "error: dtlane.csv is not loaded.\n"
		<< "\tlane.csv needs dtlane.csv"
		<< std::endl;
      std::exit(1);
    }
    marker.ns = "lane";
    for (i=0; i<lanes.size(); i++) {
      if (lanes[i].lnid <= 0) {
	continue;
      }
      int did = lanes[i].did;
      int pid = dtlanes[did].pid;
      double ox, oy, oz, ow;

      marker.type = visualization_msgs::Marker::CUBE;

      ox = 0.0;
      oy = 0.0;
      oz = sin(dtlanes[did].dir / 2);
      ow = cos(dtlanes[did].dir / 2);

      if((lanes[i].span == 0 || (dtlanes[did].lw + dtlanes[did].rw) == 0)) continue;
      set_marker_data(&marker,
		      pointclasses[pid].bx - lanes[i].span/2, pointclasses[pid].ly - (dtlanes[did].lw + dtlanes[did].rw)/2, pointclasses[pid].h,
		      ox, oy, oz, ow,
		      lanes[i].span, dtlanes[did].lw + dtlanes[did].rw, 0.1,
		      0.5, 0, 0, 0.3);

      push_marker(&marker, &marker_array);
    }
  }

  // pole
  if(poles.size() > 0 && (poleclasses.size() <= 0 || vectorclasses.size() <= 0)) {
    std::cerr << "error: pole.csv or vector.csv is not loaded \n"
	      << "\tpoledata.csv needs pole.csv and vector.csv "
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "pole";
  for (i=0; i<poles.size(); i++) {
    if (poles[i].id <= 0) {
      continue;
    }
    int plid = poles[i].plid;
    set_poleclass_data(poleclasses[plid], 
		       1, 1, 1, 1,
		       vectorclasses,
		       pointclasses,
		       &marker, &marker_array);
  }

  // signal
  if(signals.size() > 0 && (poleclasses.size() <= 0 || vectorclasses.size() <= 0)) {
    std::cerr << "error: pole.csv or vector.csv is not loaded.\n"
	      << "\tsignaldata.csv needs pole.csv and vector.csv "
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "signal";
  for (i=0; i<signals.size(); i++) {
    if (signals[i].id <= 0) {
      continue;
    }
    int vid = signals[i].vid;
    int pid = vectorclasses[vid].pid;

    marker.type = visualization_msgs::Marker::SPHERE;

    double ox, oy, oz, ow;
    calc_ang_to_xyzw(vectorclasses[vid].vang, vectorclasses[vid].hang,
		     &ox, &oy, &oz, &ow);

    double r = 0, g = 0, b = 0, a = 1;
    switch (signals[i].type) {
    case 1:
      r = 1;
      break;
    case 2:
      b = 1;
      break;
    case 3:
      r = 1;
      g = 1;
      break;
    case 4:
      marker.type = visualization_msgs::Marker::CUBE;
      r = 1;
      break;
    case 5:
      marker.type = visualization_msgs::Marker::CUBE;
      b = 1;
      break;
    default:
      break;
    }

    set_marker_data(&marker,
		    pointclasses[pid].bx, 
		    pointclasses[pid].ly, 
		    pointclasses[pid].h,
		    ox, oy, oz, ow,
		    0.5, 0.5, 0.5,
		    r, g, b, a);
    push_marker(&marker, &marker_array);


    // signal pole
    if (signals[i].type == 2) { // blue
      int plid = signals[i].plid;

      if (plid > 0) {
	set_poleclass_data(poleclasses[plid],
			   0.5, 0.5, 0.5, 1,
			   vectorclasses,
			   pointclasses,
			   &marker, &marker_array);
      }
    }
  }

  // roadsigns
  if(roadsigns.size() > 0 && (poleclasses.size() <= 0 || vectorclasses.size() <= 0)) {
    std::cerr << "error: pole.csv or vector.csv is not loaded\n"
	      << "\troadsign.csv needs pole.csv and vector.csv "
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "road sign";
  for (i=0; i<roadsigns.size(); i++) {
    if (roadsigns[i].id <= 0) {
      continue;
    }
    int vid = roadsigns[i].vid;
    int pid = vectorclasses[vid].pid;

    marker.type = visualization_msgs::Marker::SPHERE;

    double ox, oy, oz, ow;
    calc_ang_to_xyzw(vectorclasses[vid].vang, vectorclasses[vid].hang,
		     &ox, &oy, &oz, &ow);

    set_marker_data(&marker,
		    pointclasses[pid].bx, 
		    pointclasses[pid].ly, 
		    pointclasses[pid].h,
		    ox, oy, oz, ow,
		    1.0, 0.1, 1.0,
		    1, 1, 1, 1);
    push_marker(&marker, &marker_array);

    // road sign pole
    int plid = roadsigns[i].plid;
    if(plid > 0) {
      set_poleclass_data(poleclasses[plid],
			 1, 1, 1, 1,
			 vectorclasses,
			 pointclasses,
			 &marker, &marker_array);
    }
  }

  // cross walk
  if(crosswalks.size() > 0 && (areaclasses.size() <= 0 || lines.size() <= 0)) {
    std::cerr << "error: area.csv or line.csv is not loaded.\n"
	      << "\tcrosswalk.csv needs area.csv and line.csv"
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "cross walk";
  for (i=0; i<crosswalks.size(); i++) {
    if (crosswalks[i].id <= 0) {
      continue;
    }
    int aid = crosswalks[i].aid;
    int slid = areaclasses[aid].slid;
    int elid = areaclasses[aid].elid;

    marker.type = visualization_msgs::Marker::LINE_STRIP;

    set_marker_data(&marker,
		    0, 0, 0,
		    0, 0, 0, 1,
		    0.1, 0, 0,
		    1, 1, 1, 1);
    
    marker.points.clear();
    for(int lid = slid; ; lid = lines[lid].flid) {
      int pid = lines[lid].bpid;
      add_marker_points_pointclass(&marker, pointclasses[pid]);
      if (lid == elid) {
	pid = lines[lid].fpid;
	add_marker_points_pointclass(&marker, pointclasses[pid]);
	break;
      }
    }

    push_marker(&marker, &marker_array);
  }

  // zebrazone
  if(zebrazones.size() > 0 && (areaclasses.size() <= 0 || lines.size() <= 0)) {
    std::cerr << "error: area.csv or line.csv is not loaded.\n"
	      << "\tzebrazone.csv needs area.csv and line.csv."
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "zebrazone";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  set_areaclass_data(zebrazones, 0.1,
		     1, 1, 1, 1,
		     areaclasses,
		     lines,
		     pointclasses,
		     &marker, &marker_array);


  // white line
  if(whitelines.size() > 0 && lines.size() <= 0) {
    std::cerr << "err: line.csv is not loaded.\n"
	      << "\twhiteline.csv needs line.csv"
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "white line";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  for (i=0; i<whitelines.size(); i++) {
    if (whitelines[i].id <= 0) {
      continue;
    }
    int lid = whitelines[i].lid;
    if(lines[lid].blid == 0) {
      double r = 1.0, g = 1.0, b = 1.0;
      if(whitelines[i].color == 'Y') {
	g = 0.5;
	b = 0.0;
      }
      set_marker_data(&marker,
		      0, 0, 0,
		      0, 0, 0, 1,
		      whitelines[i].width, 0, 0,
		      r, g, b, 1);
    
      marker.points.clear();
    }

    int pid = lines[lid].bpid;
    add_marker_points_pointclass(&marker, pointclasses[pid]);
    if(lines[lid].flid == 0) {
      pid = lines[lid].fpid;
      add_marker_points_pointclass(&marker, pointclasses[pid]);

      push_marker(&marker, &marker_array);
    }
  }


  // roadedge
  if(roadedges.size() > 0 && (lines.size() <= 0)) {
    std::cerr << "error: line.csv is not loaded.\n"
	      << "\troadedge.csv needs line.csv"
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "road edge";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  for (i=0; i<roadedges.size(); i++) {
    if (roadedges[i].id <= 0) continue;
    int lid = roadedges[i].lid;
    if(lines[lid].blid == 0) {
      set_marker_data(&marker,
		      0, 0, 0,
		      0, 0, 0, 1,
		      0.1, 0, 0,
		      0.5, 0.5, 0.5, 1);		// grey @@@@
    
      marker.points.clear();
    }

    int pid = lines[lid].bpid;
    add_marker_points_pointclass(&marker, pointclasses[pid]);
    if(lines[lid].flid == 0) {
      pid = lines[lid].fpid;
      add_marker_points_pointclass(&marker, pointclasses[pid]);

      push_marker(&marker, &marker_array);
    }
  }


  // road_surface_mark
  if(roadmarks.size() > 0 && (areaclasses.size() <= 0 || lines.size() <= 0)) {
    std::cerr << "error: area.csv or line.csv is not loaded.\n"
	      << "\troad_surface_mark.csv needs area.csv and line.csv"
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "road surface mark";
  for (i=0; i<roadmarks.size(); i++) {
    if (roadmarks[i].id <= 0) {
      continue;
    }
    int aid = roadmarks[i].aid;
    int slid = areaclasses[aid].slid;
    int elid = areaclasses[aid].elid;
    double w = 0.2;

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    set_marker_data(&marker,
		    0, 0, 0,
		    0, 0, 0, 1,
		    w, 0, 0,
		    1, 1, 1, 1);
    marker.points.clear();
    for(int lid = slid; ; lid = lines[lid].flid) {
      int pid = lines[lid].bpid;
      add_marker_points_pointclass(&marker, pointclasses[pid]);
      if (lid == elid) {
	pid = lines[lid].fpid;
	add_marker_points_pointclass(&marker, pointclasses[pid]);
	break;
      }
    }

    push_marker(&marker, &marker_array);
  }

  // stop line
  if(stoplines.size() > 0 && lines.size() <= 0) {
    std::cerr << "error: line.csv is not loaded.\n"
	      << "\tstoplines.csv needs line.csv"
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "stop line";
  for (i=0; i<stoplines.size(); i++) {
    if (stoplines[i].id <= 0) {
      continue;
    }
    int lid = stoplines[i].lid;
    double r = 1.0, g = 1.0, b = 1.0;
    if(lines[lid].blid == 0) {
      set_marker_data(&marker,
		      0, 0, 0,
		      0, 0, 0, 1,
		      0.4, 0, 0,
		      r, g, b, 1);
    
      marker.points.clear();
    }
    int pid = lines[lid].bpid;
    add_marker_points_pointclass(&marker, pointclasses[pid]);
    if(lines[lid].flid == 0) {
      pid = lines[lid].fpid;
      add_marker_points_pointclass(&marker, pointclasses[pid]);

      push_marker(&marker, &marker_array);
    }
  }

  // cross road
  if(crossroads.size() > 0 && (areaclasses.size() <= 0 || lines.size() <= 0)) {
    std::cerr << "error: area.csv or line.csv is not loaded.\n"
	      << "\tcrossroad.csv needs area.csv and line.csv."
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "cross road";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  set_areaclass_data(crossroads, 0.1,
		     1, 1, 1, 1,
		     areaclasses,
		     lines,
		     pointclasses,
		     &marker, &marker_array);


  // side walk (hodou) 
  if(sidewalks.size() > 0 && (areaclasses.size() <= 0 || lines.size() <= 0)) {
    std::cerr << "error: area.csv or line.csv is not loaded.\n"
	      << "\tsidewalk.csv needs area.csv and line.csv."
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "side walk";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  set_areaclass_data(sidewalks, 0.1,
		     1, 1, 1, 1,
		     areaclasses,
		     lines,
		     pointclasses,
		     &marker, &marker_array);

  // gutter
  if(gutters.size() > 0 && (areaclasses.size() <= 0 || lines.size() <= 0)) {
    std::cerr << "error: area.csv or line.csv is not loaded.\n"
	      << "\tgutter.csv needs area.csv and line.csv."
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "gutter";
  marker.type = visualization_msgs::Marker::LINE_STRIP;

  for (i=0; i<gutters.size(); i++) {
    if (gutters[i].id <= 0) {
      continue;
    }
    int aid = gutters[i].aid;
    int slid = areaclasses[aid].slid;
    int elid = areaclasses[aid].elid;
    double r, g, b;

    switch(gutters[i].type) {
    case 0:
      r = 0.7, g = 0.7, b = 0.7;
      break;
    case 1:
      r = 0.8, g = 0.8, b = 0.8;
      break;
    case 2:
      r = 0.5, g = 0.5, b = 0.5;
      break;
    default:
      r = 1.0, g = 1.0, b = 1.0;
      break;
    }

    set_marker_data(&marker,
		    0, 0, 0,
		    0, 0, 0, 1,
		    0.2, 0, 0,
		    r, g, b, 1);
    marker.points.clear();
    for(int lid = slid; ; lid = lines[lid].flid) {
      int pid = lines[lid].bpid;
      add_marker_points_pointclass(&marker, pointclasses[pid]);
      if (lid == elid) {
	pid = lines[lid].fpid;
	add_marker_points_pointclass(&marker, pointclasses[pid]);
	break;
      }
    }

    push_marker(&marker, &marker_array);
  }

  // curb
  if(curbs.size() > 0 && lines.size() <= 0) {
    std::cerr << "error: line.csv is not loaded.\n"
	      << "\tcurb.csv needs line.csv."
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "curb";
  marker.type = visualization_msgs::Marker::LINE_STRIP;

  for (i=0; i<curbs.size(); i++) {
    if (curbs[i].id <= 0) {
      continue;
    }

    int lid = curbs[i].lid;
    if(lines[lid].blid == 0) {
      set_marker_data(&marker,
		      0, 0, 0,
		      0, 0, 0, 1,
		      curbs[i].width, 0, curbs[i].height,
		      0.7, 0.7, 0.7, 1);
    
      marker.points.clear();
    }

    int pid = lines[lid].bpid;
    add_marker_points_pointclass(&marker, pointclasses[pid]);
    if(lines[lid].flid == 0) {
      pid = lines[lid].fpid;
      add_marker_points_pointclass(&marker, pointclasses[pid]);

      push_marker(&marker, &marker_array);
    }
  }


  // streetlight
  if(streetlights.size() > 0 && (lines.size() <= 0 || poleclasses.size() <= 0 || vectorclasses.size() <= 0)) {
    std::cerr << "error: line.csv or pole.csv or vector.csv is not loaded.\n"
	      << "\tcurb.csv needs line.csv, pole.csv and vector.csv."
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "streetlight";

  for (i=0; i<streetlights.size(); i++) {
    if (streetlights[i].id <= 0) {
      continue;
    }

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    int lid = streetlights[i].lid;
    if(lines[lid].blid == 0) {
      set_marker_data(&marker,
		      0, 0, 0,
		      0, 0, 0, 1,
		      0.2, 0, 0,
		      1, 1, 1, 1);
      marker.points.clear();
    }

    int pid = lines[lid].bpid;
    add_marker_points_pointclass(&marker, pointclasses[pid]);
    if(lines[lid].flid == 0) {
      pid = lines[lid].fpid;
      add_marker_points_pointclass(&marker, pointclasses[pid]);

      push_marker(&marker, &marker_array);
    }

    // streetlight pole
    int plid = streetlights[i].plid;
    if(plid <= 0) continue;
    set_poleclass_data(poleclasses[plid],
		       1, 1, 1, 1,
		       vectorclasses,
		       pointclasses,
		       &marker, &marker_array);
  }

  // utilitypole
  if(utilitypoles.size() > 0 && (poleclasses.size() <= 0 || vectorclasses.size() <= 0)) {
    std::cerr << "error: pole.csv or vector.csv is not loaded \n"
	      << "\tutilitypole.csv needs pole.csv and vector.csv "
	      << std::endl;
    std::exit(1);
  }
  marker.ns = "utilitypole";
  for (i=0; i<utilitypoles.size(); i++) {
  if (utilitypoles[i].id <= 0) continue;
  int plid = utilitypoles[i].plid;
  set_poleclass_data(poleclasses[plid], 
			0.5, 0.5, 0.5, 1,
			vectorclasses,
			pointclasses,
			&marker, &marker_array);
  }

  pub.publish(marker_array);

  vmap_stat_msg.data = true;
  stat_publisher.publish(vmap_stat_msg);

#ifdef DEBUG_PRINT
  std::cerr << "publish end" << std::endl;
#endif

  ros::spin();

  return 0;
}
