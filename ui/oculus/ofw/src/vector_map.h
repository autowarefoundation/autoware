//
//  vector_map.h
//  ORR_PointCloud
//
//  Created by Kenjiro on 6/6/15.
//
//

#ifndef __ORR_PointCloud__vector_map__
#define __ORR_PointCloud__vector_map__

#include "ofMain.h"
#include <stdio.h>
#include <libgen.h>
#include <fstream>
#include <iostream>

#define FCLIP 40

/* for roadedge.csv */
struct RoadEdge {
    int id;
    int lid;
    int linkid;
};
/* for gutter.csv, guardrail.csv */
struct Gutter {
    int id;
    int aid;
    int type;
    int linkid;
};
/* for curb.csv */
struct Curb {
    int id;
    int lid;
    double height;
    double width;
    int dir;
    int linkid;
};
/* for whiteline.csv */
struct WhiteLine {
    int id;
    int lid;
    double width;
    char color;
    int type;
    int linkid;
};
/* for stopline.csv */
struct StopLine {
    int id;
    int lid;
    int tlid;
    int signid;
    int linkid;
};
/* for zebrazone.csv, sidewalk.csv, crossroads.csv */
struct ZebraZone {
    int id;
    int aid;
    int linkid;
};
/* for crosswalk.csv */
struct CrossWalk {
    int id;
    int aid;
    int type;
    int bdid;
    int linkid;
};
/* for road_surface_mark.csv */
struct RoadSurfaceMark {
    int id;
    int aid;
    std::string type;
    int linkid;
};
/* for poledata.csv, utilitypole.csv */
struct PoleData {
    int id;
    int plid;
    int linkid;
};
/* for signaldata.csv, roadsign.csv */
struct SignalData {
    int id;
    int vid;
    int plid;
    int type;
    int linkid;
};
/* for streetlight.csv */
struct StreetLight {
    int id;
    int lid;
    int plid;
    int linkid;
};
/* basic class */
/* for point.csv */
struct PointClass {
    int pid;
    double b;
    double l;
    double h;
    double bx;
    double ly;
    double ref;
    int mcode1;
    int mcode2;
    int mcode3;
};
/* for vector.csv */
struct VectorClass {
    int vid;
    int pid;
    double hang;
    double vang;
};
/* for line.csv */
struct LineClass {
    int lid;
    int bpid;
    int fpid;
    int blid;
    int flid;
};
/* for area.csv */
struct AreaClass {
    int aid;
    int slid;
    int elid;
};
/* for pole.csv */
struct PoleClass {
    int plid;
    int vid;
    double length;
    double dim;
};
/* for box.csv */
struct BoxClass {
    int bid;
    int pid1;
    int pid2;
    int pid3;
    int pid4;
    double height;
};
/* Road data */
/* for dtlane.csv */
struct DTLane {
    int did;
    double dist;
    int pid;
    double dir;
    int apara;
    double r;
    double slope;
    double cant;
    double lw;
    double rw;
};
///* for node.csv */
struct NodeData {
    int nid;
    int pid;
};
/* for lane.csv */
struct LaneData {
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
};

class VectorMap
{
public:
    std::map<int, RoadEdge>         roadedges;
    std::map<int, Gutter>           gutters;
    std::map<int, Curb>             curbs;
    std::map<int, WhiteLine>        whitelines;
    std::map<int, StopLine>         stoplines;
    std::map<int, ZebraZone>        zebrazones;
    std::map<int, CrossWalk>        crosswalks;
    std::map<int, RoadSurfaceMark>  roadsurfacemarks;
    std::map<int, PoleData>         poledatas;
    std::map<int, SignalData>       roadsigns;
    std::map<int, SignalData>       signaldatas;
    std::map<int, StreetLight>      streetlights;
    std::map<int, PoleData>         utilitypoles;
    std::map<int, PointClass>       points;
    std::map<int, VectorClass>      vectors;
    std::map<int, LineClass>        lines;
    std::map<int, AreaClass>        areas;
    std::map<int, PoleClass>        poles;
    std::map<int, BoxClass>         boxes;
    std::map<int, DTLane>           dtlanes;
    std::map<int, NodeData>         nodedatas;
    std::map<int, LaneData>         lanedatas;
    int read_roadedge(const char* filename);
    int read_gutter(const char* filename);
    int read_curb(const char* filename);
    int read_whiteline(const char* filename);
    int read_stopline(const char* filename);
    int read_zebrazone(const char* filename);
    int read_crosswalk(const char* filename);
    int read_roadsurfacemark(const char* filename);
    int read_poledata(const char* filename);
    int read_roadsign(const char *filename);
    int read_signaldata(const char *filename);
    int read_streetlight(const char *filename);
    int read_utilitypole(const char* filename);
    int read_pointclass(const char *filename);
    int read_vectorclass(const char *filename);
    int read_lineclass(const char *filename);
    int read_areaclass(const char *filename);
    int read_poleclass(const char *filename);
    int read_boxclass(const char *filename);
    int read_dtlane(const char *filename);
    int read_nodedata(const char *filename);
    int read_lanedata(const char *filename);

    void loadAll (const std::string &dirname);

    float min_x, max_x, min_y, max_y;

    void setTF(float mi_x, float ma_x, float mi_y, float ma_y){
        min_x = mi_x;
        max_x = ma_x;
        min_y = mi_y;
        max_y = ma_y;
    }

    ofPoint get_point(int pid);

    int set_roadedge();
    int set_gutter();
    int set_curb();
    int set_whiteline();
    int set_stopline();
    int set_zebrazone();
    int set_crosswalk();
    int set_roadsurfacemark();
    int set_poledata();
    int set_roadsign();
    int set_signaldata();
    int set_streetlight();
    int set_utilitypole();
//    int draw_pointclass();
//    int draw_vectorclass();
    ofVboMesh set_lineclass(ofPrimitiveMode mode, ofColor color, float width,  int lineid);
    ofVboMesh set_areaclass(ofPrimitiveMode mode, ofColor color, int aid);
    ofVboMesh set_poleclass(ofColor color, int plid);
//    int draw_boxclass();
//    int draw_dtlane();
//    int draw_nodedata();
//    int draw_lanedata();

    void set_all();
    void draw ();
    void drawWireframe ();

    vector<ofVboMesh> re_mesh;
    vector<ofVboMesh> gutter_mesh;
    vector<ofVboMesh> curb_mesh;
    vector<ofVboMesh> wline_mesh;
    vector<ofVboMesh> sline_mesh;
    vector<ofVboMesh> zz_mesh;
    vector<ofVboMesh> cw_mesh;
    vector<ofVboMesh> rsm_mesh;
    vector<ofVboMesh> pd_mesh;
    vector<ofVboMesh> rs_mesh;
    vector<ofVboMesh> sd_mesh;
    vector<ofVboMesh> sl_line_mesh;
    vector<ofVboMesh> sl_pole_mesh;
    vector<ofVboMesh> up_mesh;
};

#endif /* defined(__ORR_PointCloud__vector_map__) */
