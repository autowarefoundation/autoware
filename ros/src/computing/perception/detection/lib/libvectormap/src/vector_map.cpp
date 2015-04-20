//#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "vector_map.h"
#include <vector>
#include <map>
#include <tf/transform_listener.h>


using std::string;


int VectorMap::load_points(char *name){
  FILE *fp;
  char dummy[1000];
  Point p;

  p.pid=0;
  fp=fopen(name,"r");//point
  if(!fp)return 0;
  fscanf(fp,"%s",dummy);
  while(fscanf(fp,"%d,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d",&p.pid,&p.b,&p.l,&p.h,&p.ly,&p.bx,&p.ref,&p.mcode1,&p.mcode2,&p.mcode3)!=EOF){
    points[p.pid]= p;
  }
  fclose(fp);
  return 1;
}

int VectorMap::load_lines(char *name){
  FILE *fp;
  char dummy[1000];
  Line l;

  fp=fopen(name,"r");
  if(!fp)return 0;
  printf("%s\n",name);
  fscanf(fp,"%s",dummy);
  while(fscanf(fp,"%d,%d,%d,%d,%d",&l.lid,&l.bpid,&l.fpid,&l.blid,&l.flid)!=EOF){
    lines[l.lid]=l;
  }
 
  fclose(fp);
  return 1;
}

int VectorMap::load_lanes(char *name){
  FILE *fp;
  char dummy[1000];
  Lane l;

  fp=fopen(name,"r");
  if(!fp)return 0;
  printf("%s\n",name);
  fscanf(fp,"%s",dummy);
  printf("%s\n",dummy);
  while(fscanf(fp,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lf ,%d,%d",
	       &l.lnid, &l.did,  &l.blid, &l.flid,   &l.bnid,
	       &l.fnid, &l.jct,  &l.blid2,&l.blid3,  &l.blid4,
	       &l.flid2,&l.flid3,&l.flid4,&l.clossid,&l.span,
	       &l.lcnt, &l.lno)!=EOF){
    lanes[l.lnid]=l;
    //printf("%d %d\n",l.lnid,l.lno);
  }
  printf("fin\n");
  fclose(fp);
  return 1;
}

int VectorMap::load_vectors(char *name){
  FILE *fp;
  char dummy[1000];
  Vector v;

  
  //line
  fp=fopen(name,"r");
  if(!fp)return 0;
  printf("%s\n",name);
  fscanf(fp,"%s",dummy);
  while(fscanf(fp,"%d,%d,%lf,%lf",&v.vid,&v.pid,&v.hang,&v.vang)!=EOF){
    vectors[v.vid]=v;
  }
 
  fclose(fp);
  return 1;
}

int VectorMap::load_signals(char *name){
  FILE *fp;
  char dummy[1000];
  Signal s;

  //  signals.resize(40000);
  
  //line
  fp=fopen(name,"r");

  if(!fp)return 0;
  printf("%s\n",name);
  fscanf(fp,"%s",dummy);
  while(fscanf(fp,"%d,%d,%d,%d,%d",&s.id,&s.vid,&s.plid,&s.type,&s.linkid)!=EOF){
    signals[s.id]=s;
  }
 
  fclose(fp);
  return 1;
}


int VectorMap::load_clines(char *name){
  FILE *fp;
  char dummy[1000];
  CLine l;

  //clines.resize(40000);
  
  //line
  fp=fopen(name,"r");
  if(!fp)return 0;
  fscanf(fp,"%s",dummy);
  while(fscanf(fp,"%d,%d,%f,%c,%d,%d",
	       &l.id, &l.lid, &l.width, &l.color, &l.type, &l.linkid)!=EOF){
    clines[l.id]=l;
  }
  printf("%s\n",name); 
  fclose(fp);
  return 1;
}

int VectorMap::load_dtlanes(char *name ){
  FILE *fp;
  char dummy[1000];
  DTLane a;

  //dtlanes.resize(40000);
  //dtlane
  fp=fopen(name,"r");
  if(!fp)return 0;
  fscanf(fp,"%s",dummy);
  while(fscanf(fp,"%d,%lf,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
	       &a.did, &a.dist, &a.pid, &a.dir, &a.apara,
	       &a.r, &a.slope, &a.cant, &a.lw, &a.rw)!=EOF){
    dtlanes[a.did]=a;
  }
  fclose(fp);
  return 1;
}


void VectorMap::loadAll (const std::string &dirname)
{
	if (loaded)
		return;
	string ptname = dirname + "/point.csv";
	load_points((char*)ptname.c_str());
	string linename = dirname + "/line.csv";
	load_lines ((char*)linename.c_str());
	string dtlanename = dirname + "/dtlane.csv";
	load_dtlanes ((char*)dtlanename.c_str());
	string clinename = dirname + "/cline.csv";
	load_clines ((char*)clinename.c_str());
	string vectorname = dirname + "/vector.csv";
	load_vectors ((char*)vectorname.c_str());
	string signalname = dirname + "/signal.csv";
	load_signals ((char*)signalname.c_str());
	string lanename = dirname + "/lane.csv";
	load_lanes ((char*)lanename.c_str());
	loaded = true;
}


