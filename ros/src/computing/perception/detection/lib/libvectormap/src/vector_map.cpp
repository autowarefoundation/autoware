//#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "vector_map.h"
#include <vector>
#include <map>
#include <tf/transform_listener.h>
#include <string>
#include <fstream>

using std::string;



int VectorMap::load_points(char *name){
  FILE *fp;
  char dummy[1000];
  Point p;

  p.pid=0;
  if ( (fp=fopen(name,"r")) == NULL )//point
    {
      fprintf(stderr, "open failed: %s\n", name);
      return EXIT_FAILURE;
    }

  fscanf(fp,"%s",dummy);
  while(fscanf(fp,"%d,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d",&p.pid,&p.b,&p.l,&p.h,&p.ly,&p.bx,&p.ref,&p.mcode1,&p.mcode2,&p.mcode3)!=EOF){
    points[p.pid]= p;
  }
  fclose(fp);
  return EXIT_SUCCESS;
}


int VectorMap::load_lines(char *name){
  FILE *fp;
  char dummy[1000];
  Line l;

  if ( (fp=fopen(name,"r")) == NULL )
    {
      fprintf(stderr, "open failed: %s\n", name);
      return EXIT_FAILURE;
    }

  //  printf("%s\n",name);
  fscanf(fp,"%s",dummy);
  while(fscanf(fp,"%d,%d,%d,%d,%d",&l.lid,&l.bpid,&l.fpid,&l.blid,&l.flid)!=EOF){
    lines[l.lid]=l;
  }

  fclose(fp);
  return EXIT_SUCCESS;
}


int VectorMap::load_lanes(char *name){
  FILE *fp;
  char dummy[1000];
  Lane l;

  if ( (fp=fopen(name,"r")) == NULL )
    {
      fprintf(stderr, "open failed: %s\n", name);
      return EXIT_FAILURE;
    }

  //  printf("%s\n",name);
  fscanf(fp,"%s",dummy);
  //  printf("%s\n",dummy);
  while(fscanf(fp,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lf ,%d,%d",
	       &l.lnid, &l.did,  &l.blid, &l.flid,   &l.bnid,
	       &l.fnid, &l.jct,  &l.blid2,&l.blid3,  &l.blid4,
	       &l.flid2,&l.flid3,&l.flid4,&l.clossid,&l.span,
	       &l.lcnt, &l.lno)!=EOF){
    lanes[l.lnid]=l;
    //printf("%d %d\n",l.lnid,l.lno);
  }
  //  printf("fin\n");
  fclose(fp);
  return EXIT_SUCCESS;
}


int VectorMap::load_vectors(char *name){
  FILE *fp;
  char dummy[1000];
  Vector v;

  //line
  if ( (fp=fopen(name,"r")) == NULL )
    {
      fprintf(stderr, "open failed: %s\n", name);
      return EXIT_FAILURE;
    }

  printf("%s\n",name);
  fscanf(fp,"%s",dummy);
  while(fscanf(fp,"%d,%d,%lf,%lf",&v.vid,&v.pid,&v.hang,&v.vang)!=EOF){
    vectors[v.vid]=v;
  }

  fclose(fp);
  return EXIT_SUCCESS;
}


int VectorMap::load_signals(char *name){
  FILE *fp;
  char dummy[1000];
  Signal s;

  //  signals.resize(40000);

  //line
  if ( (fp=fopen(name,"r")) == NULL )
    {
      fprintf(stderr, "open failed: %s\n", name);
      return EXIT_FAILURE;
    }

  //  printf("%s\n",name);
  fscanf(fp,"%s",dummy);
  while(fscanf(fp,"%d,%d,%d,%d,%d",&s.id,&s.vid,&s.plid,&s.type,&s.linkid)!=EOF){
    signals[s.id]=s;
  }

  fclose(fp);
  return EXIT_SUCCESS;
}


int VectorMap::load_clines(char *name){
#if 0
  FILE *fp;
  char dummy[1000];
  CLine l;

  //clines.resize(40000);

  //line
  if ( (fp=fopen(name,"r")) == NULL )
    {
      fprintf(stderr, "open failed: %s\n", name);
      return EXIT_FAILURE;
    }

  fscanf(fp,"%s",dummy);
  while(fscanf(fp,"%d,%d,%f,%c,%d,%d",
	       &l.id, &l.lid, &l.width, &l.color, &l.type, &l.linkid)!=EOF){
    clines[l.id]=l;
  }

  //  printf("%s\n",name);
  fclose(fp);
  return EXIT_SUCCESS;
#else
  CLine tmp_l;
  std::ifstream ifs(name);
  if (ifs == 0)
    {
      fprintf(stderr, "open failed: %s\n", name);
      return EXIT_FAILURE;
    }

  std::string line_contents;
  std::getline(ifs, line_contents);  // skip first header line

  while ( getline(ifs, line_contents) )
    {
      std::istringstream ss(line_contents);
      std::vector<std::string> columns;
      std::string element;
      while(std::getline(ss, element, ',')) {
        columns.push_back(element);
      }
      tmp_l.id     = atoi((const char*)(columns[0].c_str()));
      tmp_l.lid    = atoi((const char*)(columns[1].c_str()));
      tmp_l.width  = atof((const char*)(columns[2].c_str()));
      tmp_l.color  = columns[3].c_str()[0];
      tmp_l.type   = atoi((const char*)(columns[4].c_str()));
      tmp_l.linkid = atoi((const char*)(columns[5].c_str()));

      clines[tmp_l.id] = tmp_l;
    }

#endif

}


int VectorMap::load_dtlanes(char *name ){
  FILE *fp;
  char dummy[1000];
  DTLane a;

  //dtlanes.resize(40000);
  //dtlane
  if ( (fp=fopen(name,"r")) == NULL )
    {
      fprintf(stderr, "open failed: %s\n", name);
      return EXIT_FAILURE;
    }

  fscanf(fp,"%s",dummy);
  while(fscanf(fp,"%d,%lf,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
	       &a.did, &a.dist, &a.pid, &a.dir, &a.apara,
	       &a.r, &a.slope, &a.cant, &a.lw, &a.rw)!=EOF){
    dtlanes[a.did]=a;
  }
  fclose(fp);
  return EXIT_SUCCESS;
}


void VectorMap::loadAll (const std::string &dirname)
{
	if (loaded)
		return;

	string ptname = dirname + "/point.csv";
	if (load_points((char*)ptname.c_str()) == EXIT_FAILURE) {
      exit(EXIT_FAILURE);
    }
    std::cout << ptname << "\t load complete." << std::endl;

	string linename = dirname + "/line.csv";
	if (load_lines ((char*)linename.c_str()) == EXIT_FAILURE) {
      exit(EXIT_FAILURE);
    }
    std::cout << linename << "\t load complete." << std::endl;

	string dtlanename = dirname + "/dtlane.csv";
	if (load_dtlanes ((char*)dtlanename.c_str()) == EXIT_FAILURE) {
      exit(EXIT_FAILURE);
    }
    std::cout << dtlanename << "\t load complete." << std::endl;

	//string clinename = dirname + "/cline.csv";
    string clinename = dirname + "/whiteline.csv";
	if (load_clines ((char*)clinename.c_str()) == EXIT_FAILURE) {
      exit(EXIT_FAILURE);
    }
    std::cout << clinename << "\t load complete." << std::endl;

	string vectorname = dirname + "/vector.csv";
	if (load_vectors ((char*)vectorname.c_str()) == EXIT_FAILURE) {
      exit(EXIT_FAILURE);
    }
    std::cout << vectorname << "\t load complete." << std::endl;

	//string signalname = dirname + "/signal.csv";
    string signalname = dirname + "/signaldata.csv";
	if (load_signals ((char*)signalname.c_str()) == EXIT_FAILURE) {
      exit(EXIT_FAILURE);
    }
    std::cout << signalname << "\t load complete." << std::endl;

	string lanename = dirname + "/lane.csv";
	if (load_lanes ((char*)lanename.c_str()) == EXIT_FAILURE) {
      exit(EXIT_FAILURE);
    }
    std::cout << lanename << "\t load complete." << std::endl;

	loaded = true;

    std::cout << "all vecter maps loaded." << std::endl;
}
