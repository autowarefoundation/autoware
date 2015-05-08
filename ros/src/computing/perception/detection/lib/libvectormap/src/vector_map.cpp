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

typedef std::vector< std::vector<std::string> > File_contents;

static inline File_contents read_csv(const char* filename)
{
  File_contents fileContents;
  fileContents.clear();

  std::ifstream ifs(filename);
  if (ifs == 0)
    return fileContents;

  std::string line_contents;
  std::getline(ifs, line_contents); // skip first header line

  while (std::getline(ifs, line_contents))
    {
      std::istringstream ss(line_contents);

      std::vector<std::string> column;
      std::string element;
      while (std::getline(ss, element, ',')) {
        column.push_back(element);
      }
      fileContents.push_back(column);
    }

  return fileContents;
}


int VectorMap::load_points(char *name) {

  File_contents fileContents = read_csv(name);
  if (fileContents.empty())
    return EXIT_FAILURE;

  size_t line_num = fileContents.size();
  for (int i=0; i<line_num; i++)
    {
      Point tmp_p;
      tmp_p.pid    = atoi((const char*)(fileContents[i][0].c_str()));
      tmp_p.b      = atof((const char*)(fileContents[i][1].c_str()));
      tmp_p.l      = atof((const char*)(fileContents[i][2].c_str()));
      tmp_p.h      = atof((const char*)(fileContents[i][3].c_str()));
      tmp_p.ly     = atof((const char*)(fileContents[i][4].c_str()));
      tmp_p.bx     = atof((const char*)(fileContents[i][5].c_str()));
      tmp_p.ref    = atoi((const char*)(fileContents[i][6].c_str()));
      tmp_p.mcode1 = atoi((const char*)(fileContents[i][7].c_str()));
      tmp_p.mcode2 = atoi((const char*)(fileContents[i][8].c_str()));
      tmp_p.mcode3 = atoi((const char*)(fileContents[i][9].c_str()));

      points.insert( std::map<int, Point>::value_type(tmp_p.pid, tmp_p) );
    }

  return EXIT_SUCCESS;
}


int VectorMap::load_lines(char *name) {

  File_contents fileContents = read_csv(name);
  if (fileContents.empty())
    return EXIT_FAILURE;

  size_t line_num = fileContents.size();
  for (int i=0; i<line_num; i++)
    {
      Line tmp_l;
      tmp_l.lid  = atoi((const char*)(fileContents[i][0].c_str()));
      tmp_l.bpid = atoi((const char*)(fileContents[i][1].c_str()));
      tmp_l.fpid = atoi((const char*)(fileContents[i][2].c_str()));
      tmp_l.blid = atoi((const char*)(fileContents[i][3].c_str()));
      tmp_l.flid = atoi((const char*)(fileContents[i][4].c_str()));

      lines.insert( std::map<int, Line>::value_type(tmp_l.lid, tmp_l) );
    }

  return EXIT_SUCCESS;
}


int VectorMap::load_lanes(char *name) {

  File_contents fileContents = read_csv(name);
  if (fileContents.empty())
    return EXIT_FAILURE;

  size_t line_num = fileContents.size();
  for (int i=0; i<line_num; i++)
    {
      Lane tmp_l;
      tmp_l.lnid    = atoi((const char*)(fileContents[i][0].c_str()));
      tmp_l.did     = atoi((const char*)(fileContents[i][1].c_str()));
      tmp_l.blid    = atoi((const char*)(fileContents[i][2].c_str()));
      tmp_l.flid    = atoi((const char*)(fileContents[i][3].c_str()));
      tmp_l.bnid    = atoi((const char*)(fileContents[i][4].c_str()));
      tmp_l.fnid    = atoi((const char*)(fileContents[i][5].c_str()));
      tmp_l.jct     = atoi((const char*)(fileContents[i][6].c_str()));
      tmp_l.blid2   = atoi((const char*)(fileContents[i][7].c_str()));
      tmp_l.blid3   = atoi((const char*)(fileContents[i][8].c_str()));
      tmp_l.blid4   = atoi((const char*)(fileContents[i][9].c_str()));
      tmp_l.flid2   = atoi((const char*)(fileContents[i][10].c_str()));
      tmp_l.flid3   = atoi((const char*)(fileContents[i][11].c_str()));
      tmp_l.flid4   = atoi((const char*)(fileContents[i][12].c_str()));
      tmp_l.clossid = atoi((const char*)(fileContents[i][13].c_str()));
      tmp_l.span    = atof((const char*)(fileContents[i][14].c_str()));
      tmp_l.lcnt    = atoi((const char*)(fileContents[i][15].c_str()));
      tmp_l.lno     = atoi((const char*)(fileContents[i][16].c_str()));

      lanes.insert( std::map<int, Lane>::value_type(tmp_l.lnid, tmp_l) );
    }

  return EXIT_SUCCESS;
}


int VectorMap::load_vectors(char *name) {

  File_contents fileContents = read_csv(name);
  if (fileContents.empty())
    return EXIT_FAILURE;

  size_t line_num = fileContents.size();
  for (int i=0; i<line_num; i++)
    {
      Vector tmp_v;
      tmp_v.vid  = atoi((const char*)(fileContents[i][0].c_str()));
      tmp_v.pid  = atoi((const char*)(fileContents[i][1].c_str()));
      tmp_v.hang = atof((const char*)(fileContents[i][2].c_str()));
      tmp_v.vang = atof((const char*)(fileContents[i][3].c_str()));

      vectors.insert( std::map<int, Vector>::value_type(tmp_v.vid, tmp_v) );
    }

  return EXIT_SUCCESS;
}


int VectorMap::load_signals(char *name) {

  File_contents fileContents = read_csv(name);
  if (fileContents.empty())
    return EXIT_FAILURE;

  size_t line_num = fileContents.size();
  for (int i=0; i<line_num; i++)
    {
      Signal tmp_s;
      tmp_s.id     = atoi((const char*)(fileContents[i][0].c_str()));
      tmp_s.vid    = atoi((const char*)(fileContents[i][1].c_str()));
      tmp_s.plid   = atoi((const char*)(fileContents[i][2].c_str()));
      tmp_s.type   = atoi((const char*)(fileContents[i][3].c_str()));
      tmp_s.linkid = atoi((const char*)(fileContents[i][4].c_str()));

      signals.insert( std::map<int, Signal>::value_type(tmp_s.id, tmp_s) );
    }

  return EXIT_SUCCESS;
}

int VectorMap::load_clines(char *name) {

  File_contents fileContents = read_csv(name);
  if (fileContents.empty())
    return EXIT_FAILURE;

  size_t line_num = fileContents.size();
  for (int i=0; i<line_num; i++)
    {
      CLine tmp_l;
      tmp_l.id     = atoi((const char*)(fileContents[i][0].c_str()));
      tmp_l.lid    = atoi((const char*)(fileContents[i][1].c_str()));
      tmp_l.width  = atof((const char*)(fileContents[i][2].c_str()));
      tmp_l.color  = fileContents[i][3].c_str()[0];
      tmp_l.type   = atoi((const char*)(fileContents[i][4].c_str()));
      tmp_l.linkid = atoi((const char*)(fileContents[i][5].c_str()));

      //clines[tmp_l.id] = tmp_l;
      clines.insert( std::map<int, CLine>::value_type(tmp_l.id, tmp_l) );
    }

  return EXIT_SUCCESS;
}


int VectorMap::load_dtlanes(char *name ) {

  File_contents fileContents = read_csv(name);
  if (fileContents.empty())
    return EXIT_FAILURE;

  size_t line_num = fileContents.size();
  for (int i=0; i<line_num; i++)
    {
      DTLane tmp_a;
      tmp_a.did   = atoi((const char*)(fileContents[i][0].c_str()));
      tmp_a.dist  = atof((const char*)(fileContents[i][1].c_str()));
      tmp_a.pid   = atoi((const char*)(fileContents[i][2].c_str()));
      tmp_a.dir   = atof((const char*)(fileContents[i][3].c_str()));
      tmp_a.apara = atof((const char*)(fileContents[i][4].c_str()));
      tmp_a.r     = atof((const char*)(fileContents[i][5].c_str()));
      tmp_a.slope = atof((const char*)(fileContents[i][6].c_str()));
      tmp_a.cant  = atof((const char*)(fileContents[i][7].c_str()));
      tmp_a.lw    = atof((const char*)(fileContents[i][8].c_str()));
      tmp_a.rw    = atof((const char*)(fileContents[i][9].c_str()));

      dtlanes.insert( std::map<int, DTLane>::value_type(tmp_a.did, tmp_a) );
    }

  return EXIT_SUCCESS;
}


void VectorMap::loadAll (const std::string &dirname)
{
	if (loaded)
		return;

	string ptname = dirname + "/point.csv";
	if (load_points((char*)ptname.c_str()) == EXIT_FAILURE) {
      std::cerr << ptname << "\t load fail." << std::endl;
      exit(EXIT_FAILURE);
    }
    std::cout << ptname << "\t load complete." << std::endl;

	string linename = dirname + "/line.csv";
	if (load_lines ((char*)linename.c_str()) == EXIT_FAILURE) {
      std::cerr << linename << "\t load fail." << std::endl;
      exit(EXIT_FAILURE);
    }
    std::cout << linename << "\t load complete." << std::endl;

	string dtlanename = dirname + "/dtlane.csv";
	if (load_dtlanes ((char*)dtlanename.c_str()) == EXIT_FAILURE) {
      std::cerr << dtlanename << "\t load fail." << std::endl;
      exit(EXIT_FAILURE);
    }
    std::cout << dtlanename << "\t load complete." << std::endl;

	//string clinename = dirname + "/cline.csv";
    string clinename = dirname + "/whiteline.csv";
	if (load_clines ((char*)clinename.c_str()) == EXIT_FAILURE) {
      std::cerr << clinename << "\t load fail." << std::endl;
      exit(EXIT_FAILURE);
    }
    std::cout << clinename << "\t load complete." << std::endl;

	string vectorname = dirname + "/vector.csv";
	if (load_vectors ((char*)vectorname.c_str()) == EXIT_FAILURE) {
      std::cerr << vectorname << "\t load fail." << std::endl;
      exit(EXIT_FAILURE);
    }
    std::cout << vectorname << "\t load complete." << std::endl;

	//string signalname = dirname + "/signal.csv";
    string signalname = dirname + "/signaldata.csv";
	if (load_signals ((char*)signalname.c_str()) == EXIT_FAILURE) {
      std::cerr << signalname << "\t load fail." << std::endl;
      exit(EXIT_FAILURE);
    }
    std::cout << signalname << "\t load complete." << std::endl;

	string lanename = dirname + "/lane.csv";
	if (load_lanes ((char*)lanename.c_str()) == EXIT_FAILURE) {
      std::cerr << lanename << "\t load fail." << std::endl;
      exit(EXIT_FAILURE);
    }
    std::cout << lanename << "\t load complete." << std::endl;

	loaded = true;

    std::cout << "all vecter maps loaded." << std::endl;
}
