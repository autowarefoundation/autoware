
/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *this
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
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


// Auther : Hatem Darweesh
// Date   :  23/06/2018

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <string>
#include <vector>
#include <nmea_msgs/Sentence.h>
#include "ReadNMEASentence.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <ostream>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <tinyxml.h>
#include "dirent.h"

#ifndef foreach
#define foreach BOOST_FOREACH
#endif


#define MAX_SAT_NUM 15
#define LINE_WIDTH 8
#define GROUND_RELATIVE_HRIGHT 8

static std::string nmea_topic_name = "/nmea_sentence";


int g_prev_sat_number = 0;
int g_max_sat = 0;
int g_min_sat = 100;
TiXmlElement* pHeadElem = 0;
long path_id = 0;

class GPS_Point
{
public:
	double lat;
	double lon;
	double alt;

	GPS_Point(double _lat, double _lon, double _alt)
	{
		lat = _lat;
		lon = _lon;
		alt = _alt;
	}

	GPS_Point()
	{
		lat = lon = alt = 0;
	}
};

std::vector<GPS_Point> g_line_points;

bool OpenRosBag(const std::string& bag_Name, rosbag::Bag & _bag, rosbag::View& _bagView, rosbag::View::iterator& _viewIterator)
{
	std::set<std::string> bagTopics;

    try
    {
    	_bag.open(bag_Name, rosbag::bagmode::Read);

    	rosbag::View view(_bag);
		std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
		std::vector<std::string> topics_to_subscribe;

		bagTopics.clear();

		BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos)
		{
			bagTopics.insert(info->topic);
		}

		if (bagTopics.find(nmea_topic_name) == bagTopics.end())
		{
			std::cout << "Can't Find NMEA Sentence Topic in RosBag File :" << nmea_topic_name << std::endl;
			return false;
		}
		else
		{
			topics_to_subscribe.push_back(std::string(nmea_topic_name));
		}

		_bagView.addQuery(_bag, rosbag::TopicQuery(topics_to_subscribe));
		_viewIterator = _bagView.begin();
		return true;

    }
    catch (rosbag::BagIOException& e)
    {
    	std::cout << "Can't Open Rosbag with path: " << bag_Name << std::endl;
        return false;
    }
}

int GetNextSentence(nmea_msgs::SentenceConstPtr& pOutSentence, rosbag::View& _bagView, rosbag::View::iterator& _viewIterator)
{
	if(_viewIterator != _bagView.end())
	{
		rosbag::MessageInstance m = *_viewIterator;

		pOutSentence = m.instantiate<nmea_msgs::Sentence>();
		if (pOutSentence == 0)
		{
			return 0;
		}

		_viewIterator++;
		return 1;
	}
	else
	{
		return -1;
	}
}

void CreateTemplateDocument(TiXmlDocument& doc)
{
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "UTF-8", "" );
	TiXmlElement * mainElement = new TiXmlElement( "kml");
	mainElement->SetAttribute("xmlns", "http://www.opengis.net/kml/2.2");
	mainElement->SetAttribute("xmlns:gx", "http://www.google.com/kml/ext/2.2");
	mainElement->SetAttribute("xmlns:kml", "http://www.opengis.net/kml/2.2");
	mainElement->SetAttribute("xmlns:atom", "http://www.w3.org/2005/Atom");
	mainElement->LinkEndChild(new TiXmlElement("Document"));
	mainElement->FirstChild()->LinkEndChild(new TiXmlElement("name"));
	mainElement->FirstChild()->LastChild()->LinkEndChild(new TiXmlText("Extracted KML from rosbag Data"));
	mainElement->FirstChild()->LinkEndChild(new TiXmlElement("open"));
	mainElement->FirstChild()->LastChild()->LinkEndChild(new TiXmlText("1"));

	doc.LinkEndChild( decl );
	doc.LinkEndChild( mainElement );
}

TiXmlElement* CreateStyleWithColor(TiXmlElement* pElem, std::string style_name, int r, int g, int b, int a)
{
	std::ostringstream color_hexa;
	color_hexa << std::setfill('0') << std::setw(2)<< std::hex << a <<
			std::setfill('0') << std::setw(2) << b <<
			std::setfill('0') << std::setw(2) << g <<
			std::setfill('0') << std::setw(2) << r;

	pElem->LinkEndChild(new TiXmlElement(style_name));
	TiXmlElement* pStyleElem = pElem->LastChild()->ToElement();
	pStyleElem->LinkEndChild(new TiXmlElement("color"));
	pStyleElem->LastChild()->LinkEndChild(new TiXmlText(color_hexa.str()));
	return pStyleElem;
}

void InsertNewStyle(TiXmlElement* pElem, std::string style_id, int r, int g, int b)
{
	pElem->LinkEndChild(new TiXmlElement("Style"));
	TiXmlElement* pStyleElem = pElem->LastChild()->ToElement();
	pStyleElem->SetAttribute("id", style_id);

	TiXmlElement* pLineStyleElem = CreateStyleWithColor(pStyleElem, "LineStyle", r,g,b,255);
	pLineStyleElem->LinkEndChild(new TiXmlElement("width"));
	std::ostringstream width_str;
	width_str << LINE_WIDTH;
	pLineStyleElem->LastChild()->LinkEndChild(new TiXmlText(width_str.str()));

	TiXmlElement* pPolyStyleElem = CreateStyleWithColor(pStyleElem, "PolyStyle", 120,120,120,175);
	pPolyStyleElem->LinkEndChild(new TiXmlElement("outline"));
	std::ostringstream outline_str;
	outline_str << 0;
	pPolyStyleElem->LastChild()->LinkEndChild(new TiXmlText(outline_str.str()));
}

void InitializeStyleForSateliteNumbers(TiXmlElement* pElem, int max_sat_nu)
{
	for(int i=max_sat_nu; i >= 0; i--)
	{
		double r=0, g=0, b=0;

		float norm_cost = (float)i / (float)max_sat_nu;
		if(norm_cost <= 0.5)
		{
			g = (0.5 - norm_cost)*2.0;
			r = 0;
		}
		else if(norm_cost > 0.5)
		{
			g = 0;
			r = (norm_cost - 0.5)*2.0;
		}

		std::ostringstream style_id;
		style_id<< "stl_" << i;
		InsertNewStyle(pElem,style_id.str(), r*255.0,g*255.0,b*255.0);
	}
}

void CreateLinePlaceMark(std::vector<GPS_Point>& line, int sat_no)
{
	if(line.size() == 0 ) return;

	sat_no = (float)sat_no / 2.0 +  (float)MAX_SAT_NUM/2.0;

	std::ostringstream path_name, style_name;
	path_id++;
	path_name << "path_" << path_id;
	style_name << "stl_" << sat_no;

	pHeadElem->LinkEndChild(new TiXmlElement("Placemark"));
	pHeadElem->LastChild()->LinkEndChild(new TiXmlElement("name"));
	pHeadElem->LastChild()->LastChild()->LinkEndChild(new TiXmlText(path_name.str()));

	pHeadElem->LastChild()->LinkEndChild(new TiXmlElement("styleUrl"));
	pHeadElem->LastChild()->LastChild()->LinkEndChild(new TiXmlText(style_name.str()));

	pHeadElem->LastChild()->LinkEndChild(new TiXmlElement("LineString"));

	pHeadElem->LastChild()->LastChild()->LinkEndChild(new TiXmlElement("extrude"));
	pHeadElem->LastChild()->LastChild()->LastChild()->LinkEndChild(new TiXmlText("1"));

	if(GROUND_RELATIVE_HRIGHT > 0)
	{
		pHeadElem->LastChild()->LastChild()->LinkEndChild(new TiXmlElement("tessellate"));
		pHeadElem->LastChild()->LastChild()->LastChild()->LinkEndChild(new TiXmlText("0"));

		pHeadElem->LastChild()->LastChild()->LinkEndChild(new TiXmlElement("altitudeMode"));
		pHeadElem->LastChild()->LastChild()->LastChild()->LinkEndChild(new TiXmlText("relativeToGround"));
	}

	pHeadElem->LastChild()->LastChild()->LinkEndChild(new TiXmlElement("coordinates"));

	TiXmlElement* pCoordsElem = pHeadElem->LastChild()->LastChild()->LastChild()->ToElement();

	std::ostringstream coords ;
	coords.precision(16);
	for(unsigned int i=0 ; i < line.size(); i++)
	{
		coords << line.at(i).lon << "," << line.at(i).lat << "," << line.at(i).alt << " ";
	}

	pCoordsElem->LinkEndChild(new TiXmlText(coords.str()));
}

void ParseAndWrite(const nmea_msgs::SentenceConstPtr& msg, std::ofstream& _ofs)
{
  if(_ofs.is_open())
  {
	  _ofs << msg->sentence << std::endl;
  }

  NMEA_PARSER::ReadNMEASentence _parser;
  for(unsigned int i=0; i < msg->sentence.size(); i++)
  {
	  _parser.Parse(msg->sentence.at(i));
  }

  if(_parser.gpgga.count == 1)
  {
	  if(_parser.gpgga.satellites > MAX_SAT_NUM)
		  _parser.gpgga.satellites = MAX_SAT_NUM;

	  if(_parser.gpgga.satellites > g_max_sat)
		  g_max_sat = _parser.gpgga.satellites;

	  if(_parser.gpgga.satellites < g_min_sat)
		  g_min_sat = _parser.gpgga.satellites;

	  if(_parser.gpgga.satellites != g_prev_sat_number)
	  {
		  if(GROUND_RELATIVE_HRIGHT > 0)
			  g_line_points.push_back(GPS_Point(_parser.gpgga.latitude, _parser.gpgga.longitude, GROUND_RELATIVE_HRIGHT));
		  else
			  g_line_points.push_back(GPS_Point(_parser.gpgga.latitude, _parser.gpgga.longitude, _parser.gpgga.altitude));

		  CreateLinePlaceMark(g_line_points, g_prev_sat_number);
		  g_prev_sat_number = _parser.gpgga.satellites;
		  g_line_points.clear();
	  }


	  if(_parser.gpgga.satellites == g_prev_sat_number)
	  {
		  if(GROUND_RELATIVE_HRIGHT > 0)
			  g_line_points.push_back(GPS_Point(_parser.gpgga.latitude, _parser.gpgga.longitude, GROUND_RELATIVE_HRIGHT));
		  else
			  g_line_points.push_back(GPS_Point(_parser.gpgga.latitude, _parser.gpgga.longitude, _parser.gpgga.altitude));
	  }

	  std::cout << "GPGGA: " << _parser.gpgga.longitude << ", " << _parser.gpgga.latitude <<", " << _parser.gpgga.altitude << ", " << g_max_sat << ", " << g_min_sat << std::endl;
  }
}

void GetFileNameInFolder(const std::string& path, std::vector<std::string>& out_list)
{
	out_list.clear();
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (path.c_str())) != NULL)
	{
	  while ((ent = readdir (dir)) != NULL)
	  {
		  std::string str(ent->d_name);
		  if(str.compare(".") !=0 && str.compare("..") !=0)
			  out_list.push_back(path+str);
	  }
	  closedir (dir);
	}
}

void ConvertOneFile(const std::string& fileName)
{
	path_id = 0;
	g_prev_sat_number = 0;
	g_line_points.clear();
	std::string output_kml = fileName;
	output_kml.replace(fileName.size() - 4, fileName.size() , ".kml");
	std::string output_csv = fileName;
	output_csv.replace(fileName.size() - 4, fileName.size() , ".csv");

	rosbag::Bag bag;
	rosbag::View bagView;
	rosbag::View::iterator viewIterator;
	TiXmlDocument xml_doc;

	CreateTemplateDocument(xml_doc);
	pHeadElem = xml_doc.FirstChildElement()->FirstChildElement("Document");
	InitializeStyleForSateliteNumbers(pHeadElem, MAX_SAT_NUM);

	std::ofstream ofs;
	ofs.open(output_csv.c_str(), std::ios::app);

	OpenRosBag(fileName, bag, bagView, viewIterator);

	while(1)
	{
	  nmea_msgs::SentenceConstPtr pOutSentence;
	  int ret = GetNextSentence(pOutSentence, bagView, viewIterator);
	  if(ret == 1)
		  ParseAndWrite(pOutSentence, ofs);
	  else if(ret == -1)
		  break;
	}
	CreateLinePlaceMark(g_line_points, g_prev_sat_number);
	xml_doc.SaveFile(output_kml);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "nmea2kml");

  if(argc < 2)
  {
    std::cout << "Usage: rosrun autoware_bag_tools nmea2kml bag_file_name.bag \n       rosrun autoware_bag_tools nmea2kml rosbags_folder/ " << std::endl;
    exit (1);
  }

  std::vector<std::string> bag_files;

  bool bDirectory = false;

  boost::filesystem::path p_param(argv[1]);

  if(!exists(p_param))
  {
	  std::cout << "Wrong argument: Doesn't Exist" <<std::endl;
	  exit(1);
  }

  if(is_directory(p_param))
  {
	  std::vector<std::string> all_files;
	  GetFileNameInFolder(p_param.string(), all_files);

	  for (unsigned int i=0; i < all_files.size(); i++)
	  {
		  boost::filesystem::path _f(all_files.at(i));
		  std::string file_ext = _f.extension().string();
		  std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::toupper);
		  if(file_ext.compare(".BAG")==0)
		  {
			  bag_files.push_back(_f.string());
			  std::cout << "Bag File: " << _f.string() << std::endl;
		  }
	  }
  }
  else if(is_regular_file(p_param))
  {
	  std::string file_ext = p_param.extension().string();
	  std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::toupper);
	  if(file_ext.compare(".BAG")==0)
	  {
		  bag_files.push_back(p_param.string());
		  std::cout << "Bag File: " << p_param.string() << std::endl;
	  }
  }
  else
  {
	  std::cout << "Wrong argument: Not a directory or a file !" <<std::endl;
	  exit(1);
  }

  for (unsigned int i=0; i < bag_files.size(); i++)
  {
	  ConvertOneFile(bag_files.at(i));
  }

  return 0;
}
