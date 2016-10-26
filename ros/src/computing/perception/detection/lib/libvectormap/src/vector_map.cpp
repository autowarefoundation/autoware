//#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "vector_map.h"
#include <vector>
#include <map>
#include <tf/transform_listener.h>

void VectorMap::load_points(const vector_map::PointArray& msg)
{
  for (const auto& point : msg.data)
    {
      Point tmp;
      tmp.pid	 = point.pid;
      tmp.b      = point.b;
      tmp.l      = point.l;
      tmp.h      = point.h;
      tmp.bx	 = point.ly;
      tmp.ly	 = point.bx;
      tmp.ref	 = point.ref;
      tmp.mcode1 = point.mcode1;
      tmp.mcode2 = point.mcode2;
      tmp.mcode3 = point.mcode3;
      points.insert( std::map<int, Point>::value_type(tmp.pid, tmp) );
    }
  std::cout << "load points complete. element num: " << points.size() << std::endl;
} /* void VectorMap::load_points() */


void VectorMap::load_lines(const vector_map::LineArray& msg)
{
  for (const auto& line : msg.data)
    {
      Line tmp;
      tmp.lid  = line.lid;
      tmp.bpid = line.bpid;
      tmp.fpid = line.fpid;
      tmp.blid = line.blid;
      tmp.flid = line.flid;

      lines.insert( std::map<int, Line>::value_type(tmp.lid, tmp) );
    }
  std::cout << "load lines complete." << std::endl;
} /* void VectorMap::load_lines() */


void VectorMap::load_lanes(const vector_map::LaneArray& msg)
{
  for (const auto& lane : msg.data)
    {
      Lane tmp;
      tmp.lnid    = lane.lnid;
      tmp.did     = lane.did;
      tmp.blid    = lane.blid;
      tmp.flid    = lane.flid;
      tmp.bnid    = lane.bnid;
      tmp.fnid    = lane.fnid;
      tmp.jct     = lane.jct;
      tmp.blid2   = lane.blid2;
      tmp.blid3   = lane.blid3;
      tmp.blid4   = lane.blid4;
      tmp.flid2   = lane.flid2;
      tmp.flid3   = lane.flid3;
      tmp.flid4   = lane.flid4;
      tmp.clossid = lane.clossid;
      tmp.span    = lane.span;
      tmp.lcnt    = lane.lcnt;
      tmp.lno     = lane.lno;

      lanes.insert( std::map<int, Lane>::value_type(tmp.lnid, tmp) );
    }
  std::cout << "load lanes complete." << std::endl;
} /* void VectorMap::load_lanes() */


void VectorMap::load_vectors(const vector_map::VectorArray& msg)
{
  for (const auto& vector : msg.data)
    {
      Vector tmp;
      tmp.vid  = vector.vid;
      tmp.pid  = vector.pid;
      tmp.hang = vector.hang;
      tmp.vang = vector.vang;

      vectors.insert( std::map<int, Vector>::value_type(tmp.vid, tmp) );
    }
  std::cout << "load vectors complete. element num: " << vectors.size() << std::endl;
} /* void VectorMap::load_vectors() */


void VectorMap::load_signals(const vector_map::SignalArray& msg)
{
  for (const auto& signal : msg.data)
    {
      Signal tmp;
      tmp.id     = signal.id;
      tmp.vid    = signal.vid;
      tmp.plid   = signal.plid;
      tmp.type   = signal.type;
      tmp.linkid = signal.linkid;

      signals.insert( std::map<int, Signal>::value_type(tmp.id, tmp) );
    }
  std::cout << "load signals complete. element num: " << signals.size() << std::endl;
} /* void VectorMap::load_signals() */


void VectorMap::load_whitelines(const vector_map::WhiteLineArray& msg)
{
  for (const auto& white_line : msg.data)
    {
      WhiteLine tmp;
      tmp.id     = white_line.id;
      tmp.lid    = white_line.lid;
      tmp.width  = white_line.width;
      tmp.color  = white_line.color;
      tmp.type   = white_line.type;
      tmp.linkid = white_line.linkid;

      whitelines.insert( std::map<int, WhiteLine>::value_type(tmp.id, tmp) );
    }
  std::cout << "load whitelines complete." << std::endl;
} /* void VectorMap::load_whitelines() */


void VectorMap::load_dtlanes(const vector_map::DTLaneArray& msg)
{
  for (const auto& dtlane : msg.data)
    {
      DTLane tmp;
      tmp.did   = dtlane.did;
      tmp.dist  = dtlane.dist;
      tmp.pid   = dtlane.pid;
      tmp.dir   = dtlane.dir;
      tmp.apara = dtlane.apara;
      tmp.r     = dtlane.r;
      tmp.slope = dtlane.slope;
      tmp.cant  = dtlane.cant;
      tmp.lw    = dtlane.lw;
      tmp.rw    = dtlane.rw;

      dtlanes.insert( std::map<int, DTLane>::value_type(tmp.did, tmp) );
    }
  std::cout << "load dtlanes complete." << std::endl;
} /* void VectorMap::load_dtlanes() */

