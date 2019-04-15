/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <vector_map_converter/autoware2vectormap.hpp>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <vector_map/vector_map.h>
template <class T>
void write(std::ofstream &ofs, std::vector<T> objs)
{
  for( auto obj : objs)
  {
    ofs << obj << std::endl;
  }
}

using uint_pair = std::pair<unsigned int, unsigned int>;

void writeVectorMapMsgs(const std::string output_dir,
                        const std::vector<vector_map_msgs::Area> &vmap_areas,
                        const std::vector<vector_map_msgs::CrossRoad> &vmap_cross_roads,
                        const std::vector<vector_map_msgs::CrossWalk> &vmap_cross_walks,
                        const std::vector<vector_map_msgs::DTLane> &vmap_dtlanes,
                        const std::vector<vector_map_msgs::Lane> &vmap_lanes,
                        const std::vector<vector_map_msgs::Line> &vmap_lines,
                        const std::vector<vector_map_msgs::Node> &vmap_nodes,
                        const std::vector<vector_map_msgs::Point> &vmap_points,
                        const std::vector<vector_map_msgs::Pole> &vmap_poles,
                        const std::vector<vector_map_msgs::RoadSign> &vmap_road_signs,
                        const std::vector<vector_map_msgs::Signal> &vmap_signals,
                        const std::vector<vector_map_msgs::StopLine> &vmap_stop_lines,
                        const std::vector<vector_map_msgs::UtilityPole> &vmap_utility_poles,
                        const std::vector<vector_map_msgs::Vector> &vmap_vectors,
                        const std::vector<vector_map_msgs::WayArea> &vmap_way_areas,
                        const std::vector<vector_map_msgs::WhiteLine> &vmap_white_lines
                        )
{
  std::ofstream ofs;
  std::string filename;

  //areas
  if(!vmap_areas.empty())
  {
    filename = output_dir + "/area.csv";
    ofs.open(filename);
    ofs << "AID,SLID,ELID" << std::endl;
    write(ofs, vmap_areas);
    ofs.close();
  }

  //cross_roads
  if(!vmap_cross_roads.empty())
  {
    filename = output_dir + "/intersection.csv";
    ofs.open(filename);
    ofs << "ID,AID,LinkID" << std::endl;
    write(ofs, vmap_cross_roads);
    ofs.close();
  }
  //cross_walk
  if(!vmap_cross_walks.empty())
  {
    filename = output_dir + "/crosswalk.csv";
    ofs.open(filename);
    ofs << "ID,AID,Type,BdID,LinkID" << std::endl;
    write(ofs, vmap_cross_walks);
    ofs.close();
  }

  //DTLane
  if(!vmap_dtlanes.empty())
  {

    filename = output_dir + "/dtlane.csv";
    ofs.open(filename);
    ofs << "DID,Dist,PID,Dir,Apara,r,slope,cant,LW,RW" << std::endl;
    write(ofs, vmap_dtlanes);
    ofs.close();
  }
  //Lanes
  if(!vmap_lanes.empty())
  {
    filename = output_dir + "/lane.csv";
    ofs.open(filename);
    ofs << "LnID,DID,BLID,FLID,BNID,FNID,JCT,BLID2,BLID3,BLID4,FLID2,FLID3,FLID4,ClossID,Span,LCnt,Lno,LaneType,LimitVel,RefVel,RoadSecID,LaneChgFG,LinkWAID" << std::endl;
    write(ofs, vmap_lanes);
    ofs.close();
  }
  //Lanes
  if(!vmap_lines.empty())
  {
    filename = output_dir + "/line.csv";
    ofs.open(filename);
    ofs << "LID,BPID,FPID,BLID,FLID" << std::endl;
    write(ofs, vmap_lines);
    ofs.close();
  }
  //nodes
  if(!vmap_nodes.empty())
  {
    filename = output_dir + "/node.csv";
    ofs.open(filename);
    ofs << "NID,PID" << std::endl;
    write(ofs, vmap_nodes);
    ofs.close();
  }
  //points
  if(!vmap_points.empty())
  {
    filename = output_dir + "/point.csv";
    ofs.open(filename);
    ofs << "PID,B,L,H,Bx,Ly,ReF,MCODE1,MCODE2,MCODE3" << std::endl;
    write(ofs, vmap_points);
    ofs.close();
  }
  //poles
  if(!vmap_poles.empty())
  {
    filename = output_dir + "/pole.csv";
    ofs.open(filename);
    ofs << "PLID,VID,Length,Dim" << std::endl;
    write(ofs, vmap_poles);
    ofs.close();
  }
  //utility poles
  if(!vmap_utility_poles.empty())
  {
    filename = output_dir + "/poledata.csv";
    ofs.open(filename);
    ofs << "ID,PLID,LinkID" << std::endl;
    write(ofs, vmap_utility_poles);
    ofs.close();
  }
  //road_signs
  if(!vmap_road_signs.empty())
  {
    filename = output_dir + "/roadsign.csv";
    ofs.open(filename);
    ofs << "ID,VID,PLID,Type,LinkID" << std::endl;
    write(ofs, vmap_road_signs);
    ofs.close();
  }
  //signals
  if(!vmap_signals.empty())
  {
    filename = output_dir + "/signaldata.csv";
    ofs.open(filename);
    ofs << "ID,VID,PLID,Type,LinkID" << std::endl;
    write(ofs, vmap_signals);
    ofs.close();
  }
  //stop_line
  if(!vmap_stop_lines.empty())
  {
    filename = output_dir + "/stopline.csv";
    ofs.open(filename);
    ofs << "ID,LID,TLID,SignID,LinkID" << std::endl;
    write(ofs, vmap_stop_lines);
    ofs.close();
  }
  //vectors
  if(!vmap_vectors.empty())
  {
    filename = output_dir + "/vector.csv";
    ofs.open(filename);
    ofs << "VID,PID,Hang,Vang" << std::endl;
    write(ofs, vmap_vectors);
    ofs.close();
  }
  //wayareas
  if(!vmap_way_areas.empty())
  {
    filename = output_dir + "/wayarea.csv";
    ofs.open(filename);
    ofs << "ID,AID" << std::endl;
    write(ofs, vmap_way_areas);
    ofs.close();
  }

  if(!vmap_white_lines.empty())
  {
    filename = output_dir + "/whiteline.csv";
    ofs.open(filename);
    ofs << "ID,LID,Width,Color,type,LinkID" << std::endl;
    write(ofs, vmap_white_lines);
    ofs.close();
  }
}
void getVectorMapMsgs(
  const autoware_map::AutowareMapHandler &map_handler,
  std::vector<vector_map_msgs::Area> &vmap_areas,
  std::vector<vector_map_msgs::CrossRoad> &vmap_cross_roads,
  std::vector<vector_map_msgs::CrossWalk> &vmap_cross_walks,
  std::vector<vector_map_msgs::DTLane> &vmap_dtlanes,
  std::vector<vector_map_msgs::Lane> &vmap_lanes,
  std::vector<vector_map_msgs::Line> &vmap_lines,
  std::vector<vector_map_msgs::Node> &vmap_nodes,
  std::vector<vector_map_msgs::Point> &vmap_points,
  std::vector<vector_map_msgs::Pole> &vmap_dummy_poles,
  std::vector<vector_map_msgs::RoadSign> &vmap_road_signs,
  std::vector<vector_map_msgs::Signal> &vmap_signals,
  std::vector<vector_map_msgs::StopLine> &vmap_stop_lines,
  std::vector<vector_map_msgs::UtilityPole> &vmap_dummy_utility_poles,
  std::vector<vector_map_msgs::Vector> &vmap_vectors,
  std::vector<vector_map_msgs::WayArea> &vmap_way_areas,
  std::vector<vector_map_msgs::WhiteLine> &vmap_white_lines
  )
{
  createAreas(map_handler, vmap_areas, vmap_lines);
  createNodes(map_handler, vmap_nodes);
  createPoints(map_handler, vmap_points);
  createDTLanes( map_handler, vmap_dtlanes,vmap_lanes);
  createCrossWalks(map_handler, vmap_cross_walks, vmap_areas, vmap_lines, vmap_points);
  createCrossRoads(map_handler, vmap_cross_roads);
  createSignals(map_handler, vmap_signals, vmap_vectors,vmap_dummy_poles);
  createStopLines(map_handler, vmap_lines, vmap_points, vmap_stop_lines, vmap_road_signs, vmap_vectors, vmap_dummy_poles);
  createWayAreas(map_handler, vmap_way_areas);
  createWayAreasFromLanes(map_handler, vmap_way_areas, vmap_areas, vmap_lines, vmap_points);
  createDummyUtilityPoles(vmap_dummy_poles, vmap_dummy_utility_poles);
  // createWhitelines(map_handler, vmap_points, vmap_lines, vmap_white_lines);
}

void createAreas(const autoware_map::AutowareMapHandler &map_handler, std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines)
{
  int line_id = getMaxId(vmap_lines) + 1;
  for ( auto awmap_area : map_handler.findByFilter( [&](autoware_map::Area a){return true; }))
  {
    vector_map_msgs::Area vmap_area;
    vmap_area.aid = awmap_area.area_id;
    vmap_area.slid = line_id;

    //create lines that represent area
    auto end_itr = awmap_area.point_ids.end();
    auto begin_itr = awmap_area.point_ids.begin();
    //assumes that point id of corresponding points in AutowareMap and VectorMap are the same. (please check createPoints function)
    for (auto point_itr = begin_itr; point_itr != end_itr; point_itr++)
    {
      vector_map_msgs::Line line;
      line.lid =line_id;
      line.bpid = *point_itr;

      if(point_itr == begin_itr)
      {
        line.blid = 0;
      }
      else{
        line.blid = line_id - 1;
      }

      if(point_itr + 1 == end_itr)
      {
        line.flid = 0;
        line.fpid = *begin_itr;           //close the loop
      }
      else{
        line.flid = line_id + 1;
        line.fpid = *(point_itr + 1);
      }
      vmap_lines.push_back(line);
      vmap_area.elid = line_id;
      line_id++;
    }
    vmap_areas.push_back(vmap_area);
  }
}


void convertPoint(vector_map_msgs::Point &vmap_point,const autoware_map_msgs::Point awmap_point)
{
  if(isJapaneseCoordinate(awmap_point.epsg))
  {
    vmap_point.bx = awmap_point.x;
    vmap_point.ly = awmap_point.y;
  }
  else{
    vmap_point.bx = awmap_point.y;
    vmap_point.ly = awmap_point.x;
  }
  vmap_point.ref = convertESPGToRef(awmap_point.epsg);
  vmap_point.pid = awmap_point.point_id;
  vmap_point.b = convertDecimalToDDMMSS( awmap_point.lat );
  vmap_point.l = convertDecimalToDDMMSS( awmap_point.lng );
  vmap_point.h = awmap_point.z;

  //cannot convert mcodes from autoware_map_format
  vmap_point.mcode1 = 0;
  vmap_point.mcode2 = 0;
  vmap_point.mcode3 = 0;
}


void createPoints(const autoware_map::AutowareMapHandler &awmap, std::vector<vector_map_msgs::Point> &vmap_points)
{

  bool epsg_fail_flag = false;
  for ( auto awmap_pt : awmap.findByFilter( [&](autoware_map::Point pt){return true; }) )
  {
    if(isJapaneseCoordinate(awmap_pt.epsg) && !epsg_fail_flag) {
      epsg_fail_flag =true;
      ROS_WARN_STREAM("no corresponding Japanese Plane Rectangular CS Number for specified epsg value" );
    }
    vector_map_msgs::Point vmap_point;
    convertPoint(vmap_point, awmap_pt);
    vmap_points.push_back(vmap_point);
  }
}

void createNodes(const autoware_map::AutowareMapHandler &awmap, std::vector<vector_map_msgs::Node> &vmap_nodes)
{
  for ( auto awmap_wp : awmap.findByFilter([&](autoware_map::Waypoint){return true; } ))
  {
    vector_map_msgs::Node vmap_node;
    vmap_node.nid = awmap_wp.waypoint_id;
    vmap_node.pid = awmap_wp.waypoint_id;
    vmap_nodes.push_back(vmap_node);
  }
}

std::vector<int> findBranchingIdx(const std::vector<autoware_map::WaypointRelation> &relation, int root_index, const std::unordered_map<uint_pair, unsigned int, boost::hash<uint_pair>> &lookup_table)
{
  std::vector<int> branching_indices;
  for(const auto &r : relation)
  {
    if(r.waypoint_id == root_index){
      auto key = std::make_pair(r.waypoint_id, r.next_waypoint_id);
      branching_indices.push_back(lookup_table.at(key));
    }
  }
  return branching_indices;
}
std::vector<int> findMergingIdx(const std::vector<autoware_map::WaypointRelation> &relation, int merged_index,const std::unordered_map<uint_pair, unsigned int, boost::hash<uint_pair>> &lookup_table)
{
  std::vector<int> merging_indices;

  for(const auto &r : relation)
  {
    if(r.waypoint_id == merged_index)
    {
      auto key = std::make_pair(r.waypoint_id, r.next_waypoint_id);
      merging_indices.push_back(lookup_table.at(key));
    }
  }
  return merging_indices;
}

int getJunctionType(const std::vector<autoware_map::WaypointRelation> &awmap_waypoint_relations,
                    std::vector<int> branching_idx,
                    std::vector<int> merging_idx)
{
  if(branching_idx.size() <= 1 && merging_idx.size() <= 1 )
  {
    return vector_map_msgs::Lane::NORMAL;
  }

  // if(merging_idx.size() > 1) return vector_map_msgs::Lane::RIGHT_MERGING;
  // else         return vector_map_msgs::Lane::NORMAL;
  sort(merging_idx.begin(), merging_idx.end(), [&](const int x, const int y){ return awmap_waypoint_relations.at(x).blinker < awmap_waypoint_relations.at(y).blinker; });
  sort(branching_idx.begin(), branching_idx.end(), [&](const int x, const int y){ return awmap_waypoint_relations.at(x).blinker < awmap_waypoint_relations.at(y).blinker; });

  int left_branching_cnt = 0;
  int right_branching_cnt = 0;
  int straight_branching_cnt = 0;
  int left_merging_cnt = 0;
  int right_merging_cnt = 0;
  int straight_merging_cnt = 0;

  for(auto idx : branching_idx)
  {
    if( awmap_waypoint_relations.at(idx).blinker == 1 )
    {
      left_branching_cnt++;
    }
    if( awmap_waypoint_relations.at(idx).blinker == 2 )
    {
      right_branching_cnt++;
    }else{
      straight_branching_cnt++;
    }
  }
  for(auto idx : merging_idx)
  {
    if( awmap_waypoint_relations.at(idx).blinker == 1 )
    {
      left_merging_cnt++;
    }
    if( awmap_waypoint_relations.at(idx).blinker == 2 )
    {
      right_merging_cnt++;
    }else{
      straight_merging_cnt++;
    }
  }

  if(branching_idx.size() >= 2 && merging_idx.size() >= 2 )
  {
    return vector_map_msgs::Lane::COMPOSITION;
  }
  if ( right_branching_cnt >= 1 )
  {
    return vector_map_msgs::Lane::RIGHT_BRANCHING;
  }
  if ( left_branching_cnt >= 1 )
  {
    return vector_map_msgs::Lane::LEFT_BRANCHING;
  }
  if( straight_branching_cnt >= 2) {
    return vector_map_msgs::Lane::LEFT_BRANCHING;
  }
  if ( right_merging_cnt >= 1 )
  {
    return vector_map_msgs::Lane::RIGHT_MERGING;
  }
  if ( left_merging_cnt >= 1 )
  {
    return vector_map_msgs::Lane::LEFT_MERGING;
  }
  if( straight_merging_cnt >= 2) {
    return vector_map_msgs::Lane::LEFT_MERGING;
  }

  ROS_ERROR_STREAM("could not find appropriate junction type!!!!!!!");

  return vector_map_msgs::Lane::NORMAL;
}



void createDTLanes(const autoware_map::AutowareMapHandler &awmap,
                   std::vector<vector_map_msgs::DTLane> &vmap_dtlanes,
                   std::vector<vector_map_msgs::Lane> &vmap_lanes)
{
  std::vector<autoware_map::WaypointRelation> awmap_waypoint_relations = awmap.findByFilter( [&](autoware_map::WaypointRelation wr){return true; });

  //create lookup table:
  // key = waypoint_relation
  // value = lnid;
  std::unordered_map<uint_pair, unsigned int, boost::hash<uint_pair>> wp_relation2lnid;
  unsigned int id = 1;
  for ( auto r : awmap_waypoint_relations ){
     auto key = std::make_pair(r.waypoint_id,r.next_waypoint_id);
     wp_relation2lnid[key] = id;
     id++;
  }

  id = 1;
  for ( auto awmap_waypoint_relation : awmap_waypoint_relations )
  {
    if(id % 1000 == 0)
      std::cout << id << "/" << awmap_waypoint_relations.size() << std::endl;
    autoware_map::Waypoint awmap_waypoint = *(awmap_waypoint_relation.getWaypointPtr());
    autoware_map::Waypoint awmap_next_waypoint = *(awmap_waypoint_relation.getNextWaypointPtr());
    //create dtlane
    vector_map_msgs::DTLane vmap_dtlane;
    vmap_dtlane.did = id;
    vmap_dtlane.dist = awmap_waypoint_relation.distance;
    vmap_dtlane.pid = awmap_waypoint_relation.getWaypointPtr()->point_id;
    vmap_dtlane.dir = convertDecimalToDDMMSS(awmap_waypoint_relation.yaw);
    vmap_dtlane.apara = 0;
    vmap_dtlane.r = 90000000000;
    autoware_map_msgs::Point pt1, pt2;
    pt1 = *(awmap_waypoint.getPointPtr());
    pt2 = *(awmap_next_waypoint.getPointPtr());
    double horizontal_dist = hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    double vertical_dist = pt2.z - pt1.z;
    vmap_dtlane.slope = vertical_dist / horizontal_dist * 100;     //decimal to percentage
    vmap_dtlane.cant = 0;
    vmap_dtlane.lw = awmap_waypoint.left_width;
    vmap_dtlane.rw = awmap_waypoint.right_width;
    vmap_dtlanes.push_back(vmap_dtlane);

    //create lane
    vector_map_msgs::Lane vmap_lane;
    vmap_lane.lnid = id;
    vmap_lane.did = id;

    std::vector<int> merging_idx = findMergingIdx(awmap_waypoint.getStdWaypointRelations(), awmap_waypoint.waypoint_id, wp_relation2lnid);
    std::vector<int> branching_idx = findBranchingIdx(awmap_next_waypoint.getStdWaypointRelations(), awmap_next_waypoint.waypoint_id,wp_relation2lnid);
    // //change order of branch/merge lanes according to blinkers. (staright < left turn < right turn)
    vmap_lane.jct = getJunctionType(awmap_waypoint_relations, branching_idx, merging_idx);
    vmap_lane.blid = 0;
    vmap_lane.flid = 0;
    vmap_lane.blid2 = 0;
    vmap_lane.blid3 = 0;
    vmap_lane.blid4 = 0;
    vmap_lane.flid2 = 0;
    vmap_lane.flid3 = 0;
    vmap_lane.flid4 = 0;
    if(merging_idx.size() >= 1)
      vmap_lane.blid = merging_idx.at(0) + 1;
    if(merging_idx.size() >= 2)
      vmap_lane.blid2 = merging_idx.at(1) + 1;
    if(merging_idx.size() >= 3)
      vmap_lane.blid3 = merging_idx.at(2) + 1;
    if(merging_idx.size() >= 4)
      vmap_lane.blid4 = merging_idx.at(3) + 1;
    if(branching_idx.size() >= 1)
      vmap_lane.flid = branching_idx.at(0) + 1;
    if(branching_idx.size() >= 2)
      vmap_lane.flid2 = branching_idx.at(1) + 1;
    if(branching_idx.size() >= 3)
      vmap_lane.flid3 = branching_idx.at(2) + 1;
    if(branching_idx.size() >= 4)
      vmap_lane.flid4 = branching_idx.at(3) + 1;
    vmap_lane.bnid = awmap_waypoint.point_id;
    vmap_lane.fnid = awmap_next_waypoint.point_id;
    vmap_lane.span =  awmap_waypoint_relation.distance;

    auto awmap_lane = *(awmap_waypoint.getBelongingLanes().front());
    vmap_lane.lcnt = awmap_lane.num_of_lanes;
    vmap_lane.lanetype = awmap_waypoint_relation.blinker;
    vmap_lane.limitvel = awmap_lane.speed_limit;
    vmap_lane.refvel = awmap_waypoint.velocity;
    vmap_lane.lanecfgfg = (vmap_lane.lcnt > 1) ? 1 : 0;
    vmap_lane.lno = awmap_lane.lane_number;
    vmap_lane.roadsecid = 0;
    vmap_lane.linkwaid = 0;
    vmap_lanes.push_back(vmap_lane);
    id++;
  }
}

void createCrossRoads(const autoware_map::AutowareMapHandler &awmap, std::vector<vector_map_msgs::CrossRoad> &vmap_cross_roads)
{
  unsigned int id = 1;
  for ( auto awmap_relation : awmap.findByFilter( [&](autoware_map::LaneAttributeRelation lar){return lar.attribute_type == autoware_map_msgs::LaneAttributeRelation::INTERSECTION; }) )
  {
    //check whether same cross_road is already created
    if(std::find_if(vmap_cross_roads.begin(), vmap_cross_roads.end(), [&](vector_map_msgs::CrossRoad cr){return cr.aid == awmap_relation.area_id; }) != vmap_cross_roads.end())
    {
      continue;
    }
    vector_map_msgs::CrossRoad cross_road;
    cross_road.id = id++;
    cross_road.aid = awmap_relation.area_id;
    cross_road.linkid = 0;
    vmap_cross_roads.push_back(cross_road);
  }
}


int createSquareArea(const double x, const double y, const double z, const double length,
                     std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines,
                     std::vector<vector_map_msgs::Point> &vmap_points)
{
  vector_map_msgs::Point v1, v2, v3, v4;
  int point_id = getMaxId(vmap_points) + 1;
  v1.pid = point_id++;
  v1.bx = x - length / 2;
  v1.ly = y - length / 2;
  v1.h = z;

  v2.pid = point_id++;
  v2.bx = x - length / 2;
  v2.ly = y + length / 2;
  v2.h = z;

  v3.pid = point_id++;
  v3.bx = x + length / 2;
  v3.ly = y + length / 2;
  v3.h = z;

  v4.pid = point_id++;
  v4.bx = x + length / 2;
  v4.ly = y - length / 2;
  v4.h = z;

  vmap_points.push_back(v1);
  vmap_points.push_back(v2);
  vmap_points.push_back(v3);
  vmap_points.push_back(v4);

  vector_map_msgs::Line line;
  int lid, start_lid;
  lid =start_lid= getMaxId(vmap_lines) + 1;
  line.lid = lid;
  line.bpid = v1.pid;
  line.fpid = v2.pid;
  line.blid = 0;
  line.flid = lid + 1;
  vmap_lines.push_back(line);
  lid++;
  line.lid = lid;
  line.bpid = v2.pid;
  line.fpid = v3.pid;
  line.blid = lid - 1;
  line.flid = lid + 1;
  vmap_lines.push_back(line);
  lid++;
  line.lid = lid;
  line.bpid = v3.pid;
  line.fpid = v4.pid;
  line.blid = lid - 1;
  line.flid = lid + 1;
  vmap_lines.push_back(line);
  lid++;
  line.lid = lid;
  line.bpid = v4.pid;
  line.fpid = v1.pid;
  line.blid = lid - 1;
  line.flid = 0;
  vmap_lines.push_back(line);

  vector_map_msgs::Area area;
  area.aid = getMaxId(vmap_areas) + 1;
  area.slid = start_lid;
  area.elid = lid;
  vmap_areas.push_back(area);

  return area.aid;
}

void createCrossWalks(const autoware_map::AutowareMapHandler &awmap, std::vector<vector_map_msgs::CrossWalk> &vmap_cross_walks,
                      std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::Point> &vmap_points)
{
  unsigned int id = 1;
  for( auto awmap_relation : awmap.findByFilter([&](const autoware_map::LaneAttributeRelation lar){ return lar.attribute_type == autoware_map_msgs::LaneAttributeRelation::CROSS_WALK; }) )
  {
    //skip if area is already added
    if(std::find_if(vmap_cross_walks.begin(), vmap_cross_walks.end(), [&](vector_map_msgs::CrossWalk cw){return cw.aid == awmap_relation.area_id; }) != vmap_cross_walks.end())
    {
      continue;
    }

    int parent_id = id++;
    vector_map_msgs::CrossWalk cross_walk;
    cross_walk.id = parent_id;
    cross_walk.aid = awmap_relation.area_id;
    cross_walk.type = 0;
    cross_walk.linkid = 0;
    vmap_cross_walks.push_back(cross_walk);

    cross_walk.id = id++;
    cross_walk.aid = awmap_relation.area_id;
    cross_walk.type = 1;
    cross_walk.bdid = parent_id;
    cross_walk.linkid = 0;
    vmap_cross_walks.push_back(cross_walk);

    //create stripes for detection area
    autoware_map_msgs::Area awmap_area = awmap.findById<autoware_map::Area>(awmap_relation.area_id);
    std::vector<autoware_map_msgs::Point> vertices;
    for ( int vertex : awmap_area.point_ids)
    {
      vertices.push_back(awmap.findById<autoware_map::Point>(vertex));
    }

    autoware_map_msgs::Point min,max;
    getMinMax(min,max,vertices);

    double height = (min.z + max.z) / 2;

    double resolution = 2;
    for (double x = min.x - resolution; x < max.x + resolution; x += resolution / 2)
    {
      for (double y = min.y - resolution; y < max.y + resolution; y += resolution / 2)
      {
        if( isWithinArea(x,y,vertices) )
        {
          int area_id;
          if(isJapaneseCoordinate(vertices.front().epsg))
          {
            area_id = createSquareArea(x,y,height,resolution, vmap_areas, vmap_lines, vmap_points);
          }
          else
          {
            area_id = createSquareArea(y,x,height,resolution, vmap_areas, vmap_lines, vmap_points);
          }

          cross_walk.id = id++;
          cross_walk.aid = area_id;
          cross_walk.type = 1;
          cross_walk.bdid = parent_id;
          cross_walk.linkid = 0;
          vmap_cross_walks.push_back(cross_walk);
        }
      }
    }
  }
}

vector_map_msgs::Pole createDummyPole(const int plid,const int vid)
{
  vector_map_msgs::Pole vmap_pole;
  vmap_pole.plid = plid;
  vmap_pole.vid = vid;
  vmap_pole.length = 1;
  vmap_pole.dim = 0.02;
  return vmap_pole;
}

vector_map_msgs::RoadSign createDummyRoadSign(const int id, const int vid, const int plid)
{
  vector_map_msgs::RoadSign road_sign;
  road_sign.id = id;
  road_sign.vid = vid;
  road_sign.plid = plid;
  road_sign.type = 1;
  road_sign.linkid = 0;
  return road_sign;
}

vector_map_msgs::Vector createDummyVector(int vid, int pid)
{
  vector_map_msgs::Vector vmap_vector;
  vmap_vector.vid = vid;
  vmap_vector.pid = pid;
  vmap_vector.hang = 0;
  vmap_vector.vang = 0;
  return vmap_vector;
}

void createSignals( const autoware_map::AutowareMapHandler &awmap,
                    std::vector<vector_map_msgs::Signal> &vmap_signals,
                    std::vector<vector_map_msgs::Vector> &vmap_vectors,
                    std::vector<vector_map_msgs::Pole> &vmap_dummy_poles)
{
  std::vector<autoware_map::SignalLight> awmap_signal_lights = awmap.findByFilter( [&](autoware_map::SignalLight sl){return true; });
  std::vector<autoware_map::WaypointRelation> awmap_waypoint_relations = awmap.findByFilter( [&](autoware_map::WaypointRelation wr){return true; });
  std::vector<autoware_map::LaneSignalLightRelation> awmap_lane_signal_relations = awmap.findByFilter( [&](autoware_map::LaneSignalLightRelation lslr){return true; });

  int vector_id = getMaxId(vmap_vectors) + 1;

  for ( auto awmap_signal_light : awmap_signal_lights)
  {
    //create dummy poles if not created yet
    auto pole = std::find_if( vmap_dummy_poles.begin(),
                              vmap_dummy_poles.end(),
                              [&](vector_map_msgs::Pole pole){ return pole.plid == awmap_signal_light.signal_id; });
    if ( pole == vmap_dummy_poles.end()) {
      vector_map_msgs::Vector pole_vector = createDummyVector(vector_id, awmap_signal_light.point_id);
      // pole_vector.vang = 90;
      vmap_vectors.push_back(pole_vector);
      vmap_dummy_poles.push_back(createDummyPole(awmap_signal_light.signal_id, vector_id));
      vector_id++;
    }

    vector_map_msgs::Signal vmap_signal;
    vmap_signal.id = awmap_signal_light.signal_light_id;
    vmap_signal.plid = awmap_signal_light.signal_id;
    //color:{1 = red, 2=green, 3=yellow}
    if(awmap_signal_light.color_type <=3 && awmap_signal_light.arrow_type == 0)
    {
      vmap_signal.type = awmap_signal_light.color_type;
    }
    else{
      vmap_signal.type = 9;       //other
    }

    //create Vector to describe signal direction
    vector_map_msgs::Vector vmap_vector;
    vmap_vector.vid = vector_id;
    vmap_vector.pid = awmap_signal_light.point_id;
    vmap_vector.hang =  convertDecimalToDDMMSS( awmap_signal_light.horizontal_angle );
    vmap_vector.vang =  convertDecimalToDDMMSS( awmap_signal_light.vertical_angle );
    vmap_vectors.push_back(vmap_vector);
    vmap_signal.vid = vector_id;
    vector_id += 1;

    //find link to lane
    auto lane_signal_itr = std::find_if(   awmap_lane_signal_relations.begin(),
                                           awmap_lane_signal_relations.end(),
                                           [awmap_signal_light](autoware_map_msgs::LaneSignalLightRelation lslr){ return lslr.signal_light_id == awmap_signal_light.signal_light_id; });
    autoware_map_msgs::Lane awmap_lane;
    if (lane_signal_itr != awmap_lane_signal_relations.end())
    {
      awmap_lane = awmap.findById<autoware_map::Lane>(lane_signal_itr->lane_id);
    }

    int linkid = 0;
    for(auto itr = awmap_waypoint_relations.begin(); itr != awmap_waypoint_relations.end(); itr++)
    {
      if(itr->next_waypoint_id == awmap_lane.end_waypoint_id)
      {
        auto awmap_waypoint_lane_relations = awmap.findByFilter([&](autoware_map_msgs::WaypointLaneRelation wlr){return wlr.waypoint_id == itr->waypoint_id; });
        for(auto awmap_waypoint_lane_relation : awmap_waypoint_lane_relations)
        {
          if( awmap_waypoint_lane_relation.lane_id == awmap_lane.lane_id)
          {
            linkid = std::distance(awmap_waypoint_relations.begin(), itr) + 1;
          }
        }
      }
    }
    if(linkid == 0)
    {
      ROS_ERROR_STREAM("failed to find valid linkid for signal");
    }

    vmap_signal.linkid = linkid;
    vmap_signals.push_back(vmap_signal);
  }
}

void createStopLines( const autoware_map::AutowareMapHandler &awmap,
                      std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::Point> &vmap_points,
                      std::vector<vector_map_msgs::StopLine> &vmap_stop_lines,
                      std::vector<vector_map_msgs::RoadSign> &vmap_road_signs,
                      std::vector<vector_map_msgs::Vector> &vmap_vectors,
                      std::vector<vector_map_msgs::Pole> &vmap_poles)
{
  const std::vector<autoware_map::WaypointRelation> awmap_waypoint_relations = awmap.findByFilter([&](autoware_map::WaypointRelation ){return true; });
  int line_id = getMaxId(vmap_lines) + 1;
  int point_id = getMaxId(vmap_points) + 1;
  int stop_line_id = getMaxId(vmap_stop_lines) + 1;

  for(auto wp : awmap.findByFilter([] (const autoware_map::Waypoint wp){return wp.stop_line == 1; }))
  {

    autoware_map::Point awmap_pt = *(wp.getPointPtr());
    if(wp.getNextWaypoints().size() == 0) continue;
    autoware_map::Waypoint next_wp = *(wp.getNextWaypoints().front());
    autoware_map::Point awmap_next_pt = *(next_wp.getPointPtr());
    auto next_waypoint_relation = std::find_if(awmap_waypoint_relations.begin(),
                                               awmap_waypoint_relations.end(),
                                               [&](autoware_map_msgs::WaypointRelation wr){return wr.waypoint_id == wp.waypoint_id; });


    double yaw = atan2(awmap_next_pt.y - awmap_pt.y, awmap_next_pt.x - awmap_pt.x);
    double angle_left, angle_right;
    if(isJapaneseCoordinate(awmap_pt.epsg)) {
      angle_left = addAngles(yaw, -M_PI / 2);
      angle_right = addAngles(yaw, M_PI / 2);
    }else
    {
      angle_left = addAngles(yaw, M_PI / 2);
      angle_right = addAngles(yaw, -M_PI / 2);
    }

    vector_map_msgs::Point start_point, end_point;
    start_point.pid = point_id++;

    double r = (wp.left_width > wp.right_width) ? wp.left_width: wp.right_width;
    //stop line cannot be right on waypoint with current rebuild_decision_maker
    double epsilon_x = cos(yaw) * 0.001;
    double epsilon_y = sin(yaw) * 0.001;

    //stop line must intersect with waypoints left side of the line, lengthen left side
    start_point.bx = awmap_pt.x + r * cos(angle_left) + epsilon_x;
    start_point.ly = awmap_pt.y + r * sin(angle_left) + epsilon_y;
    start_point.h = awmap_pt.z;

    end_point.pid = point_id++;
    end_point.bx = awmap_pt.x + r * cos(angle_right) + epsilon_x;
    end_point.ly = awmap_pt.y + r * sin(angle_right) + epsilon_y;
    end_point.h = awmap_pt.z;

    // make sure that stop line does not intersect with other lanes.
    for(auto awmap_wp_relation : awmap_waypoint_relations )
    {
      if(awmap_wp_relation.waypoint_id == wp.waypoint_id || awmap_wp_relation.next_waypoint_id == wp.waypoint_id) {
        continue;
      }
      autoware_map::Waypoint wp1 = *(awmap_wp_relation.getWaypointPtr());
      autoware_map::Waypoint wp2 = *(awmap_wp_relation.getNextWaypointPtr());
      autoware_map::Point p1 = *(wp1.getPointPtr());
      autoware_map::Point p2 = *(wp2.getPointPtr());

      double intersect_x, intersect_y;
      if ( getIntersect(p1.x, p1.y, p2.x, p2.y,
                        start_point.bx, start_point.ly, end_point.bx, end_point.ly,
                        intersect_x, intersect_y))
      {
        double distance = std::hypot( awmap_pt.x - intersect_x, awmap_pt.y - intersect_y );
        r = distance * 0.98;           //shorten length of stop line so that it does not cross any other lanes

        start_point.bx = awmap_pt.x + r * cos(angle_left) + epsilon_x;
        start_point.ly = awmap_pt.y + r * sin(angle_left) + epsilon_y;
        end_point.bx = awmap_pt.x + r * cos(angle_right) + epsilon_x;
        end_point.ly = awmap_pt.y + r * sin(angle_right) + epsilon_y;
      }
    }

    //swap x and y if the coordinate is not in japanese rectangular coordinate system
    if(!isJapaneseCoordinate(awmap_pt.epsg))
    {
      double tmp = start_point.bx;
      start_point.bx = start_point.ly;
      start_point.ly = tmp;
      tmp = end_point.bx;
      end_point.bx = end_point.ly;
      end_point.ly = tmp;
    }

    vmap_points.push_back(start_point);
    vmap_points.push_back(end_point);

    vector_map_msgs::Line vmap_line;
    vmap_line.lid = line_id++;
    vmap_line.bpid = start_point.pid;
    vmap_line.fpid = end_point.pid;
    vmap_line.blid = vmap_line.flid = 0;
    vmap_lines.push_back(vmap_line);

    // only create road sign if stopline is not related to signal
    std::vector<autoware_map::WaypointSignalRelation> wsr_vector = awmap.findByFilter([&](const autoware_map::WaypointSignalRelation wsr){ return wsr.waypoint_id == wp.waypoint_id; });
    vector_map_msgs::StopLine vmap_stop_line;
    vmap_stop_line.id = stop_line_id++;
    vmap_stop_line.lid = vmap_line.lid;
    vmap_stop_line.tlid = 0;
    if( wsr_vector.empty()) {
      int vector_id = getMaxId(vmap_vectors) + 1;
      vmap_vectors.push_back(createDummyVector(vector_id, vmap_line.bpid));

      int pole_id = getMaxId(vmap_poles)+1;
      vmap_poles.push_back(createDummyPole(pole_id, vector_id));

      int road_sign_id = getMaxId(vmap_road_signs) + 1;
      vmap_road_signs.push_back(createDummyRoadSign(road_sign_id, vector_id, pole_id));
      vmap_stop_line.signid = road_sign_id;
    }

    vmap_stop_line.linkid = std::distance(awmap_waypoint_relations.begin(), next_waypoint_relation) + 1;
    vmap_stop_lines.push_back(vmap_stop_line);
  }
}


void createWayAreas(const autoware_map::AutowareMapHandler &awmap, std::vector<vector_map_msgs::WayArea> &vmap_way_areas)
{
  for ( auto awmap_area : awmap.findByFilter( [&](autoware_map::Wayarea wa){return true; }))
  {
    vector_map_msgs::WayArea way_area;
    way_area.waid = awmap_area.wayarea_id;
    way_area.aid = awmap_area.area_id;
    vmap_way_areas.push_back(way_area);
  }
}


void createWayAreasFromLanes(const autoware_map::AutowareMapHandler &awmap,
                             std::vector<vector_map_msgs::WayArea> &vmap_way_areas,
                             std::vector<vector_map_msgs::Area> &vmap_areas,
                             std::vector<vector_map_msgs::Line> &vmap_lines,
                             std::vector<vector_map_msgs::Point> &vmap_points )
{
  const std::vector<autoware_map::WaypointRelation> awmap_waypoint_relations = awmap.findByFilter([&](autoware_map::WaypointRelation wr){return true; });
  int line_id = getMaxId(vmap_lines) + 1;
  int point_id = getMaxId(vmap_points) + 1;
  int area_id = getMaxId(vmap_areas) + 1;
  int wayarea_id = getMaxId(vmap_way_areas) + 1;

  std::unordered_map<int, WaypointWithYaw> wp_yaw_map;
  for(auto relation : awmap_waypoint_relations)
  {
    autoware_map::Waypoint awmap_wp = *(relation.getWaypointPtr());
    autoware_map::Waypoint awmap_next_wp = *(relation.getNextWaypointPtr());
    autoware_map::Point awmap_pt = *(awmap_wp.getPointPtr());
    autoware_map::Point awmap_next_pt = *(awmap_next_wp.getPointPtr());

    // autoware_map_msgs::Waypoint awmap_wp = awmap.findById<autoware_map::Waypoint>(relation.waypoint_id);
    // autoware_map_msgs::Waypoint awmap_next_wp = awmap.findById<autoware_map::Waypoint>(relation.next_waypoint_id);
    // autoware_map_msgs::Point awmap_pt = awmap.findById<autoware_map::Point>(awmap_wp.point_id);
    // autoware_map_msgs::Point awmap_next_pt = awmap.findById<autoware_map::Point>(awmap_next_wp.point_id);

    double yaw = atan2(awmap_next_pt.y - awmap_pt.y, awmap_next_pt.x - awmap_pt.x);

    WaypointWithYaw wp_yaw;
    if(wp_yaw_map.find(relation.waypoint_id) != wp_yaw_map.end()) {
      wp_yaw = wp_yaw_map.at(relation.waypoint_id);
    }
    wp_yaw.waypoint = awmap_wp;
    wp_yaw.point = awmap_pt;
    wp_yaw.yaws.push_back(yaw);
    wp_yaw_map[relation.waypoint_id] = wp_yaw;

    WaypointWithYaw next_wp_yaw;
    if(wp_yaw_map.find(relation.next_waypoint_id) != wp_yaw_map.end()) {
      next_wp_yaw = wp_yaw_map.at(relation.next_waypoint_id);
    }
    next_wp_yaw.waypoint = awmap_next_wp;
    next_wp_yaw.point = awmap_next_pt;
    next_wp_yaw.yaws.push_back(yaw);
    wp_yaw_map[relation.next_waypoint_id] = next_wp_yaw;
  }

  for( auto &item : wp_yaw_map)
  {
    WaypointWithYaw wp_yaw = item.second;
    wp_yaw.yaw_avg = getAngleAverage(wp_yaw.yaws);
    double yaw = wp_yaw.yaw_avg;
    double left_width = wp_yaw.waypoint.left_width;
    double right_width = wp_yaw.waypoint.right_width;

    double angle_left, angle_right;
    if(isJapaneseCoordinate(wp_yaw.point.epsg)) {
      angle_left = addAngles(yaw, -M_PI / 2);
      angle_right = addAngles(yaw, M_PI / 2);
    }else
    {
      angle_left = addAngles(yaw, M_PI / 2);
      angle_right = addAngles(yaw, -M_PI / 2);
    }

    autoware_map_msgs::Point left_point,right_point;
    left_point.x = wp_yaw.point.x + left_width * cos(angle_left);
    left_point.y = wp_yaw.point.y + left_width * sin(angle_left);
    left_point.z = wp_yaw.point.z;
    left_point.point_id = point_id++;
    wp_yaw.left_point = left_point;

    right_point.x = wp_yaw.point.x + right_width * cos(angle_right);
    right_point.y = wp_yaw.point.y + right_width * sin(angle_right);
    right_point.z = wp_yaw.point.z;
    right_point.point_id = point_id++;
    wp_yaw.right_point = right_point;
    item.second = wp_yaw;
  }

  for(auto relation : awmap_waypoint_relations)
  {
    WaypointWithYaw wp_yaw = wp_yaw_map.at(relation.waypoint_id);
    WaypointWithYaw next_wp_yaw = wp_yaw_map.at(relation.next_waypoint_id);

    vector_map_msgs::Point pt1, pt2, pt3, pt4;

    convertPoint(pt1, wp_yaw.left_point);
    pt1.pid = wp_yaw.left_point.point_id;
    convertPoint(pt2, next_wp_yaw.left_point);
    pt2.pid = next_wp_yaw.left_point.point_id;
    convertPoint(pt3, next_wp_yaw.right_point);
    pt3.pid = next_wp_yaw.right_point.point_id;
    convertPoint(pt4, wp_yaw.right_point);
    pt4.pid = wp_yaw.right_point.point_id;

    vmap_points.push_back(pt1);
    vmap_points.push_back(pt2);
    vmap_points.push_back(pt3);
    vmap_points.push_back(pt4);

    vector_map_msgs::Line vmap_line;
    vmap_line.lid = line_id;
    vmap_line.bpid = pt1.pid;
    vmap_line.fpid = pt2.pid;
    vmap_line.blid = 0;
    vmap_line.flid = line_id + 1;
    vmap_lines.push_back(vmap_line);
    line_id++;

    vmap_line.lid = line_id;
    vmap_line.bpid = pt2.pid;
    vmap_line.fpid = pt3.pid;
    vmap_line.blid = line_id - 1;
    vmap_line.flid = line_id + 1;
    vmap_lines.push_back(vmap_line);
    line_id++;

    vmap_line.lid = line_id;
    vmap_line.bpid = pt3.pid;
    vmap_line.fpid = pt4.pid;
    vmap_line.blid = line_id - 1;
    vmap_line.flid = line_id + 1;
    vmap_lines.push_back(vmap_line);
    line_id++;

    vmap_line.lid = line_id;
    vmap_line.bpid = pt4.pid;
    vmap_line.fpid = pt1.pid;
    vmap_line.blid = line_id - 1;
    vmap_line.flid = 0;
    vmap_lines.push_back(vmap_line);
    line_id++;

    vector_map_msgs::Area vmap_area;
    vmap_area.aid = area_id++;
    vmap_area.slid = line_id - 4;
    vmap_area.elid = line_id - 1;
    vmap_areas.push_back(vmap_area);

    vector_map_msgs::WayArea vmap_way_area;
    vmap_way_area.aid = vmap_area.aid;
    vmap_way_area.waid = wayarea_id++;
    vmap_way_areas.push_back(vmap_way_area);
  }
}

//move point to specified direction, where direction is angle from north in clockwise
autoware_map_msgs::Point movePointByDirection(autoware_map::Point point,double direction_rad, double length)
{
  autoware_map_msgs::Point moved_point = point;
  moved_point.x = point.x + length * cos(direction_rad);
  moved_point.y = point.y + length * sin(direction_rad);
  moved_point.z = point.z;
  return moved_point;
}

double getAngleToNextWaypoint(std::shared_ptr<autoware_map::Waypoint> waypoint, int lane_id)
{
  double yaw = 0;
  autoware_map::Waypoint next =*( waypoint->getNextWaypoint(lane_id) );
  yaw = atan2(next.getPointPtr()->y - waypoint->getPointPtr()->y, next.getPointPtr()->x - waypoint->getPointPtr()->x);
  return yaw;
}

void createWhitelines(const autoware_map::AutowareMapHandler &awmap,
                      std::vector<vector_map_msgs::Point> &vmap_points,
                      std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::WhiteLine> &vmap_white_lines)
{
  int white_line_id = getMaxId(vmap_white_lines)+ 1;
  int line_id = getMaxId(vmap_lines)+ 1;
  int point_id = getMaxId(vmap_points) + 1;
  for ( auto awmap_lane : awmap.findByFilter([] (autoware_map::Lane l){return true; }))
  {
    vector_map_msgs::Point current_vmap_point_left,current_vmap_point_right;
    vector_map_msgs::Point prev_vmap_point_left,prev_vmap_point_right;

    auto waypoint = awmap_lane.getStartWaypoint();
    double prev_yaw = getAngleToNextWaypoint(awmap_lane.getStartWaypoint(),awmap_lane.lane_id);
    double next_yaw = getAngleToNextWaypoint(awmap_lane.getStartWaypoint(),awmap_lane.lane_id);

    for(auto waypoint : awmap_lane.getWaypoints())
    {
      autoware_map_msgs::WaypointRelation tmp;
      if(waypoint->waypoint_id == awmap_lane.end_waypoint_id ) {
        next_yaw = prev_yaw;
      }else{
        next_yaw = getAngleToNextWaypoint(waypoint,awmap_lane.lane_id);
      }

      double yaw = prev_yaw;
      double angle_left, angle_right;
      if(isJapaneseCoordinate(waypoint->getPointPtr()->epsg)) {
        angle_left = addAngles(yaw, -M_PI / 2);
        angle_right = addAngles(yaw, M_PI / 2);
      }else
      {
        angle_left = addAngles(yaw, M_PI / 2);
        angle_right = addAngles(yaw, -M_PI / 2);
      }

      autoware_map_msgs::Point awmap_right_pt = movePointByDirection(*(waypoint->getPointPtr()), angle_right,waypoint->right_width);
      autoware_map_msgs::Point awmap_left_pt = movePointByDirection(*(waypoint->getPointPtr()), angle_left,waypoint->left_width);
      // autoware_map_msgs::Point awmap_right_pt = movePointByDirection(*(waypoint->getPointPtr()), angle_right,1);
      // autoware_map_msgs::Point awmap_left_pt = movePointByDirection(*(waypoint->getPointPtr()), angle_left,1);
      awmap_right_pt.point_id = point_id++;
      awmap_left_pt.point_id = point_id++;
      convertPoint(current_vmap_point_right,awmap_right_pt);
      convertPoint(current_vmap_point_left,awmap_left_pt);
      current_vmap_point_right.pid = awmap_right_pt.point_id;
      current_vmap_point_left.pid = awmap_left_pt.point_id;
      vmap_points.push_back(current_vmap_point_right);
      vmap_points.push_back(current_vmap_point_left);

      //continue if this is initial point of lane
      if(prev_vmap_point_left.pid == 0 || prev_vmap_point_right.pid == 0) {
        prev_vmap_point_left = current_vmap_point_left;
        prev_vmap_point_right = current_vmap_point_right;
        prev_yaw = next_yaw;
        continue;
      }

      //right line
      vector_map_msgs::Line vmap_line_right;
      vmap_line_right.lid = line_id;
      vmap_line_right.bpid = prev_vmap_point_right.pid;
      vmap_line_right.fpid = current_vmap_point_right.pid;
      vmap_line_right.blid = vmap_line_right.flid = 0;
      vmap_lines.push_back(vmap_line_right);

      vector_map_msgs::WhiteLine vmap_white_line_right;
      vmap_white_line_right.id = white_line_id++;
      vmap_white_line_right.lid = line_id++;
      vmap_white_line_right.width = 0.1;
      if(awmap_lane.lane_number == 0)       //intersection
      {
        vmap_white_line_right.color = 'W';
        vmap_white_line_right.type =vector_map_msgs::WhiteLine::DASHED_LINE_BLANK;
      }else if(awmap_lane.lane_number == awmap_lane.num_of_lanes)
      {
        vmap_white_line_right.color = 'Y';
        vmap_white_line_right.type = vector_map_msgs::WhiteLine::SOLID_LINE;
      }
      else{
        vmap_white_line_right.color = 'W';
        vmap_white_line_right.type =vector_map_msgs::WhiteLine::DASHED_LINE_SOLID;
      }
      vmap_white_line_right.linkid = 0;
      vmap_white_lines.push_back(vmap_white_line_right);

      //left line
      vector_map_msgs::Line vmap_line_left;
      vmap_line_left.lid = line_id;
      vmap_line_left.bpid = prev_vmap_point_left.pid;
      vmap_line_left.fpid = current_vmap_point_left.pid;
      vmap_line_left.blid = vmap_line_left.flid = 0;
      vmap_lines.push_back(vmap_line_left);

      vector_map_msgs::WhiteLine vmap_white_line_left;
      vmap_white_line_left.id = white_line_id++;
      vmap_white_line_left.lid = line_id++;
      vmap_white_line_left.width = 0.1;
      if(awmap_lane.lane_number == 0)
      {
        vmap_white_line_left.color = 'W';
        vmap_white_line_left.type =vector_map_msgs::WhiteLine::DASHED_LINE_BLANK;
      }
      else if(awmap_lane.lane_number == 1)
      {
        vmap_white_line_left.color = 'W';
        vmap_white_line_left.type = vector_map_msgs::WhiteLine::SOLID_LINE;
      }
      else{
        vmap_white_line_left.color = 'W';
        vmap_white_line_left.type =vector_map_msgs::WhiteLine::DASHED_LINE_SOLID;
      }
      vmap_white_line_left.linkid = 0;
      vmap_white_lines.push_back(vmap_white_line_left);

      //update previous points
      prev_vmap_point_left = current_vmap_point_left;
      prev_vmap_point_right = current_vmap_point_right;
      prev_yaw = next_yaw;

    }
  }
}

void createDummyUtilityPoles(const std::vector<vector_map_msgs::Pole> vmap_poles,
                             std::vector<vector_map_msgs::UtilityPole> &vmap_utility_poles)
{
    int id = 1;
    for(auto pole: vmap_poles)
    {
      vector_map_msgs::UtilityPole utility_pole;
      utility_pole.id = id++;
      utility_pole.plid = pole.plid;
      utility_pole.linkid = 0;
      vmap_utility_poles.push_back(utility_pole);
    }
}

//keep angles within (M_PI, -M_PI]
double addAngles(double angle1, double angle2)
{
  double sum = angle1 + angle2;
  while( sum > M_PI ) sum -= 2 * M_PI;
  while( sum <= -M_PI ) sum += 2 * M_PI;
  return sum;
}

double angleAverage(double angle1, double angle2)
{
  double x1, x2, y1, y2;
  x1 = cos(angle1);
  y1 = sin(angle1);
  x2 = cos(angle2);
  y2 = sin(angle2);
  return atan2((y2 + y1)/2, (x2 + x1)/2);
}

double getAngleAverage(const std::vector<double> angles)
{
  autoware_map_msgs::Point avg, sum;
  for ( double angle : angles)
  {
    sum.x += cos(angle);
    sum.y += sin(angle);
  }
  avg.x = sum.x / angles.size();
  avg.y = sum.y / angles.size();
  return atan2(avg.y, avg.x);
}

double convertDecimalToDDMMSS(const double decimal)
{
  int degree, minutes,seconds;
  degree = floor(decimal);
  minutes = floor( (decimal - degree ) * 60);
  seconds = floor( (decimal - degree - minutes * 1.0 / 60) * 3600);
  return degree + minutes * 0.01 + seconds * 0.0001;
}


void getMinMax(autoware_map_msgs::Point &min, autoware_map_msgs::Point &max, const std::vector<autoware_map_msgs::Point>points)
{
  min = max = points.front();
  for (auto pt : points)
  {
    min.x = min.x < pt.x ? min.x : pt.x;
    min.y = min.y < pt.y ? min.y : pt.y;
    min.z = min.z < pt.z ? min.z : pt.z;
    max.x = max.x > pt.x ? max.x : pt.x;
    max.y = max.y > pt.y ? max.y : pt.y;
    max.z = max.z > pt.z ? max.z : pt.z;
  }
}

//determine whether point lies within an area by counting winding number
bool isWithinArea(double x, double y, const std::vector<autoware_map_msgs::Point> vertices)
{
  std::vector<double> angles;
  for (auto pt : vertices)
  {
    if(pt.x == x && pt.y== y) return false;
    angles.push_back( atan2(pt.y - y, pt.x - x));
  }

  double angle_sum = 0;
  for (unsigned int idx = 0; idx < vertices.size(); idx++)
  {
    double angle1, angle2;
    angle1 = angles.at(idx);
    if(idx + 1 >= vertices.size())
    {
      angle2 = angles.front();
    }else
    {
      angle2 = angles.at(idx + 1);
    }
    double angle_diff = addAngles(angle1, -angle2);
    angle_sum += angle_diff;
  }
  //allow some precision error
  if(fabs(angle_sum) < 1e-3) return false;
  else return true;
}

bool getIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double &intersect_x, double &intersect_y )
{
  //let p1(x1, y1), p2(x2, y2), p3(x3, y3), p4(x4,y4)
  //intersect of line segment p1 to p2 and p3 to p4 satisfies
  // p1 + r(p2 - p1) = p3 + s(p4 - p3)
  // 0 <= r <= 1
  // 0 <= s <= 1
  double denominator = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

  if(denominator == 0) {
    //line is parallel
    return false;
  }

  double r = ( (y4 - y3) * (x3 - x1) - (x4 - x3) * (y3 - y1) ) / denominator;
  double s = ( (y2 - y1) * (x3 - x1) - (x2 - x1) * (y3 - y1) ) / denominator;

  if( r >= 0 && r <= 1 && s >= 0 && s <= 1) {
    intersect_x = x1 + r * (x2 - x1);
    intersect_y = y1 + r * (y2 - y1);
    return true;
  }else{
    return false;
  }
}

int convertESPGToRef(int epsg)
{
  if(epsg >= 2443 && epsg <= 2461)
  {
    return epsg - 2442;
  }
  if(epsg >= 6669 && epsg <= 6687)
  {
    return epsg - 6668;
  }
  return 0;
}


int getMaxId(std::vector<vector_map_msgs::Point> points)
{
  int max = 0;
  for (auto p : points)
  {
    if (p.pid > max) {
      max = p.pid;
    }
  }
  return max;
}

int getMaxId(std::vector<vector_map_msgs::Line> lines)
{
  int max = 0;
  for (auto l : lines)
  {
    if (l.lid > max) {
      max = l.lid;
    }
  }
  return max;
}
int getMaxId(std::vector<vector_map_msgs::StopLine> stop_lines)
{
  int max = 0;
  for (auto l : stop_lines)
  {
    if (l.id > max) {
      max = l.id;
    }
  }
  return max;
}
int getMaxId(std::vector<vector_map_msgs::RoadSign> signs)
{
  int max = 0;
  for (auto s : signs)
  {
    if (s.id > max) {
      max = s.id;
    }
  }
  return max;
}

int getMaxId(std::vector<vector_map_msgs::Area> areas)
{
  int max = 0;
  for (auto a : areas)
  {
    if (a.aid > max) {
      max = a.aid;
    }
  }
  return max;
}

int getMaxId(const std::vector<vector_map_msgs::Vector> vectors)
{
  int max = 0;
  for (auto v : vectors)
  {
    if (v.vid > max) {
      max = v.vid;
    }
  }
  return max;
}

int getMaxId(const std::vector<vector_map_msgs::Pole> poles)
{
  int max = 0;
  for (auto p : poles)
  {
    if (p.plid > max) {
      max = p.plid;
    }
  }
  return max;
}
int getMaxId(const std::vector<vector_map_msgs::WhiteLine> white_lines)
{
  int max = 0;
  for (auto w : white_lines)
  {
    if (w.id > max) {
      max = w.id;
    }
  }
  return max;
}
int getMaxId(const std::vector<vector_map_msgs::WayArea> wayareas)
{
  int max = 0;
  for (auto w : wayareas)
  {
    if (w.waid > max) {
      max = w.waid;
    }
  }
  return max;
}
