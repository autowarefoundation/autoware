/*
 *  Copyright (c) 2018, Nagoya University
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
 ********************/

#include "objects2costmap.h"


// Constructor
Objects2Costmap::Objects2Costmap()
{
}

Objects2Costmap::~Objects2Costmap() {}

grid_map::Polygon makePolygonFromObject(const autoware_msgs::DetectedObject& object)
{
  //forward_right, forward_left, backward_right, backward_left
  grid_map::Position forward_right
  grid_map::Polygon polygon;
  polygon.setFrameId(object.header.frame_id);
  polygon.addVertex(Position( 0.480,  0.000));
  // use amc's code for generating 4 points
  return polygon;
}

void setCostForPolygon(const grid_map::Polygon& polygon,const std::string& gridmap_layer_name,
                       grid_map::GridMap& objects_costmap)
{
  for (grid_map::PolygonIterator iterator(objects_costmap, polygon);
    !iterator.isPastEnd(); ++iterator)
  {
    // change cost depending on object.score?
    objects_costmap.at(gridmap_layer_name, *iterator) = 1.0;
  }
}

grid_map::GridMap Objects2Costmap::makeCostmapFromObjects(const grid_map::GridMap& costmap,
                                                            const std::string& gridmap_layer_name,
                                                            const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects)
{
  grid_map::GridMap objects_costmap = costmap;
  for (const auto& object: in_objects->objects)
  {
    // grid_map::Polygon polygon = makePolygonFromObject(object);
    // calculateCostForPolygon(polygon, gridmap_layer_name, objects_costmap);
  }
  return objects_costmap;
}
