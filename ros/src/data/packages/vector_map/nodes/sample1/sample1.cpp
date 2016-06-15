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

#include <ros/console.h>
#include <vector_map/vector_map.h>

using vector_map::VectorMap;
using vector_map::Category;
using vector_map::Key;
using vector_map::Point;
using vector_map::Vector;
using vector_map::Signal;

namespace
{
VectorMap g_vmap;

bool isRed(const Signal& signal)
{
  return signal.type == Signal::RED;
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample1");

  ros::NodeHandle nh;
  g_vmap.subscribe(nh, Category::POINT | Category::VECTOR | Category::SIGNAL);

  ROS_INFO_STREAM("@@@ Point entries [1-5]");
  ROS_INFO_STREAM("PID,B,L,H,Bx,Ly,ReF,MCODE1,MCODE2,MCODE3");
  ROS_INFO_STREAM(g_vmap.findByKey(Key<Point>(1)));
  ROS_INFO_STREAM(g_vmap.findByKey(Key<Point>(2)));
  ROS_INFO_STREAM(g_vmap.findByKey(Key<Point>(3)));
  std::vector<Point> points = g_vmap.findByFilter([](const Point& point){return point.pid >= 4 && point.pid <= 5;});
  for (const auto& p : points)
    ROS_INFO_STREAM(p);

  ROS_INFO_STREAM("@@@ Red signal coodinates");
  ROS_INFO_STREAM("Bx,Ly");
  std::vector<Signal> signals = g_vmap.findByFilter(isRed);
  for (const auto& signal : signals) {
    Vector vector = g_vmap.findByKey(Key<Vector>(signal.vid));
    Point point = g_vmap.findByKey(Key<Point>(vector.pid));
    ROS_INFO_STREAM(point.bx << "," << point.ly);
  }

  return EXIT_SUCCESS;
}
