/*
 * Copyright (C) 2016, DFKI GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of DFKI GmbH nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Martin Günther <martin.guenther@dfki.de>
 *         Jochen Sprickerhof <ros@jochen.sprickerhof.de>
 *
 * Based on LD-MRS example code by SICK AG.
 *
 */

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sick_ldmrs_msgs/sick_ldmrs_point_type.h>
#include <pcl/point_cloud.h>

typedef std::vector<std::pair<double, double> > ResolutionMap;
typedef sick_ldmrs_msgs::SICK_LDMRS_Point PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#ifndef deg2rad
#define deg2rad 0.01745329251994329576923690768   // (PI / 180.0)
#endif
#ifndef rad2deg
#define rad2deg 57.29577951308232087679815481   // (180.0 / PI)
#endif

std::string doubleToString(double val,
                           int digits_after_decimal_point)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(digits_after_decimal_point) << val;

  return ss.str();
}

inline bool almostEqual(double a, double b)
{
  return std::abs(a - b) < 1e-6f;
}

inline double getHAngle(const PointT &p)
{
  return atan2(p.y, p.x);
}

std::string sectorToString(const ResolutionMap& sector)
{
  size_t i = 0;
  std::ostringstream oss;
  oss << std::endl;
  for (ResolutionMap::const_iterator s = sector.begin(); s != sector.end(); ++s)
  {
    oss  << "Sector (" << i << "): start=" << doubleToString(s->first * rad2deg, 3) << " (= " << (int)(round(s->first * rad2deg * 32.0)) << ") res=" << doubleToString(s->second * rad2deg, 3) << std::endl;
    i++;
  }

  return oss.str();
}

PointCloudT::const_iterator getNextPointInSameLayer(PointCloudT::const_iterator iter, const PointCloudT::ConstPtr cloud)
{
  PointCloudT::const_iterator ret = iter;
  for (++iter /*start with the next point*/ ; iter != cloud->end(); ++iter)
  {
    if (iter->layer == ret->layer)
    {
      ret = iter;
      break;
    }
  }
  return ret;

}

PointCloudT::const_iterator getNextPoint(PointCloudT::const_iterator iter, const PointCloudT::ConstPtr cloud)
{
  PointCloudT::const_iterator ret = iter;
  ++iter;
  if (iter != cloud->end())
  {
    ret = iter;
  }
  return ret;

}

void checkResolution(const PointCloudT::ConstPtr cloud)
{
  if (cloud->size() < 10)
  {
    // do not process on scans with too few scan points
    return;
  }

  // iterate through all scan points and make a diff of the angles
  PointCloudT::const_iterator p;
  double angleDiff; // compute differences

  ResolutionMap sectors; // first = start angle, second = resolution

  for (p = cloud->begin(); p != cloud->end(); ++p)
  {
    PointCloudT::const_iterator p2 = getNextPoint(p, cloud);
    PointCloudT::const_iterator p3 = getNextPointInSameLayer(p, cloud);
    double interlaced = 1.;
    if (almostEqual(getHAngle(*p2), getHAngle(*p3)))
    {
      // we are close to the border -> only 2 scan layers left
      interlaced = 0.5;

    }
    angleDiff = std::fabs(getHAngle(*p2) - getHAngle(*p)) * interlaced;

    if (angleDiff > 1e-6f)
    {
      if (almostEqual(angleDiff, 0.125 * deg2rad)
          || almostEqual(angleDiff, 0.25 * deg2rad)
          || almostEqual(angleDiff, 0.5 * deg2rad)
          || almostEqual(angleDiff, 1.0 * deg2rad))
      {
        if (sectors.size() == 0)
        {
          sectors.push_back(std::make_pair(getHAngle(*p), angleDiff));
        }
        else
        {
          // we detected a new resolution
          if (almostEqual(double(sectors.back().second), angleDiff) == false)
          {
            sectors.push_back(std::make_pair(getHAngle(*p), angleDiff));
          }
        }
      }
    }
  }

  // filter out sectors < 3° or with same resolution as next sector
  ResolutionMap sectors_filtered;
  bool changed = true;
  while (changed)
  {
    changed = false;
    for (size_t i = 0; i < sectors.size() - 1; ++i)
    {
      if (sectors[i].first - sectors[i + 1].first > (3.0 + 1e-6f) * deg2rad &&
          !almostEqual(sectors[i].second, sectors[i + 1].second))
      {
        sectors_filtered.push_back(sectors[i]);
      }
      else
      {
        sectors[i + 1].first = sectors[i].first;
        changed = true;
      }
    }
    sectors_filtered.push_back(sectors.back());
    sectors.swap(sectors_filtered);
    sectors_filtered.clear();
  }

  std::ostringstream oss;
  oss << "Angular resolutions:" << sectorToString(sectors);
  ROS_INFO("%s", oss.str().c_str());
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{

  PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>();
  pcl::fromROSMsg(*pc, *cloud);

  checkResolution(cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_ldmrs_print_resolution");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("cloud", 1, callback);

  ros::spin();

  return 0;
}
