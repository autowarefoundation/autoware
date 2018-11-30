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

#include "costmap_generator.h"


// Constructor
CostmapGenerator::CostmapGenerator() :
  use_vectormap_(true),
  use_waypoint_(true),
  private_nh_("~")
{
}

CostmapGenerator::~CostmapGenerator() {}

void CostmapGenerator::init()
{
}

void CostmapGenerator::run()
{
  pub_costmap_ = nh_.advertise<grid_map_msgs::GridMap>("/semantics/costmap", 1);

  // if(use_vectormap_)
  // {
  //   sub_vectormap_ = nh_.subscribe("/poitns_no_ground", 1, &CostmapGenerator::pointsCallback, this);
  // }


  if(!use_objects_)
  {
    // sub_points_ = nh_.subscribe("/poitns_no_ground", 1, &CostmapGenerator::pointsCallback, this);
  }
  else
  {
    sub_sync_points_ptr_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/points_no_ground", 1));
    sub_sync_objects_ptr_.reset(
      new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(nh_, "/detection/lidar_tracker/objects", 1));
    sync_ptr_.reset(
      new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,
      autoware_msgs::DetectedObjectArray>
      (*sub_sync_points_ptr_, *sub_sync_objects_ptr_, 10));

    sync_ptr_->registerCallback(boost::bind(&CostmapGenerator::syncedCallback, this, _1, _2));
  }
}

void CostmapGenerator::syncedCallback(const sensor_msgs::PointCloud2::ConstPtr& in_points,
                                const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects)
{
  // if(checkSubscripton())
  // generatePointsCostmap()
  // generateObjectsCostmap()
  // generateVectormapCostmap()
  // generateWaypointCostmap()
  // generateCombinedCostmap()
  // pub(costmap)
}
