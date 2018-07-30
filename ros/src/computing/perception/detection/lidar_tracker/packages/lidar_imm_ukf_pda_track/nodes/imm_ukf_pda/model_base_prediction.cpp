#include <stack>

#include "model_base_prediction.h"

ModelBasePrediction::ModelBasePrediction()
{
  num_path_points_ = 10;

  // // initial state vector
  // x_p_ = Eigen::MatrixXd(4,1);
  //
  // // initial state vector
  // x_l_ = Eigen::MatrixXd(4,1);

  // initial state vector
  x_p_ = Eigen::VectorXd(4);

  // initial state vector
  x_l_ = Eigen::VectorXd(4);

  // initial covariance matrix
  p_p_ = Eigen::MatrixXd(4, 4);
}

void ModelBasePrediction::adasMapAssitDirectionAndPrediction(const autoware_msgs::DetectedObjectArray& input,
                                                             const tf::TransformListener &tf_listen_,
                                              std::vector<IMM_RAUKF> &targets,
                                              visualization_msgs::MarkerArray &directionMarkers,
                                              visualization_msgs::MarkerArray &predictionMarkers)
{
  for (size_t i = 0; i < targets.size(); i++)
  {
    if (targets[i].is_vis_bb_ )
    {
      geometry_msgs::PoseStamped world_pose, map_pose, vel_pose;
      world_pose.header.stamp = input.header.stamp;
      world_pose.header.frame_id = "/world";
      world_pose.pose   = targets[i].jsk_bb_.pose;
      tf::Matrix3x3 mat;
      mat.setEulerYPR(targets[i].x_merge_(3), 0, 0);  // yaw, pitch, roll
      tf::Quaternion quat;
      mat.getRotation(quat);
      world_pose.pose.orientation.x = quat.getX();
      world_pose.pose.orientation.y = quat.getY();
      world_pose.pose.orientation.z = quat.getZ();
      world_pose.pose.orientation.w = quat.getW();

      // tf::Quaternion q(targets[i].jsk_bb_.pose.orientation.x,
      //   targets[i].jsk_bb_.pose.orientation.y,
      //   targets[i].jsk_bb_.pose.orientation.z,
      //   targets[i].jsk_bb_.pose.orientation.w);
      // double roll, pitch, yaw;
      // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
      // std::cout << "world "<< yaw << std::endl;
      tf_listen_.waitForTransform("/world", "/map", input.header.stamp, ros::Duration(1.0));
      tf_listen_.transformPose("/map", ros::Time(0), world_pose, "world", map_pose);
      // tf::Quaternion q2(map_pose.pose.orientation.x,
      //                  map_pose.pose.orientation.y,
      //                  map_pose.pose.orientation.z,
      //                  map_pose.pose.orientation.w);
      // double roll2, pitch2, yaw2;
      // tf::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);
      // std::cout << "map "<< map_pose.pose << std::endl;

      if(targets[i].objectPaths.size() == 0)
      {
        // todo: not init objectPaths if target is in interaction or nearest node(fnid) is branching node
        // todo: multiple paths : now only one branching path can be make
        initObjectPaths(map_pose, targets[i]);
      }
      else
      {
        for(size_t ith_path = 0; ith_path < targets[i].objectPaths.size(); ith_path++)
        {
          // double adas_yaw = 0;
          vector_map::Point fp = vmap_.findByKey(vector_map::Key<vector_map::Point>( targets[i].objectPaths[ith_path][0].fpid));
          double px = map_pose.pose.position.x;
          double py = map_pose.pose.position.y;
          double dist = ((py - fp.bx) * (py - fp.bx) + (px - fp.ly) * (px - fp.ly));
          // double bp2fp_yaw = atan2(fp.bx - bp.bx, fp.ly - bp.ly);

          if(dist < 1.0)
          {
            if(ith_path == 0)
            {
              double adas_yaw = targets[i].objectPaths[0][0].direction;

              tf::Matrix3x3 mat;
              mat.setEulerYPR(adas_yaw, 0, 0);  // yaw, pitch, roll
              tf::Quaternion quat;
              mat.getRotation(quat);

              // map_pose.pose.orientation.x = quat.getX();
              // map_pose.pose.orientation.y = quat.getY();
              // map_pose.pose.orientation.z = quat.getZ();
              // map_pose.pose.orientation.w = quat.getW();


              // visualization_msgs::Marker arrow;
              // arrow.lifetime = ros::Duration(0.1);
              // arrow.header.frame_id = "map";
              // arrow.header.stamp = input.header.stamp;
              // arrow.ns = "arrow";
              // arrow.action = visualization_msgs::Marker::ADD;
              // arrow.type = visualization_msgs::Marker::ARROW;
              // // green
              // arrow.color.g = 1.0f;
              // arrow.color.a = 1.0;
              //
              // arrow.color.r = 1.0;
              // arrow.id = i*10+ith_path;
              //
              // // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
              // arrow.pose = map_pose.pose;
              //
              // // Set the scale of the arrow -- 1x1x1 here means 1m on a side
              // arrow.scale.x = 3;
              // arrow.scale.y = 0.5;
              // arrow.scale.z = 0.5;
              //
              // directionMarkers.markers.push_back(arrow);
            }

            //add new lane point
            vector_map::Lane last_lane = vmap_.findByKey(vector_map::Key<vector_map::Lane>(
                                                        targets[i].objectPaths[ith_path][num_path_points_ - 1].lnid));
            vector_map::Lane next_lane = vmap_.findByKey(vector_map::Key<vector_map::Lane>(last_lane.flid));
            vector_map::DTLane  dtlane = vmap_.findByKey(vector_map::Key<vector_map::DTLane>(next_lane.did));
            vector_map::Node  bn       = vmap_.findByKey(vector_map::Key<vector_map::Node>(next_lane.bnid));
            vector_map::Point bp       = vmap_.findByKey(vector_map::Key<vector_map::Point>(bn.pid));
            vector_map::Node  fn       = vmap_.findByKey(vector_map::Key<vector_map::Node>(next_lane.fnid));
            vector_map::Point fp       = vmap_.findByKey(vector_map::Key<vector_map::Point>(fn.pid));

            double direction = atan2(fp.bx - bp.bx, fp.ly - bp.ly);

            double l_direction = direction + M_PI/2;
            while (l_direction > M_PI)
              l_direction -= 2. * M_PI;
            while (l_direction < -M_PI)
              l_direction += 2. * M_PI;

            double r_direction = direction - M_PI/2;
            while (r_direction > M_PI)
              r_direction -= 2. * M_PI;
            while (r_direction < -M_PI)
              r_direction += 2. * M_PI;

            double l_ly = cos(l_direction)*dtlane.lw + bp.ly;
            double l_bx = sin(l_direction)*dtlane.lw + bp.bx;

            // std::cout << "left " << ((l_bx - bp.bx) * (l_bx - bp.bx) + (l_ly - bp.ly) * (l_ly - bp.ly)) << std::endl;

            double r_ly = cos(r_direction)*dtlane.rw + bp.ly;
            double r_bx = sin(r_direction)*dtlane.rw + bp.bx;

            // std::cout << "right " << ((r_bx - bp.bx) * (r_bx - bp.bx) + (r_ly - bp.ly) * (r_ly - bp.ly)) << std::endl;

            geometry_msgs::PoseStamped map_fp_pose;
            map_fp_pose.header = map_pose.header;
            map_fp_pose.pose.position.x = fp.ly;
            map_fp_pose.pose.position.y = fp.bx;
            map_fp_pose.pose.position.z = fp.h;
            map_fp_pose.pose.orientation.x = 0.0;
            map_fp_pose.pose.orientation.y = 0.0;
            map_fp_pose.pose.orientation.z = 0.0;
            map_fp_pose.pose.orientation.w = 1.0;

            LanePoint lp;
            lp.lnid         = next_lane.lnid;
            lp.fpid         = fn.pid;
            lp.direction    = direction;
            lp.curvature    = 1/dtlane.r;
            lp.lw           = dtlane.lw;
            lp.rw           = dtlane.rw;
            lp.lw_ly        = l_ly;
            lp.lw_bx        = l_bx;
            lp.rw_ly        = r_ly;
            lp.rw_bx        = r_bx;
            lp.map_fp_pose  = map_fp_pose;

            // std::cout << "might be about to die1 " << std::endl;
            // add another lane if there is
            if(last_lane.jct == 1 || last_lane.jct == 2)
            {
              // std::cout << "might be about to die2" << std::endl;
              std::vector<LanePoint> new_path = targets[i].objectPaths[ith_path];

              vector_map::Lane next_lane = vmap_.findByKey(vector_map::Key<vector_map::Lane>(last_lane.flid2));
              vector_map::DTLane  dtlane = vmap_.findByKey(vector_map::Key<vector_map::DTLane>(next_lane.did));
              vector_map::Node  bn       = vmap_.findByKey(vector_map::Key<vector_map::Node>(next_lane.bnid));
              vector_map::Point bp       = vmap_.findByKey(vector_map::Key<vector_map::Point>(bn.pid));
              vector_map::Node  fn       = vmap_.findByKey(vector_map::Key<vector_map::Node>(next_lane.fnid));
              vector_map::Point fp       = vmap_.findByKey(vector_map::Key<vector_map::Point>(fn.pid));
              double direction = atan2(fp.bx - bp.bx, fp.ly - bp.ly);

              double l_direction = direction + M_PI/2;
              while (l_direction > M_PI)
                l_direction -= 2. * M_PI;
              while (l_direction < -M_PI)
                l_direction += 2. * M_PI;

              double r_direction = direction - M_PI/2;
              while (r_direction > M_PI)
                r_direction -= 2. * M_PI;
              while (r_direction < -M_PI)
                r_direction += 2. * M_PI;

              double l_ly = cos(l_direction)*dtlane.lw + bp.ly;
              double l_bx = sin(l_direction)*dtlane.lw + bp.bx;

              double r_ly = cos(r_direction)*dtlane.rw + bp.ly;
              double r_bx = sin(r_direction)*dtlane.rw + bp.bx;



              geometry_msgs::PoseStamped map_fp_pose;
              map_fp_pose.header = map_pose.header;
              map_fp_pose.pose.position.x = fp.ly;
              map_fp_pose.pose.position.y = fp.bx;
              map_fp_pose.pose.position.z = fp.h;
              map_fp_pose.pose.orientation.x = 0.0;
              map_fp_pose.pose.orientation.y = 0.0;
              map_fp_pose.pose.orientation.z = 0.0;
              map_fp_pose.pose.orientation.w = 1.0;

              LanePoint lp;
              lp.lnid         = next_lane.lnid;
              lp.fpid         = fn.pid;
              lp.direction    = direction;
              lp.curvature    = 1/dtlane.r;
              lp.lw           = dtlane.lw;
              lp.rw           = dtlane.rw;
              lp.lw_ly        = l_ly;
              lp.lw_bx        = l_bx;
              lp.rw_ly        = r_ly;
              lp.rw_bx        = r_bx;
              lp.map_fp_pose  = map_fp_pose;

              new_path.push_back(lp);
              new_path.erase(new_path.begin());
              targets[i].objectPaths.push_back(new_path);
            }

            targets[i].objectPaths[ith_path].push_back(lp);
            targets[i].objectPaths[ith_path].erase(targets[i].objectPaths[ith_path].begin());

            maneuverRecognition(map_pose, targets[i]);
            // std::cout << "num path " <<  targets[i].objectPaths.size() << std::endl;
          }
        }

        // for (const auto &lane : lanes_)
        // {
        //   vector_map::Node  bn = vmap_.findByKey(vector_map::Key<vector_map::Node>(lane.bnid));
        //   vector_map::Point bp = vmap_.findByKey(vector_map::Key<vector_map::Point>(bn.pid));
        //   vector_map::Node  fn = vmap_.findByKey(vector_map::Key<vector_map::Node>(lane.fnid));
        //   vector_map::Point fp = vmap_.findByKey(vector_map::Key<vector_map::Point>(fn.pid));
        //   // ROS_INFO(" lane bn=(%f,%f) fn=(%f,%f)", bp.ly, bp.bx, fp.ly, fp.bx);
        //
        //   double adas_yaw = 0;
        //   double px = map_pose.pose.position.x;
        //   double py = map_pose.pose.position.y;
        //   double dist = ((py - fp.bx) * (py - fp.bx) + (px - fp.ly) * (px - fp.ly));
        //   // double bp2fp_yaw = atan2(fp.bx - bp.bx, fp.ly - bp.ly);
        //
        //   if(dist < 3.0)
        //   {
            // not branching path
            // if(targets[i].objectPaths[0][0].fpid == targets[i].objectPaths[1][0].fpid)
            // {
              // pick up latest direction
              // adas_yaw = targets[i].objectPaths[0][0].direction;
              //
              // tf::Matrix3x3 mat;
              // mat.setEulerYPR(adas_yaw, 0, 0);  // yaw, pitch, roll
              // tf::Quaternion quat;
              // mat.getRotation(quat);
              //
              // map_pose.pose.orientation.x = quat.getX();
              // map_pose.pose.orientation.y = quat.getY();
              // map_pose.pose.orientation.z = quat.getZ();
              // map_pose.pose.orientation.w = quat.getW();
              //
              //
              // visualization_msgs::Marker arrow;
              // arrow.lifetime = ros::Duration(0.1);
              // arrow.header.frame_id = "map";
              // arrow.header.stamp = input.header.stamp;
              // arrow.ns = "arrow";
              // arrow.action = visualization_msgs::Marker::ADD;
              // arrow.type = visualization_msgs::Marker::ARROW;
              // // green
              // arrow.color.g = 1.0f;
              // arrow.color.a = 1.0;
              //
              // arrow.color.r = 1.0;
              // arrow.id = i;
              //
              // // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
              // arrow.pose = map_pose.pose;
              //
              // // Set the scale of the arrow -- 1x1x1 here means 1m on a side
              // arrow.scale.x = 3;
              // arrow.scale.y = 0.5;
              // arrow.scale.z = 0.5;
              //
              // directionMarkers.markers.push_back(arrow);

              // // if fnid is next lane point
              // if(fn.pid == targets[i].objectPaths[0][0].fpid)
              // {
              //   //add new lane point
              //   vector_map::Lane last_lane = vmap_.findByKey(vector_map::Key<vector_map::Lane>(
              //                                                               targets[i].objectPaths[0][numPathPoints - 1].lnid));
              //   vector_map::Lane next_lane = vmap_.findByKey(vector_map::Key<vector_map::Lane>(last_lane.flid));
              //   vector_map::Node  bn       = vmap_.findByKey(vector_map::Key<vector_map::Node>(next_lane.bnid));
              //   vector_map::Point bp       = vmap_.findByKey(vector_map::Key<vector_map::Point>(bn.pid));
              //   vector_map::Node  fn       = vmap_.findByKey(vector_map::Key<vector_map::Node>(next_lane.fnid));
              //   vector_map::Point fp       = vmap_.findByKey(vector_map::Key<vector_map::Point>(fn.pid));
              //   double direction = atan2(fp.bx - bp.bx, fp.ly - bp.ly);
              //
              //   geometry_msgs::PoseStamped map_fp_pose;
              //   map_fp_pose.header = map_pose.header;
              //   map_fp_pose.pose.position.x = fp.ly;
              //   map_fp_pose.pose.position.y = fp.bx;
              //   map_fp_pose.pose.position.z = fp.h;
              //   map_fp_pose.pose.orientation.x = 0.0;
              //   map_fp_pose.pose.orientation.y = 0.0;
              //   map_fp_pose.pose.orientation.z = 0.0;
              //   map_fp_pose.pose.orientation.w = 1.0;
              //
              //   LanePoint lp;
              //   lp.lnid = next_lane.lnid;
              //   lp.fpid = fn.pid;
              //   lp.direction = direction;
              //   lp.map_fp_pose  = map_fp_pose;
              //
              //   targets[i].objectPaths[0].push_back(lp);
              //   targets[i].objectPaths[0].erase(targets[i].objectPaths[0].begin());
              // }
            // }// end if branching_path
            // else // not brancing at index 0(objectPaths[0][0])
            // {
            //   double path1_diff_direction = path1
            // }
          // }// end if dist < 3.0
        // }// end lanes_ Loop



        // start making predictionMarkers
        // std::cout << "ukf_id "    << targets[i].ukf_id_ << std::endl;
        // std::cout << "num paths " << targets[i].objectPaths.size() << std::endl;
        for (size_t ith_path = 0; ith_path < targets[i].objectPaths.size(); ith_path ++)
        {
          visualization_msgs::Marker line;
          line.lifetime = ros::Duration(0.1);
          // line.header.frame_id = pointcloud_frame_;
          line.header.frame_id = "/map";
          line.header.stamp = input.header.stamp;
          line.ns = "line";
          line.action = visualization_msgs::Marker::ADD;
          line.type = visualization_msgs::Marker::LINE_STRIP;
          // green
          line.color.g = 1.0f;
          line.color.a = 1.0;

          line.id = ith_path + i*10;
          line.scale.x = 0.1;
          for (size_t path_point_i = 0; path_point_i < num_path_points_; path_point_i ++)
          {
            geometry_msgs::PoseStamped map_fp_pose;
            map_fp_pose = targets[i].objectPaths[ith_path][path_point_i].map_fp_pose;
            // tf_listen_.waitForTransform("/map", "/velodyne", map_fp_pose.header.stamp, ros::Duration(1.0));
            // tf_listen_.transformPose("/velodyne", ros::Time(0), map_fp_pose, "map", velo_fp_pose);

            geometry_msgs::Point p;
            // p.x = velo_fp_pose.pose.position.x;
            // p.y = velo_fp_pose.pose.position.y;
            p.x = map_fp_pose.pose.position.x;
            p.y = map_fp_pose.pose.position.y;
            p.z = map_fp_pose.pose.position.z;
            line.points.push_back(p);
          }
          predictionMarkers.markers.push_back(line);
        }
      }
    }// end if for isvis target
  }//end loop for targets.size()
}

void ModelBasePrediction::initObjectPaths(const geometry_msgs::PoseStamped& map_pose, IMM_RAUKF &target)
{
  int min_fnid = 0;
  int min_flid = 0;
  int target_flid = 0;
  double min_dist = 9999;
  geometry_msgs::PoseStamped min_fn_pose;
  double px = map_pose.pose.position.x;
  double py = map_pose.pose.position.y;

  for (const auto &lane : lanes_)
  {
    vector_map::Node  bn = vmap_.findByKey(vector_map::Key<vector_map::Node>(lane.bnid));
    vector_map::Point bp = vmap_.findByKey(vector_map::Key<vector_map::Point>(bn.pid));
    vector_map::Node  fn = vmap_.findByKey(vector_map::Key<vector_map::Node>(lane.fnid));
    vector_map::Point fp = vmap_.findByKey(vector_map::Key<vector_map::Point>(fn.pid));
    // std::cout << "px " << px << "py "<< py <<std::endl;
    // ROS_INFO(" lane bn=(%f,%f)", fp.ly, fp.bx);
    double dist = ((py - fp.bx) * (py - fp.bx) + (px - fp.ly) * (px - fp.ly));
    // std::cout << "dist " << dist << std::endl;
    if(dist < min_dist)
    {
      // std::cout << "id " << input.objects[i].id << std::endl;
      // std::cout << "dist "<<dist << std::endl;
      // std::cout << "estimated yaw " << tyaw << std::endl;
      tf::Matrix3x3 mat;
      mat.setEulerYPR(atan2(fp.bx - bp.bx, fp.ly - bp.ly), 0, 0);  // yaw, pitch, roll
      tf::Quaternion quat;
      mat.getRotation(quat);

      min_fn_pose = map_pose;
      min_fn_pose.pose.orientation.x = quat.getX();
      min_fn_pose.pose.orientation.y = quat.getY();
      min_fn_pose.pose.orientation.z = quat.getZ();
      min_fn_pose.pose.orientation.w = quat.getW();
      min_dist = dist;
      min_fnid = lane.fnid;
      min_flid = lane.flid;
    }
  }


  int num_path_points_ = 10;
  bool is_continue = true;
  std::stack<std::vector<LanePoint>> path_stack;
  std::vector<LanePoint> one_path;


  while(is_continue)
  {
    // restore stacked path
    if(!path_stack.empty())
    {
      one_path = path_stack.top();
      path_stack.pop();
      target_flid = one_path.back().flid;
    }
    else
    {
      target_flid = min_flid;
    }

    const size_t path_size = one_path.size();
    for(size_t i = 0; i < (num_path_points_ - path_size); i++)
    {
      vector_map::Lane  front_lane    = vmap_.findByKey(vector_map::Key<vector_map::Lane>(target_flid));
      vector_map::DTLane  dtlane = vmap_.findByKey(vector_map::Key<vector_map::DTLane>(front_lane.did));
      vector_map::Node  bn = vmap_.findByKey(vector_map::Key<vector_map::Node>(front_lane.bnid));
      vector_map::Point bp = vmap_.findByKey(vector_map::Key<vector_map::Point>(bn.pid));
      vector_map::Node  fn = vmap_.findByKey(vector_map::Key<vector_map::Node>(front_lane.fnid));
      vector_map::Point fp = vmap_.findByKey(vector_map::Key<vector_map::Point>(fn.pid));
      int front_lane_fpid = fn.pid;
      double direction = atan2(fp.bx - bp.bx, fp.ly - bp.ly);

      double l_direction = direction + M_PI/2;
      while (l_direction > M_PI)
        l_direction -= 2. * M_PI;
      while (l_direction < -M_PI)
        l_direction += 2. * M_PI;

      double r_direction = direction - M_PI/2;
      while (r_direction > M_PI)
        r_direction -= 2. * M_PI;
      while (r_direction < -M_PI)
        r_direction += 2. * M_PI;

      double l_ly = cos(l_direction)*dtlane.lw + bp.ly;
      double l_bx = sin(l_direction)*dtlane.lw + bp.bx;

      double r_ly = cos(r_direction)*dtlane.rw + bp.ly;
      double r_bx = sin(r_direction)*dtlane.rw + bp.bx;

      geometry_msgs::PoseStamped in_pose, out_pose;
      in_pose = map_pose;


      LanePoint lp;
      lp.lnid = target_flid;
      lp.flid = front_lane.flid;
      lp.fpid = front_lane_fpid;
      lp.direction = direction;
      lp.curvature    = 1/dtlane.r;
      lp.lw           = dtlane.lw;
      lp.rw           = dtlane.rw;
      lp.lw_ly        = l_ly;
      lp.lw_bx        = l_bx;
      lp.rw_ly        = r_ly;
      lp.rw_bx        = r_bx;

      in_pose.pose.position.x = fp.ly;
      in_pose.pose.position.y = fp.bx;
      in_pose.pose.position.z = fp.h;
      in_pose.pose.orientation.x = 0.0;
      in_pose.pose.orientation.y = 0.0;
      in_pose.pose.orientation.z = 0.0;
      in_pose.pose.orientation.w = 1.0;
      lp.map_fp_pose  = in_pose;

      if(front_lane.jct == 1 || front_lane.jct == 2)
      {
        lp.flid = front_lane.flid2;
        one_path.push_back(lp);
        path_stack.push(one_path);
      }
      else
      {
        one_path.push_back(lp);
      }
      target_flid = front_lane.flid;
    }// end num_path_points_ loop
    // std::cout << "one path size " << one_path.size() << std::endl;
    target.objectPaths.push_back(one_path);

    if(path_stack.empty())
    {
      is_continue = false;
    }
  }//end while loop
}

void ModelBasePrediction::maneuverRecognition(geometry_msgs::PoseStamped& map_pose, IMM_RAUKF& target)
{
  for(size_t ith_path = 0; ith_path < target.objectPaths.size(); ith_path++)
  {
    double px = map_pose.pose.position.x;
    double py = map_pose.pose.position.y;

    // std::cout << "map pose "<< px << " " << py << std::endl;
    // std::cout << "l edge pose "<< target.objectPaths[ith_path][0].lw_ly << " " << target.objectPaths[ith_path][0].lw_bx << std::endl;

    double l_edge_x = target.objectPaths[ith_path][0].lw_ly;
    double l_edge_y = target.objectPaths[ith_path][0].lw_bx;
    double l_edge_dist = sqrt((px - l_edge_x)*(px - l_edge_x) + (py - l_edge_y)*(py - l_edge_y));
    // std::cout << "target to left edge " << l_edge_dist << std::endl;

    double r_edge_x = target.objectPaths[ith_path][0].rw_ly;
    double r_edge_y = target.objectPaths[ith_path][0].rw_bx;
    double r_edge_dist = sqrt((px - r_edge_x)*(px - r_edge_x) + (py - r_edge_y)*(py - r_edge_y));
    // std::cout << "target to right edge " << r_edge_dist << std::endl;

    tf::Quaternion q(map_pose.pose.orientation.x,
                     map_pose.pose.orientation.y,
                     map_pose.pose.orientation.z,
                     map_pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double target_curvature = target.x_merge_(4)/target.x_merge_(2);
    // std::cout <<"curvature " <<target_curvature << std::endl;

    x_p_ << l_edge_dist, r_edge_dist, yaw, target_curvature;

    x_l_ << target.objectPaths[ith_path][0].lw,
            target.objectPaths[ith_path][0].rw,
            target.objectPaths[ith_path][0].direction,
            target.objectPaths[ith_path][0].curvature;

    double v = target.x_merge_(2);
    double w = target.x_merge_(4);
    double var_v = target.p_merge_(2,2);
    double var_w = target.p_merge_(4,4);
    double var_pos = (target.p_merge_(0, 0) + target.p_merge_(1, 1))/2;
    double var_yaw = target.p_merge_(3,3);
    double var_curvature = (w*v/(v*v - var_v))*(w*v/(v*v - var_v)) - (w*w - var_w)/(v*v - var_v);

    p_p_ << var_pos, 0, 0, 0,
            0,var_pos,0,0,
            0,0, var_yaw, 0,
            0,0,0, var_curvature;

    // double nis = (x_l_ - x_p_).transpose()*p_p_.inverse()*(x_l_ - x_p_);
    // double nis = (x_l_ - x_p_).transpose()*(x_l_ - x_p_);
    // double nis = ((x_l_ - x_p_).transpose()*var_curvature.inverse()*(x_l_ - x_p_))(0,0);
    double nis = ((x_l_ - x_p_).transpose()*p_p_.inverse()*(x_l_ - x_p_));
    // std::cout << "x_p " << x_p_ <<std::endl;
    // std::cout << "x_l " << x_l_ <<std::endl;
    // std::cout << "ukf_id "<< target.ukf_id_ << std::endl;
    // std::cout << "ith_path " << ith_path << std::endl;
    // std::cout << "nis " << nis << std::endl;
    target.nis_paths_[ith_path] = nis;
  }
  // std::cout << "ukf_id "<< target.ukf_id_ << std::endl;
  if(target.ukf_id_ == 0)
  {
    if(target.objectPaths.size() == 2)
    {
      if(target.nis_paths_[0] < target.nis_paths_[1])
      {
        std::cout << "Lane Change" << std::endl;
      }
      else{
        std::cout << "Keep Lane" << std::endl;
      }
      // target.nis_paths_[ith_path] = nis;
      // target.nis_paths_[ith_path] = nis;
    }
    else
    {
      std::cout << "Keep Lane" << std::endl;
    }
  }
}
