#ifndef OBJECT_TRACKING_MODEL_BASE_PREDICTION_H
#define OBJECT_TRACKING_MODEL_BASE_PREDICTION_H

#include <ros/ros.h>
#include <vector_map/vector_map.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

// #include "autoware_msgs/CloudCluster.h"
// #include "autoware_msgs/CloudClusterArray.h"

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"


#include "ukf.h"
// #include "imm_ukf_pda.h"

class ModelBasePrediction
{
private:
  int num_path_points_;
  vector_map::VectorMap vmap_;
  std::vector<vector_map_msgs::Lane> lanes_;

  // Eigen::MatrixXd x_p_;
  // Eigen::MatrixXd x_l_;

  Eigen::VectorXd x_p_;
  Eigen::VectorXd x_l_;

  Eigen::MatrixXd p_p_;

public:
  ModelBasePrediction();
  void setLanes(std::vector<vector_map_msgs::Lane>& lanes){lanes_ = lanes;}
  void setVMap(vector_map::VectorMap& v_map){vmap_ = v_map;}
  void adasMapAssitDirectionAndPrediction(const autoware_msgs::DetectedObjectArray& input,
                                          const tf::TransformListener &tf_listen_,
                                                std::vector<UKF> &targets,
                                                visualization_msgs::MarkerArray &directionMarkers,
                                                visualization_msgs::MarkerArray &predictionMarkers);
  void initObjectPaths(const geometry_msgs::PoseStamped& map_pose, UKF &target);
  void maneuverRecognition(geometry_msgs::PoseStamped& map_pose, UKF& target);
};

#endif  // OBJECT_TRACKING_MODEL_BASE_PREDICTION_H
