#ifndef OBJECT_TRACKING_IMM_UKF_JPDAF_H
#define OBJECT_TRACKING_IMM_UKF_JPDAF_H

#include <ros/ros.h>

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <tf/transform_listener.h>


#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"

#include "ukf.h"


// using namespace pcl;
// using namespace Eigen;



class ImmUkfPda
{
private:
  bool init_;
  double timestamp_ ;

  std::vector<UKF> targets_;
  // std::vector<int> trackNumVec_;

  // probabilistic data association params
  double gating_thres_;//9.22; // 99%
  double gate_probability_;//0.99;
  // extern double gammaG_ = 5.99; // 99%
  // extern double pG_ = 0.95;
  // extern double gammaG_ = 15.22; // 99%
  double detection_probability_;//0.9;

  //bbox association param
  double distance_thres_;//0.25;
  int life_time_thres_;//8;
  //bbox update params
  double bb_yaw_change_thres_;//0.2;
  // double bb_area_change_thres_;//0.5;

  double static_distance_thres_;

  double init_yaw_;

  std::string input_topic_;
  std::string output_topic_;

  std::string pointcloud_frame_;
  // std::vector<UKF> targets_;
  // std::vector<int> trackNumVec_;

  tf::TransformListener* tran_;

  ros::NodeHandle node_handle_;
  ros::Subscriber sub_cloud_array_;
  ros::Publisher pub_cloud_array_;

  void callBack(autoware_msgs::CloudClusterArray input);
  void transformPoseToGlobal(autoware_msgs::CloudClusterArray& input);
  void transformPoseToLocal(autoware_msgs::CloudClusterArray& input);
  void findMaxZandS(const UKF target, Eigen::VectorXd& max_det_z, Eigen::MatrixXd& max_det_s);
  void measurementValidation(const autoware_msgs::CloudClusterArray input, UKF& target, const bool second_init,
                 const Eigen::VectorXd max_det_z, const Eigen::MatrixXd max_det_s,
                 std::vector<autoware_msgs::CloudCluster>& cluster_vec,
                 std::vector<int>& matching_vec);
  void filterPDA(UKF& target, const std::vector<autoware_msgs::CloudCluster> cluster_vec, std::vector<double>& lambda_vec);
  void getNearestEuclidCluster(const UKF target, const std::vector<autoware_msgs::CloudCluster> cluster_vec,
                autoware_msgs::CloudCluster& cluster, double& min_dist);
  void getRightAngleBBox(const std::vector<double> nearest_bbox, std::vector<double>& rightAngle_bbox);
  void associateBB(const std::vector<autoware_msgs::CloudCluster> cluster_vec,
                     UKF& target);
  double getBboxArea(const pcl::PointCloud<pcl::PointXYZ> bbox);
  double getBBoxYaw(const UKF target);
  double getJskBBoxArea(const jsk_recognition_msgs::BoundingBox jsk_bb);
  double getJskBBoxYaw(const jsk_recognition_msgs::BoundingBox jsk_bb);
  void updateBB(UKF& target);
  double getIntersectCoef(const double vec1x, const double vec1y, const double vec2x, const double vec2y,
                          const double p_x, const double p_y, const double cp_x, const double cp_y);
  void mergeOverSegmentation(const std::vector<UKF> targets);

  void updateLabel(UKF target, autoware_msgs::CloudCluster& cc);

  void initTracker(autoware_msgs::CloudClusterArray input, double timestamp);
  void secondInit(double dt, std::vector<autoware_msgs::CloudCluster> clusterVec, UKF &target);

  void updateTrackingNum(std::vector<autoware_msgs::CloudCluster> cluster_vec, UKF& target);

  void probabilisticDataAssociation(autoware_msgs::CloudClusterArray input,
                                    double dt, double det_explode_param, std::vector<int>& matching_vec,
                                    std::vector<double>& lambda_vec, UKF& target, bool& is_skip_target);
  void makeNewTargets(double timestamp, autoware_msgs::CloudClusterArray input, std::vector<int> matching_vec);

  void staticClassification();

  void makeOutput(autoware_msgs::CloudClusterArray input,
                  autoware_msgs::CloudClusterArray& output);

  void tracker(autoware_msgs::CloudClusterArray input,
                 autoware_msgs::CloudClusterArray& output);


public:
  ImmUkfPda();
};

#endif /* OBJECT_TRACKING_IMM_UKF_JPDAF_H */
