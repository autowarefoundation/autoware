#include <gtest/gtest.h>

#include "ukf.h"

TEST(UKF, initializeProperly)
{
  double px = 0.7;
  double py = 10.9;
  double timestamp = 1;
  int target_id = 1;
  Eigen::VectorXd init_meas = Eigen::VectorXd(2);
  init_meas << px, py;
  UKF ukf;
  ukf.initialize(init_meas, timestamp, target_id);
  EXPECT_DOUBLE_EQ(0.7, ukf.x_merge_(0));
  EXPECT_DOUBLE_EQ(10.9, ukf.z_pred_cv_(1));
}

TEST(UKF, initializeCovarianceQProperly)
{
  UKF ukf;
  double dt = 0.1;
  double yaw = 0;
  ukf.initCovarQs(dt, yaw);
  EXPECT_NEAR(6.920451055917801e-310, ukf.q_cv_(0, 0), 0.00001);
}

TEST(UKF, checkPredictionForSUKF)
{
  double px = -9.61675;
  double py = -3.49989;
  double timestamp = 1.31701e+09;
  int target_id = 1;
  Eigen::VectorXd init_meas = Eigen::VectorXd(2);
  init_meas << px, py;
  UKF ukf;
  ukf.initialize(init_meas, timestamp, target_id);
  double dt = 0.103288;
  bool has_subscribed_vectormap = false;
  ukf.predictionSUKF(dt, has_subscribed_vectormap);
  EXPECT_NEAR(-9.61675, ukf.x_ctrv_(0, 0), 0.00001);
  EXPECT_NEAR(-3.49989, ukf.z_pred_ctrv_(1), 0.00001);
}

TEST(UKF, checkUpdateForSUKF)
{
  // frame 0-------
  double px = -9.61675;
  double py = -3.49989;
  double timestamp = 1.31701e+09;
  int target_id = 1;
  Eigen::VectorXd init_meas = Eigen::VectorXd(2);
  init_meas << px, py;
  UKF ukf;
  ukf.initialize(init_meas, timestamp, target_id);
  // frame 1-----
  double dt = 0.103288;
  bool has_subscribed_vectormap = false;
  ukf.predictionSUKF(dt, has_subscribed_vectormap);
  std::vector<autoware_msgs::DetectedObject> object_vec;
  autoware_msgs::DetectedObject dd;
  dd.pose.position.x = -9.86449;
  dd.pose.position.y = -2.81236;
  object_vec.push_back(dd);
  ukf.updateSUKF(object_vec);
  double tested_x = ukf.x_ctrv_(0, 0);
  double tested_p = ukf.p_ctrv_(0, 0);
  EXPECT_NEAR(-9.8544286784548873, tested_x, 0.00001);
  EXPECT_NEAR(0.021651, tested_p, 0.00001);
}

TEST(UKF, checkPredictionForIMMUKFPDA)
{
  double px = -9.61675;
  double py = -3.49989;
  double timestamp = 1.31701e+09;
  int target_id = 1;
  Eigen::VectorXd init_meas = Eigen::VectorXd(2);
  init_meas << px, py;
  UKF ukf;
  ukf.initialize(init_meas, timestamp, target_id);
  double dt = 0.103288;
  bool has_subscribed_vectormap = false;
  ukf.predictionIMMUKF(dt, has_subscribed_vectormap);
  double cv_x_0 = ukf.x_cv_(0, 0);
  double rm_x_1 = ukf.x_rm_(1, 0);
  EXPECT_NEAR(-9.61675, cv_x_0, 0.00001);
  EXPECT_NEAR(-3.49989, rm_x_1, 0.00001);
}

TEST(UKF, checkUpdateForIMMUKFPDA)
{
  double px = -9.61675;
  double py = -3.49989;
  double timestamp = 1.31701e+09;
  int target_id = 1;
  Eigen::VectorXd init_meas = Eigen::VectorXd(2);
  init_meas << px, py;
  UKF ukf;
  ukf.initialize(init_meas, timestamp, target_id);
  double dt = 0.103288;
  bool has_subscribed_vectormap = false;
  ukf.predictionIMMUKF(dt, has_subscribed_vectormap);
  std::vector<autoware_msgs::DetectedObject> object_vec;
  autoware_msgs::DetectedObject dd;
  dd.pose.position.x = -9.86449;
  dd.pose.position.y = -2.81236;
  object_vec.push_back(dd);
  ukf.updateSUKF(object_vec);
  double test_x_cv_2 = ukf.x_cv_(2, 0);
  double test_p_cv_1 = ukf.p_cv_(1, 1);
  EXPECT_NEAR(0.0, test_x_cv_2, 0.00001);
  EXPECT_NEAR(0.5, test_p_cv_1, 0.00001);
}

TEST(UKF, checkFaultDetectionForRAFilter)
{
  double px = -9.61675;
  double py = -3.49989;
  double timestamp = 1.31701e+09;
  int target_id = 1;
  Eigen::VectorXd init_meas = Eigen::VectorXd(2);
  init_meas << px, py;
  UKF ukf;
  ukf.initialize(init_meas, timestamp, target_id);
  double dt = 0.103288;
  bool has_subscribed_vectormap = false;
  ukf.predictionIMMUKF(dt, has_subscribed_vectormap);
  std::vector<autoware_msgs::DetectedObject> object_vec;
  autoware_msgs::DetectedObject dd;
  dd.pose.position.x = -9.86449;
  dd.pose.position.y = -2.81236;
  object_vec.push_back(dd);
  ukf.updateSUKF(object_vec);
  ukf.raukf_chi_thres_param_ = 10.59;
  bool use_lane_direction = false;
  bool is_fault = ukf.faultDetection(MotionModel::CTRV, use_lane_direction);
  EXPECT_TRUE(!is_fault);
}

TEST(UKF, checkPassFaultDetectionForRAFilter)
{
  double px = -9.61675;
  double py = -3.49989;
  double timestamp = 1.31701e+09;
  int target_id = 1;
  Eigen::VectorXd init_meas = Eigen::VectorXd(2);
  init_meas << px, py;
  UKF ukf;
  ukf.initialize(init_meas, timestamp, target_id);
  double dt = 0.103288;
  bool has_subscribed_vectormap = false;
  ukf.predictionIMMUKF(dt, has_subscribed_vectormap);
  std::vector<autoware_msgs::DetectedObject> object_vec;
  autoware_msgs::DetectedObject dd;
  dd.pose.position.x = -12.61675;
  dd.pose.position.y = -3.49;
  object_vec.push_back(dd);
  ukf.updateSUKF(object_vec);
  ukf.raukf_chi_thres_param_ = 10.59;
  bool use_lane_direction = false;
  bool is_fault = ukf.faultDetection(MotionModel::CTRV, use_lane_direction);
  EXPECT_TRUE(is_fault);
}

TEST(UKF, checkRAFilter)
{
  double px = -9.61675;
  double py = -3.49989;
  double timestamp = 1.31701e+09;
  int target_id = 1;
  Eigen::VectorXd init_meas = Eigen::VectorXd(2);
  init_meas << px, py;
  UKF ukf;
  ukf.initialize(init_meas, timestamp, target_id);
  double dt = 0.103288;
  bool has_subscribed_vectormap = false;
  ukf.predictionIMMUKF(dt, has_subscribed_vectormap);
  std::vector<autoware_msgs::DetectedObject> object_vec;
  autoware_msgs::DetectedObject dd;
  dd.pose.position.x = -12.6165;
  dd.pose.position.y = -3.49;
  object_vec.push_back(dd);
  ukf.updateSUKF(object_vec);

  ukf.raukf_chi_thres_param_ = 10.59;
  ukf.raukf_q_param_ = 0.2;
  ukf.raukf_r_param_ = 0.2;
  bool use_lane_direction = false;
  ukf.adaptiveAdjustmentQ(MotionModel::CTRV, use_lane_direction);
  ukf.adaptiveAdjustmentR(MotionModel::CTRV, use_lane_direction);
  ukf.estimationUpdate(MotionModel::CTRV, use_lane_direction);
  double test_x_ctrv_0 = ukf.x_ctrv_(0);
  double test_p_ctrv_1 = ukf.p_ctrv_(1, 1);
  EXPECT_NEAR(-12.4948, test_x_ctrv_0, 0.0001);
  EXPECT_NEAR(0.0215346, test_p_ctrv_1, 0.00001);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
