// Bring in gtest
#include <gtest/gtest.h>

#include "imm_raukf.h"

TEST(ImmRaukf, initializeProperly)
{
  double px        = 0.7;
  double py        = 10.9;
  double timestamp = 111111;
  int target_id    = 1;
  Eigen::VectorXd init_meas = Eigen::VectorXd(2);
  init_meas << px, py;
  IMM_RAUKF ukf;
  ukf.initialize(init_meas, timestamp, target_id);
  EXPECT_DOUBLE_EQ( 0.7, ukf.x_merge_(0));
  EXPECT_DOUBLE_EQ(10.9, ukf.z_pred_cv_(1));
}

TEST(ImmRaukf, initializeCovarianceQProperly)
{
  IMM_RAUKF ukf;
  double dt  = 0.1;
  double yaw = 0;
  ukf.initCovarQs(dt, yaw);
  EXPECT_NEAR(6.920451055917801e-310, ukf.q_cv_(0,0), 0.00001);
}

TEST(ImmRaukf, checkCTRV)
{
  IMM_RAUKF ukf;
  double dt  = 0.1;
  double yaw = 0;
  ukf.initCovarQs(dt, yaw);
  EXPECT_NEAR(6.920451055917801e-310, ukf.q_cv_(0,0), 0.00001);
}


// // Declare another test
// TEST(TestSuite, testCase2)
// {
//     <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
// }

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
