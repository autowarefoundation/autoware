#include <ros/ros.h>
#include <gtest/gtest.h>

#include "op_utility/utility.h"

class TestSuite : public ::testing::Test
{
public:
  TestSuite() {}
  ~TestSuite() {}
};

TEST(TestSuite, UtilityH_fixNegativeAngle) {
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::fixNegativeAngle(0));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::fixNegativeAngle(2 * M_PI));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::fixNegativeAngle(4 * M_PI));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::fixNegativeAngle(-2 * M_PI));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::fixNegativeAngle(-4 * M_PI));

  ASSERT_EQ(M_PI, op_utility_ns::UtilityH::fixNegativeAngle(M_PI));
  ASSERT_EQ(M_PI, op_utility_ns::UtilityH::fixNegativeAngle(3 * M_PI));
  ASSERT_EQ(M_PI, op_utility_ns::UtilityH::fixNegativeAngle(-M_PI));
  ASSERT_EQ(M_PI, op_utility_ns::UtilityH::fixNegativeAngle(-3 * M_PI));

  ASSERT_EQ(1.0, op_utility_ns::UtilityH::fixNegativeAngle(1.0));
  ASSERT_EQ(1.0, op_utility_ns::UtilityH::fixNegativeAngle(1.0 + 2 * M_PI));
  ASSERT_EQ(1.0, op_utility_ns::UtilityH::fixNegativeAngle(1.0 - 2 * M_PI));
  ASSERT_EQ(1.0 + M_PI, op_utility_ns::UtilityH::fixNegativeAngle(1.0 - M_PI));
}

TEST(TestSuite, UtilityH_splitPositiveAngle) {
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::splitPositiveAngle(0));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::splitPositiveAngle(2.0 * M_PI));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::splitPositiveAngle(4.0 * M_PI));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::splitPositiveAngle(-2.0 * M_PI));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::splitPositiveAngle(-4.0 * M_PI));

  ASSERT_EQ(-M_PI, op_utility_ns::UtilityH::splitPositiveAngle(-M_PI));
  ASSERT_EQ(-M_PI, op_utility_ns::UtilityH::splitPositiveAngle(3.0 * M_PI));
  ASSERT_EQ(-M_PI, op_utility_ns::UtilityH::splitPositiveAngle(M_PI));
  ASSERT_EQ(-M_PI, op_utility_ns::UtilityH::splitPositiveAngle(-3.0 * M_PI));

  ASSERT_EQ(1.0, op_utility_ns::UtilityH::splitPositiveAngle(1.0));
  ASSERT_EQ(1.0, op_utility_ns::UtilityH::splitPositiveAngle(1.0 + 2.0 * M_PI));
  ASSERT_EQ(1.0, op_utility_ns::UtilityH::splitPositiveAngle(1.0 - 2.0 * M_PI));

  ASSERT_EQ(-1.0, op_utility_ns::UtilityH::splitPositiveAngle(-1.0));
  ASSERT_EQ(-1.0, op_utility_ns::UtilityH::splitPositiveAngle(-1.0 + 2.0 * M_PI));
  ASSERT_EQ(-1.0, op_utility_ns::UtilityH::splitPositiveAngle(-1.0 - 2.0 * M_PI));

  ASSERT_EQ(4.0 - 2.0 * M_PI, op_utility_ns::UtilityH::splitPositiveAngle(4.0));
  ASSERT_EQ(-4.0 + 2.0 * M_PI, op_utility_ns::UtilityH::splitPositiveAngle(-4.0));
}

TEST(TestSuite, UtilityH_inverseAngle) {
  ASSERT_EQ(M_PI, op_utility_ns::UtilityH::inverseAngle(0.0));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::inverseAngle(M_PI));
  ASSERT_EQ(1.0 + M_PI, op_utility_ns::UtilityH::inverseAngle(1.0));
  ASSERT_EQ(1.0, op_utility_ns::UtilityH::inverseAngle(1.0 + M_PI));
}

TEST(TestSuite, UtilityH_angleBetweenTwoAnglesPositive) {
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(0.0, 0.0));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(0.0, 2.0 * M_PI));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(M_PI, M_PI));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(-M_PI, -M_PI));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(-M_PI, M_PI));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(M_PI, -M_PI));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(M_PI, 3 * M_PI));

  ASSERT_EQ(M_PI, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(0.0, M_PI));
  ASSERT_EQ(M_PI, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(M_PI, 0.0));
  ASSERT_EQ(M_PI, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(-M_PI, 0.0));
  ASSERT_EQ(M_PI, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(0.0, -M_PI));

  ASSERT_EQ(M_PI_2, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(0.0, M_PI_2));
  ASSERT_EQ(M_PI_2, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(M_PI_2, 0.0));
  ASSERT_EQ(M_PI_2, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(-M_PI_2, 0.0));
  ASSERT_EQ(M_PI_2, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(0.0, -M_PI_2));

  ASSERT_EQ(M_PI, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(-M_PI_2, M_PI_2));
  ASSERT_EQ(M_PI, op_utility_ns::UtilityH::angleBetweenTwoAnglesPositive(M_PI_2, -M_PI_2));
}

TEST(TestSuite, UtilityH_getSign) {
  ASSERT_EQ(1, op_utility_ns::UtilityH::getSign(0.0));
  ASSERT_EQ(1, op_utility_ns::UtilityH::getSign(1.0));
  ASSERT_EQ(1, op_utility_ns::UtilityH::getSign(1e20));
  ASSERT_EQ(1, op_utility_ns::UtilityH::getSign(1e-20));

  ASSERT_EQ(-1, op_utility_ns::UtilityH::getSign(-1.0));
  ASSERT_EQ(-1, op_utility_ns::UtilityH::getSign(-1e20));
  ASSERT_EQ(-1, op_utility_ns::UtilityH::getSign(-1e-20));
}

TEST(TestSuite, PIDController_null) {
  // With all coefficients set to 0, control should be zero
  auto controller = op_utility_ns::PIDController{};
  ASSERT_EQ(0.0, controller.getPID(0.0, 0.0));
  ASSERT_EQ(0.0, controller.getPID(0.0, 1.0));
  ASSERT_EQ(0.0, controller.getPID(1.0, 0.0));
  ASSERT_EQ(0.0, controller.getPID(-1.0, 0.0));
  ASSERT_EQ(0.0, controller.getPID(0.0, -1.0));

  ASSERT_EQ(0.0, controller.getPID(0.0));
  ASSERT_EQ(0.0, controller.getPID(-1.0));
  ASSERT_EQ(0.0, controller.getPID(1.0));
}

TEST(TestSuite, PIDController_stable) {
  // A system in the desired state should not get any control
  auto controller = op_utility_ns::PIDController{1.0, 1.0, 1.0};
  ASSERT_EQ(0.0, controller.getPID(0.0, 0.0));
  ASSERT_EQ(0.0, controller.getPID(1.0, 1.0));
  ASSERT_EQ(0.0, controller.getPID(-1.0, -1.0));
}


TEST(TestSuite, PIDController_lowpass) {
  // A proportional controller should provide fixed control proportional to error
  auto controller = op_utility_ns::PIDController{0.9, 0.0, 0.0};
  ASSERT_EQ(0.0, controller.getPID(0.0, 0.0));
  ASSERT_EQ(0.9, controller.getPID(0.0, 1.0));
  ASSERT_EQ(-0.9, controller.getPID(1.0, 0.0));
  ASSERT_EQ(0.9, controller.getPID(-1.0, 0.0));
  ASSERT_EQ(-0.9, controller.getPID(0.0, -1.0));

  ASSERT_EQ(0.0, controller.getPID(0.0));
  ASSERT_EQ(0.9, controller.getPID(1.0));
  ASSERT_EQ(-0.9, controller.getPID(-1.0));

  controller.init(0.8, 0.0, 0.0);
  ASSERT_EQ(0.0, controller.getPID(0.0, 0.0));
  ASSERT_EQ(0.8, controller.getPID(0.0, 1.0));
  ASSERT_EQ(-0.8, controller.getPID(1.0, 0.0));
  ASSERT_EQ(0.8, controller.getPID(-1.0, 0.0));
  ASSERT_EQ(-0.8, controller.getPID(0.0, -1.0));

  auto v = 1.0;
  auto target = 2.0;
  for (auto i = 0; i < 1000; ++i) {
    v += controller.getPID(v, target);
  }
  ASSERT_DOUBLE_EQ(target, v);
}

TEST(TestSuite, PIDController_fullController) {
  // Example taken from https://upload.wikimedia.org/wikipedia/commons/3/33/PID_Compensation_Animated.gif
  auto controller = op_utility_ns::PIDController{1.0, 0.5, 0.1};
  auto v = 0.0;
  auto target = 1.0;
  for (auto i = 0; i < 20; ++i) {
    v += controller.getPID(v, target);
  }

  ASSERT_NEAR(target, v, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
