#include <ros/ros.h>
#include <gtest/gtest.h>

#include "op_utility/utility.h"

class TestSuite : public ::testing::Test
{
public:
  TestSuite() {}
  ~TestSuite() {}
};

TEST(TestSuite, fixNegativeAngle) {
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

TEST(TestSuite, splitPositiveAngle) {
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

TEST(TestSuite, inverseAngle) {
  ASSERT_EQ(M_PI, op_utility_ns::UtilityH::inverseAngle(0.0));
  ASSERT_EQ(0.0, op_utility_ns::UtilityH::inverseAngle(M_PI));
  ASSERT_EQ(1.0 + M_PI, op_utility_ns::UtilityH::inverseAngle(1.0));
  ASSERT_EQ(1.0, op_utility_ns::UtilityH::inverseAngle(1.0 + M_PI));
}

TEST(TestSuite, angleBetweenTwoAnglesPositive) {
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

TEST(TestSuite, getSign) {
  ASSERT_EQ(1, op_utility_ns::UtilityH::getSign(0.0));
  ASSERT_EQ(1, op_utility_ns::UtilityH::getSign(1.0));
  ASSERT_EQ(1, op_utility_ns::UtilityH::getSign(1e20));
  ASSERT_EQ(1, op_utility_ns::UtilityH::getSign(1e-20));

  ASSERT_EQ(-1, op_utility_ns::UtilityH::getSign(-1.0));
  ASSERT_EQ(-1, op_utility_ns::UtilityH::getSign(-1e20));
  ASSERT_EQ(-1, op_utility_ns::UtilityH::getSign(-1e-20));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
