#include <ros/ros.h>
#include <gtest/gtest.h>

#include "op_utility/UtilityH.h"

class TestSuite : public ::testing::Test
{
public:
  TestSuite() {}
  ~TestSuite() {}
};

TEST(TestSuite, UtilityH_fixNegativeAngle) {
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::FixNegativeAngle(0));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::FixNegativeAngle(2 * M_PI));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::FixNegativeAngle(4 * M_PI));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::FixNegativeAngle(-2 * M_PI));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::FixNegativeAngle(-4 * M_PI));

  ASSERT_EQ(M_PI, UtilityHNS::UtilityH::FixNegativeAngle(M_PI));
  ASSERT_EQ(M_PI, UtilityHNS::UtilityH::FixNegativeAngle(3 * M_PI));
  ASSERT_EQ(M_PI, UtilityHNS::UtilityH::FixNegativeAngle(-M_PI));
  ASSERT_EQ(M_PI, UtilityHNS::UtilityH::FixNegativeAngle(-3 * M_PI));

  ASSERT_EQ(1.0, UtilityHNS::UtilityH::FixNegativeAngle(1.0));
  ASSERT_EQ(1.0, UtilityHNS::UtilityH::FixNegativeAngle(1.0 + 2 * M_PI));
  ASSERT_EQ(1.0, UtilityHNS::UtilityH::FixNegativeAngle(1.0 - 2 * M_PI));
  ASSERT_EQ(1.0 + M_PI, UtilityHNS::UtilityH::FixNegativeAngle(1.0 - M_PI));
}

TEST(TestSuite, UtilityH_splitPositiveAngle) {
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::SplitPositiveAngle(0));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::SplitPositiveAngle(2.0 * M_PI));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::SplitPositiveAngle(4.0 * M_PI));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::SplitPositiveAngle(-2.0 * M_PI));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::SplitPositiveAngle(-4.0 * M_PI));

  ASSERT_EQ(-M_PI, UtilityHNS::UtilityH::SplitPositiveAngle(-M_PI));
  ASSERT_EQ(-M_PI, UtilityHNS::UtilityH::SplitPositiveAngle(3.0 * M_PI));
  ASSERT_EQ(-M_PI, UtilityHNS::UtilityH::SplitPositiveAngle(M_PI));
  ASSERT_EQ(-M_PI, UtilityHNS::UtilityH::SplitPositiveAngle(-3.0 * M_PI));

  ASSERT_EQ(1.0, UtilityHNS::UtilityH::SplitPositiveAngle(1.0));
  ASSERT_EQ(1.0, UtilityHNS::UtilityH::SplitPositiveAngle(1.0 + 2.0 * M_PI));
  ASSERT_EQ(1.0, UtilityHNS::UtilityH::SplitPositiveAngle(1.0 - 2.0 * M_PI));

  ASSERT_EQ(-1.0, UtilityHNS::UtilityH::SplitPositiveAngle(-1.0));
  ASSERT_EQ(-1.0, UtilityHNS::UtilityH::SplitPositiveAngle(-1.0 + 2.0 * M_PI));
  ASSERT_EQ(-1.0, UtilityHNS::UtilityH::SplitPositiveAngle(-1.0 - 2.0 * M_PI));

  ASSERT_EQ(4.0 - 2.0 * M_PI, UtilityHNS::UtilityH::SplitPositiveAngle(4.0));
  ASSERT_EQ(-4.0 + 2.0 * M_PI, UtilityHNS::UtilityH::SplitPositiveAngle(-4.0));
}

TEST(TestSuite, UtilityH_inverseAngle) {
  ASSERT_EQ(M_PI, UtilityHNS::UtilityH::InverseAngle(0.0));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::InverseAngle(M_PI));
  ASSERT_EQ(1.0 + M_PI, UtilityHNS::UtilityH::InverseAngle(1.0));
  ASSERT_EQ(1.0, UtilityHNS::UtilityH::InverseAngle(1.0 + M_PI));
}

TEST(TestSuite, UtilityH_angleBetweenTwoAnglesPositive) {
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(0.0, 0.0));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(0.0, 2.0 * M_PI));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(M_PI, M_PI));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(-M_PI, -M_PI));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(-M_PI, M_PI));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(M_PI, -M_PI));
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(M_PI, 3 * M_PI));

  ASSERT_EQ(M_PI, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(0.0, M_PI));
  ASSERT_EQ(M_PI, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(M_PI, 0.0));
  ASSERT_EQ(M_PI, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(-M_PI, 0.0));
  ASSERT_EQ(M_PI, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(0.0, -M_PI));

  ASSERT_EQ(M_PI_2, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(0.0, M_PI_2));
  ASSERT_EQ(M_PI_2, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(M_PI_2, 0.0));
  ASSERT_EQ(M_PI_2, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(-M_PI_2, 0.0));
  ASSERT_EQ(M_PI_2, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(0.0, -M_PI_2));

  ASSERT_EQ(M_PI, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(-M_PI_2, M_PI_2));
  ASSERT_EQ(M_PI, UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(M_PI_2, -M_PI_2));
}

TEST(TestSuite, UtilityH_getSign) {
  ASSERT_EQ(1, UtilityHNS::UtilityH::GetSign(0.0));
  ASSERT_EQ(1, UtilityHNS::UtilityH::GetSign(1.0));
  ASSERT_EQ(1, UtilityHNS::UtilityH::GetSign(1e20));
  ASSERT_EQ(1, UtilityHNS::UtilityH::GetSign(1e-20));

  ASSERT_EQ(-1, UtilityHNS::UtilityH::GetSign(-1.0));
  ASSERT_EQ(-1, UtilityHNS::UtilityH::GetSign(-1e20));
  ASSERT_EQ(-1, UtilityHNS::UtilityH::GetSign(-1e-20));
}

TEST(TestSuite, UtilityH_getTimeDiff) {
  ASSERT_EQ(0.0, UtilityHNS::UtilityH::GetTimeDiff(timespec{0, 0}, timespec{0, 0}));
  ASSERT_EQ(1.0, UtilityHNS::UtilityH::GetTimeDiff(timespec{0, 0}, timespec{1, 0}));
  ASSERT_EQ(-1.0, UtilityHNS::UtilityH::GetTimeDiff(timespec{1, 0}, timespec{0, 0}));
  ASSERT_EQ(1.0e-9, UtilityHNS::UtilityH::GetTimeDiff(timespec{0, 0}, timespec{0, 1}));
  ASSERT_EQ(1.0e-6, UtilityHNS::UtilityH::GetTimeDiff(timespec{0, 0}, timespec{0, 1000}));
  ASSERT_EQ(1.0e-3, UtilityHNS::UtilityH::GetTimeDiff(timespec{0, 0}, timespec{0, 1000000}));
}

TEST(TestSuite, UtilityH_tsCompare) {
  // Clear cases
  ASSERT_EQ(0, UtilityHNS::UtilityH::tsCompare(timespec{0, 0}, timespec{0, 0}));
  ASSERT_EQ(1, UtilityHNS::UtilityH::tsCompare(timespec{1, 0}, timespec{0, 0}));
  ASSERT_EQ(-1, UtilityHNS::UtilityH::tsCompare(timespec{0, 0}, timespec{1, 0}));

  // Just within tolerance
  ASSERT_EQ(0, UtilityHNS::UtilityH::tsCompare(timespec{0, 0}, timespec{0, 10}, 10));
  ASSERT_EQ(0, UtilityHNS::UtilityH::tsCompare(timespec{0, 10}, timespec{0, 0}, 10));
  ASSERT_EQ(0, UtilityHNS::UtilityH::tsCompare(timespec{0, 999999990}, timespec{1, 0}, 10));
  ASSERT_EQ(0, UtilityHNS::UtilityH::tsCompare(timespec{1, 0}, timespec{0, 999999990}, 10));

  // Just outside of tolerance
  ASSERT_EQ(-1, UtilityHNS::UtilityH::tsCompare(timespec{0, 0}, timespec{0, 11}, 10));
  ASSERT_EQ(1, UtilityHNS::UtilityH::tsCompare(timespec{0, 11}, timespec{0, 0}, 10));
  ASSERT_EQ(-1, UtilityHNS::UtilityH::tsCompare(timespec{0, 999999989}, timespec{1, 0}, 10));
  ASSERT_EQ(1, UtilityHNS::UtilityH::tsCompare(timespec{1, 0}, timespec{0, 999999989}, 10));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
