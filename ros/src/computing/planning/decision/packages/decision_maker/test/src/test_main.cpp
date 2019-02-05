#include <gtest/gtest.h>
#include <ros/ros.h>

class TestSuite : public ::testing::Test {
public:
  TestSuite() {}
  ~TestSuite() {}
};

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "DecisionMakerTestNode");
  return RUN_ALL_TESTS();
}
