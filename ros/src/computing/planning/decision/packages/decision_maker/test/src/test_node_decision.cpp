#include <gtest/gtest.h>
#include <ros/ros.h>

#include "amathutils_lib/amathutils.hpp"
#include "decision_maker_node.hpp"

#include "test_class.hpp"

namespace decision_maker {

class TestSuite : public ::testing::Test {
public:
  TestSuite() {}
  ~TestSuite() {}

  TestClass test_obj_;

protected:
  virtual void SetUp() {
    int argc;
    char **argv;
    test_obj_.dmn = new DecisionMakerNode(argc, argv);
  };
  virtual void TearDown() { delete test_obj_.dmn; };
};

TEST_F(TestSuite, isArrivedGoal) {
	test_obj_.createFinalWaypoints();

	test_obj_.setCurrentPose(100, 0, 0);
	test_obj_.setCurrentVelocity(0.0);
  ASSERT_TRUE(test_obj_.isArrivedGoal()) << "Current pose is outside the target range."
                               << "It should be true";

  test_obj_.setCurrentPose(100, 0, 0);
  test_obj_.setCurrentVelocity(3.0);
  ASSERT_FALSE(test_obj_.isArrivedGoal()) << "Current pose is inside the target range."
                                << "It should be false";

  test_obj_.setCurrentPose(90, 0, 0);
  test_obj_.setCurrentVelocity(0.0);
  ASSERT_FALSE(test_obj_.isArrivedGoal()) << "Current pose is inside the target range."
                                << "It should be false";

  test_obj_.setCurrentPose(90, 0, 0);
  test_obj_.setCurrentVelocity(3.0);
  ASSERT_FALSE(test_obj_.isArrivedGoal()) << "Current pose is inside the target range."
                                << "It should be false";
}

} // namespace decision_maker
