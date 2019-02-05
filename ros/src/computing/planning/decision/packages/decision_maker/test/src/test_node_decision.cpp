#include <ros/ros.h>
#include <gtest/gtest.h>

#include "decision_maker_node.hpp"
#include "amathutils_lib/amathutils.hpp"

namespace decision_maker
{

class TestSuite : public ::testing::Test {
public:
  TestSuite(){}
  ~TestSuite(){}

protected:
  virtual void SetUp()
  {
    int argc;
    char** argv;
    dmn = new DecisionMakerNode(argc, argv);
  };
  virtual void TearDown()
  {
    delete dmn;
  };

  DecisionMakerNode* dmn;

  void createFinalWaypoints()
  {
    autoware_msgs::Lane final_lane;
    for (int idx = 0; idx < 100; idx++)
    {
      static autoware_msgs::Waypoint wp;
      wp.gid = idx;
      wp.lid = idx;
      wp.pose.pose.position.x = 0.0 + (double)idx;
      wp.pose.pose.position.y = 0.0;
      wp.pose.pose.position.z = 0.0;
      wp.twist.twist.linear.x = 5.0;
      wp.twist.twist.angular.z = 0.0;

      final_lane.waypoints.push_back(wp);
    }

    dmn->current_status_.finalwaypoints = final_lane;
  }

  void setCurrentPose(double x, double y, double yaw)
  {
    geometry_msgs::Pose current_pose;
    current_pose.position.x = x;
    current_pose.position.y = y;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);
    quaternionTFToMsg(quaternion, current_pose.orientation);

    dmn->current_status_.pose = current_pose;
  }

  void setCurrentVelocity(double vel)
  {
    dmn->current_status_.velocity = vel;
  }

  bool isArrivedGoal() { return dmn->isArrivedGoal(); }
};

TEST_F(TestSuite, isArrivedGoal)
{
  createFinalWaypoints();

  setCurrentPose(100, 0, 0);
  setCurrentVelocity(0.0);
  ASSERT_TRUE(isArrivedGoal()) << "Current pose is outside the target range."
                               << "It should be true";

  setCurrentPose(100, 0, 0);
  setCurrentVelocity(3.0);
  ASSERT_FALSE(isArrivedGoal()) << "Current pose is inside the target range."
                                << "It should be false";

  setCurrentPose(90, 0, 0);
  setCurrentVelocity(0.0);
  ASSERT_FALSE(isArrivedGoal()) << "Current pose is inside the target range."
                                << "It should be false";

  setCurrentPose(90, 0, 0);
  setCurrentVelocity(3.0);
  ASSERT_FALSE(isArrivedGoal()) << "Current pose is inside the target range."
                                << "It should be false";

}

} // namespace decision_maker
