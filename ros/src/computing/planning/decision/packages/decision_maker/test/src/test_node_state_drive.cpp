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

      wp.wpstate.steering_state = autoware_msgs::WaypointState::STR_STRAIGHT;
      wp.wpstate.stop_state = autoware_msgs::WaypointState::NULLSTATE;
      wp.wpstate.event_state = autoware_msgs::WaypointState::TYPE_EVENT_NULL;

      final_lane.waypoints.push_back(wp);
    }

    dmn->current_status_.finalwaypoints = final_lane;
  }

  void setSteeringState(int index, uint8_t state)
  {
    dmn->current_status_.finalwaypoints.waypoints.at(index).wpstate.steering_state = state;
  }

  void setEventState(int index, uint8_t state)
  {
    dmn->current_status_.finalwaypoints.waypoints.at(index).wpstate.event_state = state;
  }

  void setStopState(int index, uint8_t state)
  {
    dmn->current_status_.finalwaypoints.waypoints.at(index).wpstate.stop_state = state;
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

  DecisionMakerNode* dmn;
  uint8_t getSteeringStateFromWaypoint() { return dmn->getSteeringStateFromWaypoint(); }
  uint8_t getEventStateFromWaypoint() { return dmn->getEventStateFromWaypoint(); }
  std::pair<uint8_t, int> getStopSignStateFromWaypoint() { return dmn->getStopSignStateFromWaypoint(); }
};

TEST_F(TestSuite, getSteeringStateFromWaypoint)
{
  createFinalWaypoints();
  ASSERT_EQ(getSteeringStateFromWaypoint(), autoware_msgs::WaypointState::STR_STRAIGHT)
    << "Could not get the expected state"
    << "It should be "
    << autoware_msgs::WaypointState::STR_STRAIGHT;

  setSteeringState(20, autoware_msgs::WaypointState::STR_LEFT);
  ASSERT_EQ(getSteeringStateFromWaypoint(), autoware_msgs::WaypointState::STR_LEFT)
    << "Could not get the expected state"
    << "It should be "
    << autoware_msgs::WaypointState::STR_LEFT;

  setSteeringState(20, autoware_msgs::WaypointState::STR_RIGHT);
  ASSERT_EQ(getSteeringStateFromWaypoint(), autoware_msgs::WaypointState::STR_RIGHT)
    << "Could not get the expected state"
    << "It should be "
    << autoware_msgs::WaypointState::STR_RIGHT;
}

TEST_F(TestSuite, getEventStateFromWaypoint)
{
  setCurrentPose(0, 0, 0);

  createFinalWaypoints();
  ASSERT_EQ(getEventStateFromWaypoint(), autoware_msgs::WaypointState::TYPE_EVENT_NULL)
    << "Could not get the expected state"
    << "It should be "
    << autoware_msgs::WaypointState::TYPE_EVENT_NULL;

  setEventState(20, autoware_msgs::WaypointState::TYPE_EVENT_GOAL);
  ASSERT_EQ(getEventStateFromWaypoint(), autoware_msgs::WaypointState::TYPE_EVENT_GOAL)
    << "Could not get the expected state"
    << "It should be "
    << autoware_msgs::WaypointState::TYPE_EVENT_GOAL;

  setEventState(20, autoware_msgs::WaypointState::TYPE_EVENT_MIDDLE_GOAL);
  ASSERT_EQ(getEventStateFromWaypoint(), autoware_msgs::WaypointState::TYPE_EVENT_MIDDLE_GOAL)
    << "Could not get the expected state"
    << "It should be "
    << autoware_msgs::WaypointState::TYPE_EVENT_MIDDLE_GOAL;

  setEventState(20, autoware_msgs::WaypointState::TYPE_EVENT_POSITION_STOP);
  ASSERT_EQ(getEventStateFromWaypoint(), autoware_msgs::WaypointState::TYPE_EVENT_POSITION_STOP)
    << "Could not get the expected state"
    << "It should be "
    << autoware_msgs::WaypointState::TYPE_EVENT_POSITION_STOP;

  setEventState(20, autoware_msgs::WaypointState::TYPE_EVENT_BUS_STOP);
  ASSERT_EQ(getEventStateFromWaypoint(), autoware_msgs::WaypointState::TYPE_EVENT_BUS_STOP)
    << "Could not get the expected state"
    << "It should be "
    << autoware_msgs::WaypointState::TYPE_EVENT_BUS_STOP;

  setEventState(20, autoware_msgs::WaypointState::TYPE_EVENT_PARKING);
  ASSERT_EQ(getEventStateFromWaypoint(), autoware_msgs::WaypointState::TYPE_EVENT_PARKING)
    << "Could not get the expected state"
    << "It should be "
    << autoware_msgs::WaypointState::TYPE_EVENT_PARKING;
}

TEST_F(TestSuite, getStopSignStateFromWaypoint)
{
  setCurrentPose(0, 0, 0);
  setCurrentVelocity(10.0);

  createFinalWaypoints();
  std::pair<uint8_t, int> ret1 = getStopSignStateFromWaypoint();
  ASSERT_EQ(ret1.first, autoware_msgs::WaypointState::NULLSTATE)
    << "Could not get the expected state"
    << "It should be "
    << autoware_msgs::WaypointState::NULLSTATE;
  ASSERT_EQ(ret1.second, -1)
    << "Could not get the expected state"
    << "It should be "
    << -1;

  setStopState(20, autoware_msgs::WaypointState::TYPE_STOPLINE);
  std::pair<uint8_t, int> ret2 = getStopSignStateFromWaypoint();
  ASSERT_EQ(ret2.first, autoware_msgs::WaypointState::TYPE_STOPLINE)
    << "Could not get the expected state"
    << "It should be "
    << autoware_msgs::WaypointState::TYPE_STOPLINE;
  ASSERT_EQ(ret2.second, 20)
    << "Could not get the expected state"
    << "It should be "
    << 20;

  setStopState(20, autoware_msgs::WaypointState::TYPE_STOP);
  std::pair<uint8_t, int> ret3 = getStopSignStateFromWaypoint();
  ASSERT_EQ(ret3.first, autoware_msgs::WaypointState::TYPE_STOP)
    << "Could not get the expected state"
    << "It should be "
    << autoware_msgs::WaypointState::TYPE_STOP;
  ASSERT_EQ(ret3.second, 20)
    << "Could not get the expected state"
    << "It should be "
    << 20;
}

} // namespace decision_maker
