#include "jsk_recognition_utils/tf_listener_singleton.h"
#include <ros/ros.h>
#include <gtest/gtest.h>


TEST(TfListenerSingleton, testLookupTransformWithDuration){
  boost::mutex mutex;
  tf::TransformListener* tf_listener;

  mutex.lock();
  tf_listener = new tf::TransformListener(ros::Duration(30.0));
  mutex.unlock();

  std::string from_frame("base");
  std::string to_frame("head");
  tf::StampedTransform transform;
  transform = jsk_recognition_utils::lookupTransformWithDuration(
    /*listener=*/tf_listener,
    /*to_frame=*/to_frame,
    /*from_frame=*/from_frame,
    /*time=*/ros::Time(),
    /*duration=*/ros::Duration(1.0));
  ASSERT_STREQ("base", transform.frame_id_.c_str());
  ASSERT_STREQ("head", transform.child_frame_id_.c_str());
  ASSERT_EQ(0, transform.getOrigin().getX());
  ASSERT_EQ(0, transform.getOrigin().getY());
  ASSERT_EQ(1, transform.getOrigin().getZ());
  ASSERT_EQ(0, transform.getRotation().getX());
  ASSERT_EQ(0, transform.getRotation().getY());
  ASSERT_EQ(0, transform.getRotation().getZ());
  ASSERT_EQ(1, transform.getRotation().getW());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "simple_lookup_transform");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
