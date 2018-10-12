#include "naive_motion_predict.h"

NaiveMotionPredict::NaiveMotionPredict() : nh_(), private_nh_("~")
{
  // private_nh_.param<bool>("publish_objects", publish_objects_, true);
  // private_nh_.param<bool>("publish_points", publish_points_, true);
  private_nh_.param<double>("publish_rate", publish_rate_, 10.0);

  predicted_objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/prediction/objects", 1);
  predicted_paths_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/prediction/motion_predictor/path_markers", 1);


  // TODO:remap topic in launch file later
  // detected_objects_sub_ = nh_.subscribe("/detection/objects", 1, &NaiveMotionPredict::objectsCallback, this);
  detected_objects_sub_ = nh_.subscribe("/detection/object_tracker/objects", 1, &NaiveMotionPredict::objectsCallback, this);
}

NaiveMotionPredict::~NaiveMotionPredict()
{
}

// void NaiveMotionPredict::run()
// {
//   ros::Rate rate(publish_rate_);
//
//   while (ros::ok())
//   {
//     ros::spinOnce();
//     // updateFakes();
//     // publishFakes();
//     rate.sleep();
//   }
// }

void NaiveMotionPredict::objectsCallback(const autoware_msgs::DetectedObjectArray& input)
{
}
