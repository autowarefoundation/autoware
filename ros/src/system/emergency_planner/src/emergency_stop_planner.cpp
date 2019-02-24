#include <emergency_planner/libemergency_planner.h>

class EmergencyStopPlanner : public EmergencyPlanner
{
public:
  EmergencyStopPlanner(std::string name) : EmergencyPlanner(name)
  {
    ros::NodeHandle pnh("~");
    pnh.param<int>("stop_signal_secs", stop_signal_secs_, 30);
  }
  virtual void goalCallback()
  {
    ros::Time start_time = ros::Time::now();
    ros::Rate r(50);
    while (ros::ok())
    {
      const ros::Duration diff = ros::Time::now() - start_time;
      if (diff.toSec() > stop_signal_secs_)
      {
        break;
      }
      autoware_msgs::VehicleCmd vehicle_cmd;
      vehicle_cmd.header.stamp = ros::Time::now();
      // TODO: get emergency value and put it!!!
      // vehicle_cmd.emergency = ;
      publishFeedback(vehicle_cmd);
      r.sleep();
    }
    setSucceeded();
  }
private:
  int stop_signal_secs_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "emergency_stop_planner");
  EmergencyStopPlanner esp(ros::this_node::getName());
  ros::spin();
  return 0;
}
