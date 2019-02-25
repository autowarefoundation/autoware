#include <emergency_planner/libemergency_planner.h>

class EmergencyStopPlanner : public EmergencyPlanner
{
public:
  EmergencyStopPlanner(std::string name) : EmergencyPlanner(name),
    new_order_(false)
  {
    ros::NodeHandle pnh("~");
    pnh.param<double>("stop_signal_secs", stop_signal_secs_, 10.0);
  }
  virtual void goalCallback()
  {
    new_order_ = true;
  }
  void run()
  {
    ros::Rate r(50);
    while (ros::ok())
    {
      if (new_order_)
      {
        new_order_ = false;
        runPlannerServer();
      }
      ros::spinOnce();
      r.sleep();
    }
  }
  void runPlannerServer()
  {
    ros::Time start_time = ros::Time::now();
    ros::Rate r(50);
    while (ros::ok() && !new_order_)
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
      ros::spinOnce();
      r.sleep();
    }
    new_order_ ? setAborted() : setSucceeded();
  }
private:
  double stop_signal_secs_;
  bool new_order_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "emergency_stop_planner");
  EmergencyStopPlanner esp(ros::this_node::getName());
  esp.run();
  return 0;
}
