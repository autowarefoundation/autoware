#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>
#include <tf/transform_datatypes.h>

#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/Lane.h"
#include <utils.h>
#include <lowpass_filter.h>

using vector = std::vector<double>;



class MPCFollower {
public:
  MPCFollower();
  ~MPCFollower();

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_steering_, pub_twist_cmd_, pub_debug_filtered_traj_;
  ros::Subscriber sub_ref_path_, sub_twist_, sub_pose_;
  std::string path_frame_id_;


  /* algorithm */
  geometry_msgs::TwistStamped current_twist_;
  geometry_msgs::PoseStamped current_pose_;
  autoware_msgs::Lane current_ref_path_;
  class Trajectory {
  public:
    vector x;
    vector y;
    vector z;
    vector yaw;
    vector relative_time;
    void push_back(const double &xp, const double &yp, const double &zp, const double &yawp, const double &tp){
      x.push_back(xp);
      y.push_back(yp);
      z.push_back(zp);
      yaw.push_back(yawp);
      relative_time.push_back(tp);
    };
    void clear(){
      x.clear();
      y.clear();
      z.clear();
      yaw.clear();
      relative_time.clear();
    };
  };
  MPCFollower::Trajectory ref_traj_;
  Butter2D path_filter_;

  /* parameters */
  double traj_dt_;
  double ctrl_dt_;
  double path_lowpass_cutoff_hz_;


  /* flags */
  bool use_path_smoothing_;

  void callbackRefPath(const autoware_msgs::Lane::ConstPtr&);
  void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr&);
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr&);

  void calcRelativeTimeForPath(const autoware_msgs::Lane&, std::vector<double> &);
  void resamplePath2Traj(const autoware_msgs::Lane &path, const vector &time, const double &resample_dt, MPCFollower::Trajectory &ref_traj_);
  void convertTraj2Marker(const Trajectory &traj, visualization_msgs::Marker &markers);

};

MPCFollower::MPCFollower()
    : nh_(""), pnh_("~"), path_frame_id_("") {

  pnh_.param("use_path_smoothing", use_path_smoothing_, bool(true));
  pnh_.param("traj_dt", traj_dt_, double(0.05));
  pnh_.param("path_lowpass_cutoff_hz", path_lowpass_cutoff_hz_, double(0.01));

  pub_steering_  = nh_.advertise<autoware_msgs::ControlCommandStamped>("/ctrl_cmd", 1);
  pub_twist_cmd_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_cmd", 1);
  pub_debug_filtered_traj_ = pnh_.advertise<visualization_msgs::Marker>("debug/filtered_traj", 1);

  sub_ref_path_ = nh_.subscribe("/final_waypoints", 1, &MPCFollower::callbackRefPath, this);
  sub_twist_ = nh_.subscribe("/current_velocity", 1, &MPCFollower::callbackTwist, this);
  sub_pose_ = nh_.subscribe("/current_pose", 1, &MPCFollower::callbackPose, this);

  path_filter_.initialize(traj_dt_, path_lowpass_cutoff_hz_);

};


void MPCFollower::callbackRefPath(const autoware_msgs::Lane::ConstPtr& msg){
  current_ref_path_ = *msg;
  path_frame_id_ = msg->header.frame_id;
  printf("callbackRefPath start\n");
  printf("current_ref_path_.size() = %d\n",current_ref_path_.waypoints.size());

  // calculate relative time
  vector relative_time;
  MPCFollower::calcRelativeTimeForPath(current_ref_path_, relative_time);
  printf("relative_time.end() = %f\n", relative_time.back());
  printf("relative_time.size() = %d\n",relative_time.size());

  // resampling
  MPCFollower::resamplePath2Traj(current_ref_path_, relative_time, traj_dt_, ref_traj_);
printf("ref_traj_.x.size() = %d\n",ref_traj_.x.size());
printf("ref_traj_.y.size() = %d\n",ref_traj_.y.size());
printf("ref_traj_.z.size() = %d\n",ref_traj_.z.size());
printf("ref_traj_.yaw.size() = %d\n",ref_traj_.yaw.size());
printf("ref_traj_.relative_time.size() = %d\n",ref_traj_.relative_time.size());

  // low pass filter
  if (use_path_smoothing_) {
    double cutoff_hz = 0.1;
    path_filter_.initialize(traj_dt_, cutoff_hz);
    path_filter_.filtfilt_vector(ref_traj_.relative_time, ref_traj_.x);
    path_filter_.filtfilt_vector(ref_traj_.relative_time, ref_traj_.y);
    path_filter_.filtfilt_vector(ref_traj_.relative_time, ref_traj_.z);
    path_filter_.filtfilt_vector(ref_traj_.relative_time, ref_traj_.yaw);
  }

  // publish trajectory for visualize
  visualization_msgs::Marker markers;
  convertTraj2Marker(ref_traj_, markers);
  pub_debug_filtered_traj_.publish(markers);
};






void MPCFollower::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg){
  current_twist_ = *msg;
};

void MPCFollower::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_pose_ = *msg;
};

void MPCFollower::calcRelativeTimeForPath(const autoware_msgs::Lane &path,
                                          std::vector<double> &path_time) {
printf("calcRelativeTimeForPath start\n");
  double t = 0.0;
  path_time.clear();
  path_time.push_back(t);
  for (int i = 0; i < (int)path.waypoints.size() - 1; ++i) {
    const double x0 = path.waypoints.at(i).pose.pose.position.x;
    const double y0 = path.waypoints.at(i).pose.pose.position.y;
    const double z0 = path.waypoints.at(i).pose.pose.position.z;
    const double x1 = path.waypoints.at(i+1).pose.pose.position.x;
    const double y1 = path.waypoints.at(i+1).pose.pose.position.y;
    const double z1 = path.waypoints.at(i+1).pose.pose.position.z;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dz = z1 - z0;
    const double dist = sqrt(dx * dx + dy * dy + dz * dz);
    double v = std::max(std::fabs(path.waypoints.at(i).twist.twist.linear.x), 0.001);
    t += dist / v;
    path_time.push_back(t);
    // printf("i = %d, t = %f, dist = %f, v = %f, dx = %f, dy = %f, dz = %f\n",i,t,dist,v,dx,dy,dz);
  }

  for (int i = 0; i < path_time.size(); i++) {
     printf("%f, ", path_time[i]);
 }
 printf("\n");
 // printf("path_time.size() = %d\n", path_time.size());
 // printf("path_time.end() = %f\n", path_time.back());
 // printf("path_time.begin() = %f\n", path_time.front());
  printf("calcRelativeTimeForPath end\n");
}

void MPCFollower::resamplePath2Traj(const autoware_msgs::Lane &path, const vector &time, const double &dt, MPCFollower::Trajectory &ref_traj_){
  printf("resamplePath2Traj start\n");
  // relative time should be monotonicly incresed
  ref_traj_.clear();
  double t = 0.0;
  printf("time = [%f ,~ %f]\n", time.front(), time.back());

  while (t < time.back()) {
    uint j = 1;
    while (t > time.at(j)) {
      ++j;
      if (j > time.size() - 1){
        printf("\n\n\n[MPC resampleByDt] something wrong\n\n\n");
        printf("t = %f, time.at(j-1)=%f\n", t, time.at(j-1));
        return;
      }

    }

    const double a = t - time.at(j - 1);
    const double path_dt_j = time.at(j) - time.at(j-1);
    const geometry_msgs::Pose pos0 = path.waypoints.at(j-1).pose.pose;
    const geometry_msgs::Pose pos1 = path.waypoints.at(j).pose.pose;
    const double x = ((path_dt_j - a) * pos0.position.x + a * pos1.position.x) / path_dt_j;
    const double y = ((path_dt_j - a) * pos0.position.y + a * pos1.position.y) / path_dt_j;
    const double z = ((path_dt_j - a) * pos0.position.z + a * pos1.position.z) / path_dt_j;
    const double yaw = (a * tf2::getYaw(pos0.orientation) +
                        (path_dt_j - a) * tf2::getYaw(pos1.orientation)) / path_dt_j;
                        printf("t = %f, j = %d, a = %f, dt = %f\n", t, j, a, path_dt_j);
                        printf("x0 = %f, x = %f, x1 = %f, ", pos0.position.x, x, pos1.position.x);
                        printf("y0 = %f, y = %f, y1 = %f, ", pos0.position.y, y, pos1.position.y);
                        printf("z0 = %f, z = %f, z1 = %f, ", pos0.position.z, z, pos1.position.z);
                        printf("yaw0 = %f, yaw = %f, yaw1 = %f\n", tf2::getYaw(pos0.orientation), yaw, tf2::getYaw(pos1.orientation));
    ref_traj_.push_back(x, y, z, yaw, t);
    t += dt;
  }
  // printf("x = ");
  // for (int i = 0; i < ref_traj_.x.size(); ++i){
  //   printf("%f, ", ref_traj_.x[i]);
  // }
  // printf("\n");
  // printf("y = ");
  // for (int i = 0; i < ref_traj_.y.size(); ++i){
  //   printf("%f, ", ref_traj_.y[i]);
  // }
  // printf("\n");
  printf("resamplePath2Traj end\n");
}



void MPCFollower::convertTraj2Marker(const Trajectory &traj, visualization_msgs::Marker &marker){
  marker.points.clear();
  marker.header.frame_id = path_frame_id_;
  marker.header.stamp = ros::Time();
  marker.ns = "traj";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  for (uint i = 0; i < traj.x.size(); ++i){
    geometry_msgs::Point p;
    p.x = traj.x.at(i);
    p.y = traj.y.at(i);
    p.z = traj.z.at(i);
    marker.points.push_back(p);
  }
  // printf("marker x = ");
  // for (int i = 0; i < markers.markers.size(); ++i){
  //   printf("%f, ", markers.markers[i].pose.position.x);
  // }
  // printf("\n");
  // printf("marker y = ");
  // for (int i = 0; i < markers.markers.size(); ++i){
  //   printf("%f, ", markers.markers[i].pose.position.y);
  // }
  // printf("\n");
  // printf("marker z = ");
  // for (int i = 0; i < markers.markers.size(); ++i){
  //   printf("%f, ", markers.markers[i].pose.position.z);
  // }
  // printf("\n");
  // printf("marker y = ");
  // for (int i = 0; i < markers.markers.size(); ++i){
  //   printf("%f, ", markers.markers[i].pose.orientation.w);
  // }
}




MPCFollower::~MPCFollower(){};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpc_follower");
  MPCFollower obj;

  ros::spin();

  return 0;
};
