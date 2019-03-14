#include "get_PCD.hpp"
// catkin_create_pkg pcd_tutorial std_msgs roscpp pcl_ros pcl_msgs pcl_conversions libpcl-all-dev

static void check_arguments(int argc, char* argv[])
{
  if (argc != 3){
    cout << "Please set arguments like below\n'rosrun data_preprocessor get_PCD save_dir topic_name'\n";
    exit(EXIT_FAILURE);
  }
}

void SavePCD::save_pcd(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  stringstream ss;
  ss << msg->header.stamp;
  string file_name = save_path_ + ss.str() + ".pcd";
  pcl::PointCloud<pcl::PointXYZ> points;
  pcl::fromROSMsg(*msg, points);
  pcl::io::savePCDFileASCII(file_name, points);
}

void SavePCD::sub_pcd(int argc, char* argv[])
{
  ros::init(argc, argv, "PCD_Subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(topic_name_, 1, &SavePCD::save_pcd, this);
  ros::spin();
}

int main(int argc, char* argv[])
{
  check_arguments(argc, argv);
  SavePCD si;
  string path = argv[1];
  if (path[path.size() - 1] == '/'){
    path.erase(path.begin() + path.size()-1);
  }
  si.save_path_ = path + '/';
  si.topic_name_ = argv[2];
  si.sub_pcd(argc, argv);
  cout << "finish\n";
  return 0;
}
