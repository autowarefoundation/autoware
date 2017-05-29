// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

namespace jsk_topic_tools
{
  class PoseStampedTransformer
  {
  public:
    typedef boost::shared_ptr<PoseStampedTransformer> Ptr;
    PoseStampedTransformer(double x, double y, double z,
                           double yaw, double pitch, double roll,
                           std::string from_topic,
                           std::string to_topic);
    virtual ~PoseStampedTransformer();
  protected:
    void transform(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    ros::Subscriber sub_;
    ros::Publisher pub_;
    double x_;
    double y_;
    double z_;
    double roll_;
    double pitch_;
    double yaw_;
  private:
    
  };

  PoseStampedTransformer::PoseStampedTransformer(
    double x, double y, double z,
    double yaw, double pitch, double roll,
    std::string from_topic, std::string to_topic):
    x_(x), y_(y), z_(z), yaw_(yaw), pitch_(pitch), roll_(roll)
  {
    ros::NodeHandle nh;
    pub_ = nh.advertise<geometry_msgs::PoseStamped>(
      to_topic, 1);
    sub_ = nh.subscribe(from_topic, 1,
                        &PoseStampedTransformer::transform, this);
  }

  PoseStampedTransformer::~PoseStampedTransformer()
  {

  }

  void PoseStampedTransformer::transform(
    const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
  {
    Eigen::Affine3d input_transform;
    tf::poseMsgToEigen(pose_msg->pose, input_transform);
    Eigen::AngleAxisd roll_angle(roll_, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yaw_angle(yaw_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitch_angle(pitch_, Eigen::Vector3d::UnitX());

    Eigen::Affine3d transformation
      = Eigen::Translation3d(x_, y_, z_) * roll_angle * yaw_angle * pitch_angle;
    Eigen::Affine3d output_transform = input_transform * transformation;
    geometry_msgs::PoseStamped output_pose_msg;
    tf::poseEigenToMsg(output_transform, output_pose_msg.pose);
    output_pose_msg.header = pose_msg->header;
    pub_.publish(output_pose_msg);
  }
  
}

int usage(char** argv)
{
  std::cerr << "Usage:: " << argv[0]
            << " x y z yaw pitch roll from_topic to_topic"
            << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_transform_pose_stamped",
            ros::init_options::AnonymousName);
  // x y z yaw pitch roll
  if (argc != 9) {
    usage(argv);
    exit(1);
  }
  // parse argument
  jsk_topic_tools::PoseStampedTransformer p(::atof(argv[1]),
                                            ::atof(argv[2]),
                                            ::atof(argv[3]),
                                            ::atof(argv[4]),
                                            ::atof(argv[5]),
                                            ::atof(argv[6]),
                                            argv[7],
                                            argv[8]);
  ros::spin();
}
