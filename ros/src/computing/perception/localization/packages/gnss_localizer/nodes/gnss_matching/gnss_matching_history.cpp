#include "gnss_matching_history.h"

namespace gnss_matching {

Gnss_Matching_History::Gnss_Matching_History()
    : max_history_size(10)
{
}

Gnss_Matching_History::~Gnss_Matching_History()
{
}

void Gnss_Matching_History::history_resize()
{
    if(pose_history.size() > max_history_size)
    {
        pose_history.resize(max_history_size);
        speed_history.resize(max_history_size);
        transform_history.resize(max_history_size);
    }
}

void Gnss_Matching_History::set_max_history_size(unsigned int max_history_size_)
{
    max_history_size = max_history_size_;
    history_resize();
}

void Gnss_Matching_History::push_gnss_pose(const geometry_msgs::PoseStamped posedata_,
                                           const autoware_msgs::gnss_surface_speed speeddata_,
                                           const Eigen::Matrix4f transformhistory_)
{
    pose_history.insert(pose_history.begin(),posedata_);
    speed_history.insert(speed_history.begin(),speeddata_);
    transform_history.insert(transform_history.begin(),transformhistory_);
    history_resize();
    //for(int i=0;i<pose_history.size();i++) std::cout<<transform_history[i]<<std::endl;
    //std::cout<<std::endl;
}

geometry_msgs::PoseStamped Gnss_Matching_History::get_pose(unsigned int history_num) const
{
    return pose_history[history_num];
}

Eigen::Matrix4f Gnss_Matching_History::get_transform(unsigned int history_num) const
{
    return transform_history[history_num];
}

unsigned int Gnss_Matching_History::get_data_size() const
{
    return pose_history.size();
}

}
