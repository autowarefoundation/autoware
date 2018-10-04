#ifndef GNSS_MATCHING_HISTORY
#define GNSS_MATCHING_HISTORY
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/gnss_surface_speed.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace gnss_matching {

class Gnss_Matching_History
{
private:
    unsigned int max_history_size;
    std::vector<geometry_msgs::PoseStamped> pose_history;
    std::vector<autoware_msgs::gnss_surface_speed> speed_history;
    std::vector<Eigen::Matrix4f> transform_history;
    void history_resize();
public:
    Gnss_Matching_History();
    ~Gnss_Matching_History();

    void set_max_history_size(unsigned int max_history_);
    void push_gnss_pose(const geometry_msgs::PoseStamped posedata_,
                        const autoware_msgs::gnss_surface_speed speeddata_,
                        const Eigen::Matrix4f transformhistory_);
    geometry_msgs::PoseStamped get_pose(unsigned int history_num) const;
    Eigen::Matrix4f get_transform(unsigned int history_num) const;
    unsigned int get_data_size() const;
};

}
#endif
