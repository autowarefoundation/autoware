#include <ros/ros.h>
#include <istream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_msgs/gnss_standard_deviation.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

const int max_localizer_count = 2;
const int SYNC_FRAMES = 10;

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped, autoware_msgs::NDTStat>
    NdtlocalizerSync;

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped, autoware_msgs::gnss_standard_deviation>
    RTKlocalizerSync;

class TopicList
{
private:
    ros::NodeHandle nh_, private_nh_;

    message_filters::Subscriber<geometry_msgs::PoseStamped>             *base_link_pose_sub_;
    message_filters::Subscriber<geometry_msgs::TwistStamped>            *estimate_twist_sub_;
    message_filters::Subscriber<geometry_msgs::PoseStamped>             *localizer_pose_sub_;
    message_filters::Subscriber<autoware_msgs::NDTStat>                 *ndt_status_sub_;
    message_filters::Subscriber<autoware_msgs::gnss_standard_deviation> *gnss_deviation_sub_;
    message_filters::Synchronizer<NdtlocalizerSync> *sync_ndt_;
    message_filters::Synchronizer<RTKlocalizerSync> *sync_RTK_;

    //Use localizer topic group
    std::string base_link_pose_topic_, estimate_twist_topic_, localizer_pose_topic_;
    //localizer's approach
    int approach_;
    //status topic to use when approach is ndt
    std::string ndt_status_topic_;
    //status topic to use when approach is GNSS(RTK)
    std::string gnss_deviation_topic_;

    const void NdtlocalizerCallback(const geometry_msgs::PoseStampedConstPtr &base_link_pose_msg,
                                    const geometry_msgs::TwistStampedConstPtr &estimate_twist_msg,
                                    const geometry_msgs::PoseStampedConstPtr &localizer_pose_msg,
                                    const autoware_msgs::NDTStatConstPtr &ndt_stConstPtratus_msg)
    {
        std::cout << "aaa\n";
    }

    const void RTKlocalizerCallback(const geometry_msgs::PoseStampedConstPtr &base_link_pose_msg,
                                    const geometry_msgs::TwistStampedConstPtr &estimate_twist_msg,
                                    const geometry_msgs::PoseStampedConstPtr &localizer_pose_msg,
                                    const autoware_msgs::gnss_standard_deviationConstPtr &gnss_deviation_msg)
    {
        std::cout << "bbb\n";
    }
public:
    TopicList(ros::NodeHandle nh, ros::NodeHandle private_nh,
              std::string baseLinkPoseTopic, std::string estimateTwistTopic, std::string localizerPoseTopic,
              int approachFlag, std::string ndtStatusTopic, std::string gnssDeviationTopic)
        : nh_(nh)
        , private_nh_(private_nh)
    {
        base_link_pose_topic_ = baseLinkPoseTopic;   estimate_twist_topic_ = estimateTwistTopic;
        localizer_pose_topic_ = localizerPoseTopic;
        approach_ = approachFlag;
        ndt_status_topic_ = ndtStatusTopic;
        gnss_deviation_topic_ = gnssDeviationTopic;
    }

    std::string get_base_link_pose_topic() {return base_link_pose_topic_; }
    std::string get_estimate_twist_topic() {return estimate_twist_topic_; }
    std::string get_localizer_pose_topic() {return localizer_pose_topic_; }
    int         get_approach() {return approach_; }
    std::string get_ndt_status_topic() {return ndt_status_topic_; }
    std::string get_gnss_deviation_topic() {return gnss_deviation_topic_; }

    void callback_run()
    {
        // subscriber
        base_link_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, base_link_pose_topic_, 10);
        estimate_twist_sub_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, estimate_twist_topic_, 10);
        localizer_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, localizer_pose_topic_, 10);
        switch(approach_)
        {
        case 0://ndt
            {
                ndt_status_sub_ = new message_filters::Subscriber<autoware_msgs::NDTStat>(nh_, ndt_status_topic_, 10);
                sync_ndt_ = new message_filters::Synchronizer<NdtlocalizerSync>(NdtlocalizerSync(SYNC_FRAMES),
                                  *base_link_pose_sub_, *estimate_twist_sub_, *localizer_pose_sub_, *ndt_status_sub_);
                sync_ndt_->registerCallback(boost::bind(&TopicList::NdtlocalizerCallback, this, _1, _2, _3, _4));
            }
        case 1://gnss(RTK)
            {
                gnss_deviation_sub_ = new message_filters::Subscriber<autoware_msgs::gnss_standard_deviation>(nh_, gnss_deviation_topic_, 10);
                sync_RTK_ = new message_filters::Synchronizer<RTKlocalizerSync>(RTKlocalizerSync(SYNC_FRAMES),
                                  *base_link_pose_sub_, *estimate_twist_sub_, *localizer_pose_sub_, *gnss_deviation_sub_);
                sync_RTK_->registerCallback(boost::bind(&TopicList::RTKlocalizerCallback, this, _1, _2, _3, _4));
            }
        }
    }
};

class LocalizerSwitch
{
private:
    std::vector<TopicList> topic_list_;
    int trigger_topic_number_;//
public:
    LocalizerSwitch(std::vector<TopicList> list, int trigger_topic_number)
    {
        topic_list_ = list;
        trigger_topic_number_ = trigger_topic_number;

        for(int i=0; i<topic_list_.size(); i++)
        {
            topic_list_[i].callback_run();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_matching");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::vector<TopicList> topicListArray;

    for(int cou=1; cou<=max_localizer_count; cou++)
    {
        std::string base_link_pose_topic, estimate_twist_topic, localizer_pose_topic;
        std::stringstream base_link_pose_name, estimate_twist_name, localizer_pose_name, base_link_tf_name, alignment_mechanism_name;

        base_link_pose_name << "base_link_pose" << cou;
        private_nh.param<std::string>(base_link_pose_name.str(), base_link_pose_topic, std::string(""));
        std::cout << base_link_pose_name.str() << " : " << base_link_pose_topic << std::endl;

        estimate_twist_name << "estimate_twist" << cou;
        private_nh.param<std::string>(estimate_twist_name.str(), estimate_twist_topic, std::string(""));
        std::cout << estimate_twist_name.str() << " : " << estimate_twist_topic << std::endl;

        localizer_pose_name << "localizer_pose" << cou;
        private_nh.param<std::string>(localizer_pose_name.str(), localizer_pose_topic, std::string(""));
        std::cout << localizer_pose_name.str() << " : " << localizer_pose_topic << std::endl;

        alignment_mechanism_name << "alignment_mechanism" << cou;
        int alignment_mechanism;
        private_nh.param<int>(alignment_mechanism_name.str(), alignment_mechanism, 0);
        std::cout << alignment_mechanism_name.str() << " : " << alignment_mechanism << std::endl;

        std::string ndt_status_topic="", gnss_deviation_topic="";
        std::stringstream ndt_status_name, gnss_deviation_name;

        switch(alignment_mechanism)
        {
        case 0://ndt
            {
                ndt_status_name << "ndt_status" << cou;
                private_nh.param<std::string>(ndt_status_name.str(), ndt_status_topic, std::string(""));
                std::cout << ndt_status_name.str() << " : " << ndt_status_topic << std::endl;
                break;
            }
        case 1://GNSS(RTK)
            {
                gnss_deviation_name << "gnss_deviation" << cou;
                private_nh.param<std::string>(gnss_deviation_name.str(), gnss_deviation_topic, std::string(""));
                std::cout << gnss_deviation_name.str() << " : " << gnss_deviation_topic << std::endl;
                break;
            }
        }

        TopicList list(nh, private_nh,
                       base_link_pose_topic, estimate_twist_topic, localizer_pose_topic,
                       alignment_mechanism, ndt_status_topic, gnss_deviation_topic);
        topicListArray.push_back(list);
    }

    std::string trigger_topic_number_str;
    private_nh.param<std::string>("trigger_topic_number", trigger_topic_number_str, std::string("1"));
    int trigger_topic_number;
    try{ trigger_topic_number = stoi(trigger_topic_number_str); }
    catch(std::invalid_argument)
    {
        std::cerr << "error : trigger_topic_number : No conversion to numbers was done" << std::endl;
        exit(-1);
    }
    catch(std::out_of_range)
    {
        std::cerr << "error : trigger_topic_number : Numeric value is out of range" << std::endl;
        exit(-1);
    }
    std::cout << "trigger_topic_number : " << trigger_topic_number << std::endl;

    LocalizerSwitch localizer_switch(topicListArray, trigger_topic_number-1);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
