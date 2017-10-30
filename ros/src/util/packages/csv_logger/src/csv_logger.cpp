#include <csv_logger.h>
#define DEBUG_MODE

namespace csv_logger_namespace{
//Constructor
csv_logger::csv_logger() : private_nh_("~"){
#ifdef DEBUG_MODE
    std::cout<<"Constructor has been loaded"<<std::endl;
#endif
    initForROS();
};    
    
void csv_logger::initForROS(){
#ifdef DEBUG_MODE
    std::cout<<"initForROS has been loaded"<<std::endl;
#endif
    //Set parameters
    private_nh_.param<bool>("ndt_matching_flag", ndt_matching_flag, false);
    private_nh_.param<bool>("obj_detection_flag", obj_detection_flag, false);
    
    //Setup subscribers
    if(ndt_matching_flag){
    sub_ndt_pose = nh_.subscribe("/ndt_pose", 1, &csv_logger::ndt_pose_callback, this);
    sub_ndt_stat = nh_.subscribe("/ndt_stat", 1, &csv_logger::ndt_stat_callback, this);
#ifdef DEBUG_MODE
    std::cout<<"Subscriber for ndt_matching has been created"<<std::endl;
#endif
    }
    if(obj_detection_flag){
    sub_obj_detection = nh_.subscribe("/obj_car/image_obj_ranged", 1, &csv_logger::obj_detection_callback, this);
#ifdef DEBUG_MODE
    std::cout<<"Subscriber for obj_detection has been created"<<std::endl;
#endif
    }
    
    //Setup file for log
    char buffer[80];
    std::time_t now = std::time(NULL);
    std::tm* pnow = std::localtime(&now);
    std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
    filename = "csv_logger" + std::string(buffer) + ".csv";
    ofs.open(filename.c_str(), std::ios::app);
#ifdef DEBUG_MODE
    std::cout<<"Log file has been created"<<std::endl;
#endif    
    ofs<<"Time_sec, Time_nsed, ndt_flag, obj_flag";
    if(ndt_matching_flag){
        ofs<<"nTime_sec,nTime_nsec,velocity,acceleration,x,y,z,a,b,c,d,";
    }
    if(obj_detection_flag){
        ofs<<"oTime_sec,oTime_nsec,obj_num,x,y,width,height,score,range";
    }
    ofs<<std::endl;
    
    //Fill zero into variables
    time = ros::Time(0);
    
    ndt_matching.time = ros::Time(0);
    for(int i = 0; i < 7; i++)ndt_matching.pose[i] = 0;
    ndt_matching.velocity = 0;
    ndt_matching.acceleration = 0;
    
    obj_detection.time = ros::Time(0);
    obj_detection.obj_num = 0;
    for(int i = 0; i < 20; i ++)for(int j = 0; j < 6; j++)
        obj_detection.rect[i][j] = 0;
};    
 
void csv_logger::run(){
#ifdef DEBUG_MODE
    std::cout<<"run has been loaded"<<std::endl;
#endif
    ros::spin();
};
    
void csv_logger::log_data(){
#ifdef DEBUG_MODE
    std::cout<<"log_data has been loaded"<<std::endl;
#endif
    time = ros::Time::now();
    
    if(!ofs){
        std::cerr<<"Could not open the file."<<std::endl;
        exit(1);
    }
    
    ofs<<time.sec<<","<<time.nsec<<","<<ndt_matching_flag<<","<<obj_detection_flag<<",";
    
    if(ndt_matching_flag){
        ofs<<ndt_matching.time.sec<<","<<ndt_matching.time.nsec<<","
            <<ndt_matching.velocity<<","<<ndt_matching.acceleration<<",";
        for(int i = 0; i < 7; i++) ofs<<ndt_matching.pose[i]<<",";
#ifdef DEBUG_MODE
        std::cout<<"ndt_matching has been logged"<<std::endl;
#endif
    }
    
    if(obj_detection_flag) {
        ofs<<obj_detection.time.sec<<","<<obj_detection.time.nsec<<","<<obj_detection.obj_num<<",";
        for(int i = 0; i < obj_detection.obj_num; i++)for(int j = 0; j < 6; j++) ofs<<obj_detection.rect[i][j]<<",";
#ifdef DEBUG_MODE
        std::cout<<"obj_detection has been logged"<<std::endl;
#endif
    }
    
    ofs<<std::endl;
};

void csv_logger::ndt_pose_callback(const geometry_msgs::PoseStamped& msg){
    ndt_matching.pose[0] = msg.pose.position.x;
    ndt_matching.pose[1] = msg.pose.position.y;
    ndt_matching.pose[2] = msg.pose.position.z;
    ndt_matching.pose[3] = msg.pose.orientation.x;
    ndt_matching.pose[4] = msg.pose.orientation.y;
    ndt_matching.pose[5] = msg.pose.orientation.z;
    ndt_matching.pose[6] = msg.pose.orientation.w;
};
    
void csv_logger::ndt_stat_callback(const autoware_msgs::ndt_stat& msg){
    ndt_matching.time = msg.header.stamp;
    ndt_matching.velocity = msg.velocity;
    ndt_matching.acceleration = msg.acceleration;
    
    log_data();
};
    
void csv_logger::obj_detection_callback(const autoware_msgs::image_obj_ranged& msg){
    obj_detection.time = msg.header.stamp;
    obj_detection.obj_num = msg.obj.size();
    for(int i = 0; i < obj_detection.obj_num; i++){
        obj_detection.rect[i][0] = msg.obj.at(i).rect.x;
        obj_detection.rect[i][1] = msg.obj.at(i).rect.y;
        obj_detection.rect[i][2] = msg.obj.at(i).rect.width;
        obj_detection.rect[i][3] = msg.obj.at(i).rect.height;
        obj_detection.rect[i][4] = msg.obj.at(i).rect.score;
        obj_detection.rect[i][5] = msg.obj.at(i).range;
    }
    
    log_data();
};

}