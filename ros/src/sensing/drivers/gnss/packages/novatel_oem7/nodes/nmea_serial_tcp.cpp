#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <nmea_msgs/Sentence.h>
#include <string>

namespace {
    ros::Publisher nmea_pub;
}

void publish(const char buf[], const int bufSize)
{
    std::stringstream stream(buf);
    std::string str;
    while (std::getline(stream, str, '\n'))
    {
        //ROS_INFO("%c\n\n",str.c_str());
        std::stringstream stream2(str);
        std::string str2;
        std::getline(stream2, str2, '\r');

        if(str.compare(0,6,"$GPGGA")==0)
        {
            //ROS_INFO("%s\n\n",str.c_str());
            std::cout<<str<<std::endl;
            nmea_msgs::Sentence sentence;
            sentence.header.stamp=ros::Time::now();
            sentence.header.frame_id="gps";
            sentence.sentence=str2;
            nmea_pub.publish(sentence);
        }
    }

    /*nmea_msgs::Sentence sentence;
    sentence.header.stamp=ros::Time::now();
    sentence.header.frame_id="gps";
    sentence.sentence=buf;
    nmea_pub.publish(sentence);*/
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"nmea_serial");
    ros::NodeHandle nh;

    int sock = socket(PF_INET, SOCK_STREAM, 0);//ソケットの作成

    //接続先指定用構造体の準備
    struct sockaddr_in server;
    server.sin_family = PF_INET;
    server.sin_port = htons(5001);
    server.sin_addr.s_addr = inet_addr("192.168.1.150");

    //サーバに接続
    connect(sock, (struct sockaddr *)&server, sizeof(server));

    nmea_pub=nh.advertise<nmea_msgs::Sentence>("nmea_sentence",100);

    //ros::Rate rate(1);
    while(ros::ok())
    {
        char buf[300];
        int readSize=read(sock, buf, sizeof(buf));
        publish(buf,readSize);
        //ROS_INFO("%s\n\n",buf);
        //rate.sleep();
    }

    close(sock);
    return 0;
}
