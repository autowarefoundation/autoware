#include <ros/ros.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
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

        if(str.compare(0,6,"$GPVTG")==0 || str.compare(0,6,"$GPGGA")==0 ||
                str.compare(0,6,"$PASHR")==0 || str.compare(0,6,"$GPHDT")==0 ||
                str.compare(0,6,"$GPGST") ||str.compare(0,6,"$GPGSA"))
        {
            //ROS_INFO("%s\n\n",str.c_str());
            //std::cout<<str<<std::endl;
            nmea_msgs::Sentence sentence;
            sentence.header.stamp=ros::Time::now();
            sentence.header.frame_id="gps";
            sentence.sentence=str2;
            nmea_pub.publish(sentence);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"nmea_serial");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string port;
    int baud=0;
    private_nh.getParam("port", port);
    private_nh.getParam("baud", baud);
    if(baud == 110) baud = B110;
    else if(baud == 300) baud = B300;
    else if(baud == 1200) baud = B1200;
    else if(baud == 2400) baud = B2400;
    else if(baud == 4800) baud = B4800;
    else if(baud == 9600) baud = B9600;
    else if(baud == 19200) baud = B19200;
    else if(baud == 38400) baud = B38400;
    else if(baud == 57600) baud = B57600;
    else if(baud == 115200) baud = B115200;
    else if(baud == 230400) baud = B230400;
    else
    {
        std::cout<<"The baud rate can not be specified."<<std::endl;
        return -1;
    }

    const std::string SERIAL_PORT=port; //"/dev/ttyUSB0";//デバイス名は適宜変えてください
    unsigned char msg[] = "serial port open...\n";
    unsigned char buf[255];             // バッファ
    int fd;                             // ファイルディスクリプタ
    struct termios tio;                 // シリアル通信設定
    int baudRate = baud;
    int i;
    int len;
    int ret;
    int size;

    fd = open(SERIAL_PORT.c_str(), O_RDWR);     // デバイスをオープンする
    if (fd < 0) {
        printf("open error\n");
        return -1;
    }

    tio.c_cflag += CREAD;               // 受信有効
    tio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
    tio.c_cflag += CS8;                 // データビット:8bit
    tio.c_cflag += 0;                   // ストップビット:1bit
    tio.c_cflag += 0;                   // パリティ:None

    cfsetispeed( &tio, baudRate );
    cfsetospeed( &tio, baudRate );

    cfmakeraw(&tio);                    // RAWモードros advertise true

    tcsetattr( fd, TCSANOW, &tio );     // デバイスに設定を行う

    ioctl(fd, TCSETS, &tio);            // ポートの設定を有効にする

    nmea_pub=nh.advertise<nmea_msgs::Sentence>("nmea_sentence",100,false);

    //ros::Rate rate(1);
    while(ros::ok())
    {
        char buf[300];
        len = read(fd, buf, sizeof(buf));
        //std::cout<<buf<<std::endl;
        //std::cout<<len<<std::endl;
        buf[len]='\0';
        publish(buf,len);
        //printf("%s\n\n",buf);
        //rate.sleep();
    }

    close(fd);
    return 0;
}
