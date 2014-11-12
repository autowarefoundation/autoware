#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/time.h>

#include "ros/ros.h"
#include "input/gearData.h"

#define ROS
#define DEFAULT_PORT	(12345)

static void getConnect(int);
#ifdef ROS
static int getSensorValue(ros::Publisher);
#else /* ROS */
static int getSensorValue();
#endif /* ROS */
static int sendSignal();

static int sock_num;
static int sock0;

int main(int argc, char **argv)
{
	int count=0;
	struct timeval start_timeval, end_timeval;
	double sec_timeofday;
#ifdef ROS
	ros::Publisher pub;
	int port;

	ros::init(argc, argv, "gear");
	ros::NodeHandle n;
	pub = n.advertise<input::gearData>("gear_data", 1);
	n.param<int>("/input/gear_port", port, DEFAULT_PORT);
	printf("port=%d\n", port);
#else /* ROS */
	int port = DEFAULT_PORT;
#endif /* ROS */

	//get connect to android
	getConnect(port);
	printf("get connect.\n");
	
	gettimeofday( &start_timeval, NULL );
#ifdef ROS
	while(1) {
		if(getSensorValue(pub) == -1)
			break;
		if(sendSignal() == -1)
			break;
		count++;
	}
#else /* ROS */
	while(count < 100000){
		//printf("count: %d\n", count);
		//get value of sensor from android	
		if(getSensorValue() == -1)
			break;
		if(sendSignal() == -1)
			break;
		count++;
	}
#endif /* ROS */
	gettimeofday( &end_timeval, NULL );
	sec_timeofday = (end_timeval.tv_sec - start_timeval.tv_sec)
			+ (end_timeval.tv_usec - start_timeval.tv_usec) / 1000000.0;

	printf("%f\n",sec_timeofday);
	return 0;
}

void getConnect(int port)
{
	struct sockaddr_in addr;
	struct sockaddr_in client;
	socklen_t len;
	int sock;
	int yes = 1;

	sock0 = socket(AF_INET, SOCK_STREAM, 0);

	addr.sin_family = AF_INET;
	addr.sin_port = htons(port); addr.sin_addr.s_addr = INADDR_ANY;
	//make it available immediately to connect
	setsockopt(sock0,SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes));
	bind(sock0, (struct sockaddr *)&addr, sizeof(addr));
	listen(sock0, 5);
	len = sizeof(client);
	sock = accept(sock0, (struct sockaddr *)&client, &len);
	if(sock == -1){
		fprintf(stderr, "ERROR: cannot accept\n");
		return ;
	}
	sock_num = sock;
}

#ifdef ROS
int getSensorValue(ros::Publisher pub)
{
#else /* ROS */
int getSensorValue(){
#endif /* ROS */
	int sensorInfo[4];
	if(recv(sock_num, &sensorInfo, 16, 0) == -1){
		fprintf(stderr, "ERROR: can not recieve message\n");
		return -1;
	}

	printf("pitch: %d, accelerator: %d, brake %d, gearNum: %d\n", sensorInfo[0], sensorInfo[1], sensorInfo[2], sensorInfo[3]);

	/*
	 * ギアボックス
	 * B:0演じブレーキ強
	 * R:1後退
	 * N:2ニュートラル
	 * D:3ドライブ
	 */

	if(sensorInfo[1] == 999 && sensorInfo[2] == 999){
		printf("STOP Andorive!!!\n");
		if(close(sock0)<0){
			printf("ERROR: can not close sock0\n");
			return -1;
		}
		if(close(sock_num)<0){
			printf("ERROR: can not close sock_num\n");
			return -1;
		}
		return -1;
	}	

#ifdef ROS
	input::gearData msg;
	msg.pitch = sensorInfo[0];
	msg.accelerator = sensorInfo[1];
	msg.brake = sensorInfo[2];
	msg.gearNum = sensorInfo[3];
	pub.publish(msg);
#endif /* ROS */

	return 0;
}

int sendSignal(){
	int signal = 0;

	if(send(sock_num, &signal, 4, 0) == -1){
		printf("ERROR: can not send signal\n");
		return -1;
	}
	return 0;
}
