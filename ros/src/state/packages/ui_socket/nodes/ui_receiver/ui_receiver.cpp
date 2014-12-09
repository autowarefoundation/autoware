#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/time.h>

//#undef NDEBUG
#include <assert.h>

#include "ros/ros.h"
#include "ui_socket/gear_info.h"
#include "ui_socket/mode_info.h"

#define NODE_NAME	"ui_receiver"
#define TOPIC_NR	(2)

#define DEFAULT_PORT	(12345)

static int getConnect(int, int *, int *);
static int getSensorValue(int, ros::Publisher[TOPIC_NR]);
static int sendSignal(int);

int main(int argc, char *argv[])
{
	ros::Publisher pub[TOPIC_NR];
	int port;
	int sock, asock;

	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;
	pub[0] = node.advertise<ui_socket::gear_info>("gear_info", 1);
	pub[1] = node.advertise<ui_socket::mode_info>("mode_info", 1);
	node.param<int>("ui_receiver/port", port, DEFAULT_PORT);
	fprintf(stderr, "listen port=%d\n", port);

	//get connect to android
	sock = -1;
	while (getConnect(port, &sock, &asock) != -1) {
		struct timeval tv[2];
		double sec;
		int count;

		fprintf(stderr, "get connect.\n");
		gettimeofday(tv, NULL);
		for (count = 0; ; count++) {
			if(getSensorValue(asock, pub) == -1)
				break;
			if(sendSignal(asock) == -1)
				break;
		}
		close(asock);
		gettimeofday(tv+1, NULL);
		sec = (tv[1].tv_sec - tv[0].tv_sec) +
			(tv[1].tv_usec - tv[0].tv_usec) / 1000000.0;
		fprintf(stderr, "done, %f sec\n",sec);
	}

	return 0;
}

static int getConnect(int port, int *sock, int *asock)
{
	struct sockaddr_in addr;
	struct sockaddr_in client;
	socklen_t len;
	int yes = 1;

	assert(sock != NULL);
	assert(asock != NULL);

	if (*sock < 0) {
		*sock = socket(AF_INET, SOCK_STREAM, 0);
		if (*sock == -1) {
			perror("socket");
			return -1;
		}

		std::memset(&addr, 0, sizeof(sockaddr_in));
		addr.sin_family = AF_INET;
		addr.sin_port = htons(port);
		addr.sin_addr.s_addr = htonl(INADDR_ANY);
		//make it available immediately to connect
		setsockopt(*sock, SOL_SOCKET, SO_REUSEADDR,
			(const char *)&yes, sizeof(yes));
		if (bind(*sock, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
			perror("bind");
			return -1;
		}
		if (listen(*sock, 5) == -1) {
			perror("listen");
			return -1;
		}
	}

	len = sizeof(client);
	*asock = accept(*sock, (struct sockaddr *)&client, &len);
	if(*asock == -1) {
		perror("accept");
		return -1;
	}

	return 0;
}

static int getSensorValue(int sock, ros::Publisher pub[TOPIC_NR])
{
	int info[2];

	if(recv(sock, info, sizeof(info), 0) == -1) {
		perror("recv");
		return -1;
	}
	fprintf(stderr, "info=%d value=%d\n", info[0], info[1]);

	switch(info[0]) {
	case 1: { // GEAR
		ui_socket::gear_info msg;
		msg.gear_num = info[1];
		pub[0].publish(msg);
		break;
	}
	case 2: { // MODE
		ui_socket::mode_info msg;
		msg.mode_num = info[1];
		pub[1].publish(msg);
		break;
	}
	default: // TERMINATOR
		fprintf(stderr, "receive %d, terminated.\n", info[0]);
		return -1;
	}

	return 0;
}

static int sendSignal(int sock)
{
	int signal = 0;

	if(send(sock, &signal, sizeof(signal), 0) == -1) {
		perror("send");
		return -1;
	}
	return 0;
}
