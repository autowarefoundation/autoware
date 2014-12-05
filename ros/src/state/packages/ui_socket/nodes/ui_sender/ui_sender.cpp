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
#include "ui_socket/error_info.h"

static constexpr uint32_t_QUEUE_SIZE = 1000;
static constexpr int DEFAULT_PORT = 23456;

static port;

static void subscribe_error_info(const ui_socket::error_info& msg)
{
	int info[2];

	info[0] = 3;
	info[1] = msg.error;
}

int main(int argc, char **argv)
{
	int sock;
	int asock;

	ros::init(argc, argv, "ui_sender");

	ros::NodeHandle n;

	n.param<int>("ui_sender/port", port, DEFAULT_PORT);

	ros::Subscriber sub;

	sub = n.subscribe("error_info", QUEUE_SIZE, subscribe_error_info);

	ros::spin();

	return 0;
}
