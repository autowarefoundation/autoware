#include <mutex>

#include <netinet/in.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <ui_socket/error_info.h>
#include <ui_socket/mode_info.h>

#include <vehicle_socket/CanInfo.h>

#include "request.h"

static constexpr int DEFAULT_PORT = 23456;
static constexpr int LISTEN_BACKLOG = 10;
static constexpr uint32_t QUEUE_SIZE = 1000;

static int port;
static int connfd;
static volatile bool socket_ok;
std::mutex mtx;

static void subscribe_error_info(const ui_socket::error_info& msg)
{
	error_request request(msg);
	int response;
	ssize_t nbytes;

	std::lock_guard<std::mutex> lock(mtx);

	nbytes = send(connfd, &request, sizeof(request), 0);
	if (nbytes < 0) {
		ROS_ERROR("send: %s", strerror(errno));
		socket_ok = false;
		return;
	}
	if ((size_t)nbytes < sizeof(request)) {
		ROS_WARN("send: %zd bytes remaining",
			 sizeof(request) - nbytes);
		return;
	}

	nbytes = recv(connfd, &response, sizeof(response), 0);
	if (nbytes < 0) {
		ROS_ERROR("recv: %s", strerror(errno));
		socket_ok = false;
		return;
	}
	if ((size_t)nbytes < sizeof(response)) {
		ROS_WARN("recv: %zd bytes remaining",
			 sizeof(response) - nbytes);
		return;
	}
}

static void subscribe_can_info(const vehicle_socket::CanInfo& msg)
{
	can_request request(msg);
	int response;
	ssize_t nbytes;

	std::lock_guard<std::mutex> lock(mtx);

	nbytes = send(connfd, &request, sizeof(request), 0);
	if (nbytes < 0) {
		ROS_ERROR("send: %s", strerror(errno));
		socket_ok = false;
		return;
	}
	if ((size_t)nbytes < sizeof(request)) {
		ROS_WARN("send: %zd bytes remaining",
			 sizeof(request) - nbytes);
		return;
	}

	nbytes = recv(connfd, &response, sizeof(response), 0);
	if (nbytes < 0) {
		ROS_ERROR("recv: %s", strerror(errno));
		socket_ok = false;
		return;
	}
	if ((size_t)nbytes < sizeof(response)) {
		ROS_WARN("recv: %zd bytes remaining",
			 sizeof(response) - nbytes);
		return;
	}
}

static void subscribe_mode_info(const ui_socket::mode_info& msg)
{
	mode_request request(msg);
	int response;
	ssize_t nbytes;

	std::lock_guard<std::mutex> lock(mtx);

	nbytes = send(connfd, &request, sizeof(request), 0);
	if (nbytes < 0) {
		ROS_ERROR("send: %s", strerror(errno));
		socket_ok = false;
		return;
	}
	if ((size_t)nbytes < sizeof(request)) {
		ROS_WARN("send: %zd bytes remaining",
			 sizeof(request) - nbytes);
		return;
	}

	nbytes = recv(connfd, &response, sizeof(response), 0);
	if (nbytes < 0) {
		ROS_ERROR("recv: %s", strerror(errno));
		socket_ok = false;
		return;
	}
	if ((size_t)nbytes < sizeof(response)) {
		ROS_WARN("recv: %zd bytes remaining",
			 sizeof(response) - nbytes);
		return;
	}
}

int main(int argc, char **argv)
{
	int listenfd;
	sockaddr_in addr;

	ros::init(argc, argv, "ui_sender");

	ros::NodeHandle n;
	n.param<int>("ui_sender/port", port, DEFAULT_PORT);
	if (!(port >= 0 && port < 65536)) {
		ROS_ERROR("Invalid port value %d\n", port);
		return -1;
	}

	listenfd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (listenfd < 0) {
		ROS_ERROR("socket: %s", strerror(errno));
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(listenfd, (const struct sockaddr *)&addr,
		 sizeof(addr)) < 0) {
		ROS_ERROR("bind: %s", strerror(errno));
		close(listenfd);
		return -1;
	}

	if (listen(listenfd, LISTEN_BACKLOG) < 0) {
		ROS_ERROR("listen: %s", strerror(errno));
		close(listenfd);
		return -1;
	}

	ros::Subscriber sub_error_info = n.subscribe("error_info", QUEUE_SIZE,
						     subscribe_error_info);
	ros::Subscriber sub_can_info = n.subscribe("can_info", QUEUE_SIZE,
						   subscribe_can_info);
	ros::Subscriber sub_mode_info = n.subscribe("mode_info", QUEUE_SIZE,
						    subscribe_mode_info);

	while (true) {
		connfd = accept(listenfd, (struct sockaddr *)nullptr,
				nullptr);
		if (connfd < 0) {
			ROS_ERROR("accept: %s", strerror(errno));
			close(listenfd);
			return -1;
		}

		socket_ok = true;
		while (socket_ok) {
			ros::spinOnce();
		}

		close(connfd);
	}

	close(listenfd);

	return 0;
}
