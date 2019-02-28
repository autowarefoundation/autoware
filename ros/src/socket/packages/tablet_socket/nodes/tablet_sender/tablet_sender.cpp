/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mutex>

#include <netinet/in.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <signal.h>

#include <std_msgs/Bool.h>
#include <tablet_socket_msgs/error_info.h>
#include <tablet_socket_msgs/mode_info.h>
#include "autoware_can_msgs/CANInfo.h"
#include "autoware_msgs/NDTStat.h"

static constexpr int DEFAULT_PORT = 5777;
static constexpr int LISTEN_BACKLOG = 10;
static constexpr uint32_t QUEUE_SIZE = 1;
static constexpr double SUBSCRIBE_HZ = 1;

static constexpr int32_t ERROR_INFO_TYPE = 1;
static constexpr int32_t CAN_INFO_TYPE = 2;
static constexpr int32_t MODE_INFO_TYPE = 3;
static constexpr int32_t NDT_STAT_TYPE = 4;
static constexpr int32_t LF_STAT_TYPE = 5;

static int port;
static int connfd;
static volatile bool socket_ok;
static std::mutex mtx;

struct error_request {
	int32_t type;
	int32_t error;

	error_request(const tablet_socket_msgs::error_info& msg)
	: type(ERROR_INFO_TYPE), error(msg.error) {
	}
};

struct can_request {
	int32_t type;
	int32_t driveshift;

	can_request(const autoware_can_msgs::CANInfo& msg)
	: type(CAN_INFO_TYPE), driveshift(msg.driveshift) {
	}
};

struct mode_request {
	int32_t type;
	int32_t mode;

	mode_request(const tablet_socket_msgs::mode_info& msg)
	: type(MODE_INFO_TYPE), mode(msg.mode) {
	}
};

struct ndt_request {
	int32_t type;
	float exe_time;
	int32_t iteration;
	float score;
	float velocity;
	float acceleration;
	int32_t use_predict_pose;

	ndt_request(const autoware_msgs::NDTStat& msg) {
		type = NDT_STAT_TYPE;
		exe_time = msg.exe_time;
		iteration = msg.iteration;
		score = msg.score;
		velocity = msg.velocity;
		acceleration = msg.acceleration;
		use_predict_pose = msg.use_predict_pose;
	}
};

struct lf_request {
	int32_t type;
	int32_t data;

	lf_request(const std_msgs::Bool& msg) {
		type = LF_STAT_TYPE;
		data = msg.data ? 1 : 0;
	}
};

static void subscribe_error_info(const tablet_socket_msgs::error_info& msg)
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

static void subscribe_can_info(const autoware_can_msgs::CANInfo& msg)
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

static void subscribe_mode_info(const tablet_socket_msgs::mode_info& msg)
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

static void subscribe_ndt_stat(const autoware_msgs::NDTStat& msg)
{
	ndt_request request(msg);
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

static void subscribe_lf_stat(const std_msgs::Bool& msg)
{
	lf_request request(msg);
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

static bool send_beacon(void)
{
	int request[2];
	int response;
	ssize_t nbytes;

	memset(request, 0, sizeof(request));

	std::lock_guard<std::mutex> lock(mtx);

	nbytes = send(connfd, request, sizeof(request), 0);
	if (nbytes < 0) {
		socket_ok = false;
		return false;
	}

	nbytes = recv(connfd, &response, sizeof(response), 0);
	if (nbytes < 0) {
		socket_ok = false;
		return false;
	}

	return true;
}

int main(int argc, char **argv)
{
	int listenfd, on;
	sockaddr_in addr;

	ros::init(argc, argv, "tablet_sender");

	ros::NodeHandle n;
	n.param<int>("tablet_sender/port", port, DEFAULT_PORT);
	if (!(port >= 0 && port < 65536)) {
		ROS_ERROR("Invalid port value %d\n", port);
		return -1;
	}

	listenfd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (listenfd < 0) {
		ROS_ERROR("socket: %s", strerror(errno));
		return -1;
	}

	on = 1;
	if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on))
	    < 0) {
		ROS_ERROR("setsockopt: %s", strerror(errno));
		close(listenfd);
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(listenfd, (const struct sockaddr *)&addr, sizeof(addr)) < 0) {
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
	ros::Subscriber sub_ndt_stat = n.subscribe("ndt_stat", QUEUE_SIZE,
						   subscribe_ndt_stat);
	ros::Subscriber sub_lf_stat = n.subscribe("wf_stat", QUEUE_SIZE,
						  subscribe_lf_stat);

	ros::Rate loop_rate(SUBSCRIBE_HZ);

	struct sigaction act;
	sigaction(SIGINT, NULL, &act);
	act.sa_flags &= ~SA_RESTART;
	sigaction(SIGINT, &act, NULL);

	while (true) {
		connfd = accept(listenfd, (struct sockaddr *)nullptr,
				nullptr);
		if (connfd < 0) {
			ROS_ERROR("accept: %s", strerror(errno));
			close(listenfd);
			return -1;
		}

		socket_ok = true;
		while (socket_ok && send_beacon()) {
			ros::spinOnce();
			loop_rate.sleep();
		}

		close(connfd);
	}

	close(listenfd);

	return 0;
}
