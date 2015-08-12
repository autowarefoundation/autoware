/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <mutex>

#include <netinet/in.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Bool.h>
#include <tablet_socket/error_info.h>
#include <tablet_socket/mode_info.h>
#include <vehicle_socket/CanInfo.h>

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

	error_request(const tablet_socket::error_info& msg)
	: type(ERROR_INFO_TYPE), error(msg.error) {
	}
};

struct can_request {
	int32_t type;
	char tm[32];
	int32_t devmode;
	int32_t drvcontmode;
	int32_t drvoverridemode;
	int32_t drvservo;
	int32_t drivepedal;
	int32_t targetpedalstr;
	int32_t inputpedalstr;
	double targetveloc;
	double speed;
	int32_t driveshift;
	int32_t targetshift;
	int32_t inputshift;
	int32_t strmode;
	int32_t strcontmode;
	int32_t stroverridemode;
	int32_t strservo;
	int32_t targettorque;
	int32_t torque;
	double angle;
	double targetangle;
	int32_t bbrakepress;
	int32_t brakepedal;
	int32_t brtargetpedalstr;
	int32_t brinputpedalstr;
	double battery;
	int32_t voltage;
	double anp;
	int32_t battmaxtemparature;
	int32_t battmintemparature;
	double maxchgcurrent;
	double maxdischgcurrent;
	double sideacc;
	double accellfromp;
	double anglefromp;
	double brakepedalfromp;
	double speedfr;
	double speedfl;
	double speedrr;
	double speedrl;
	double velocfromp2;
	int32_t drvmode;
	int32_t devpedalstrfromp;
	int32_t rpm;
	double velocflfromp;
	int32_t ev_mode;
	int32_t temp;
	int32_t shiftfrmprius;
	int32_t light;
	int32_t gaslevel;
	int32_t door;
	int32_t cluise;

	can_request(const vehicle_socket::CanInfo& msg) {
		type = CAN_INFO_TYPE;
		msg.tm.copy(tm, 32, 0);
		devmode = msg.devmode;
		drvcontmode = msg.drvcontmode;
		drvoverridemode = msg.drvoverridemode;
		drvservo = msg.drvservo;
		drivepedal = msg.drivepedal;
		targetpedalstr = msg.targetpedalstr;
		inputpedalstr = msg.inputpedalstr;
		targetveloc = msg.targetveloc;
		speed = msg.speed;
		driveshift = msg.driveshift;
		targetshift = msg.targetshift;
		inputshift = msg.inputshift;
		strmode = msg.strmode;
		strcontmode = msg.strcontmode;
		stroverridemode = msg.stroverridemode;
		strservo = msg.strservo;
		targettorque = msg.targettorque;
		torque = msg.torque;
		angle = msg.angle;
		targetangle = msg.targetangle;
		bbrakepress = msg.bbrakepress;
		brakepedal = msg.brakepedal;
		brtargetpedalstr = msg.brtargetpedalstr;
		brinputpedalstr = msg.brinputpedalstr;
		battery = msg.battery;
		voltage = msg.voltage;
		anp = msg.anp;
		battmaxtemparature = msg.battmaxtemparature;
		battmintemparature = msg.battmintemparature;
		maxchgcurrent = msg.maxchgcurrent;
		maxdischgcurrent = msg.maxdischgcurrent;
		sideacc = msg.sideacc;
		accellfromp = msg.accellfromp;
		anglefromp = msg.anglefromp;
		brakepedalfromp = msg.brakepedalfromp;
		speedfr = msg.speedfr;
		speedfl = msg.speedfl;
		speedrr = msg.speedrr;
		speedrl = msg.speedrl;
		velocfromp2 = msg.velocfromp2;
		drvmode = msg.drvmode;
		devpedalstrfromp = msg.devpedalstrfromp;
		rpm = msg.rpm;
		velocflfromp = msg.velocflfromp;
		ev_mode = msg.ev_mode;
		temp = msg.temp;
		shiftfrmprius = msg.shiftfrmprius;
		light = msg.light;
		gaslevel = msg.gaslevel;
		door = msg.door;
		cluise = msg.cluise;
	}
};

struct mode_request {
	int32_t type;
	int32_t mode;

	mode_request(const tablet_socket::mode_info& msg)
	: type(MODE_INFO_TYPE), mode(msg.mode) {
	}
};

struct ndt_request {
	int32_t type;
	int32_t data;

	ndt_request(const std_msgs::Bool& msg) {
		type = NDT_STAT_TYPE;
		data = msg.data ? 1 : 0;
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

static void subscribe_error_info(const tablet_socket::error_info& msg)
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

static void subscribe_mode_info(const tablet_socket::mode_info& msg)
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

static void subscribe_ndt_stat(const std_msgs::Bool& msg)
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
