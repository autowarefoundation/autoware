/*
 * SocketClient.h
 *
 *  Created on: Feb 13, 2017
 *      Author: user
 */

#ifndef SOCKETCLIENT_H_
#define SOCKETCLIENT_H_

#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include "RosHelpers.h"

namespace WayPlannerNS
{
class HMISocketClient {
public:
	HMISocketClient(int port);
	virtual ~HMISocketClient();
	HMI_MSG WaitForMSG();

private:
	int m_ConnPort;
	int m_Socket;
};
}
#endif /* SOCKETCLIENT_H_ */
