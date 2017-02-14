/*
 * SocketServer.h
 *
 *  Created on: Feb 13, 2017
 *      Author: user
 */

#ifndef SOCKETSERVER_H_
#define SOCKETSERVER_H_


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
class HMISocketServer
{
public:
	HMISocketServer();
	virtual ~HMISocketServer();
	int InitSocket(int port);
	void SendMSG(HMI_MSG msg);

private:
	int m_ConnPort;
	int m_Socket;
	pthread_mutex_t sock_mutex;
	pthread_t sock_thread_tid;
	sockaddr_in m_Client;
	socklen_t m_Len;
	HMI_MSG m_msg;
	bool m_bLatestMsg;
	bool m_bExitMainLoop;

	static void* ThreadMainLoop(void* pSock);

};

}

#endif /* SOCKETSERVER_H_ */
