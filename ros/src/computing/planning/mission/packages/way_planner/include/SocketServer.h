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
	int InitSocket(int port_send, int port_receive);
	void SendMSG(HMI_MSG msg);
	int GetLatestMSG(HMI_MSG& msg);

private:
	int m_ConnPortSend;
	int m_ConnPortReceive;
	int m_Socket_send;
	int m_Socket_receive;
	pthread_mutex_t sock_mutex_send;
	pthread_t sock_thread_tid_send;
	HMI_MSG m_msg_send;
	bool m_bLatestMsg_send;
	pthread_mutex_t sock_mutex_receive;
	pthread_t sock_thread_tid_receive;
	HMI_MSG m_msg_receive;
	bool m_bLatestMsg_receive;

	//socklen_t m_Len;
	bool m_bExitMainLoop;

	static void* ThreadMainSend(void* pSock);
	static void* ThreadMainReceive(void* pSock);

};

}

#endif /* SOCKETSERVER_H_ */
