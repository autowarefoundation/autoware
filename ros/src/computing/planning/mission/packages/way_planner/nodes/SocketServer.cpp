/*
 * SocketServer.cpp
 *
 *  Created on: Feb 13, 2017
 *      Author: user
 */

#include "SocketServer.h"

namespace WayPlannerNS
{


HMISocketServer::HMISocketServer()
{
	m_ConnPort = 1006;
	m_Socket = 0;
	sock_mutex =  PTHREAD_MUTEX_INITIALIZER;
	sock_thread_tid = 0;
	m_bLatestMsg = false;
	m_bExitMainLoop = false;
	m_Len = 0;
}

HMISocketServer::~HMISocketServer()
{
	HMISocketServer* pRet;
	m_bExitMainLoop = true;
	if(sock_thread_tid>0)
		pthread_join(sock_thread_tid, (void**)&pRet);

	 shutdown(m_Socket, SHUT_RDWR);
	 close(m_Socket);
}

int HMISocketServer::InitSocket(int port)
{
	if(port > 0)
		m_ConnPort = port;

	m_Socket = socket(AF_INET, SOCK_STREAM, 0);
	  if(m_Socket == -1){
		std::perror("socket");
		return -1;
	  }

	  sockaddr_in addr;

	  m_Len = sizeof(m_Client);

	  std::memset(&addr, 0, sizeof(sockaddr_in));
	  addr.sin_family = PF_INET;
	  addr.sin_port = htons(m_ConnPort);
	  addr.sin_addr.s_addr = INADDR_ANY;

	  int ret = bind(m_Socket, (struct sockaddr *)&addr, sizeof(addr));
	  if(ret == -1)
	  {
		std::perror("bind");
		return -1;
	  }

	  ret = listen(m_Socket, 1);
	  if(ret == -1)
	  {
		std::perror("listen");
		return -1;
	  }

	  if(pthread_create(&sock_thread_tid, nullptr,&HMISocketServer::ThreadMainLoop , this) != 0)
	  {
		   std::perror("pthread_create");
	  }

	  return 1;
}

void* HMISocketServer::ThreadMainLoop(void* pSock)
{
	HMISocketServer* pS = (HMISocketServer*)pSock;

	while(!pS->m_bExitMainLoop)
	{
		//pthread_mutex_lock(&pS->sock_mutex);
		bool bNewMsg = pS->m_bLatestMsg;
		HMI_MSG msg = pS->m_msg;
		//pthread_mutex_unlock(&pS->sock_mutex);

		if(!bNewMsg)
		{
			//std::cout << "Waiting for Valid Message..." << std::endl;
			//usleep(10);
			continue;
		}

//		pthread_mutex_lock(&pS->sock_mutex);
//		pS->m_bLatestMsg = false;
//		pthread_mutex_unlock(&pS->sock_mutex);



		std::cout << "Waiting access..." << std::endl;

		int *client_sock = new int();
		*client_sock = accept(pS->m_Socket, reinterpret_cast<sockaddr*>(&pS->m_Client), &pS->m_Len);
		if(*client_sock == -1){
		  std::perror("accept");
		  break;
		}

		std::cout << "get connect." << std::endl;

		  std::ostringstream oss;
		  oss << msg.type << ",";
		  oss << "op:";
		  for(unsigned int i=0; i< msg.options.size(); i++)
			  oss << msg.options.at(i) << ";";
		  oss << msg.bErr << ",";
		  oss << msg.err_msg << ",";

		  std::string cmd(oss.str());
		  ssize_t n = write(*client_sock, cmd.c_str(), cmd.size());
		  if(n < 0){
		    std::perror("write");
		    return nullptr;
		  }

		  if(close(*client_sock) == -1){
		    std::perror("close");
		    return nullptr;
		  }

		  std::cout << "cmd: " << cmd << ", size: " << cmd.size() << std::endl;

	}
	return 0;
}

void HMISocketServer::SendMSG(HMI_MSG msg)
{
	pthread_mutex_lock(&sock_mutex);
	m_bLatestMsg = true;
	m_msg = msg;
	pthread_mutex_unlock(&sock_mutex);
}

}



