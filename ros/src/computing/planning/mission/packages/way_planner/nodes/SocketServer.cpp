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
	m_ConnPortSend = 10001;
	m_ConnPortReceive = 10002;
	m_Socket_send = 0;
	m_Socket_receive = 0;
	sock_mutex_send =  PTHREAD_MUTEX_INITIALIZER;
	sock_mutex_receive =  PTHREAD_MUTEX_INITIALIZER;
	sock_thread_tid_send = 0;
	sock_thread_tid_receive = 0;
	m_bLatestMsg_send = false;
	m_bLatestMsg_receive = false;
	m_bExitMainLoop = false;
}

HMISocketServer::~HMISocketServer()
{
	std::cout << " >> Call The Constructor !!!!! " << std::endl;
	HMISocketServer* pRet;

	shutdown(m_Socket_send, SHUT_RDWR);
	close(m_Socket_send);

	shutdown(m_Socket_receive, SHUT_RDWR);
	 close(m_Socket_receive);

	m_bExitMainLoop = true;
	if(sock_thread_tid_send>0)
		pthread_join(sock_thread_tid_send, (void**)&pRet);

	if(sock_thread_tid_receive>0)
		pthread_join(sock_thread_tid_receive, (void**)&pRet);

	std::cout << " >> Destroy everything !!!!! " << std::endl;

}

int HMISocketServer::InitSocket(int port_send, int port_receive)
{
	if(port_send > 0)
		m_ConnPortSend = port_send;
	if(port_receive > 0)
		m_ConnPortReceive = port_receive;

	// Step 1: Creat Socket
	m_Socket_send = socket(AF_INET, SOCK_STREAM, 0);
	  if(m_Socket_send == -1){
		std::perror("socket");
		return -1;
	  }

	  sockaddr_in addr;

	  std::memset(&addr, 0, sizeof(sockaddr_in));
	  addr.sin_family = PF_INET;
	  addr.sin_port = htons(m_ConnPortSend);
	  addr.sin_addr.s_addr = INADDR_ANY;

	  int ret = bind(m_Socket_send, (struct sockaddr *)&addr, sizeof(addr));
	  if(ret == -1)
	  {
		std::perror("bind");
		return -1;
	  }

	  ret = listen(m_Socket_send, 5);
	  if(ret == -1)
	  {
		std::perror("listen");
		return -1;
	  }

	  if(pthread_create(&sock_thread_tid_send, nullptr,&HMISocketServer::ThreadMainSend , this) != 0)
	  {
		   std::perror("pthread_create");
	  }


	  m_Socket_receive = socket(AF_INET, SOCK_STREAM, 0);
	  	  if(m_Socket_receive == -1){
	  		std::perror("socket");
	  		return -1;
	  	  }

	  	  sockaddr_in addr_receive;

	  	  std::memset(&addr_receive, 0, sizeof(sockaddr_in));
	  	addr_receive.sin_family = PF_INET;
	  	addr_receive.sin_port = htons(m_ConnPortReceive);
	  	addr_receive.sin_addr.s_addr = INADDR_ANY;

	  	  int ret_r = bind(m_Socket_receive, (struct sockaddr *)&addr_receive, sizeof(addr_receive));
	  	  if(ret_r == -1)
	  	  {
	  		std::perror("bind");
	  		return -1;
	  	  }

	  	  ret = listen(m_Socket_receive, 5);
	  	  if(ret == -1)
	  	  {
	  		std::perror("listen");
	  		return -1;
	  	  }

	  if(pthread_create(&sock_thread_tid_receive, nullptr,&HMISocketServer::ThreadMainReceive , this) != 0)
	  {
		   std::perror("pthread_create");
	  }

	  return 1;
}

void* HMISocketServer::ThreadMainSend(void* pSock)
{
	HMISocketServer* pS = (HMISocketServer*)pSock;

	while(!pS->m_bExitMainLoop)
	{
		pthread_mutex_lock(&pS->sock_mutex_send);
		HMI_MSG msg = pS->m_msg_send;
		if(!pS->m_bLatestMsg_send)
			msg.options.clear();
		pS->m_bLatestMsg_send = false;
		pthread_mutex_unlock(&pS->sock_mutex_send);



		//std::cout << "Waiting access..." << std::endl;

		int client_sock = 0;
		  sockaddr_in client;
		  socklen_t len = sizeof(client);

		client_sock = accept(pS->m_Socket_send, reinterpret_cast<sockaddr*>(&client), &len);
		if(client_sock == -1){
		  std::perror("accept");
		  usleep(500);
		continue;
		}

		  std::ostringstream oss;
		  oss << msg.type << ",";
		  for(unsigned int i=0; i< msg.options.size(); i++)
			  oss << msg.options.at(i) << ";";

		  oss << "," << msg.current;
		  oss << "," << msg.currID;
		  oss << "," << msg.bErr ;
		  oss << "," << msg.err_msg << ",";

		  std::string cmd(oss.str());
		  ssize_t n = write(client_sock, cmd.c_str(), cmd.size());
		  if(n < 0){
		    std::perror("write");
		    usleep(500);
			continue;
		  }

		  shutdown(client_sock, SHUT_RDWR);
		  if(close(client_sock) == -1){
		    std::perror("close");
		    usleep(500);
			continue;
		  }

		  std::cout << "cmd: " << cmd << ", size: " << cmd.size() << std::endl;

	}
	return 0;
}

void* HMISocketServer::ThreadMainReceive(void* pSock)
{
	HMISocketServer* pS = (HMISocketServer*)pSock;

	while(!pS->m_bExitMainLoop)
	{
		//std::cout << "Waiting access..." << std::endl;

		int client_sock = 0;
		 sockaddr_in client;
		 socklen_t len = sizeof(client);


		client_sock = accept(pS->m_Socket_receive, reinterpret_cast<sockaddr*>(&client), &len);
		if(client_sock == -1){
		  std::perror("accept");
		  usleep(500);
		continue;
		}

		char recvdata[1024];
		  std::string can_data("");
		  ssize_t nr = 0;
		  while(true){
			nr = recv(client_sock, recvdata, sizeof(recvdata), 0);

			if(nr<0){
			  std::perror("recv");
			  can_data = "";
			  break;
			}else if(nr == 0){
			  break;
			}

			can_data.append(recvdata,nr);
		  }

		  //std::cout << "Command Recieved >> " << can_data << std::endl;

			 shutdown(client_sock, SHUT_RDWR);

			  if(close(client_sock)<0)
			  {
				  std::perror("close");
				  usleep(500);
				continue;
			  }

			 if(can_data.size() > 0)
			 {
				pthread_mutex_lock(&pS->sock_mutex_receive);
				pS->m_bLatestMsg_receive = true;
				pS->m_msg_receive = HMI_MSG::FromString(can_data);
				//std::cout << "Command Recieved >> " << can_data << std::endl;
				pthread_mutex_unlock(&pS->sock_mutex_receive);
			 }


	}
	return 0;
}

void HMISocketServer::SendMSG(HMI_MSG msg)
{
	pthread_mutex_lock(&sock_mutex_send);
	m_bLatestMsg_send = true;
	m_msg_send = msg;
	pthread_mutex_unlock(&sock_mutex_send);
}

int HMISocketServer::GetLatestMSG(HMI_MSG& msg)
{
	int res = -1;
	pthread_mutex_lock(&sock_mutex_receive);
	if(m_bLatestMsg_receive)
	{
		msg = m_msg_receive;
		m_bLatestMsg_receive = false;
		res = 1;
	}
	pthread_mutex_unlock(&sock_mutex_receive);

	return res;
}

}



