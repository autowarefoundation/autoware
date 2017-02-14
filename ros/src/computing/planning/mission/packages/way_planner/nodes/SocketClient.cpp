/*
 * SocketClient.cpp
 *
 *  Created on: Feb 13, 2017
 *      Author: user
 */

#include "SocketClient.h"
namespace WayPlannerNS
{
HMISocketClient::HMISocketClient(int port) {

	m_ConnPort = 1005;
	m_Socket = 0;

}

HMISocketClient::~HMISocketClient() {

}

}
