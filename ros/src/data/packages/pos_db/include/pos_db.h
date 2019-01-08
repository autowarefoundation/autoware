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

#ifndef _POS_DB_H_
#define _POS_DB_H_

#include <cstdint>
#include <string>
#include <netinet/in.h>
#define USE_LIBSSH2
#ifdef USE_LIBSSH2
#include <libssh2.h>
#endif

#define DB_HOSTNAME     "db3.ertl.jp"
#define DB_PORT         (5678)
#ifdef USE_LIBSSH2
#define SSHPUBKEY       "/.ssh/id_rsa.pub"
#define SSHPRIVATEKEY   "/.ssh/id_rsa"
#define SSHPORT         (22)
#define SSHTUNNELHOST	"localhost"
#endif

#define POS_DB_HEAD_LEN	(16)

class SendData {
private:
	std::string host_name_;
	int port_;
#ifdef USE_LIBSSH2
	std::string sshuser_;
	std::string sshtunnelhost_;
	std::string sshpubkey_;
	std::string sshprivatekey_;
	int sshport_;
	LIBSSH2_SESSION *session;
	LIBSSH2_CHANNEL *channel;
#endif
	int sock;
	bool connected;
	struct sockaddr_in server;

public:
	SendData();
	explicit SendData(const std::string& host_name, int port, char *sshuser,
			  std::string& sshpubkey, std::string& sshprivatekey,
			  int sshport, std::string& sshtunnelhost);

	int Sender(const std::string& value, std::string& res, int insert_num);
	int ConnectDB();
	int DisconnectDB(const char *msg);
};

extern std::string make_header(int32_t sql_inst, int32_t sql_num);
extern int probe_mac_addr(char *mac_addr);
#define MAC_ADDRBUFSIZ	20

#endif /* _POS_DB_H_ */
