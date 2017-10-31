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
