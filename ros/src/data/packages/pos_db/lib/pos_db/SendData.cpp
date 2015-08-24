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

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <alloca.h>
#include <string>
#include <sys/time.h>

#include <pos_db.h>

#ifdef USE_LIBSSH2
#include <libssh2.h>
#endif /* USE_LIBSSH2 */

#define TIMEOUT_SEC	10

SendData::SendData()
{
}

SendData::SendData(const std::string& host_name, int port, char *sshuser, std::string& sshpubkey, std::string& sshprivatekey, int sshport, std::string& sshtunnelhost)
	: host_name_(host_name), port_(port),
	  sshuser_(sshuser), sshtunnelhost_(sshtunnelhost),
	  sshpubkey_(sshpubkey), sshprivatekey_(sshprivatekey),
	  sshport_(sshport)
{
	connected = false;
#ifdef USE_LIBSSH2
	session = NULL;
	channel = NULL;
#endif
}

int SendData::ConnectDB()
{
	int r;
	unsigned int **addrptr;

	if(connected) {
		std::cout << "The database is already connected" << std::endl;
		return -3;
	}

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		perror("socket");
		return -1;
	}

	server.sin_family = AF_INET;
	server.sin_port = htons(sshport_); // HTTP port is 80

	server.sin_addr.s_addr = inet_addr(host_name_.c_str());
	if (server.sin_addr.s_addr == 0xffffffff) {
		struct hostent *host;

		host = gethostbyname(host_name_.c_str());
		if (host == nullptr) {
			if (h_errno == HOST_NOT_FOUND) {
				// h_errno is defined in extern
				printf("host not found : %s\n", host_name_.c_str());
			} else {

				printf("%s : %s\n", hstrerror(h_errno), host_name_.c_str());
			}
			close(sock);
			return -1;
		}

		addrptr = (unsigned int **)host->h_addr_list;

		while (*addrptr != nullptr) {
			server.sin_addr.s_addr = *(*addrptr);

			// if connect() is succeed , exit loop
			if (connect(sock,
				    (struct sockaddr *)&server,
				    sizeof(server)) == 0) {
				break;
			}

			addrptr++;
			// if connect is failed, try next address
		}

		// the case of connection failed
		if (*addrptr == nullptr) {
			perror("connect");
			return -1;
		}
	} else {
		if (connect(sock,
			    (struct sockaddr *)&server, sizeof(server)) != 0) {
			perror("connect");
			return -1;
		}
	}
#ifdef USE_LIBSSH2
	session = libssh2_session_init();
	if (session == NULL) {
		fprintf(stderr, "libssh2_session_init failed.\n");
		close(sock);
		return -2;
	}
	r = libssh2_session_handshake(session, sock);
	if (r) {
		fprintf(stderr, "libssh2_session_handshake failed (%d)\n", r);
		close(sock);
		return -2;
	}
	libssh2_hostkey_hash(session, LIBSSH2_HOSTKEY_HASH_SHA1);
	if (libssh2_userauth_publickey_fromfile(session, sshuser_.c_str(), sshpubkey_.c_str(), sshprivatekey_.c_str(), "")) {
		fprintf(stderr, "libssh2_userauth_publickey_fromfile failed\n");
		libssh2_session_disconnect(session, "auth failed");
		libssh2_session_free(session);
		close(sock);
		return -2;
	}
#endif /* USE_LIBSSH2 */

	connected = true;

	return 0;
}

int SendData::DisconnectDB(const char *msg)
{
	if(connected) {
#ifdef USE_LIBSSH2
		if (channel) {
			libssh2_channel_free(channel);
			channel = NULL;
		}
	        if (session) {
			if(msg == NULL) 
				libssh2_session_disconnect(session, "normal exit");
			else
				libssh2_session_disconnect(session, msg);
			libssh2_session_free(session);
		}
#endif
		close(sock);
		connected = false;
	}

	return 0;
}

static int count_line(char *buf)
{
	int ret = 0;
	char *p;

	for (p = buf; *p; p++)
		ret += (*p == '\n') ? 1:0;

	return ret;
}

int SendData::Sender(const std::string& value, std::string& res, int insert_num) 
{
	/*********************************************
	   format data to send
	*******************************************/
	char recvdata[1024];
	int n;
	char *cvalue;
	int maxfd;
	fd_set readfds, writefds;
	struct timeval timeout;

	if(!connected) {
		std::cout << "There is no connection to the database, sock=" << sock << std::endl;
		n = ConnectDB();
		if(n < 0) return -3;
		std::cout << "connected to database\n";
	}

#ifdef USE_LIBSSH2
	channel = libssh2_channel_direct_tcpip_ex(session,
			sshtunnelhost_.c_str(), port_,
			host_name_.c_str(), sshport_);
	if (channel == NULL) {
		fprintf(stderr, "libssh2_channel_direct_tcpip_ex failed\n");
		DisconnectDB("tunnel failed");
		return -2;
	}
	libssh2_channel_flush(channel);
#endif /* USE_LIBSSH2 */

	if(value == ""){
		fprintf(stderr,"no data\n");
		return -1;
	}

	//    printf("send data : %s\n",value.c_str());
	std::cout << "send data : \"" << value << "\"" << std::endl;

	FD_ZERO(&readfds);
	FD_ZERO(&writefds);
	FD_SET(sock, &readfds);
	FD_SET(sock, &writefds);
	maxfd = sock;
	timeout.tv_sec = TIMEOUT_SEC; timeout.tv_usec = 0;
	select(maxfd+1, NULL, &writefds, NULL, &timeout);
	if(FD_ISSET(sock, &writefds)) {
		cvalue = (char *)alloca(value.size());
		memcpy(cvalue, value.c_str(), value.size());
		for (int i=0; i<16; i++)
			cvalue[i] &= 0x7f; // TODO: see make_header()
#ifdef USE_LIBSSH2
		n = libssh2_channel_write(channel, cvalue, value.size());
#else /* USE_LIBSSH2 */
		n = write(sock, cvalue, value.size());
#endif /* USE_LIBSSH2 */
		if (n < 0) {
			perror("write");
			return -1;
		}
		std::cout << "write return n=" << n << std::endl;
	} else {
		DisconnectDB("tunnel failed");
		return -5;
	}

	//Caution : If server do not close in one communication,loop forever because read() do not return 0.
	timeout.tv_sec = TIMEOUT_SEC; timeout.tv_usec = 0;
	select(maxfd+1, &readfds, NULL, NULL, &timeout);
	if(FD_ISSET(sock, &readfds)) {
		int len;
#ifdef USE_LIBSSH2
		n = libssh2_channel_read(channel, (char *)&len, sizeof(len));
#else /* USE_LIBSSH2 */
		n = read(sock, (char *)&len, sizeof(len));
#endif /* USE_LIBSSH2 */
		if (n < 0) {
			perror("recv error");
			return -1;
		}
		len = ntohl(len);
		std::cerr << "recv len=" << len << std::endl;
		if(len == insert_num) return 0;

		for (int j = 0; j < len; ) {
			memset(recvdata, 0, sizeof(recvdata));
#ifdef USE_LIBSSH2
			n = libssh2_channel_read(channel, recvdata, sizeof(recvdata)-1);
#else /* USE_LIBSSH2 */
			n = read(sock, recvdata, sizeof(recvdata)-1);
#endif /* USE_LIBSSH2 */

			if (n < 0) {
				perror("recv error");
				return -1;
			} else if (n==0) {
				break;
			}
			std::cout << "read return n=" << n << std::endl;

			recvdata[n] = '\0';
			int hlen = 0;
			for (int i=0; i<n && hlen<4; i++, hlen++)
				if(recvdata[i] == 0) recvdata[i] |= 0x80;
			if (strlen(recvdata) >= sizeof(recvdata)) {
				res.append(recvdata, sizeof(recvdata));
			} else {
				res.append(recvdata, strlen(recvdata));
			}

			j += count_line(recvdata);
			std::cerr << "count_line=" << j << std::endl;
		}
	} else {
		DisconnectDB("tunnel failed");
		return -5;
	}

#ifdef USE_LIBSSH2
	if (channel) {
		libssh2_channel_free(channel);
		channel = NULL;
	}
#endif

	return 0;
}
