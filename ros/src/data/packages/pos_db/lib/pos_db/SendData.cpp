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
			close(sock);
			return -1;
		}
	} else {
		if (connect(sock,
			    (struct sockaddr *)&server, sizeof(server)) != 0) {
			perror("connect");
			close(sock);
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
		libssh2_session_free(session);
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
			session = NULL;
		}
#endif
		close(sock);
		sock = -1;
		connected = false;
	}

	return 0;
}

static int count_line(const char *buf)
{
	int ret = 0;
	const char *p;

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
#ifndef USE_LIBSSH2
	int maxfd;
	fd_set readfds, writefds;
	struct timeval timeout;
#endif /* !USE_LIBSSH2 */

	if(!connected) {
		std::cout << "There is no connection to the database, sock=" << sock << std::endl;
		n = ConnectDB();
		if(n < 0) return -3;
		std::cout << "connected to database\n";
	}

#ifdef USE_LIBSSH2
	long oldtoutmsec = libssh2_session_get_timeout(session);
	libssh2_session_set_timeout(session, TIMEOUT_SEC*1000);
	std::cout << "timeout set " << oldtoutmsec << "msec to "
		<< TIMEOUT_SEC*1000 << "msec" << std::endl;
	time_t t1 = time(NULL);
	channel = libssh2_channel_direct_tcpip_ex(session,
			sshtunnelhost_.c_str(), port_,
			host_name_.c_str(), sshport_);
	time_t t2 = time(NULL);
	if (channel == NULL) {
		std::cerr << "libssh2_channel_direct_tcpip_ex failed, diff="
			<< t2-t1 << std::endl;
		DisconnectDB("tunnel failed");
		return -2;
	}
	std::cout << "channel done, diff=" << t2-t1 << ", time=" << t2 << std::endl;
	libssh2_channel_flush(channel);
#endif /* USE_LIBSSH2 */

	if(value == ""){
		fprintf(stderr,"no data\n");
		return -1;
	}

#ifdef POS_DB_VERBOSE
	std::cout << "send data : \"" << value.substr(POS_DB_HEAD_LEN) << "\"" << std::endl;
#endif /* POS_DB_VERBOSE */

	cvalue = (char *)alloca(value.size());
	memcpy(cvalue, value.c_str(), value.size());
	for (int i=0; i<POS_DB_HEAD_LEN; i++)
		cvalue[i] &= 0x7f; // TODO: see make_header()
#ifdef USE_LIBSSH2
	n = libssh2_channel_write(channel, cvalue, value.size());
#else /* USE_LIBSSH2 */
	FD_ZERO(&readfds);
	FD_ZERO(&writefds);
	FD_SET(sock, &writefds);
	maxfd = sock;
	timeout.tv_sec = TIMEOUT_SEC; timeout.tv_usec = 0;
	select(maxfd+1, NULL, &writefds, NULL, &timeout);
	if(FD_ISSET(sock, &writefds)) {
		n = write(sock, cvalue, value.size());
	} else {
		n = 0;
	}
#endif /* USE_LIBSSH2 */
	if (n < 0) {
		perror("write");
		DisconnectDB("tunnel failed");
		return -1-n;
	} else if (n == 0) {
		std::cerr << "write timed out" << std::endl;
		DisconnectDB("tunnel failed");
		return -1-n;
	}
	std::cout << "write done, size=" << value.size() << ", n=" << n << std::endl;

	//Caution : If server do not close in one communication,loop forever because read() do not return 0.
	int len;
#ifdef USE_LIBSSH2
	n = libssh2_channel_read(channel, (char *)&len, sizeof(len));
#else /* USE_LIBSSH2 */
	timeout.tv_sec = TIMEOUT_SEC; timeout.tv_usec = 0;
	FD_ZERO(&readfds);
	FD_ZERO(&writefds);
	FD_SET(sock, &readfds);
	select(maxfd+1, &readfds, NULL, NULL, &timeout);
	if(FD_ISSET(sock, &readfds)) {
		n = read(sock, (char *)&len, sizeof(len));
	} else {
		n = 0;
	}
#endif /* USE_LIBSSH2 */
	if (n < 0) {
		perror("read");
		DisconnectDB("tunnel failed");
		return -1-n;
	} else if (n == 0) {
		std::cerr << "read timed out" << std::endl;
		DisconnectDB("tunnel failed");
		return -1;
	}
	len = ntohl(len);
	std::cerr << "read count done, len=" << len << std::endl;
	if(len == insert_num) return 0;

	for (int j = 0; j < len; ) {
		memset(recvdata, 0, sizeof(recvdata));
#ifdef USE_LIBSSH2
		n = libssh2_channel_read(channel, recvdata, sizeof(recvdata)-1);
#else /* USE_LIBSSH2 */
		timeout.tv_sec = TIMEOUT_SEC; timeout.tv_usec = 0;
		FD_ZERO(&readfds);
		FD_ZERO(&writefds);
		FD_SET(sock, &readfds);
		select(maxfd+1, &readfds, NULL, NULL, &timeout);
		if(FD_ISSET(sock, &readfds)) {
			n = read(sock, recvdata, sizeof(recvdata)-1);
		} else {
			n = 0;
		}
#endif /* USE_LIBSSH2 */
		if (n < 0) {
			perror("read");
			DisconnectDB("tunnel failed");
			return -1-n;
		} else if (n == 0) {
			std::cerr << "read timed out" << std::endl;
			DisconnectDB("tunnel failed");
			return -1;
		}
#ifdef POS_DB_VERBOSE
		std::cerr << "read return n=" << n << std::endl;
#endif /* POS_DB_VERBOSE */

		recvdata[n] = '\0';
		int hlen = 0;
		for (int i=0; i<n && hlen<4; i++, hlen++)
			if(recvdata[i] == 0) recvdata[i] |= 0x80;
		if (strlen(recvdata) >= sizeof(recvdata)) {
			res.append(recvdata, sizeof(recvdata));
		} else {
			res.append(recvdata, strlen(recvdata));
		}

		j = count_line(res.c_str());
#ifdef POS_DB_VERBOSE
		std::cerr << "count_line=" << j << std::endl;
#endif /* POS_DB_VERBOSE */
	}
	std::cerr << "read data done, count_line=" << count_line(res.c_str()) << std::endl;

#ifdef USE_LIBSSH2
	if (channel) {
		libssh2_channel_free(channel);
		channel = NULL;
	}
#endif

	return 0;
}
