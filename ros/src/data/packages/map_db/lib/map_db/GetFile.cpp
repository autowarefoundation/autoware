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
#include <fstream>
#include <sstream>

#include <map_db.h>

#define TIMEOUT_SEC	10

GetFile::GetFile()
	: GetFile(HTTP_HOSTNAME, HTTP_PORT)
{
}

GetFile::GetFile(const std::string& host_name, int port)
	: host_name_(host_name), port_(port)
{
	connected = false;
}

int GetFile::ConnectHTTP()
{
	unsigned int **addrptr;

#if 0
	if(connected) {
		std::cout << "The database is already connected" << std::endl;
		return -3;
	}
#endif

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		perror("socket");
		return -1;
	}

	server.sin_family = AF_INET;
	server.sin_port = htons(port_); // HTTP port is 80

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

	connected = true;

	return 0;
}

int GetFile::DisconnectHTTP(const char *msg)
{
	if(connected) {
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

int GetFile::GetHTTPFile(const std::string& value) 
{
	char send_buf[256];
	int n;
	char recvdata[1024];
	std::string res;

	ConnectHTTP();
	snprintf(send_buf, sizeof(send_buf), "GET %s HTTP/1.1\r\n", value.c_str());
	write(sock, send_buf, strlen(send_buf));
	snprintf(send_buf, sizeof(send_buf), "Host: %s:%d\r\n", host_name_.c_str(), port_);
	write(sock, send_buf, strlen(send_buf));
	snprintf(send_buf, sizeof(send_buf), "\r\n");
	write(sock, send_buf, strlen(send_buf));

	while(1) {
		n = read(sock, recvdata, sizeof(recvdata)-1);
		if (n < 0) {
			perror("recv error");
			return -1;
		} else if (n==0) {
			break;
		}
		recvdata[n] = '\0';

		if (strlen(recvdata) >= sizeof(recvdata)) {
			res.append(recvdata, sizeof(recvdata));
		} else {
			res.append(recvdata, strlen(recvdata));
		}
	}

	if(res.find("HTTP/1.1 200 OK", 0) == std::string::npos) return -1;

	std::istringstream ss(res);
	std::string tbuf;
	for(int i = 0; i < 10; i++) {
		std::getline(ss, tbuf);	// ignore http header
		if(tbuf.size() <= 1) break;
	}

	std::ofstream ofs;
	ofs.open("/tmp/"+value);
	while(std::getline(ss, tbuf)) ofs << tbuf << std::endl;
	ofs.close();

	return 0;
}

int GetFile::Sender(const std::string& value, std::string& res, int insert_num) 
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
		n = ConnectHTTP();
		if(n < 0) return -3;
		std::cout << "connected to database\n";
	}

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
#if 0	///???
		for (int i=0; i<16; i++)
			cvalue[i] &= 0x7f; // TODO: see make_header()
#endif
		n = write(sock, cvalue, value.size());
		if (n < 0) {
			perror("write");
			return -1;
		}
		std::cout << "write return n=" << n << std::endl;
	} else {
		DisconnectHTTP("tunnel failed");
		return -5;
	}

	//Caution : If server do not close in one communication,loop forever because read() do not return 0.
	timeout.tv_sec = TIMEOUT_SEC; timeout.tv_usec = 0;
	select(maxfd+1, &readfds, NULL, NULL, &timeout);
	if(FD_ISSET(sock, &readfds)) {
		int len;
		n = read(sock, (char *)&len, sizeof(len));
		if (n < 0) {
			perror("recv error");
			return -1;
		}
		len = ntohl(len);
		std::cerr << "recv len=" << len << std::endl;
		if(len == insert_num) return 0;

		for (int j = 0; j < len; ) {
			memset(recvdata, 0, sizeof(recvdata));
			n = read(sock, recvdata, sizeof(recvdata)-1);

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
		DisconnectHTTP("tunnel failed");
		return -5;
	}

	return 0;
}
