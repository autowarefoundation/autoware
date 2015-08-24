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
#include <string>

#include <obj_db.h>

SendData::SendData()
	: SendData("db1.ertl.jp", 5700)
{
}

SendData::SendData(const std::string& host_name, int port)
	: host_name_(host_name), port_(port)
{
}

int SendData::Sender(const std::string& value, std::string& res) const
{
	/*********************************************
	   format data to send
	*******************************************/
	char recvdata[1024];
	int n;

	struct sockaddr_in server;
	unsigned int **addrptr;

	int sock = socket(AF_INET, SOCK_STREAM, 0);
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

	if(value == ""){
		fprintf(stderr,"no data\n");
		return -1;
	}

	//    printf("send data : %s\n",value.c_str());
	std::cout << "send data : " << value << std::endl;
	n = write(sock, value.c_str(), value.size());
	if (n < 0) {
		perror("write");
		return -1;
	}

	//Caution : If server do not close in one communication,loop forever because read() do not return 0.
	while(1){
		memset(recvdata, 0, sizeof(recvdata));
		n = read(sock, recvdata, sizeof(recvdata));
		if (n < 0) {
			perror("recv error");
			return -1;
		}else if(n==0){
			break;
		}

		if(strlen(recvdata)>=sizeof(recvdata)){
			res.append(recvdata,sizeof(recvdata));
		}else{
			res.append(recvdata,strlen(recvdata));
		}
	}

	close(sock);
	return 0;
}
