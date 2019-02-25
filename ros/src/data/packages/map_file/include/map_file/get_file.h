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

#ifndef _GET_FILE_H_
#define _GET_FILE_H_

#include <cstdint>
#include <string>
#include <netinet/in.h>

#define HTTP_HOSTNAME     "133.6.148.90"
#define HTTP_PORT         (80)
#define HTTP_USER         ""
#define HTTP_PASSWORD     ""

class GetFile {
private:
	std::string host_name_;
	int port_;
	std::string user_;
	std::string password_;
	int sock;
	struct sockaddr_in server;

public:
	GetFile();
	explicit GetFile(const std::string& host_name, int port, const std::string& user, const std::string& password);

	int GetHTTPFile(const std::string& value);
};


#endif /* _GET_FILE_H_ */
