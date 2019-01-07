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
#include <unistd.h>
#include <string>
#include <fstream>
#include <sstream>
#include <ext/stdio_filebuf.h>
#include <curl/curl.h>

#include <map_file/get_file.h>


GetFile::GetFile()
	: GetFile(HTTP_HOSTNAME, HTTP_PORT, HTTP_USER, HTTP_PASSWORD)
{
}

GetFile::GetFile(const std::string& host_name, int port, const std::string& user, const std::string& password)
	: host_name_(host_name), port_(port), user_(user), password_(password)
{
}

typedef std::basic_ofstream<char>::__filebuf_type buffer_t;
typedef __gnu_cxx::stdio_filebuf<char> io_buffer_t; 
static FILE* cfile(buffer_t* const fb) {
	return (static_cast<io_buffer_t* const>(fb))->file();
}

int GetFile::GetHTTPFile(const std::string& value) 
{
	CURL *curl;

	curl = curl_easy_init();
	if (! curl) {
		std::cerr << "curl_easy_init failed" << std::endl;
		return -1;
	}

	std::string filepath("/tmp/" + value);
	std::ofstream ofs;
	ofs.open(filepath);
	if (! ofs.is_open()) {
		std::cerr << "cannot open /tmp/" << value << std::endl;
		curl_easy_cleanup(curl);
		return -2;
	}

	std::ostringstream urlss;
	urlss << "http://" << host_name_ << ":" << port_ << "/" << value;
	curl_easy_setopt(curl, CURLOPT_URL, urlss.str().c_str());
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
	curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 5);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, fwrite);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, cfile(ofs.rdbuf()));
	if (user_ != "" && password_ != "") {
		std::string userpwd = user_ + ":" + password_;
		curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_DIGEST);
		curl_easy_setopt(curl, CURLOPT_USERPWD, userpwd.c_str());
	}
	CURLcode res = curl_easy_perform(curl);
	if (res != CURLE_OK) {
		std::cerr << "curl_easy_perform failed: " <<
			curl_easy_strerror(res) << std::endl;
		curl_easy_cleanup(curl);
		ofs.close();
		unlink(filepath.c_str());
		return -3;
	}
	long response_code;
	res = curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
	if (res != CURLE_OK) {
		std::cerr << "curl_easy_getinfo failed: " <<
			curl_easy_strerror(res) << std::endl;
		curl_easy_cleanup(curl);
		ofs.close();
		unlink(filepath.c_str());
		return -4;
	}
	curl_easy_cleanup(curl);
	ofs.close();
	if (response_code != 200) {
		unlink(filepath.c_str());
		return -5;
	}

	return 0;
}
