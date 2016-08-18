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
