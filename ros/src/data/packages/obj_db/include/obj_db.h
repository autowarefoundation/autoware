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

#ifndef _OBJ_DB_H_
#define _OBJ_DB_H_

#include <cstdint>
#include <string>

class SendData {
private:
	std::string host_name_;
	int port_;

public:
	SendData();
	explicit SendData(const std::string& host_name, int port);

	int Sender(const std::string& value, std::string& res) const;
};

extern std::string make_header(int32_t sql_inst, int32_t sql_num);

#endif /* _OBJ_DB_H_ */
