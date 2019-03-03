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

#ifndef UDON_SOCKET_UDON_HPP
#define UDON_SOCKET_UDON_HPP

#include <cstdint>

#include <sys/types.h>

namespace udon_socket {

namespace udon {

constexpr std::int32_t MODE_NORMAL	= 0;
constexpr std::int32_t MODE_AUTO	= 1;
constexpr std::int32_t MODE_ERROR	= 2;

struct Location {
	double x;
	double y;
	double z;
	double d;

	bool operator !=(const Location& location) const;
};

ssize_t send_request(int fd);
ssize_t send_response(int fd);
ssize_t send_mode(int fd, std::int32_t mode);
ssize_t send_location(int fd, const Location& location);

} // namespace udon

} // namespace udon_socket

#endif // UDON_SOCKET_UDON_HPP
