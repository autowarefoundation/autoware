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

#include <cstddef>
#include <cstring>

#include <sys/socket.h>

#include <udon_socket/udon.hpp>

namespace udon_socket {

namespace udon {

namespace {

constexpr std::int32_t TYPE_BEACON	= 0;
constexpr std::int32_t TYPE_MODE	= 3;
constexpr std::int32_t TYPE_LOCATION	= 6;

constexpr std::size_t SIZE_REQUEST	= 0x08;
constexpr std::size_t SIZE_RESPONSE	= 0x04;
constexpr std::size_t SIZE_MODE		= 0x08;
constexpr std::size_t SIZE_LOCATION	= 0x28;

constexpr std::ptrdiff_t OFFSET_TYPE		= 0x00;
constexpr std::ptrdiff_t OFFSET_VALUE		= 0x04;
constexpr std::ptrdiff_t OFFSET_LENGTH		= 0x04;
constexpr std::ptrdiff_t OFFSET_LOCATION_X	= 0x08;
constexpr std::ptrdiff_t OFFSET_LOCATION_Y	= 0x10;
constexpr std::ptrdiff_t OFFSET_LOCATION_Z	= 0x18;
constexpr std::ptrdiff_t OFFSET_LOCATION_D	= 0x20;

void set_type(std::uint8_t *buf, std::int32_t type)
{
	std::memcpy(buf + OFFSET_TYPE, &type, sizeof(type));
}

void set_value(std::uint8_t *buf, std::int32_t value)
{
	std::memcpy(buf + OFFSET_VALUE, &value, sizeof(value));
}

void set_length(std::uint8_t *buf, std::int32_t length)
{
	std::memcpy(buf + OFFSET_LENGTH, &length, sizeof(length));
}

void set_location_x(std::uint8_t *buf, double x)
{
	std::memcpy(buf + OFFSET_LOCATION_X, &x, sizeof(x));
}

void set_location_y(std::uint8_t *buf, double y)
{
	std::memcpy(buf + OFFSET_LOCATION_Y, &y, sizeof(y));
}

void set_location_z(std::uint8_t *buf, double z)
{
	std::memcpy(buf + OFFSET_LOCATION_Z, &z, sizeof(z));
}

void set_location_d(std::uint8_t *buf, double d)
{
	std::memcpy(buf + OFFSET_LOCATION_D, &d, sizeof(d));
}

} // namespace

bool Location::operator !=(const Location& location) const
{
	return !(x == location.x && y == location.y && z == location.z && d == location.d);
}

ssize_t send_request(int fd)
{
	std::uint8_t buf[SIZE_REQUEST];
	set_type(buf, TYPE_BEACON);
	set_value(buf, 0);

	return send(fd, &buf, sizeof(buf), 0);
}

ssize_t send_response(int fd)
{
	std::uint8_t buf[SIZE_RESPONSE];
	set_type(buf, TYPE_BEACON);

	return send(fd, &buf, sizeof(buf), 0);
}

ssize_t send_mode(int fd, std::int32_t mode)
{
	std::uint8_t buf[SIZE_MODE];
	set_type(buf, TYPE_MODE);
	set_value(buf, (mode > 0) ? 1:0);

	return send(fd, &buf, sizeof(buf), 0);
}

ssize_t send_location(int fd, const Location& location)
{
	std::uint8_t buf[SIZE_LOCATION];
	set_type(buf, TYPE_LOCATION);
	set_length(buf, sizeof(double) * 4);
	set_location_x(buf, location.x);
	set_location_y(buf, location.y);
	set_location_z(buf, location.z);
	set_location_d(buf, location.d);

	return send(fd, &buf, sizeof(buf), 0);
}

} // namespace udon

} // namespace udon_socket
