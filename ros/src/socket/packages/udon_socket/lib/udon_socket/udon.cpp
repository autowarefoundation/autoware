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
