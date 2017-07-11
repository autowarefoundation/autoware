/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  File:    EFStream.hpp
 *  Author:  Hilton Bristow
 *  Created: Jun 27, 2012
 */
#ifndef EFSTREAM_HPP_
#define EFSTREAM_HPP_

#include <fstream>
#include <algorithm>

/*! @class EFStream
 *  @brief Endian-swap File Stream
 *
 *  fstream subclass with ability to do byte swapping
 *  (change endianness) of the incoming stream upon
 *  a read request
 */
class EFStream : public std::fstream {
private:
    bool byte_swap_;
public:
    // default constructor
    EFStream() : byte_swap_(false) {}

    // get and set byte swap methods
    bool byteSwap(void) { return byte_swap_; }
    void setByteSwap(bool state) { byte_swap_ = state; }

    // method to swap the Endianness of a stream
    void swapEndian(char *s, std::streamsize N) {
        for (unsigned int n = 0; n < N; n+=2) std::swap(s[n], s[n+1]);
    }

    // overloaded fstream read method with 
    // byte swapping capacity
    std::istream& read(char *s, std::streamsize n) {
        // call the parent read
        std::istream& stream = std::fstream::read(s,n);
        // swap the endianness if necessary
        if (byte_swap_ && n%2 == 0) swapEndian(s,n);
        return stream;
    }
};

#endif
