/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011-2013 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef PANGOLIN_IMAGE_COMMON_H
#define PANGOLIN_IMAGE_COMMON_H

#include <pangolin/platform.h>
#include <pangolin/utils/type_convert.h>
#include <pangolin/utils/uri.h>
#include <exception>
#include <string>
#include <map>

namespace pangolin
{

struct PANGOLIN_EXPORT VideoException : std::exception
{
    VideoException(std::string str) : desc(str) {}
    VideoException(std::string str, std::string detail) {
        desc = str + "\n\t" + detail;
    }
    ~VideoException() throw() {}
    const char* what() const throw() { return desc.c_str(); }
    std::string desc;
};

struct PANGOLIN_EXPORT VideoPixelFormat
{
    // Previously, VideoInterface::PixFormat returned a string.
    // For compatibility, make this string convertable
    inline operator std::string() const { return format; }
    
    std::string  format;
    unsigned int channels;
    unsigned int channel_bits[4];
    unsigned int bpp;
    bool planar;
};

struct PANGOLIN_EXPORT ImageDim
{
    inline ImageDim() : x(0), y(0) {}
    inline ImageDim(size_t x, size_t y) : x(x), y(y) {}
    size_t x;
    size_t y;
};

struct PANGOLIN_EXPORT ImageRoi
{
    inline ImageRoi() : x(0), y(0), w(0), h(0) {}
    inline ImageRoi(size_t x, size_t y, size_t w, size_t h) : x(x), y(y), w(w), h(h) {}
    size_t x; size_t y;
    size_t w; size_t h;
};

//! Return Pixel Format properties given string specification in
//! FFMPEG notation.
PANGOLIN_EXPORT
VideoPixelFormat VideoFormatFromString(const std::string& format);

}

#endif // PANGOLIN_IMAGE_COMMON_H

