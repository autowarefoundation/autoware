/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove
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

#include <pangolin/image/image_common.h>
#include <pangolin/utils/file_utils.h>

#include <vector>

namespace pangolin
{

const VideoPixelFormat SupportedVideoPixelFormats[] =
{
    {"GRAY8", 1, {8}, 8, false},
    {"GRAY10", 1, {10}, 10, false},
    {"GRAY12", 1, {12}, 12, false},
    {"GRAY16LE", 1, {16}, 16, false},
    {"Y400A", 2, {8,8}, 16, false},
    {"RGB24", 3, {8,8,8}, 24, false},
    {"BGR24", 3, {8,8,8}, 24, false},
    {"YUYV422", 3, {4,2,2}, 16, false},
    {"RGBA",  4, {8,8,8,8}, 32, false},
    {"GRAY32F", 1, {32}, 32, false},
    {"RGB96F",  3, {32,32,32}, 96, false},
    {"",0,{0,0,0,0},0,0}
};

VideoPixelFormat VideoFormatFromString(const std::string& format)
{
    for(int i=0; !SupportedVideoPixelFormats[i].format.empty(); ++i)
        if(!format.compare(SupportedVideoPixelFormats[i].format))
            return SupportedVideoPixelFormats[i];
    throw VideoException("Unknown Format",format);
}

}
