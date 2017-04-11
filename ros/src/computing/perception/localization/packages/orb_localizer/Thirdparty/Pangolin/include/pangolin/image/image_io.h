/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2013 Steven Lovegrove
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

#ifndef PANGOLIN_IMAGE_IO_H
#define PANGOLIN_IMAGE_IO_H

#include <pangolin/image/image.h>
#include <pangolin/image/image_common.h>
#include <pangolin/utils/file_extension.h>

namespace pangolin {

struct TypedImage : public Image<unsigned char>
{
    inline TypedImage()
        : Image<unsigned char>()
    {
    }
    
    inline TypedImage(size_t w, size_t h, size_t pitch, unsigned char* ptr, const VideoPixelFormat& fmt)
        : Image<unsigned char>(w,h,pitch,ptr), fmt(fmt)
    {
    }    
    
    inline void Alloc(size_t w, size_t h, const VideoPixelFormat& fmt)
    {
        this->fmt = fmt;
        Image<unsigned char>::Alloc(w, h, w*fmt.bpp / 8);
    }
    
    inline void Alloc(size_t w, size_t h, const VideoPixelFormat& fmt, size_t pitch)
    {
        this->fmt = fmt;
        Image<unsigned char>::Alloc(w, h, pitch);
    }
    
    VideoPixelFormat fmt;
};

PANGOLIN_EXPORT
TypedImage LoadImage(const std::string& filename, ImageFileType file_type);

PANGOLIN_EXPORT
TypedImage LoadImage(const std::string& filename);

PANGOLIN_EXPORT
void SaveImage(const Image<unsigned char>& image, const pangolin::VideoPixelFormat& fmt, const std::string& filename, bool top_line_first = true);

PANGOLIN_EXPORT
void SaveImage(const TypedImage& image, const std::string& filename, bool top_line_first = true);

PANGOLIN_EXPORT
void FreeImage(TypedImage& img);

}

#endif // PANGOLIN_IMAGE_IO_H
