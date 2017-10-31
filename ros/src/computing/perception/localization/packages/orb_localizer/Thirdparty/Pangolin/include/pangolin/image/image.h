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

#ifndef PANGOLIN_IMAGE_H
#define PANGOLIN_IMAGE_H

#include <cstddef>

namespace pangolin
{

// Simple image wrapper
template<typename T>
struct Image {
    inline Image()
        : pitch(0), ptr(0), w(0), h(0)
    {
    }

    inline Image(size_t w, size_t h, size_t pitch, T* ptr)
        : pitch(pitch), ptr(ptr), w(w), h(h)
    {
    }
    
    void Dealloc()
    {
        if(ptr) {
            delete[] ptr;
            ptr = NULL;
        }
    }
    
    void Alloc(size_t w, size_t h, size_t pitch)
    {
        Dealloc();
        this->w = w;
        this->h = h;
        this->pitch = pitch;
        this->ptr = new unsigned char[h*pitch];
    }

    size_t SizeBytes() const
    {
        return pitch * h;
    }
    
    size_t Area() const
    {
        return w * h;
    }

    template<typename To>
    Image<To> Reinterpret()
    {
        return Image<To>(w,h,pitch, (To*)ptr);
    }

    T* RowPtr(int r)
    {
        return (T*)((char*)ptr + r*pitch);
    }

    size_t pitch;
    T* ptr;
    size_t w;
    size_t h;
};

}

#endif // PANGOLIN_IMAGE_H
