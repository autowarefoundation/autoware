/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2014 Steven Lovegrove
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

#ifndef PANGOLIN_GLTEXTURECACHE_H
#define PANGOLIN_GLTEXTURECACHE_H

#include <pangolin/gl/gl.h>
#include <pangolin/gl/glformattraits.h>
#include <pangolin/gl/glpixformat.h>
#include <pangolin/compat/memory.h>
#include <pangolin/image/image.h>

namespace pangolin
{

class PANGOLIN_EXPORT TextureCache
{
public:
    static TextureCache& I();

    GlTexture& GlTex(int w, int h, GLint internal_format, GLint glformat, GLenum gltype)
    {
        const long key =
            (((long)internal_format)<<20) ^
            (((long)glformat)<<10) ^ gltype;

        // Lookup texture
        boostd::shared_ptr<GlTexture>& ptex = texture_map[key];
        if(!ptex) {
            ptex = boostd::shared_ptr<GlTexture>(new GlTexture());
        }
        GlTexture& tex = *ptex;

        // Initialise if it didn't already exist or the size was too small
        if(!tex.tid || tex.width < w || tex.height < h) {
            tex.Reinitialise(
                std::max(tex.width,w), std::max(tex.height,h),
                internal_format, default_sampling_linear, 0,
                glformat, gltype
            );
        }

        return tex;
    }

    template<typename T>
    GlTexture& GlTex(int w, int h)
    {
        return GlTex( w,h,
            GlFormatTraits<T>::glinternalformat,
            GlFormatTraits<T>::glformat,
            GlFormatTraits<T>::gltype
        );
    }

protected:
    bool default_sampling_linear;
    std::map<long, boostd::shared_ptr<GlTexture> > texture_map;

    // Protected constructor
    TextureCache()
        : default_sampling_linear(true)
    {
    }
};

template<typename T>
void RenderToViewport(Image<T>& image, bool flipx=false, bool flipy=false)
{
    // Retrieve texture that is at least as large as image and of appropriate type.
    GlTexture& tex = TextureCache::I().GlTex<T>(image.w, image.h);
    tex.Upload(image.ptr,0,0, image.w, image.h, GlFormatTraits<T>::glformat, GlFormatTraits<T>::gltype);
    tex.RenderToViewport(Viewport(0,0,image.w, image.h), flipx, flipy);
}

// This method may dissapear in the future
inline void RenderToViewport(
    Image<unsigned char>& image,
    const pangolin::GlPixFormat& fmt,
    bool flipx=false, bool flipy=false,
    bool linear_sampling = true
) {
    pangolin::GlTexture& tex = pangolin::TextureCache::I().GlTex(image.w, image.h, fmt.scalable_internal_format, fmt.glformat, fmt.gltype);
    tex.Bind();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, linear_sampling ? GL_LINEAR : GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, linear_sampling ? GL_LINEAR : GL_NEAREST);
    tex.Upload(image.ptr,0,0, image.w, image.h, fmt.glformat, fmt.gltype);
    tex.RenderToViewport(pangolin::Viewport(0,0,image.w, image.h), flipx, flipy);
}

}

#endif // PANGOLIN_GLTEXTURECACHE_H
