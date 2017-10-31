/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2015 Steven Lovegrove
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

#ifndef PANGOLIN_GLFONT_H
#define PANGOLIN_GLFONT_H

#include <pangolin/gl/gltext.h>

#include <cstdio>
#include <cstdarg>

namespace pangolin {

class PANGOLIN_EXPORT GlFont
{
public:
    // Singleton instance if requested.
    static GlFont& I();

    // Load GL Font data. Delay uploading as texture until first use.
    GlFont(const unsigned char* ttf_buffer, float pixel_height, int tex_w=512, int tex_h=512);
    GlFont(const std::string& filename, float pixel_height, int tex_w=512, int tex_h=512);

    virtual ~GlFont();

    // Generate renderable GlText object from this font.
    GlText Text( const char* fmt, ... );

    GlText Text( const std::string& str );

    inline float Height() const {
        return font_height_px;
    }
    
protected:
    void InitialiseFont(const unsigned char* ttf_buffer, float pixel_height, int tex_w, int tex_h);

    // This can only be called once GL context is initialised
    void InitialiseGlTexture();

    const static int FIRST_CHAR = 32;
    const static int NUM_CHARS = 96;

    float font_height_px;

    int tex_w;
    int tex_h;
    unsigned char* font_bitmap;
    GlTexture mTex;

    GlChar chardata[NUM_CHARS];
    GLfloat kern_table[NUM_CHARS*NUM_CHARS];
};

}

#endif // PANGOLIN_GLFONT_H
