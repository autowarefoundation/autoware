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

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#include <pangolin/gl/glfont.h>
#include <pangolin/gl/glstate.h>
#include <pangolin/image/image_io.h>
#include <pangolin/utils/type_convert.h>

#if !defined(HAVE_GLES) || defined(HAVE_GLES_2)
#include <pangolin/gl/glsl.h>
#endif

#define MAX_TEXT_LENGTH 500

// Embedded fonts:
extern "C" const unsigned char AnonymousPro_ttf[];

namespace pangolin
{

GlFont& GlFont::I()
{
    static GlFont s_font(AnonymousPro_ttf, 15);
    return s_font;
}

GlFont::GlFont(const unsigned char* ttf_buffer, float pixel_height, int tex_w, int tex_h)
{
    InitialiseFont(ttf_buffer, pixel_height, tex_w, tex_h);
}

GlFont::GlFont(const std::string& filename, float pixel_height, int tex_w, int tex_h)
{
    unsigned char* ttf_buffer = new unsigned char[1<<20];
    const size_t bytes_read = fread(ttf_buffer, 1, 1<<20, fopen(filename.c_str(), "rb"));
    if(bytes_read > 0) {
        InitialiseFont(ttf_buffer, pixel_height, tex_w, tex_h);
    }else{
        throw std::runtime_error("Unable to read font from file.");
    }
    delete[] ttf_buffer;
}

GlFont::~GlFont()
{
    delete[] font_bitmap;
}

void GlFont::InitialiseFont(const unsigned char* ttf_buffer, float pixel_height, int tex_w, int tex_h)
{
    font_height_px = pixel_height;
    this->tex_w = tex_w;
    this->tex_h = tex_h;
    font_bitmap = new unsigned char[tex_w*tex_h];
    const int offset = 0;

    stbtt_fontinfo f;
    if (!stbtt_InitFont(&f, ttf_buffer, offset)) {
       throw std::runtime_error("Unable to initialise font");
    }

    float scale = stbtt_ScaleForPixelHeight(&f, pixel_height);

    STBTT_memset(font_bitmap, 0, tex_w*tex_h);
    int x = 1;
    int y = 1;
    int bottom_y = 1;

    // Generate bitmap and char indices
    for (int i=0; i < NUM_CHARS; ++i) {
       int advance, lsb, x0,y0,x1,y1,gw,gh;
       int g = stbtt_FindGlyphIndex(&f, FIRST_CHAR + i);
       stbtt_GetGlyphHMetrics(&f, g, &advance, &lsb);
       stbtt_GetGlyphBitmapBox(&f, g, scale,scale, &x0,&y0,&x1,&y1);
       gw = x1-x0;
       gh = y1-y0;
       if (x + gw + 1 >= tex_w)
          y = bottom_y, x = 1; // advance to next row
       if (y + gh + 1 >= tex_h) // check if it fits vertically AFTER potentially moving to next row
          throw std::runtime_error("Unable to initialise font");
       STBTT_assert(x+gw < tex_w);
       STBTT_assert(y+gh < tex_h);
       stbtt_MakeGlyphBitmap(&f, font_bitmap+x+y*tex_w, gw,gh,tex_w, scale,scale, g);

       // Adjust offset for edges of pixels
       chardata[i] = GlChar(tex_w,tex_h, x, y, gw, gh, scale*advance, x0 -0.5, -y0 -0.5);

       x = x + gw + 1;
       if (y+gh+1 > bottom_y)
          bottom_y = y+gh+1;
    }

    // Generate kern table
    for (int i=0; i < NUM_CHARS; ++i) {
        for (int j=0; j < NUM_CHARS; ++j) {
            kern_table[i*NUM_CHARS+j] = scale * stbtt_GetCodepointKernAdvance(&f,i,j);
        }
    }
}

void GlFont::InitialiseGlTexture()
{
    if(font_bitmap) {
        mTex.Reinitialise(tex_w,tex_h, GL_ALPHA, true, 0, GL_ALPHA,GL_UNSIGNED_BYTE, font_bitmap);
//        mTex.SetNearestNeighbour();
        delete[] font_bitmap;
        font_bitmap = 0;
    }
}

GlText GlFont::Text( const char* fmt, ... )
{
    if(!mTex.IsValid()) InitialiseGlTexture();

    GlText ret(mTex);
    
    char text[MAX_TEXT_LENGTH];          
    va_list ap;
    
    if( fmt != NULL ) {
        va_start( ap, fmt );
        vsnprintf( text, MAX_TEXT_LENGTH, fmt, ap );
        va_end( ap );
        
        const size_t len = strlen(text);
        char lc = ' ' - FIRST_CHAR;
        for(size_t i=0; i < len; ++i) {
            char c = text[i];
            if( !(FIRST_CHAR <= c /*&& c <FIRST_CHAR+NUM_CHARS*/) ) {
                c = ' ';
            }
            GlChar& ch = chardata[c-32];

            // Kerning
            if(i) {
                const GLfloat kern = kern_table[ (lc-32)*NUM_CHARS + (c-32) ];
                ret.AddSpace(kern);
            }

            ret.Add(c,ch);
            lc = c;
        }
    }
    return ret;
}

GlText GlFont::Text( const std::string& str )
{
    if(!mTex.IsValid()) InitialiseGlTexture();

    GlText ret(mTex);

    char lc = ' ' - FIRST_CHAR;
    for(size_t i=0; i < str.length(); ++i) {
        char c = str[i];
        if( !(FIRST_CHAR <= c /*&& c <FIRST_CHAR+NUM_CHARS*/) ) {
            c = ' ';
        }
        GlChar& ch = chardata[c-32];

        // Kerning
        if(i) {
            const GLfloat kern = kern_table[ (lc-32)*NUM_CHARS + (c-32) ];
            ret.AddSpace(kern);
        }

        ret.Add(c,ch);
        lc = c;
    }
    return ret;
}

}
