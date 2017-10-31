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

#ifndef PANGOLIN_GLCHAR_H
#define PANGOLIN_GLCHAR_H

#include <pangolin/gl/glplatform.h>
#include <map>

namespace pangolin {

struct PANGOLIN_EXPORT XYUV
{
    XYUV() {}
    XYUV(GLfloat x, GLfloat y, GLfloat tu, GLfloat tv)
        : x(x), y(y), tu(tu), tv(tv) {}

    XYUV operator+(int dx) const {
        return XYUV(x+dx,y,tu,tv);
    }    
    
    GLfloat x, y, tu, tv;
};

class PANGOLIN_EXPORT GlChar
{
public:
    GlChar();
    GlChar(int tw, int th, int x, int y, int w, int h, GLfloat x_step, GLfloat ox, GLfloat oy);
    
    inline const XYUV& GetVert(size_t i) const {
        return vs[i];
    }    
    
    inline GLfloat StepX() const {
        return x_step;
    }

    inline GLfloat YMin() const {
        return y_min;
    }

    inline GLfloat YMax() const {
        return y_max;
    }

    void Draw() const;
        
protected:
    XYUV vs[4];
    GLfloat x_step;
    GLfloat y_min, y_max;
};

}

#endif // PANGOLIN_GLCHAR_H
