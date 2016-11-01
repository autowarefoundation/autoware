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

#ifndef PANGOLIN_VIEWPORT_H
#define PANGOLIN_VIEWPORT_H

#include <pangolin/gl/glinclude.h>

namespace pangolin
{

/// Encapsulates OpenGl Viewport.
struct PANGOLIN_EXPORT Viewport
{
    Viewport() {}
    Viewport(GLint l,GLint b,GLint w,GLint h) : l(l),b(b),w(w),h(h) {}
    
    void Activate() const;
    void ActivateIdentity() const;
    void ActivatePixelOrthographic() const;

    void Scissor() const;
    void ActivateAndScissor() const;

    bool Contains(int x, int y) const;
    
    Viewport Inset(int i) const;
    Viewport Inset(int horiz, int vert) const;
    Viewport Intersect(const Viewport& vp) const;
    
    static void DisableScissor();
    
    GLint r() const { return l+w;}
    GLint t() const { return b+h;}
    GLfloat aspect() const { return (GLfloat)w / (GLfloat)h; }
    GLint l,b,w,h;
};

}

#endif // PANGOLIN_VIEWPORT_H
