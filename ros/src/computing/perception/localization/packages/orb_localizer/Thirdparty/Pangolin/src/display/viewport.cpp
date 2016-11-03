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

#include <pangolin/display/viewport.h>
#include <algorithm>

namespace pangolin {

void Viewport::Activate() const
{
    glViewport(l,b,w,h);
}

void Viewport::Scissor() const
{
    glEnable(GL_SCISSOR_TEST);
    glScissor(l,b,w,h);
}

void Viewport::ActivateAndScissor() const
{
    glViewport(l,b,w,h);
    glEnable(GL_SCISSOR_TEST);
    glScissor(l,b,w,h);
}


void Viewport::DisableScissor()
{
    glDisable(GL_SCISSOR_TEST);
}

bool Viewport::Contains(int x, int y) const
{
    return l <= x && x < (l+w) && b <= y && y < (b+h);
}

void Viewport::ActivatePixelOrthographic() const
{
    Activate();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-0.5, w-0.5, -0.5, h-0.5, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void Viewport::ActivateIdentity() const
{
    Activate();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}


Viewport Viewport::Inset(int i) const
{
    return Viewport(l+i, b+i, w-2*i, h-2*i);
}

Viewport Viewport::Inset(int horiz, int vert) const
{
    return Viewport(l+horiz, b+vert, w-horiz, h-vert);
}

Viewport Viewport::Intersect(const Viewport& vp) const
{
    GLint nl = std::max(l,vp.l);
    GLint nr = std::min(r(),vp.r());
    GLint nb = std::max(b,vp.b);
    GLint nt = std::min(t(),vp.t());
    return Viewport(nl,nb, nr-nl, nt-nb);
}

}
