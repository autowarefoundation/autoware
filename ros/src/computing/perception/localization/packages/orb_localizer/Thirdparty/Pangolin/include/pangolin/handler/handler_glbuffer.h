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

#ifndef HANDLER_GLBUFFER_H
#define HANDLER_GLBUFFER_H

#include <pangolin/handler/handler.h>
#include <pangolin/gl/gl.h>

namespace pangolin
{

struct Handler3DFramebuffer : public pangolin::Handler3D
{
    Handler3DFramebuffer(GlFramebuffer& fb, pangolin::OpenGlRenderState& cam_state, pangolin::AxisDirection enforce_up=pangolin::AxisNone, float trans_scale=0.01f);
    void GetPosNormal(pangolin::View& view, int x, int y, GLprecision p[3], GLprecision Pw[3], GLprecision Pc[3], GLprecision /*n*/[3], GLprecision default_z);

protected:
    GlFramebuffer& fb;
};

}

#endif // HANDLER_GLBUFFER_H
