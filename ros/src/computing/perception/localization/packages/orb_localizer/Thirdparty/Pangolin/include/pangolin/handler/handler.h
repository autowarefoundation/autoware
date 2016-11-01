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

#ifndef PANGOLIN_HANDLER_H
#define PANGOLIN_HANDLER_H

#include <pangolin/display/opengl_render_state.h>
#include <pangolin/handler/handler_enums.h>

#ifdef _OSX_
#define PANGO_DFLT_HANDLER3D_ZF (1.0f/50.0f)
#else
#define PANGO_DFLT_HANDLER3D_ZF (1.0f/10.0f)
#endif

namespace pangolin
{

// Forward declarations
struct View;

/// Input Handler base class.
/// Virtual methods which recurse into sub-displays.
struct PANGOLIN_EXPORT Handler
{
    virtual ~Handler() {}
    virtual void Keyboard(View&, unsigned char key, int x, int y, bool pressed);
    virtual void Mouse(View&, MouseButton button, int x, int y, bool pressed, int button_state);
    virtual void MouseMotion(View&, int x, int y, int button_state);
    virtual void PassiveMouseMotion(View&, int x, int y, int button_state);
    virtual void Special(View&, InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state);
};

struct PANGOLIN_EXPORT HandlerScroll : Handler
{
    void Mouse(View&, MouseButton button, int x, int y, bool pressed, int button_state);
    void Special(View&, InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state);
};

struct PANGOLIN_EXPORT Handler3D : Handler
{
    Handler3D(OpenGlRenderState& cam_state, AxisDirection enforce_up=AxisNone, float trans_scale=0.01f, float zoom_fraction= PANGO_DFLT_HANDLER3D_ZF);
    
    virtual void GetPosNormal(View& view, int x, int y, GLprecision p[3], GLprecision Pw[3], GLprecision Pc[3], GLprecision n[3], GLprecision default_z = 1.0);
    void Keyboard(View&, unsigned char key, int x, int y, bool pressed);
    void Mouse(View&, MouseButton button, int x, int y, bool pressed, int button_state);
    void MouseMotion(View&, int x, int y, int button_state);
    void Special(View&, InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state);
    
protected:
    OpenGlRenderState* cam_state;
    const static int hwin = 8;
    AxisDirection enforce_up;
    float tf; // translation factor
    float zf; // zoom fraction
    CameraSpec cameraspec;
    GLprecision last_z;
    float last_pos[2];
    GLprecision rot_center[3];
    
    GLprecision p[3];
    GLprecision Pw[3];
    GLprecision Pc[3];
    GLprecision n[3];
};

static Handler StaticHandler;
static HandlerScroll StaticHandlerScroll;

}

#endif // PANGOLIN_HANDLER_H
