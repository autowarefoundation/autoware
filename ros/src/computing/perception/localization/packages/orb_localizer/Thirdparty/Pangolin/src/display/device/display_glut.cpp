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

#include <pangolin/platform.h>
#include <pangolin/gl/glinclude.h>
#include <pangolin/gl/glglut.h>
#include <pangolin/display/display.h>
#include <pangolin/display/display_internal.h>
#include <pangolin/display/device/display_glut.h>

namespace pangolin
{

extern __thread PangolinGl* context;

void PangoGlutRedisplay()
{
    glutPostRedisplay();
}

void TakeGlutCallbacks()
{
    glutKeyboardFunc(&process::Keyboard);
    glutKeyboardUpFunc(&process::KeyboardUp);
    glutReshapeFunc(&process::Resize);
    glutDisplayFunc(&process::Display);
    glutMouseFunc(&process::Mouse);
    glutMotionFunc(&process::MouseMotion);
    glutPassiveMotionFunc(&process::PassiveMouseMotion);
    glutSpecialFunc(&process::SpecialFunc);
    glutSpecialUpFunc(&process::SpecialFuncUp);
    
#ifdef HAVE_APPLE_OPENGL_FRAMEWORK
    glutDisplayFunc(&PangoGlutRedisplay);
    
    // Attempt to register special smooth scroll callback
    // https://github.com/nanoant/osxglut
    typedef void (*glutScrollFunc_t)(void (*)(float, float));
    typedef void (*glutZoomFunc_t)(void (*)(float));
    typedef void (*glutRotateFunc_t)(void (*)(float));
    typedef void (*glutSubpixMotionFunc_t)(void (*)(float,float,float,float,float,float));
    
    glutScrollFunc_t glutScrollFunc = (glutScrollFunc_t)glutGetProcAddress("glutScrollFunc");
    glutZoomFunc_t glutZoomFunc = (glutZoomFunc_t)glutGetProcAddress("glutZoomFunc");
    glutRotateFunc_t glutRotateFunc = (glutRotateFunc_t)glutGetProcAddress("glutRotateFunc");
    glutSubpixMotionFunc_t glutSubpixMotionFunc = (glutSubpixMotionFunc_t)glutGetProcAddress("glutSubpixMotionFunc");
    
    if(glutScrollFunc) {
        glutScrollFunc(&process::Scroll);
    }
    if(glutZoomFunc) {
        glutZoomFunc(&process::Zoom);
    }
    if(glutRotateFunc) {
        glutRotateFunc(&process::Rotate);
    }
    
    if(glutSubpixMotionFunc) {
        glutSubpixMotionFunc(&process::SubpixMotion);
    }
#endif // HAVE_APPLE_OPENGL_FRAMEWORK
}

void CreateGlutWindowAndBind(std::string window_title, int w, int h, unsigned int mode)
{
#ifdef HAVE_FREEGLUT
    if( glutGet(GLUT_INIT_STATE) == 0)
#endif
    {
        int argc = 0;
        glutInit(&argc, 0);
        glutInitDisplayMode(mode);
    }
    glutInitWindowSize(w,h);
    glutCreateWindow(window_title.c_str());
    BindToContext(window_title);

#ifdef HAVE_GLEW
    glewInit();
#endif
    
#ifdef HAVE_FREEGLUT
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif

    // TODO: Need to implement our own key-repeat mechanism
    //glutIgnoreKeyRepeat(true);

    context->is_double_buffered = (mode & GLUT_DOUBLE) != 0;
    TakeGlutCallbacks();
}

void SwapGlutBuffersProcessGlutEvents()
{
    glutSwapBuffers();
    
#ifdef HAVE_FREEGLUT
    glutMainLoopEvent();
#endif
    
#ifdef HAVE_GLUT_APPLE_FRAMEWORK
    glutCheckLoop();
#endif
}

void FinishGlutFrame()
{    
    RenderViews();
    PostRender();
    SwapGlutBuffersProcessGlutEvents();
    context->quit |= !glutGetWindow();
}

// Implement platform agnostic version
void CreateWindowAndBind(std::string window_title, int w, int h, const Params& /*params*/ )
{
    CreateGlutWindowAndBind(window_title, w, h);
}

// Implement platform agnostic version
void FinishFrame()
{
    FinishGlutFrame();
}

void SetFullscreen(bool fullscreen)
{
    if( fullscreen != context->is_fullscreen )
    {
        if(fullscreen) {
            glutFullScreen();
        }else{
            glutReshapeWindow(context->windowed_size[0],context->windowed_size[1]);
        }
        context->is_fullscreen = fullscreen;
    }
}

void PangolinPlatformInit(PangolinGl& /*context*/)
{
}

void PangolinPlatformDeinit(PangolinGl& /*context*/)
{
}


}
