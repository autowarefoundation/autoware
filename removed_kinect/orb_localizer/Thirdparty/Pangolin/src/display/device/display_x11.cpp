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


// Code based on public domain sample at
// https://www.opengl.org/wiki/Tutorial:_OpenGL_3.0_Context_Creation_%28GLX%29

#include <pangolin/platform.h>
#include <pangolin/gl/glinclude.h>
#include <pangolin/gl/glglut.h>
#include <pangolin/display/display.h>
#include <pangolin/display/display_internal.h>

#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>

namespace pangolin
{
extern __thread PangolinGl* context;
}

Display *display = 0;
Window win = 0;
GLXContext ctx = 0;
Colormap cmap;
const long EVENT_MASKS = ButtonPressMask|ButtonReleaseMask|StructureNotifyMask|ButtonMotionMask|PointerMotionMask|KeyPressMask|KeyReleaseMask;

#define GLX_CONTEXT_MAJOR_VERSION_ARB       0x2091
#define GLX_CONTEXT_MINOR_VERSION_ARB       0x2092
typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);

// Helper to check for extension string presence.  Adapted from:
//   http://www.opengl.org/resources/features/OGLextensions/
static bool isExtensionSupported(const char *extList, const char *extension)
{
    const char *start;
    const char *where, *terminator;

    /* Extension names should not have spaces. */
    where = strchr(extension, ' ');
    if (where || *extension == '\0')
        return false;

    for (start=extList;;) {
        where = strstr(start, extension);

        if (!where)
            break;

        terminator = where + strlen(extension);

        if ( where == start || *(where - 1) == ' ' )
            if ( *terminator == ' ' || *terminator == '\0' )
                return true;

        start = terminator;
    }

    return false;
}

static bool ctxErrorOccurred = false;
static int ctxErrorHandler( Display *dpy, XErrorEvent *ev )
{
    ctxErrorOccurred = true;
    return 0;
}

int CreateX11Window(
    const std::string& title, int width, int height,
    bool glx_doublebuffer,int glx_sample_buffers, int glx_samples
) {
    display = XOpenDisplay(NULL);

    if (!display) {
        throw std::runtime_error("Pangolin X11: Failed to open X display");
    }

    // Desired attributes
    static int visual_attribs[] =
    {
        GLX_X_RENDERABLE    , True,
        GLX_DRAWABLE_TYPE   , GLX_WINDOW_BIT,
        GLX_RENDER_TYPE     , GLX_RGBA_BIT,
        GLX_X_VISUAL_TYPE   , GLX_TRUE_COLOR,
        GLX_RED_SIZE        , 8,
        GLX_GREEN_SIZE      , 8,
        GLX_BLUE_SIZE       , 8,
        GLX_ALPHA_SIZE      , 8,
        GLX_DEPTH_SIZE      , 24,
        GLX_STENCIL_SIZE    , 8,
        GLX_DOUBLEBUFFER    , glx_doublebuffer ? True : False,
        GLX_SAMPLE_BUFFERS  , glx_sample_buffers,
        GLX_SAMPLES         , glx_sample_buffers > 0 ? glx_samples : 0,
        None
    };


    int glx_major, glx_minor;
    if ( !glXQueryVersion( display, &glx_major, &glx_minor ) ||
         ( ( glx_major == 1 ) && ( glx_minor < 3 ) ) || ( glx_major < 1 ) )
    {
        // FBConfigs were added in GLX version 1.3.
        throw std::runtime_error("Pangolin X11: Invalid GLX version. Require GLX >= 1.3");
    }

    int fbcount;
    GLXFBConfig* fbc = glXChooseFBConfig(display, DefaultScreen(display), visual_attribs, &fbcount);
    if (!fbc) {
        throw std::runtime_error("Pangolin X11: Unable to retrieve framebuffer options");
    }

    int best_fbc = -1;
    int worst_fbc = -1;
    int best_num_samp = -1;
    int worst_num_samp = 999;

    // Enumerate framebuffer options, storing the best and worst that match our attribs
    for (int i=0; i<fbcount; ++i)
    {
        XVisualInfo *vi = glXGetVisualFromFBConfig( display, fbc[i] );
        if ( vi )
        {
            int samp_buf, samples;
            glXGetFBConfigAttrib( display, fbc[i], GLX_SAMPLE_BUFFERS, &samp_buf );
            glXGetFBConfigAttrib( display, fbc[i], GLX_SAMPLES       , &samples  );

            if ( (best_fbc < 0) || (samp_buf>0 && samples>best_num_samp) )
                best_fbc = i, best_num_samp = samples;

            if ( (worst_fbc < 0) || (samp_buf>0 && samples<worst_num_samp) )
                worst_fbc = i, worst_num_samp = samples;
        }
        XFree( vi );
    }

    // Select the minimum suitable option. The 'best' is often too slow.
    GLXFBConfig bestFbc = fbc[ worst_fbc ];
    XFree( fbc );

    // Get a visual
    XVisualInfo *vi = glXGetVisualFromFBConfig( display, bestFbc );

    // Create colourmap
    XSetWindowAttributes swa;
    swa.colormap = cmap = XCreateColormap( display,
                                           RootWindow( display, vi->screen ),
                                           vi->visual, AllocNone );
    swa.background_pixmap = None ;
    swa.border_pixel      = 0;
    swa.event_mask        = StructureNotifyMask;

    // Create window
    win = XCreateWindow( display, RootWindow( display, vi->screen ),
                         0, 0, width, height, 0, vi->depth, InputOutput,
                         vi->visual,
                         CWBorderPixel|CWColormap|CWEventMask, &swa );

    XFree( vi );

    if ( !win ) {
        throw std::runtime_error("Pangolin X11: Failed to create window." );
    }

    XStoreName( display, win, title.c_str() );
    XMapWindow( display, win );

    // Request to be notified of these events
    XSelectInput(display, win, EVENT_MASKS );

    // Get the default screen's GLX extension list
    const char *glxExts = glXQueryExtensionsString( display, DefaultScreen( display ) );

    glXCreateContextAttribsARBProc glXCreateContextAttribsARB =
            (glXCreateContextAttribsARBProc) glXGetProcAddressARB(
                (const GLubyte *) "glXCreateContextAttribsARB"
            );


    // Install an X error handler so the application won't exit if GL 3.0
    // context allocation fails. Handler is global and shared across all threads.
    ctxErrorOccurred = false;
    int (*oldHandler)(Display*, XErrorEvent*) = XSetErrorHandler(&ctxErrorHandler);

    if ( isExtensionSupported( glxExts, "GLX_ARB_create_context" ) && glXCreateContextAttribsARB )
    {
        int context_attribs[] = {
            GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
            GLX_CONTEXT_MINOR_VERSION_ARB, 0,
            //GLX_CONTEXT_FLAGS_ARB        , GLX_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
            None
        };

        ctx = glXCreateContextAttribsARB( display, bestFbc, 0, True, context_attribs );

        // Sync to ensure any errors generated are processed.
        XSync( display, False );
        if ( ctxErrorOccurred || !ctx ) {
            ctxErrorOccurred = false;
            // Fall back to old-style 2.x context. Implementations will return the newest
            // context version compatible with OpenGL versions less than version 3.0.
            context_attribs[1] = 1;  // GLX_CONTEXT_MAJOR_VERSION_ARB = 1
            context_attribs[3] = 0;  // GLX_CONTEXT_MINOR_VERSION_ARB = 0
            ctx = glXCreateContextAttribsARB( display, bestFbc, 0, True, context_attribs );
        }
    } else {
        // Fallback to GLX 1.3 Context
        ctx = glXCreateNewContext( display, bestFbc, GLX_RGBA_TYPE, 0, True );
    }

    // Sync to ensure any errors generated are processed.
    XSync( display, False );

    // Restore the original error handler
    XSetErrorHandler( oldHandler );

    if ( ctxErrorOccurred || !ctx )
    {
        throw std::runtime_error("Pangolin X11: Failed to create an OpenGL context");
    }

    // Verifying that context is a direct context
    if ( ! glXIsDirect ( display, ctx ) ) {
        pango_print_warn("Pangolin X11: Indirect GLX rendering context obtained\n");
    }

    glXMakeCurrent( display, win, ctx );

    return 0;
}


namespace pangolin
{

void X11Terminate()
{
    glXMakeCurrent( display, 0, 0 );
    glXDestroyContext( display, ctx );

    XDestroyWindow( display, win );
    XFreeColormap( display, cmap );
    XCloseDisplay( display );
}

void ProcessX11Events()
{
    XEvent ev;
    while(!pangolin::ShouldQuit() && XCheckWindowEvent(display,win,EVENT_MASKS,&ev))
    {
        switch(ev.type){
        case ConfigureNotify:
            pangolin::process::Resize(ev.xconfigure.width, ev.xconfigure.height);
            break;
        case ButtonPress:
        case ButtonRelease:
        {
            const int button = ev.xbutton.button-1;
            const int mask = Button1Mask << button;
            pangolin::process::Mouse(
                button,
                ev.xbutton.state & mask,
                ev.xbutton.x, ev.xbutton.y
            );
            break;
        }
        case MotionNotify:
            if(ev.xmotion.state & (Button1Mask|Button2Mask|Button3Mask) ) {
                pangolin::process::MouseMotion(ev.xmotion.x, ev.xmotion.y);
            }else{
                pangolin::process::PassiveMouseMotion(ev.xmotion.x, ev.xmotion.y);
            }
            break;
        case KeyPress:
        case KeyRelease:
            int key;
            char ch;
            KeySym sym;

            if( XLookupString(&ev.xkey,&ch,1,&sym,0) == 0) {
                switch (sym) {
                case XK_F1:        key = PANGO_SPECIAL + PANGO_KEY_F1         ; break;
                case XK_F2:        key = PANGO_SPECIAL + PANGO_KEY_F2         ; break;
                case XK_F3:        key = PANGO_SPECIAL + PANGO_KEY_F3         ; break;
                case XK_F4:        key = PANGO_SPECIAL + PANGO_KEY_F4         ; break;
                case XK_F5:        key = PANGO_SPECIAL + PANGO_KEY_F5         ; break;
                case XK_F6:        key = PANGO_SPECIAL + PANGO_KEY_F6         ; break;
                case XK_F7:        key = PANGO_SPECIAL + PANGO_KEY_F7         ; break;
                case XK_F8:        key = PANGO_SPECIAL + PANGO_KEY_F8         ; break;
                case XK_F9:        key = PANGO_SPECIAL + PANGO_KEY_F9         ; break;
                case XK_F10:       key = PANGO_SPECIAL + PANGO_KEY_F10        ; break;
                case XK_F11:       key = PANGO_SPECIAL + PANGO_KEY_F11        ; break;
                case XK_F12:       key = PANGO_SPECIAL + PANGO_KEY_F12        ; break;
                case XK_Left:      key = PANGO_SPECIAL + PANGO_KEY_LEFT       ; break;
                case XK_Up:        key = PANGO_SPECIAL + PANGO_KEY_UP         ; break;
                case XK_Right:     key = PANGO_SPECIAL + PANGO_KEY_RIGHT      ; break;
                case XK_Down:      key = PANGO_SPECIAL + PANGO_KEY_DOWN       ; break;
                case XK_Page_Up:   key = PANGO_SPECIAL + PANGO_KEY_PAGE_UP    ; break;
                case XK_Page_Down: key = PANGO_SPECIAL + PANGO_KEY_PAGE_DOWN  ; break;
                case XK_Home:      key = PANGO_SPECIAL + PANGO_KEY_HOME       ; break;
                case XK_End:       key = PANGO_SPECIAL + PANGO_KEY_END        ; break;
                case XK_Insert:    key = PANGO_SPECIAL + PANGO_KEY_INSERT     ; break;
                case XK_Shift_L:
                case XK_Shift_R:
                    key = -1;
                    if(ev.type==KeyPress) {
                        pangolin::context->mouse_state |=  pangolin::KeyModifierShift;
                    }else{
                        pangolin::context->mouse_state &= ~pangolin::KeyModifierShift;
                    }
                    break;
                case XK_Control_L:
                case XK_Control_R:
                    key = -1;
                    if(ev.type==KeyPress) {
                        pangolin::context->mouse_state |=  pangolin::KeyModifierCtrl;
                    }else{
                        pangolin::context->mouse_state &= ~pangolin::KeyModifierCtrl;
                    }
                    break;
                case XK_Alt_L:
                case XK_Alt_R:
                    key = -1;
                    if(ev.type==KeyPress) {
                        pangolin::context->mouse_state |=  pangolin::KeyModifierAlt;
                    }else{
                        pangolin::context->mouse_state &= ~pangolin::KeyModifierAlt;
                    }
                    break;
                case XK_Super_L:
                case XK_Super_R:
                    key = -1;
                    if(ev.type==KeyPress) {
                        pangolin::context->mouse_state |=  pangolin::KeyModifierCmd;
                    }else{
                        pangolin::context->mouse_state &= ~pangolin::KeyModifierCmd;
                    }
                    break;

                default: key = -1; break;
                }
            }else{
                key = ch;
            }

            if(key >=0) {
                if(ev.type == KeyPress) {
                    pangolin::process::Keyboard(key, ev.xkey.x, ev.xkey.y);
                }else{
                    pangolin::process::KeyboardUp(key, ev.xkey.x, ev.xkey.y);
                }
            }

            break;
        }
    }
}

void FinishFrame()
{
    RenderViews();
    PostRender();
    glXSwapBuffers ( display, win );
    ProcessX11Events();
}

void CreateWindowAndBind(std::string window_title, int w, int h, const Params &params)
{
    // Create Pangolin GL Context
    BindToContext(window_title);

    const bool double_buffered = params.Get(PARAM_DOUBLEBUFFER, true);
    const int  sample_buffers  = params.Get(PARAM_SAMPLE_BUFFERS, 1);
    const int  samples         = params.Get(PARAM_SAMPLES, 1);

    CreateX11Window(window_title, w, h, double_buffered, sample_buffers, samples);
    context->is_double_buffered = true;

    glewInit();

    // Process window events
    ProcessX11Events();
}

void X11ToggleFullscreen()
{
    const Atom _NET_WM_STATE_FULLSCREEN = XInternAtom(display, "_NET_WM_STATE_FULLSCREEN", True);
    const Atom _NET_WM_STATE = XInternAtom(display, "_NET_WM_STATE", True);
    XEvent e;
    e.xclient.type         = ClientMessage;
    e.xclient.window       = win;
    e.xclient.message_type = _NET_WM_STATE;
    e.xclient.format       = 32;
    e.xclient.data.l[0]    = 2;  // Toggle
    e.xclient.data.l[1]    = _NET_WM_STATE_FULLSCREEN;
    e.xclient.data.l[2]    = 0;
    e.xclient.data.l[3]    = 1;
    e.xclient.data.l[4]    = 0;

    XSendEvent(display, DefaultRootWindow(display), False, SubstructureRedirectMask | SubstructureNotifyMask, &e);
    XMoveResizeWindow(display, win, 0, 0, context->windowed_size[0], context->windowed_size[1]);
}

void StartFullScreen() {
    if(!context->is_fullscreen) {
        X11ToggleFullscreen();
        context->is_fullscreen = true;
    }
}

void StopFullScreen() {
    if(context->is_fullscreen) {
        X11ToggleFullscreen();
        context->is_fullscreen = false;
    }
}

void SetFullscreen(bool fullscreen)
{
    if(fullscreen) {
        StartFullScreen();
    }else{
        StopFullScreen();
    }
}

void PangolinPlatformInit(PangolinGl& /*context*/)
{
}

void PangolinPlatformDeinit(PangolinGl& /*context*/)
{
    X11Terminate();
}


}

