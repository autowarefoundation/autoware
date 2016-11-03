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

#ifndef PANGOLIN_DISPLAY_INTERNAL_H
#define PANGOLIN_DISPLAY_INTERNAL_H

#include <pangolin/platform.h>

#ifdef HAVE_PYTHON
#include <pangolin/console/ConsoleView.h>
#endif // HAVE_PYTHON

#include <pangolin/display/view.h>
#include <pangolin/compat/function.h>
#include <pangolin/display/user_app.h>

#include <map>
#include <queue>

#ifdef BUILD_PANGOLIN_VIDEO
#include <pangolin/video/video_output.h>
#endif // BUILD_PANGOLIN_VIDEO


namespace pangolin
{

typedef std::map<const std::string,View*> ViewMap;
typedef std::map<int,boostd::function<void(void)> > KeyhookMap;

struct PANGOLIN_EXPORT PangolinGl
{
    PangolinGl();
    ~PangolinGl();
    
    // Base container for displays
    View base;
    
    // Named views which are managed by pangolin (i.e. created / deleted by pangolin)
    ViewMap named_managed_views;

    // Optional user app
    UserApp* user_app;
    
    // Global keypress hooks
    KeyhookMap keypress_hooks;
    
    // Manage fullscreen (ToggleFullscreen is quite new)
    bool is_double_buffered;
    bool is_fullscreen;
    GLint windowed_size[2];
    
    // State relating to interactivity
    bool quit;
    int had_input;
    int has_resized;
    int mouse_state;
    View* activeDisplay;
    
    std::queue<std::pair<std::string,Viewport> > screen_capture;
    
#ifdef BUILD_PANGOLIN_VIDEO
    View* record_view;
    VideoOutput recorder;
#endif

#ifdef HAVE_PYTHON
    ConsoleView* console_view;
#endif
};

// Implemented in platform specific display file.
PANGOLIN_EXPORT
void PangolinPlatformInit(PangolinGl& context);

// Implemented in platform specific display file.
PANGOLIN_EXPORT
void PangolinPlatformDeinit(PangolinGl& context);

#ifdef BUILD_PANGOLIN_VIDEO
  void SaveFramebuffer(VideoOutput& video, const Viewport& v);
#endif // BUILD_PANGOLIN_VIDEO

}

#endif // PANGOLIN_DISPLAY_INTERNAL_H

