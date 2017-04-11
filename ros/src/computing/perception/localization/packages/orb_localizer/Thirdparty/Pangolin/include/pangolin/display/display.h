/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove, Richard Newcombe
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

#ifndef PANGOLIN_DISPLAY_H
#define PANGOLIN_DISPLAY_H

#include <pangolin/platform.h>
#include <pangolin/gl/glinclude.h>
#include <pangolin/compat/function.h>
#include <pangolin/handler/handler_enums.h>
#include <pangolin/utils/params.h>

#include <string>

/*! \file display.h
 * This file contains a number of global methods for creating and
 * querying window state as well as handling user input.
 */

namespace pangolin
{

  // CreateWindowAndBind parameter key names.
  // X11 Window options:
  extern const char* PARAM_DOUBLEBUFFER;   // bool
  extern const char* PARAM_SAMPLE_BUFFERS; // int
  extern const char* PARAM_SAMPLES;        // int

  // Forward Declarations
  struct View;
  struct Viewport;
  class  UserApp;
  
  /// Give this OpenGL context a name or switch contexts.
  /// This is required to initialise Pangolin for use with an
  /// externally defined OpenGL context. You needn't call it
  /// if you have used CreateWindowAndBind() to create a window
  /// or launched a pangolin::UserApp
  PANGOLIN_EXPORT
  void BindToContext(std::string name);

  /// Initialise OpenGL window (determined by platform) and bind context.
  /// This method will choose an available windowing system if one is present.
  PANGOLIN_EXPORT
  void CreateWindowAndBind(std::string window_title, int w = 640, int h = 480, const Params& params = Params());

  /// Launch users derived UserApp, controlling OpenGL event loop.
  /// This method will block until the application exits, calling app's
  /// Init() method to start and Render() method subsequently to draw each frame.
  /// @return exit code for use when returning from main. Currently always 0.
  PANGOLIN_EXPORT
  int LaunchUserApp(UserApp& app);
  
  /// Perform any post rendering, event processing and frame swapping.
  PANGOLIN_EXPORT
  void FinishFrame();

  /// Request that the program exit.
  PANGOLIN_EXPORT
  void Quit();

  /// Returns true if user has requested to close OpenGL window.
  PANGOLIN_EXPORT
  bool ShouldQuit();

  /// Returns true if user has interacted with the window since this was last called.
  PANGOLIN_EXPORT
  bool HadInput();

  /// Returns true if user has resized the window.
  PANGOLIN_EXPORT
  bool HasResized();

  /// Renders any views with default draw methods.
  PANGOLIN_EXPORT
  void RenderViews();
  
  /// Perform any post render events, such as screen recording.
  PANGOLIN_EXPORT
  void PostRender();

  /// Request to be notified via functor when key is pressed.
  /// Functor may take one parameter which will equal the key pressed
  PANGOLIN_EXPORT
  void RegisterKeyPressCallback(int key, boostd::function<void(void)> func);

  /// Save window contents to image.
  PANGOLIN_EXPORT
  void SaveWindowOnRender(std::string filename_prefix);
  
  PANGOLIN_EXPORT
  void SaveFramebuffer(std::string prefix, const Viewport& v);
  
  namespace process
  {
    /// Tell pangolin to process input to drive display.
    /// You will need to call this manually if you haven't let
    /// Pangolin register callbacks from your windowing system
    PANGOLIN_EXPORT
    void Keyboard( unsigned char key, int x, int y);

    PANGOLIN_EXPORT
    void KeyboardUp(unsigned char key, int x, int y);
    
    PANGOLIN_EXPORT
    void SpecialFunc(int key, int x, int y);
    
    PANGOLIN_EXPORT
    void SpecialFuncUp(int key, int x, int y);

    /// Tell pangolin base window size has changed
    /// You will need to call this manually if you haven't let
    /// Pangolin register callbacks from your windowing system
    PANGOLIN_EXPORT
    void Resize(int width, int height);

    /// Event based rendering entry point (from e.g.
    /// glutMainLoop). Not currently supported.
    PANGOLIN_EXPORT
    void Display();

    PANGOLIN_EXPORT
    void Mouse( int button, int state, int x, int y);

    PANGOLIN_EXPORT
    void MouseMotion( int x, int y);

    PANGOLIN_EXPORT
    void PassiveMouseMotion(int x, int y);

    PANGOLIN_EXPORT
    void Scroll(float x, float y);

    PANGOLIN_EXPORT
    void Zoom(float m);

    PANGOLIN_EXPORT
    void Rotate(float r);
    
    PANGOLIN_EXPORT
    void SubpixMotion(float x, float y, float pressure, float rotation, float tiltx, float tilty);

    PANGOLIN_EXPORT
    void SpecialInput(InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4);

  }

  /// Retrieve 'base' display, corresponding to entire window.
  PANGOLIN_EXPORT
  View& DisplayBase();

  /// Create or retrieve named display managed by pangolin (automatically deleted).
  PANGOLIN_EXPORT
  View& Display(const std::string& name);

  /// Create unnamed display managed by pangolin (automatically deleted).
  PANGOLIN_EXPORT
  View& CreateDisplay();

  /// Switch between windowed and fullscreen mode.
  PANGOLIN_EXPORT
  void ToggleFullscreen();

  /// Switch windows/fullscreenmode = fullscreen.
  PANGOLIN_EXPORT
  void SetFullscreen(bool fullscreen = true);

  /// Toggle display of Pangolin console
  PANGOLIN_EXPORT
  void ToggleConsole();

  /// Convenience functor for toggling pangolin::View.
  /// Use with RegisterKeyPressCallback for example
  struct ToggleViewFunctor {
      inline ToggleViewFunctor(View& view);
      inline ToggleViewFunctor(const std::string& name);
      void operator()();
      View& view;
  };

}

#endif // PANGOLIN_DISPLAY_H

