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

#ifndef PANGOLIN_VIEW_H
#define PANGOLIN_VIEW_H

#include <vector>

#include <pangolin/display/viewport.h>
#include <pangolin/display/attach.h>
#include <pangolin/display/opengl_render_state.h>
#include <pangolin/compat/function.h>

namespace pangolin
{

enum Layout
{
    LayoutOverlay,
    LayoutVertical,
    LayoutHorizontal,
    LayoutEqual,
    LayoutEqualVertical,
    LayoutEqualHorizontal
};

enum Lock {
    LockLeft = 0,
    LockBottom = 0,
    LockCenter = 1,
    LockRight = 2,
    LockTop = 2
};

// Forward declaration
struct Handler;

/// A Display manages the location and resizing of an OpenGl viewport.
struct PANGOLIN_EXPORT View
{
    View(double aspect=0.0)
        : aspect(aspect), top(1.0),left(0.0),right(1.0),bottom(0.0), hlock(LockCenter),vlock(LockCenter),
          layout(LayoutOverlay), scroll_offset(0), show(1), zorder(0), handler(0) {}
    
    virtual ~View() {}
    
    //! Activate Displays viewport for drawing within this area
    void Activate() const;
    
    //! Activate Displays and set State Matrices
    void Activate(const OpenGlRenderState& state ) const;
    
    //! Activate Displays viewport and Scissor for drawing within this area
    void ActivateAndScissor() const;
    
    //! Activate Displays viewport and Scissor for drawing within this area
    void ActivateScissorAndClear() const;
    
    //! Activate Display and set State Matrices
    void ActivateAndScissor(const OpenGlRenderState& state ) const;
    
    //! Activate Display and set State Matrices
    void ActivateScissorAndClear(const OpenGlRenderState& state ) const;
    
    //! Activate Display and setup coordinate system for 2d pixel View coordinates
    void ActivatePixelOrthographic() const;
    
    //! Activate Display and reset coordinate system to OpenGL default
    void ActivateIdentity() const;
    
    //! Return closest depth buffer value within radius of window (winx,winy)
    GLfloat GetClosestDepth(int winx, int winy, int radius) const;
    
    //! Obtain camera space coordinates of scene at pixel (winx, winy, winzdepth)
    //! winzdepth can be obtained from GetClosestDepth
    void GetCamCoordinates(const OpenGlRenderState& cam_state, double winx, double winy, double winzdepth, GLdouble& x, GLdouble& y, GLdouble& z) const;
    
    //! Obtain object space coordinates of scene at pixel (winx, winy, winzdepth)
    //! winzdepth can be obtained from GetClosestDepth
    void GetObjectCoordinates(const OpenGlRenderState& cam_state, double winx, double winy, double winzdepth, GLdouble& x, GLdouble& y, GLdouble& z) const;
    
    //! Given the specification of Display, compute viewport
    virtual void Resize(const Viewport& parent);
    
    //! Instruct all children to resize
    virtual void ResizeChildren();
    
    //! Perform any automatic rendering for this View.
    //! Default implementation simply instructs children to render themselves.
    virtual void Render();
    
    //! Instruct all children to render themselves if appropriate
    virtual void RenderChildren();
    
    //! Set this view as the active View to receive input
    View& SetFocus();

    //! Returns true iff this view currently has focus and will receive user input
    bool HasFocus() const;

    //! Set bounds for the View using mixed fractional / pixel coordinates (OpenGl view coordinates)
    View& SetBounds(Attach bottom, Attach top, Attach left, Attach right);
   
    //! Set bounds for the View using mixed fractional / pixel coordinates (OpenGl view coordinates)
    View& SetBounds(Attach bottom, Attach top, Attach left, Attach right, bool keep_aspect);
    
    //! Set bounds for the View using mixed fractional / pixel coordinates (OpenGl view coordinates)
    View& SetBounds(Attach bottom, Attach top, Attach left, Attach right, double aspect);
    
    //! Designate handler for accepting mouse / keyboard input.
    View& SetHandler(Handler* handler);
    
    //! Set drawFunc as the drawing function for this view
    View& SetDrawFunction(const boostd::function<void(View&)>& drawFunc);
    
    //! Force this view to have the given aspect, whilst fitting snuggly
    //! within the parent. A negative value with 'over-draw', fitting the
    //! smaller side of the parent.
    View& SetAspect(double aspect);
    
    //! Set how this view should be positioned relative to its parent
    View& SetLock(Lock horizontal, Lock vertical );
    
    //! Set layout policy for this view
    View& SetLayout(Layout layout);
    
    //! Add view as child
    View& AddDisplay(View& view);
    
    //! Show / hide this view
    View& Show(bool show=true);
    
    //! Toggle this views visibility
    void ToggleShow();
    
    //! Return whether this view should be shown.
    //! This method should be checked if drawing manually
    bool IsShown() const;
    
    //! Returns viewport reflecting space that will actually get drawn
    //! The minimum of vp and v
    Viewport GetBounds() const;
    
    //! Specify that this views region in the framebuffer should be saved to
    //! a file just before the buffer is flipped.
    void SaveOnRender(const std::string& filename_prefix);
    
    //! Specify that this views region in the framebuffer should be saved to
    //! a video just before the buffer is flipped
    void RecordOnRender(const std::string& record_uri);
    
    //! Uses the views default render method to draw into an FBO 'scale' times
    //! the size of the view and save to a file.
    void SaveRenderNow(const std::string& filename_prefix, float scale = 1);
    
    //! Return number of child views attached to this view
    size_t NumChildren() const;
    
    //! Return (i)th child of this view
    View& operator[](size_t i);
    
    //! Return number of visible child views attached to this view.
    size_t NumVisibleChildren() const;
    
    //! Return visible child by index.
    View& VisibleChild(size_t i);
    
    //! Return visible child at window coords x,y
    View* FindChild(int x, int y);  
    
    // Desired width / height aspect (0 if dynamic)
    double aspect;
    
    // Bounds to fit display within
    Attach top, left, right, bottom;
    Lock hlock;
    Lock vlock;
    Layout layout;
    
    int scroll_offset;
    
    // Cached client area (space allocated from parent)
    Viewport vp;
    
    // Cached absolute viewport (recomputed on resize - respects aspect)
    Viewport v;
    
    // Should this view be displayed?
    bool show;

    // Child views are rendered in order of low to high z-order
    // Views default to 0 z-order
    int zorder;
    
    // Input event handler (if any)
    Handler* handler;
    
    // Map for sub-displays (if any)
    std::vector<View*> views;
    
    // External draw function
    boostd::function<void(View&)> extern_draw_function;
    
private:
    // Private copy constructor
    View(View&) { /* Do Not copy - take reference instead*/ }
};

}

#endif // PANGOLIN_VIEW_H
