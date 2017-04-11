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

#ifndef PANGOLIN_WIDGETS_H
#define PANGOLIN_WIDGETS_H

#include <pangolin/display/view.h>
#include <pangolin/var/var.h>
#include <pangolin/handler/handler.h>
#include <pangolin/compat/function.h>
#include <pangolin/gl/glfont.h>

namespace pangolin
{

PANGOLIN_EXPORT
bool GuiVarHasChanged();

PANGOLIN_EXPORT
View& CreatePanel(const std::string& name);

struct PANGOLIN_EXPORT Panel : public View
{
    Panel();
    Panel(const std::string& auto_register_var_prefix);
    void Render();
    void ResizeChildren();
    static void AddVariable(void* data, const std::string& name, VarValueGeneric& var, bool brand_new);
};

template<typename T>
struct Widget : public View, Handler, Var<T>
{
    Widget(std::string title, VarValueGeneric& tv)
        : Var<T>(tv), title(title)
    {
        handler = this;
    }
    
    std::string title;
};

struct PANGOLIN_EXPORT Button : public Widget<bool>
{
    Button(std::string title, VarValueGeneric& tv);
    void Mouse(View&, MouseButton button, int x, int y, bool pressed, int mouse_state);
    void Render();
    
    //Cache params on resize
    void ResizeChildren();
    GlText gltext;
    GLfloat raster[2];
    bool down;
};

#ifdef CPP11_NO_BOOST
struct PANGOLIN_EXPORT FunctionButton : public Widget<boostd::function<void(void)> >
{
    FunctionButton(std::string title, VarValueGeneric& tv);
    void Mouse(View&, MouseButton button, int x, int y, bool pressed, int mouse_state);
    void Render();

    //Cache params on resize
    void ResizeChildren();
    GlText gltext;
    GLfloat raster[2];
    bool down;
};
#endif // CPP11_NO_BOOST

struct PANGOLIN_EXPORT Checkbox : public Widget<bool>
{
    Checkbox(std::string title, VarValueGeneric& tv);
    void Mouse(View&, MouseButton button, int x, int y, bool pressed, int mouse_state);
    void Render();
    
    //Cache params on resize
    void ResizeChildren();
    GlText gltext;
    GLfloat raster[2];
    Viewport vcb;
};

struct PANGOLIN_EXPORT Slider : public Widget<double>
{
    Slider(std::string title, VarValueGeneric& tv);
    void Mouse(View&, MouseButton button, int x, int y, bool pressed, int mouse_state);
    void MouseMotion(View&, int x, int y, int mouse_state);
    void Keyboard(View&, unsigned char key, int x, int y, bool pressed);
    void Render();
    
    //Cache params on resize
    void ResizeChildren();
    GlText gltext;
    GLfloat raster[2];
    bool lock_bounds;
    bool logscale;
};

struct PANGOLIN_EXPORT TextInput : public Widget<std::string>
{
    TextInput(std::string title, VarValueGeneric& tv);
    void Mouse(View&, MouseButton button, int x, int y, bool pressed, int mouse_state);
    void MouseMotion(View&, int x, int y, int mouse_state);
    void Keyboard(View&, unsigned char key, int x, int y, bool pressed);
    void Render();
    
    std::string edit;
    GlText gledit;

    //Cache params on resize
    void ResizeChildren();
    GlText gltext;
    GLfloat raster[2];
    bool do_edit;
    int sel[2];
};


}
#endif // PANGOLIN_WIDGETS_H
