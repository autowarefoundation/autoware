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

#pragma once

#include <deque>

#include <pangolin/platform.h>
#include <pangolin/gl/glfont.h>
#include <pangolin/var/var.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/colour.h>

#include <pangolin/console/ConsoleInterpreter.h>

namespace pangolin
{

class ConsoleView : public pangolin::View, pangolin::Handler
{
public:
    struct Line
    {
        Line()
        {
        }

        Line(const GlText& text, ConsoleLineType linetype = ConsoleLineTypeCmd )
            : text(text), linetype(linetype)
        {
        }

        GlText text;
        ConsoleLineType linetype;
    };


    // Construct with interpreter (and take ownership)
    ConsoleView(ConsoleInterpreter* interpreter);

    ~ConsoleView();

    // Replace implementation in View to account for hiding animation
    View& Show(bool show=true);

    // Replace implementation in View to account for hiding animation
    void ToggleShow();

    // Replace implementation in View to account for hiding animation
    bool IsShown() const;

    void Render() PANGOLIN_OVERRIDE;

    void Keyboard(View&, unsigned char key, int x, int y, bool pressed) PANGOLIN_OVERRIDE;

private:
    void DrawLine(const ConsoleView::Line& l);

    void ProcessOutputLines();

    void AddLine(const std::string& text, ConsoleLineType linetype = ConsoleLineTypeCmd);

    Line* GetLine(int id, ConsoleLineType line_type, const std::string& prefix = "");

    ConsoleInterpreter* interpreter;

    GlFont& font;

    std::map<ConsoleLineType, GlText> prompts;

    Line current_line;
    std::deque<Line> line_buffer;

    bool hiding;
    GLfloat bottom;

    Colour background_colour;
    std::map<ConsoleLineType,pangolin::Colour> line_colours;
    float animation_speed;
};

}
