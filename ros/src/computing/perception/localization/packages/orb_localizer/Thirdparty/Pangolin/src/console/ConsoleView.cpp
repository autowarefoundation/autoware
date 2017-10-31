#include <iterator>
#include <pangolin/console/ConsoleView.h>
#include <pangolin/utils/picojson.h>

namespace pangolin
{

inline Colour ParseJson(const json::value& val)
{
    return Colour(
        val.contains("r") ? val["r"].get<double>() : 0.0,
        val.contains("g") ? val["g"].get<double>() : 0.0,
        val.contains("b") ? val["b"].get<double>() : 0.0,
        val.contains("a") ? val["a"].get<double>() : 1.0
    );
}

inline json::value toJson(const Colour& colour)
{
    json::value ret(json::object_type,true);
    ret["r"] = colour.r;
    ret["g"] = colour.g;
    ret["b"] = colour.b;
    ret["a"] = colour.a;
    return ret;
}

inline std::ostream& operator<<(std::ostream& os, const Colour& colour)
{
    os << toJson(colour).serialize();
    return os;
}

inline std::istream& operator>>(std::istream& is, Colour& colour)
{
    json::value val;
    json::parse(val,is);
    colour = ParseJson(val);
    return is;
}

inline void glColour(const Colour& c)
{
    glColor4f(c.r,c.g,c.b,c.a);
}

ConsoleView::ConsoleView(ConsoleInterpreter* interpreter)
    : interpreter(interpreter),
      font(GlFont::I()),
      hiding(false),
      bottom(1.0f),
      background_colour(0.2f, 0.0f, 0.0f, 0.6f),
      animation_speed(0.2)
{
    SetHandler(this);

    line_colours[ConsoleLineTypeCmd]        = Colour(1.0,1.0,1.0,1.0);
    line_colours[ConsoleLineTypeCmdOptions] = Colour(0.9,0.9,0.9,1.0);
    line_colours[ConsoleLineTypeOutput]     = Colour(0.0,1.0,1.0,1.0);
    line_colours[ConsoleLineTypeHelp]       = Colour(1.0,0.8,1.0,1.0);
    line_colours[ConsoleLineTypeStdout]     = Colour(0.0,0.0,1.0,1.0);
    line_colours[ConsoleLineTypeStderr]     = Colour(1.0,0.8,0.8,1.0);

    Var<Colour>::Attach("pango.console.colours.Background", background_colour);
    Var<Colour>::Attach("pango.console.colours.Cmd",        line_colours[ConsoleLineTypeCmd]);
    Var<Colour>::Attach("pango.console.colours.CmdOptions", line_colours[ConsoleLineTypeCmdOptions]);
    Var<Colour>::Attach("pango.console.colours.Stdout",     line_colours[ConsoleLineTypeStdout]);
    Var<Colour>::Attach("pango.console.colours.Stderr",     line_colours[ConsoleLineTypeStderr]);
    Var<Colour>::Attach("pango.console.colours.Output",     line_colours[ConsoleLineTypeOutput]);
    Var<Colour>::Attach("pango.console.colours.Help",       line_colours[ConsoleLineTypeHelp]);

    Var<float>::Attach("pango.console.animation_speed", animation_speed);

    AddLine("Pangolin Python Command Prompt:", ConsoleLineTypeHelp);
    AddLine("===============================", ConsoleLineTypeHelp);
}

ConsoleView::~ConsoleView() {
    delete interpreter;
}

void ConsoleView::ProcessOutputLines()
{
    // empty output queue
    ConsoleLine line_in;
    while(interpreter->PullLine(line_in))
    {
        AddLine(line_in.text, line_in.linetype);
    }
}

View& ConsoleView::Show(bool should_show)
{
    if(should_show) {
        hiding = false;
        show = true;
    }else{
        hiding = true;
    }
    return *this;
}

void ConsoleView::ToggleShow()
{
    Show(!IsShown());
}

bool ConsoleView::IsShown() const
{
    return show && !hiding;
}

void ConsoleView::DrawLine(const ConsoleView::Line& l)
{
    glColour(line_colours[l.linetype]);
    l.text.Draw();
}

void ConsoleView::Render()
{
    if(hiding) {
        bottom += (1.0f - bottom) * animation_speed;
        if(1.0 - bottom < 0.01) {
            bottom = 1.0;
            show = false;
            hiding = false;
            return;
        }
    }else{
        if(bottom > 0.01f) {
            bottom -= bottom*animation_speed;
        }else{
            bottom = 0.0f;
        }
    }

    ProcessOutputLines();

#ifndef HAVE_GLES
    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT | GL_SCISSOR_BIT | GL_VIEWPORT_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);
#endif

    this->ActivatePixelOrthographic();
    glDisable(GL_DEPTH_TEST );
    glDisable(GL_LIGHTING);
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_LINE_SMOOTH);
    glDisable( GL_COLOR_MATERIAL );
    glLineWidth(1.0);

    glEnable(GL_BLEND);
    glBlendFunc( GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA );

    glColour(background_colour);

    GLfloat verts[] = { 0.0f, (GLfloat)v.h,
                        (GLfloat)v.w, (GLfloat)v.h,
                        (GLfloat)v.w, bottom*v.h,
                        0.0f, bottom*v.h };
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 0, verts);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glDisableClientState(GL_VERTEX_ARRAY);


    const double line_space = font.Height();
    glTranslated(10.0, 10.0 + bottom*v.h, 0.0 );
    DrawLine(current_line);
    glTranslated(0.0, line_space, 0.0);
    for(size_t l=0; l < line_buffer.size(); ++l) {
        DrawLine(line_buffer[l]);
        glTranslated(0.0, line_space, 0.0);
    }

#ifndef HAVE_GLES
    glPopAttrib();
#endif
}

inline std::string CommonPrefix(const std::vector<std::string>& vec)
{
    if(!vec.size()) return "";

    size_t cmn = vec[0].size();
    for(size_t i=1; i<vec.size(); ++i) {
        cmn = std::min(vec[i].size(), cmn);
        for(size_t p=0; p < cmn; ++p) {
            if(vec[i][p] != vec[0][p]) {
                cmn = p;
                break;
            }
        }
    }

    return vec[0].substr(0,cmn);
}

void ConsoleView::Keyboard(View&, unsigned char key, int x, int y, bool pressed)
{
    static int hist_id = -1;
    static std::string prefix;
    static bool edited = true;

    if(pressed) {
        if(key=='\r') key = '\n';

        GlText& txt = current_line.text;
        const std::string cmd = txt.Text();

        if(key=='`') {
            ToggleShow();
        } else if(key=='\n') {
            interpreter->PushCommand(cmd);
            line_buffer.push_front(current_line);
            hist_id = -1;
            prefix = "";
            edited = true;
            txt.Clear();
        }else if(key=='\t') {
            std::vector<std::string> options = interpreter->Complete(cmd,100);
            if(options.size()) {
                const std::string common = CommonPrefix(options);
                if(common != cmd) {
                    current_line = font.Text("%s", common.c_str());
                }else{
                    std::stringstream s;
                    std::copy(options.begin(), options.end(), std::ostream_iterator<std::string>(s,", "));
                    AddLine(s.str(), ConsoleLineTypeCmdOptions);
                }
            }
        }else if(key==PANGO_SPECIAL + PANGO_KEY_UP) {
            if(edited) {
                prefix = cmd;
                edited = false;
            }
            Line* hist_line = GetLine(hist_id+1, ConsoleLineTypeCmd, prefix);
            if(hist_line) {
                current_line = *hist_line;
                hist_id++;
            }
        }else if(key==PANGO_SPECIAL + PANGO_KEY_DOWN) {
            if(edited) {
                prefix = cmd;
                edited = false;
            }
            Line* hist_line = GetLine(hist_id-1, ConsoleLineTypeCmd, prefix);
            if(hist_line) {
                current_line = *hist_line;
                hist_id--;
            }
        }else if(key=='\b') {
            txt = font.Text("%s", txt.Text().substr(0,txt.Text().size()-1).c_str() );
            edited = true;
        }else if(key < PANGO_SPECIAL){
            txt = font.Text("%s%c", txt.Text().c_str(), key);
            edited = true;
        }
    }
}

void ConsoleView::AddLine(const std::string& text, ConsoleLineType linetype )
{
    line_buffer.push_front( Line( font.Text("%s",text.c_str()), linetype) );
}

ConsoleView::Line* ConsoleView::GetLine(int id, ConsoleLineType line_type, const std::string& prefix )
{
    int match = 0;
    for(Line& l : line_buffer)
    {
        if(l.linetype == line_type && l.text.Text().substr(0,prefix.size()) == prefix  ) {
            if(id == match) {
                return &l;
            }else{
                ++match;
            }
        }
    }

    return nullptr;
}

}
