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

#include <pangolin/plot/plotter.h>
#include <pangolin/gl/gldraw.h>

#include <iomanip>
#include <cctype>

namespace pangolin
{

std::string ReplaceChar(const std::string& str, char from, char to)
{
    std::string ret = str;
    for(size_t i=0; i<ret.length(); ++i) {
        if(ret[i] == from) ret[i] = to;
    }
    return ret;
}

std::set<int> ConvertSequences(const std::string& str, char seq_char='$', char id_char='i')
{
    std::set<int> sequences;

    for(size_t i=0; i<str.length(); ++i) {
        if(str[i] == seq_char) {
            if(i+1 < str.length() && str[i+1] == id_char) {
                sequences.insert(-1);
            }else{
                int v = 0;
                for(size_t j=i+1; std::isdigit(str[j]) && j< str.length(); ++j) {
                    v = v*10 + (str[j] - '0');
                }
                sequences.insert(v);
            }
        }
    }

    return sequences;
}

Plotter::PlotSeries::PlotSeries()
    : drawing_mode(GL_LINE_STRIP)
{

}

void Plotter::PlotSeries::CreatePlot(const std::string &x, const std::string &y, Colour colour, std::string title)
{
    static const std::string vs_header =
            "uniform float u_id_offset;\n"
            "uniform vec4 u_color;\n"
            "uniform vec2 u_scale;\n"
            "uniform vec2 u_offset;\n"
            "varying vec4 v_color;\n"
            "void main() {\n";

    static const std::string vs_footer =
            "    vec2 pos = vec2(x, y);\n"
            "    gl_Position = vec4(u_scale * (pos + u_offset),0,1);\n"
            "    v_color = u_color;\n"
            "}\n";

    static const std::string fs =
        #ifdef HAVE_GLES_2
            "precision mediump float;\n"
        #endif // HAVE_GLES_2
            "varying vec4 v_color;\n"
            "void main() {\n"
            "  gl_FragColor = v_color;\n"
            "}\n";

    attribs.clear();

    this->colour = colour;
    this->title  = GlFont::I().Text(title.c_str());
    const std::set<int> ax = ConvertSequences(x);
    const std::set<int> ay = ConvertSequences(y);
    std::set<int> as;
    as.insert(ax.begin(), ax.end());
    as.insert(ay.begin(), ay.end());
    contains_id = ( as.find(-1) != as.end() );

    std::ostringstream oss_prog;

    for(std::set<int>::const_iterator i=as.begin(); i != as.end(); ++i) {
        std::ostringstream oss;
        oss << "s" << *i;
        const std::string name = *i >= 0 ? oss.str() : "sn";
        attribs.push_back( PlotAttrib(name, *i) );
        oss_prog << "attribute float " + name + ";\n";
    }

    oss_prog << vs_header;
    if(contains_id) {
        oss_prog << "float si = sn + u_id_offset;\n";
    }
    oss_prog << "float x = " + ReplaceChar(x,'$','s') + ";\n";
    oss_prog << "float y = " + ReplaceChar(y,'$','s') + ";\n";
    oss_prog << vs_footer;

    prog.AddShader( GlSlVertexShader, oss_prog.str() );
    prog.AddShader( GlSlFragmentShader, fs );
    prog.Link();

    // Lookup attribute locations in compiled shader
    prog.SaveBind();
    for(size_t i=0; i<attribs.size(); ++i) {
        attribs[i].location = prog.GetAttributeHandle( attribs[i].name );
    }
    prog.Unbind();
}

void Plotter::PlotImplicit::CreatePlot(const std::string& code)
{
    static const std::string vs =
            "attribute vec2 a_position;\n"
            "uniform vec2 u_scale;\n"
            "uniform vec2 u_offset;\n"
            "varying float x;\n"
            "varying float y;\n"
            "void main() {\n"
            "    gl_Position = vec4(u_scale * (a_position + u_offset),0,1);\n"
            "    x = a_position.x;"
            "    y = a_position.y;"
            "}\n";

    static const std::string fs1 =
        #ifdef HAVE_GLES_2
            "precision mediump float;\n"
        #endif // HAVE_GLES_2
            "varying float x;\n"
            "varying float y;\n"
            "void main() {\n";
    static const std::string fs2 =
            "}\n";

    prog.AddShader( GlSlVertexShader, vs );
    prog.AddShader( GlSlFragmentShader, fs1 + code + fs2 );
    prog.BindPangolinDefaultAttribLocationsAndLink();
}


void Plotter::PlotImplicit::CreateColouredPlot(const std::string& code)
{
    CreatePlot(
        "  float r=1.0;\n"
        "  float g=1.0;\n"
        "  float b=1.0;\n"
        "  float a=0.5;\n" +
           code +
        "  gl_FragColor = vec4(r,g,b,a);\n"
        );
}

void Plotter::PlotImplicit::CreateInequality(const std::string& ie, Colour c)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << "if( !(" << ie << ") ) discard;\n";
    oss << "gl_FragColor = vec4(" << c.r << "," << c.g << "," << c.b << "," << c.a << ");\n";

    CreatePlot( oss.str() );
}

void Plotter::PlotImplicit::CreateDistancePlot(const std::string& dist)
{

}

Plotter::Plotter(
    DataLog* log,
    float left, float right, float bottom, float top,
    float tickx, float ticky,
    Plotter* linked_plotter_x,
    Plotter* linked_plotter_y
)   : log(log),
      colour_wheel(0.6f),
      rview_default(left,right,bottom,top), rview(rview_default), target(rview),
      selection(0,0,0,0),
      track(false), track_x("$i"), track_y(""),
      trigger_edge(0), trigger("$0"),
      linked_plotter_x(linked_plotter_x),
      linked_plotter_y(linked_plotter_y)
{
    if(!log) {
        throw std::runtime_error("DataLog not specified");
    }
    // Prevent links to ourselves - this could cause infinite recursion.
    if(linked_plotter_x == this) this->linked_plotter_x = 0;
    if(linked_plotter_y == this) this->linked_plotter_y = 0;

    // Handle our own mouse / keyboard events
    this->handler = this;
    hover[0] = 0;
    hover[1] = 0;

    // Default colour scheme
    colour_bg = Colour(0.0f, 0.0f, 0.0f);
    colour_tk = Colour(0.2f, 0.2f, 0.2f);
    colour_ax = Colour(0.5f, 0.5f, 0.5f);

    SetTicks(tickx, ticky);

    // Create shader for drawing simple primitives
    prog_lines.AddShader( GlSlVertexShader,
                         "attribute vec2 a_position;\n"
                         "uniform vec4 u_color;\n"
                         "uniform vec2 u_scale;\n"
                         "uniform vec2 u_offset;\n"
                         "varying vec4 v_color;\n"
                         "void main() {\n"
                         "    gl_Position = vec4(u_scale * (a_position + u_offset),0,1);\n"
                         "    v_color = u_color;\n"
                         "}\n"
                         );
    prog_lines.AddShader( GlSlFragmentShader,
                      #ifdef HAVE_GLES_2
                          "precision mediump float;\n"
                      #endif // HAVE_GLES_2
                         "varying vec4 v_color;\n"
                         "void main() {\n"
                         "  gl_FragColor = v_color;\n"
                         "}\n"
                         );
    prog_lines.BindPangolinDefaultAttribLocationsAndLink();

    prog_text.AddShader( GlSlVertexShader,
                         "attribute vec2 a_position;\n"
                         "attribute vec2 a_texcoord;\n"
                         "uniform vec4 u_color;\n"
                         "uniform vec2 u_scale;\n"
                         "uniform vec2 u_offset;\n"
                         "varying vec4 v_color;\n"
                         "varying vec2 v_texcoord;\n"
                         "void main() {\n"
                         "    gl_Position = vec4(u_scale * (a_position + u_offset),0,1);\n"
                         "    v_color = u_color;\n"
                         "    v_texcoord = a_texcoord;\n"
                         "}\n"
                         );
    prog_text.AddShader( GlSlFragmentShader,
                     #ifdef HAVE_GLES_2
                         "precision mediump float;\n"
                     #endif // HAVE_GLES_2
                         "varying vec4 v_color;\n"
                         "varying vec2 v_texcoord;\n"
                         "uniform sampler2D u_texture;\n"
                         "void main() {\n"
                         "  gl_FragColor = v_color;\n"
                         "  gl_FragColor.a *= texture2D(u_texture, v_texcoord).a;\n"
                         "}\n"
                         );
    prog_text.BindPangolinDefaultAttribLocationsAndLink();


    // Setup default PlotSeries
    plotseries.reserve(10);
    const std::vector<std::string>& labels = log->Labels();
    for(unsigned int i=0; i< 10; ++i) {
        std::ostringstream oss;
        oss << "$" << i;
        plotseries.push_back( PlotSeries() );
        plotseries.back().CreatePlot( "$i", oss.str(),
            colour_wheel.GetUniqueColour(),
            i < labels.size() ? log->Labels()[i] : oss.str()
        );
    }

    // Setup test PlotMarkers
    plotmarkers.reserve(100);
//    plotmarkers.push_back( Marker( Marker::Vertical, 10, Marker::GreaterThan, Colour(1,0,0,0.2)) );
//    plotmarkers.push_back( Marker( Marker::Horizontal, 1, Marker::LessThan, Colour(0,1,0,0.2)) );

    // Setup test implicit plots.
    plotimplicits.reserve(10);
//    plotimplicits.push_back( PlotImplicit() );
//    plotimplicits.back().CreateInequality("x+y <= 150.0", colour_wheel.GetUniqueColour().WithAlpha(0.2) );
//    plotimplicits.push_back( PlotImplicit() );
//    plotimplicits.back().CreateInequality("x+2.0*y <= 170.0", colour_wheel.GetUniqueColour().WithAlpha(0.2) );
//    plotimplicits.push_back( PlotImplicit() );
//    plotimplicits.back().CreateInequality("3.0*y <= 180.0", colour_wheel.GetUniqueColour().WithAlpha(0.2) );

    // Setup texture spectogram style plots
    // ...

}

Plotter::~Plotter()
{

}

template <typename T> int data_sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void Plotter::ComputeTrackValue( float track_val[2] )
{
    if(trigger_edge) {
        // Track last edge transition matching trigger_edge
        const DataLogBlock* block = log->LastBlock();
        if(block) {
            int s = (int)block->StartId() + (int)block->Samples() - 1;
            const size_t dim = block->Dimensions();
            const float* data = block->Sample(s);
            int last_sgn = 0;
            for(; s >= 0; --s, data -= dim )
            {
                const float val = data[0] - trigger_value;
                const int sgn = data_sgn(val);
                if(last_sgn * sgn == -1 && last_sgn == trigger_edge) {
                    track_val[0] = (float)s;
                    track_val[1] = 0.0f;
                    return;
                }
                last_sgn = sgn;
            }
        }
        // Fall back to simple last value tracking
    }

    track_val[0] = (float)log->Samples();
    track_val[1] = 0.0f;
}

XYRangef Plotter::ComputeAutoSelection()
{
    XYRangef range;
    range.x = target.x;

    const DataLogBlock* block = log->FirstBlock();

    if(block) {
        for(size_t i=0; i < plotseries.size(); ++i)
        {
            if( plotseries[i].attribs.size() == 2 && plotseries[i].attribs[0].plot_id == -1) {
                const int id = plotseries[i].attribs[1].plot_id;
                if( 0<= id && id < (int)block->Dimensions()) {
                    range.y.Insert(log->Stats(id).min);
                    range.y.Insert(log->Stats(id).max);
                }
            }

        }
    }

    return range;
}

void Plotter::Render()
{
    // Animate scroll / zooming
    UpdateView();

#ifndef HAVE_GLES
    glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_LINE_BIT);
#endif

    glClearColor(colour_bg.r, colour_bg.g, colour_bg.b, colour_bg.a);
    ActivateScissorAndClear();

    // Try to create smooth lines
    glDisable(GL_MULTISAMPLE);
    glLineWidth(1.5);
    glEnable(GL_LINE_SMOOTH);
    glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_LIGHTING);
    glDisable( GL_DEPTH_TEST );

    const float w = rview.x.Size();
    const float h = rview.y.Size();
    const float ox = -rview.x.Mid();
    const float oy = -rview.y.Mid();
    const float sx = 2.0f / w;
    const float sy = 2.0f / h;

    //////////////////////////////////////////////////////////////////////////
    // Draw ticks
    prog_lines.SaveBind();
    prog_lines.SetUniform("u_scale",  sx, sy);
    prog_lines.SetUniform("u_offset", ox, oy);
    prog_lines.SetUniform("u_color",  colour_tk );

    const float min_space = 80.0;
    int ta[2] = {1,1};
    while(v.w * ta[0] *tick[0].val / w < min_space) ta[0] *=2;
    while(v.h * ta[1] *tick[1].val / h < min_space) ta[1] *=2;

    const float tdelta[2] = {
        tick[0].val * ta[0],
        tick[1].val * ta[1]
    };

    const int tx[2] = {
        (int)ceil(rview.x.min / tdelta[0]),
        (int)ceil(rview.x.max / tdelta[0])
    };

    const int ty[2] = {
        (int)ceil(rview.y.min / tdelta[1]),
        (int)ceil(rview.y.max / tdelta[1])
    };

    for( int i=tx[0]; i<tx[1]; ++i ) {
        glDrawLine((i)*tdelta[0], rview.y.min,   (i)*tdelta[0], rview.y.max);
    }

    for( int i=ty[0]; i<ty[1]; ++i ) {
        glDrawLine(rview.x.min, (i)*tdelta[1],  rview.x.max, (i)*tdelta[1]);
    }
    prog_lines.Unbind();

    //////////////////////////////////////////////////////////////////////////
    // Draw axis

    prog_lines.SaveBind();
    prog_lines.SetUniform("u_color",  colour_ax );
    glDrawLine(0, rview.y.min,  0, rview.y.max );
    glDrawLine(rview.x.min,0,   rview.x.max,0  );
    prog_lines.Unbind();

    //////////////////////////////////////////////////////////////////////////
    // Draw Implicits

    for(size_t i=0; i < plotimplicits.size(); ++i) {
        PlotImplicit& im = plotimplicits[i];
        im.prog.SaveBind();

        im.prog.SetUniform("u_scale",  sx, sy);
        im.prog.SetUniform("u_offset", ox, oy);

        glDrawRect(rview.x.min,rview.y.min,rview.x.max,rview.y.max);

        im.prog.Unbind();
    }

    //////////////////////////////////////////////////////////////////////////
    // Draw series

    static size_t id_size = 0;
    static float* id_array = 0;

    for(size_t i=0; i < plotseries.size(); ++i)
    {
        PlotSeries& ps = plotseries[i];
        GlSlProgram& prog = ps.prog;
        ps.used = false;

        prog.SaveBind();
        prog.SetUniform("u_scale",  sx, sy);
        prog.SetUniform("u_offset", ox, oy);
        prog.SetUniform("u_color", ps.colour );

        const DataLogBlock* block = log->FirstBlock();
        while(block) {
            if(ps.contains_id ) {
                if(id_size < block->Samples() ) {
                    // Create index array that we can bind
                    delete[] id_array;
                    id_size = block->MaxSamples();
                    id_array = new float[id_size];
                    for(size_t k=0; k < id_size; ++k) {
                        id_array[k] = (float)k;
                    }
                }
                prog.SetUniform("u_id_offset",  (float)block->StartId() );
            }

            // Enable appropriate attributes
            bool shouldRender = true;
            for(size_t i=0; i< ps.attribs.size(); ++i) {
                if(0 <= ps.attribs[i].plot_id && ps.attribs[i].plot_id < (int)block->Dimensions() ) {
                    glVertexAttribPointer(ps.attribs[i].location, 1, GL_FLOAT, GL_FALSE, (GLsizei)(block->Dimensions()*sizeof(float)), block->DimData(ps.attribs[i].plot_id) );
                    glEnableVertexAttribArray(ps.attribs[i].location);
                }else if( ps.attribs[i].plot_id == -1 ){
                    glVertexAttribPointer(ps.attribs[i].location, 1, GL_FLOAT, GL_FALSE, 0, id_array );
                    glEnableVertexAttribArray(ps.attribs[i].location);
                }else{
                    // bad id: don't render
                    shouldRender = false;
                    break;
                }
            }

            if(shouldRender) {
                // Draw geometry
                glDrawArrays(ps.drawing_mode, 0, (GLsizei)block->Samples());
                ps.used = true;
            }

            // Disable enabled attributes
            for(size_t i=0; i< ps.attribs.size(); ++i) {
                glDisableVertexAttribArray(ps.attribs[i].location);
            }

            block = block->NextBlock();
        }
        prog.Unbind();
    }

    prog_lines.SaveBind();

    //////////////////////////////////////////////////////////////////////////
    // Draw markers
    glLineWidth(2.5f);

    for( size_t i=0; i < plotmarkers.size(); ++i) {
        const Marker& m = plotmarkers[i];
        prog_lines.SetUniform("u_color",  m.colour );
        if(m.direction == Marker::Horizontal) {
            if(m.leg == 0) {
                glDrawLine(rview.x.min, m.value,  rview.x.max, m.value );
            }else if(m.leg == -1) {
                glDrawRect(rview.x.min, rview.y.min,  rview.x.max, m.value);
            }else if(m.leg == 1) {
                glDrawRect(rview.x.min, m.value,  rview.x.max, rview.y.max);
            }
        }else{
            if(m.leg == 0) {
                glDrawLine(m.value, rview.y.min,  m.value, rview.y.max );
            }else if(m.leg == -1) {
                glDrawRect(rview.x.min, rview.y.min,  m.value, rview.y.max );
            }else if(m.leg == 1) {
                glDrawRect(m.value, rview.y.min,  rview.x.max, rview.y.max );
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////
    // Draw hover / selection
    glLineWidth(1.5f);

    // hover over
    prog_lines.SetUniform("u_color",  colour_ax.WithAlpha(0.3f) );
    glDrawLine(hover[0], rview.y.min,  hover[0], rview.y.max );
    glDrawLine(rview.x.min, hover[1],  rview.x.max, hover[1] );

    // range
    prog_lines.SetUniform("u_color",  colour_ax.WithAlpha(0.5) );
    glDrawLine(selection.x.min, rview.y.min,  selection.x.min, rview.y.max );
    glDrawLine(selection.x.max, rview.y.min,  selection.x.max, rview.y.max );
    glDrawLine(rview.x.min, selection.y.min,  rview.x.max, selection.y.min );
    glDrawLine(rview.x.min, selection.y.max,  rview.x.max, selection.y.max );
    glDrawRect(selection.x.min, selection.y.min,  selection.x.max, selection.y.max);

    prog_lines.Unbind();

    prog_text.SaveBind();

    //////////////////////////////////////////////////////////////////////////
    // Draw Key

    prog_text.SetUniform("u_scale",  2.0f / v.w, 2.0f / v.h);

    int keyid = 0;
    for(size_t i=0; i < plotseries.size(); ++i)
    {
        PlotSeries& ps = plotseries[i];
        if(ps.used) {
            prog_text.SetUniform("u_color", ps.colour );
            prog_text.SetUniform("u_offset",
                v.w-5-ps.title.Width() -(v.w/2.0f),
                v.h-15*(++keyid) -(v.h/2.0f)
            );
            ps.title.DrawGlSl();
        }
    }

    //////////////////////////////////////////////////////////////////////////
    // Draw axis text

    prog_text.SetUniform("u_scale",  2.0f / v.w, 2.0f / v.h);
    prog_text.SetUniform("u_color", colour_ax );

    for( int i=tx[0]; i<tx[1]; ++i ) {
        std::ostringstream oss;
        oss << i*tdelta[0]*tick[0].factor << tick[0].symbol;
        GlText txt = GlFont::I().Text(oss.str().c_str());
        float sx = v.w*((i)*tdelta[0]-rview.x.Mid())/w - txt.Width()/2.0f;
        prog_text.SetUniform("u_offset", sx, 15 -v.h/2.0f );
        txt.DrawGlSl();
    }

    for( int i=ty[0]; i<ty[1]; ++i ) {
        std::ostringstream oss;
        oss << i*tdelta[1]*tick[1].factor << tick[1].symbol;
        GlText txt = GlFont::I().Text(oss.str().c_str());
        float sy = v.h*((i)*tdelta[1]-rview.y.Mid())/h - txt.Height()/2.0f;
        prog_text.SetUniform("u_offset", 15 -v.w/2.0f, sy );
        txt.DrawGlSl();
    }

    prog_text.Unbind();


    glLineWidth(1.0f);

#ifndef HAVE_GLES
    glPopAttrib();
#endif

}

Plotter::Tick Plotter::FindTickFactor(float tick)
{
    Plotter::Tick ret;
    ret.val = tick;

    const float eps = 1E-6f;

    if( std::abs(tick/M_PI - floor(tick/M_PI)) < eps ) {
        ret.factor = 1.0f / (float)M_PI;
        ret.symbol = "pi";
    }else if( std::abs(tick/M_PI_2 - floor(tick/M_PI_2)) < eps ) {
        ret.factor = 1.0f / (float)M_PI;
        ret.symbol = "pi";
    }else if( std::abs(tick/M_PI_4 - floor(tick/M_PI_4)) < eps ) {
        ret.factor = 1.0f / (float)M_PI;
        ret.symbol = "pi";
    }else if( std::abs(tick/M_SQRT2 - floor(tick/M_SQRT2)) < eps ) {
        ret.factor = 1.0f / (float)M_SQRT2;
        ret.symbol = "\251 2";
    }else if( std::abs(tick/M_E - floor(tick/M_E)) < eps ) {
        ret.factor = 1.0f / (float)M_E;
        ret.symbol = "e";
    }else{
        ret.factor = 1.0f;
        ret.symbol = "";
    }
    return ret;
}

void Plotter::SetTicks(float tickx, float ticky)
{
    // Compute tick_factor, tick_label
    tick[0] = FindTickFactor(tickx);
    tick[1] = FindTickFactor(ticky);
}

void Plotter::Track(const std::string& x, const std::string& y)
{
    Plotter& p = linked_plotter_x ? *linked_plotter_x :
                    (linked_plotter_y ? *linked_plotter_y : *this);

    if( x != "$i" || y != "") {
        throw std::runtime_error("Track option not fully implemented");
    }

    p.track_x = x;
    p.track_y = y;
    p.track = !p.track_x.empty() || !p.track_y.empty();
    p.ComputeTrackValue(p.last_track_val);
}

void Plotter::ToggleTracking()
{
    Plotter& p = linked_plotter_x ? *linked_plotter_x :
                    (linked_plotter_y ? *linked_plotter_y : *this);

    p.track = !p.track;
    p.ComputeTrackValue(p.last_track_val);
}

void Plotter::Trigger(const std::string& x, int edge, float value)
{
    if( x != "$0") {
        throw std::runtime_error("Trigger option not fully implemented");
    }

    trigger = x;
    trigger_edge = edge;
    trigger_value = value;
    ComputeTrackValue(last_track_val);
}

void Plotter::ToggleTrigger()
{
    trigger_edge = trigger_edge ? 0 : -1;
    ComputeTrackValue(last_track_val);
}

void Plotter::SetBackgroundColour(const Colour& col)
{
    colour_bg = col;
}

void Plotter::SetAxisColour(const Colour& col)
{
    colour_ax = col;
}

void Plotter::SetTickColour(const Colour& col)
{
    colour_tk = col;
}

XYRangef& Plotter::GetView()
{
    return target;
}

XYRangef& Plotter::GetDefaultView()
{
    return rview_default;
}

XYRangef& Plotter::GetSelection()
{
    return selection;
}

void Plotter::FixSelection()
{
    // Make sure selection matches sign of current viewport
    if( (selection.x.min<selection.x.max) != (rview.x.min<rview.x.max) ) {
        std::swap(selection.x.min, selection.x.max);
    }
    if( (selection.y.min<selection.y.max) != (rview.y.min<rview.y.max) ) {
        std::swap(selection.y.min, selection.y.max);
    }
}

void Plotter::UpdateView()
{
    // Track value based on last log sample
    if( (track || trigger_edge) && !(linked_plotter_x || linked_plotter_y) ) {
        float newTrackVal[2];
        ComputeTrackValue(newTrackVal);
        if(target.x.max <= newTrackVal[0] ) {
            ScrollView(newTrackVal[0]-last_track_val[0], newTrackVal[1]-last_track_val[1] );
        }
        last_track_val[0] = newTrackVal[0];
        last_track_val[1] = newTrackVal[1];
    }

    const float sf = 1.0f / 20.0f;
//    XYRange d = target - rview;
//    rview += d * sf;

    if( linked_plotter_x ) {
        // Synchronise rview and target with linked plotter
        rview.x = linked_plotter_x->rview.x;
        target.x = linked_plotter_x->target.x;
    }else{
        // Animate view window toward target
        Rangef d = target.x - rview.x;
        rview.x += d * sf;
    }

    if( linked_plotter_y ) {
        // Synchronise rview and target with linked plotter
        rview.y = linked_plotter_y->rview.y;
        target.y = linked_plotter_y->target.y;
    }else{
        // Animate view window toward target
        Rangef d = target.y - rview.y;
        rview.y += d * sf;
    }
}

void Plotter::SetView(const XYRangef& range)
{
    Plotter& px = linked_plotter_x ? *linked_plotter_x : *this;
    Plotter& py = linked_plotter_y ? *linked_plotter_y : *this;

    px.rview.x = range.x;
    py.rview.y = range.y;
}

void Plotter::SetViewSmooth(const XYRangef &range)
{
    Plotter& px = linked_plotter_x ? *linked_plotter_x : *this;
    Plotter& py = linked_plotter_y ? *linked_plotter_y : *this;

    px.target.x = range.x;
    py.target.y = range.y;
}

void Plotter::SetDefaultView(const XYRangef &range)
{
    Plotter& px = linked_plotter_x ? *linked_plotter_x : *this;
    Plotter& py = linked_plotter_y ? *linked_plotter_y : *this;

    px.rview_default.x = range.x;
    py.rview_default.y = range.y;
}

void Plotter::ScrollView(float x, float y)
{
    Plotter& px = linked_plotter_x ? *linked_plotter_x : *this;
    Plotter& py = linked_plotter_y ? *linked_plotter_y : *this;

    px.target.x += x;
    py.target.y += y;
    px.rview.x += x;
    py.rview.y += y;
}

void Plotter::ScrollViewSmooth(float x, float y)
{
    Plotter& px = linked_plotter_x ? *linked_plotter_x : *this;
    Plotter& py = linked_plotter_y ? *linked_plotter_y : *this;

    px.target.x += x;
    py.target.y += y;
}

void Plotter::ScaleView(float x, float y, float cx, float cy)
{
    Plotter& px = linked_plotter_x ? *linked_plotter_x : *this;
    Plotter& py = linked_plotter_y ? *linked_plotter_y : *this;

    px.target.x.Scale(x, cx);
    py.target.y.Scale(y, cy);
    px.rview.x.Scale(x, cx);
    py.rview.y.Scale(y, cy);
}

void Plotter::ScaleViewSmooth(float x, float y, float cx, float cy)
{
    Plotter& px = linked_plotter_x ? *linked_plotter_x : *this;
    Plotter& py = linked_plotter_y ? *linked_plotter_y : *this;

    px.target.x.Scale(x,cx);
    py.target.y.Scale(y,cy);
}

void Plotter::ResetView()
{
    Plotter& px = linked_plotter_x ? *linked_plotter_x : *this;
    Plotter& py = linked_plotter_y ? *linked_plotter_y : *this;

    px.target.x = px.rview_default.x;
    py.target.y = py.rview_default.y;
}

void Plotter::Keyboard(View&, unsigned char key, int x, int y, bool pressed)
{
    const float mvfactor = 1.0f / 10.0f;

    const float c[2] = {
        track || trigger_edge ? target.x.max : rview.x.Mid(),
        rview.y.Mid()
    };

    if(pressed) {
        if(key == ' ') {
            if( selection.Area() <= 0.0f) {
                // Work out Auto zoom selection bounds
                selection = ComputeAutoSelection();
            }

            if( selection.Area() > 0.0f) {
                // Set view to equal selection
                SetViewSmooth(selection);

                // Reset selection
                selection.x.max = selection.x.min;
                selection.y.max = selection.y.min;
            }
        }else if(key == PANGO_SPECIAL + PANGO_KEY_LEFT) {
            const float w = rview.x.Size();
            const float dx = mvfactor*w;
            ScrollViewSmooth(-dx, 0);
        }else if(key == PANGO_SPECIAL + PANGO_KEY_RIGHT) {
            const float w = rview.x.Size();
            const float dx = mvfactor*w;
            ScrollViewSmooth(+dx, 0);
        }else if(key == PANGO_SPECIAL + PANGO_KEY_DOWN) {
            const float h = target.y.Size();
            const float dy = mvfactor*h;
            ScrollViewSmooth(0, -dy);
        }else if(key == PANGO_SPECIAL + PANGO_KEY_UP) {
            const float h = target.y.Size();
            const float dy = mvfactor*h;
            ScrollViewSmooth(0, +dy);
        }else if(key == '=') {
            ScaleViewSmooth(0.5, 0.5, c[0], c[1]);
        }else if(key == '-') {
            ScaleViewSmooth(2.0, 2.0, c[0], c[1]);
        }else if(key == 'r') {
            ResetView();
        }else if(key == 't') {
            ToggleTracking();
        }else if(key == 'e') {
            ToggleTrigger();
        }
    }
}

void Plotter::ScreenToPlot(int xpix, int ypix, float& xplot, float& yplot)
{
    xplot = rview.x.min + rview.x.Size() * (xpix - v.l) / (float)v.w;
    yplot = rview.y.min + rview.y.Size() * (ypix - v.b) / (float)v.h;
}

void Plotter::Mouse(View& /*view*/, MouseButton button, int x, int y, bool pressed, int button_state)
{
    // Update hover status (after potential resizing)
    ScreenToPlot(x, y, hover[0], hover[1]);

    const float scinc = 1.05f;
    const float scdec = 1.0f/scinc;

    const float c[2] = {
        track || trigger_edge ? last_track_val[0] : hover[0],
        hover[1]
    };

    if(button_state & KeyModifierShift) {
        if(button == MouseWheelUp) {
            ScaleViewSmooth(1.0f, scinc, c[0], c[1]);
        }else if(button == MouseWheelDown) {
            ScaleViewSmooth(1.0f, scdec, c[0], c[1]);
        }else if(button == MouseWheelLeft) {
            ScaleViewSmooth(scinc, 1.0f, c[0], c[1]);
        }else if(button == MouseWheelRight) {
            ScaleViewSmooth(scdec, 1.0f, c[0], c[1]);
        }
    }else if(button_state & KeyModifierCtrl) {
        if(button == MouseWheelUp) {
            ScaleViewSmooth(scinc, 1.0f, c[0], c[1]);
        }else if(button == MouseWheelDown) {
            ScaleViewSmooth(scdec, 1.0f, c[0], c[1]);
        }
    }else{
        const float mvfactor = 1.0f/20.0f;

        if(button == MouseButtonLeft) {
            // Update selected range
            if(pressed) {
                selection.x.min = hover[0];
                selection.y.min = hover[1];
                trigger_value = selection.y.min;
            }
            selection.x.max = hover[0];
            selection.y.max = hover[1];
        }else if(button == MouseWheelUp) {
            ScrollViewSmooth(0.0f, +mvfactor*rview.y.Size() );
        }else if(button == MouseWheelDown) {
            ScrollViewSmooth(0.0f, -mvfactor*rview.y.Size() );
        }else if(button == MouseWheelLeft) {
            ScrollViewSmooth(+mvfactor*rview.x.Size(), 0.0f );
        }else if(button == MouseWheelRight) {
            ScrollViewSmooth(-mvfactor*rview.x.Size(), 0.0f );
        }
    }

    FixSelection();

    last_mouse_pos[0] = x;
    last_mouse_pos[1] = y;
}

void Plotter::MouseMotion(View& view, int x, int y, int button_state)
{
    const int d[2] = {x-last_mouse_pos[0], y-last_mouse_pos[1]};
    const float is[2] = {rview.x.Size(), rview.y.Size() };
    const float df[2] = {is[0]*d[0]/(float)v.w, is[1]*d[1]/(float)v.h};

    // Update hover status (after potential resizing)
    ScreenToPlot(x, y, hover[0], hover[1]);

    if( button_state == MouseButtonLeft )
    {
        // Update selected range
        selection.x.max = hover[0];
        selection.y.max = hover[1];
    }else if(button_state == MouseButtonMiddle )
    {
        Special(view, InputSpecialScroll, df[0], df[1], 0.0f, 0.0f, 0.0f, 0.0f, button_state);
    }else if(button_state == MouseButtonRight )
    {
        const float c[2] = {
            track || trigger_edge ? last_track_val[0] : hover[0],
            hover[1]
        };

        const float scale[2] = {
            1.0f + (float)d[0] / (float)v.w,
            1.0f - (float)d[1] / (float)v.h,
        };
        ScaleView(scale[0], scale[1], c[0], c[1]);
    }

    last_mouse_pos[0] = x;
    last_mouse_pos[1] = y;
}

void Plotter::PassiveMouseMotion(View&, int x, int y, int button_state)
{
    ScreenToPlot(x, y, hover[0], hover[1]);
}

void Plotter::Special(View&, InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state)
{
    if(inType == InputSpecialScroll) {
        const float d[2] = {p1,-p2};
        const float is[2] = {rview.x.Size(),rview.y.Size() };
        const float df[2] = {is[0]*d[0]/(float)v.w, is[1]*d[1]/(float)v.h};

        ScrollView(-df[0], -df[1]);
    } else if(inType == InputSpecialZoom) {
        float scalex = 1.0;
        float scaley = 1.0;

#ifdef _OSX_
        if(button_state & KeyModifierCmd) {
#else
        if (button_state & KeyModifierCtrl) {
#endif
            scalex = 1-p1;
        }else{
            scaley = 1-p1;
        }

        const float c[2] = {
            track || trigger_edge ? last_track_val[0] : hover[0],
            hover[1]
        };

        ScaleView(scalex, scaley, c[0], c[1]);
    }

    // Update hover status (after potential resizing)
    ScreenToPlot( (int)x, (int)y, hover[0], hover[1]);
}

Marker& Plotter::AddMarker(Marker::Direction d, float value, Marker::Equality leg, Colour c )
{
    plotmarkers.push_back( Marker(d,value,leg,c) );
    return plotmarkers.back();
}

void Plotter::ClearMarkers()
{
    plotmarkers.clear();
}


}
