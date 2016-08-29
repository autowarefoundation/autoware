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

#ifndef PANGOLIN_GLVBO_H
#define PANGOLIN_GLVBO_H

#include <pangolin/gl/gl.h>

namespace pangolin
{

////////////////////////////////////////////////
// Interface
////////////////////////////////////////////////

void MakeTriangleStripIboForVbo(GlBuffer& ibo, int w, int h);

#ifdef CALLEE_HAS_RVALREF
GlBuffer MakeTriangleStripIboForVbo(int w, int h);
#endif

void RenderVbo(GlBuffer& vbo);

void RenderVboCbo(GlBuffer& vbo, GlBuffer& cbo, bool draw_color = true);

void RenderVboIbo(GlBuffer& vbo, GlBuffer& ibo, bool draw_mesh = true);

void RenderVboIboCbo(GlBuffer& vbo, GlBuffer& ibo, GlBuffer& cbo, bool draw_mesh = true, bool draw_color = true);

void RenderVboIboNbo(GlBuffer& vbo, GlBuffer& ibo, GlBuffer& nbo, bool draw_mesh = true, bool draw_normals = true);

void RenderVboIboCboNbo(GlBuffer& vbo, GlBuffer& ibo, GlBuffer& cbo, GlBuffer& nbo, bool draw_mesh = true, bool draw_color = true, bool draw_normals = true);

////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////

inline void MakeTriangleStripIboForVbo(GlBuffer& ibo, int w, int h)
{
    const int num_elements = w*(h-1)*2;
    unsigned int* buffer = new unsigned int[num_elements];
    unsigned int* ptr = buffer;
    
    for(int y=0; y < (h-1);)
    {
        for(int x=0; x<w; ++x) {
            (*ptr++) = y*w+x;
            (*ptr++) = (y+1)*w+x;
        }
        ++y;
        
        if(y>=(h-1)) break;        
        for(int x=w-1; x>=0; --x) {
            (*ptr++) = y*w+x;
            (*ptr++) = (y+1)*w+x;
        }
        ++y;
    }
    
    ibo.Reinitialise(GlElementArrayBuffer, num_elements, GL_UNSIGNED_INT, 1, GL_STATIC_DRAW );
    ibo.Upload(buffer, sizeof(unsigned int)*num_elements );
    
    delete[] buffer;
}

#ifdef CALLEE_HAS_RVALREF
inline GlBuffer MakeTriangleStripIboForVbo(int w, int h)
{
    GlBuffer ibo;
    MakeTriangleStripIboForVbo(ibo,w,h);
    return ibo;
}
#endif

inline void RenderVbo(GlBuffer& vbo)
{
    vbo.Bind();
    glVertexPointer(vbo.count_per_element, vbo.datatype, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    
    glPointSize(2.0);
    glDrawArrays(GL_POINTS, 0, vbo.num_elements);
    
    glDisableClientState(GL_VERTEX_ARRAY);
    vbo.Unbind();
    
}

inline void RenderVboCbo(GlBuffer& vbo, GlBuffer& cbo, bool draw_color)
{
    if(draw_color) {
        cbo.Bind();
        glColorPointer(cbo.count_per_element, cbo.datatype, 0, 0);
        glEnableClientState(GL_COLOR_ARRAY);
    }
    
    RenderVbo(vbo);
    
    if(draw_color) {
        glDisableClientState(GL_COLOR_ARRAY);
        cbo.Unbind();
    }
}

inline void RenderVboIbo(GlBuffer& vbo, GlBuffer& ibo, bool draw_mesh)
{
    vbo.Bind();
    glVertexPointer(vbo.count_per_element, vbo.datatype, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    
    if(draw_mesh) {
        ibo.Bind();
        glDrawElements(GL_TRIANGLE_STRIP,ibo.num_elements, ibo.datatype, 0);
        ibo.Unbind();
    }else{
        glPointSize(2.0);
        glDrawArrays(GL_POINTS, 0, vbo.num_elements);
    }
    
    glDisableClientState(GL_VERTEX_ARRAY);
    vbo.Unbind();
}

inline void RenderVboIboCbo(GlBuffer& vbo, GlBuffer& ibo, GlBuffer& cbo, bool draw_mesh, bool draw_color )
{
    if(draw_color) {
        cbo.Bind();
        glColorPointer(cbo.count_per_element, cbo.datatype, 0, 0);
        glEnableClientState(GL_COLOR_ARRAY);
    }
    
    RenderVboIbo(vbo,ibo,draw_mesh);
    
    if(draw_color) {
        glDisableClientState(GL_COLOR_ARRAY);
        cbo.Unbind();
    }
}

inline void RenderVboIboCboNbo(GlBuffer& vbo, GlBuffer& ibo, GlBuffer& cbo, GlBuffer& nbo, bool draw_mesh, bool draw_color, bool draw_normals)
{
    if(draw_color) {
        cbo.Bind();
        glColorPointer(cbo.count_per_element, cbo.datatype, 0, 0);
        glEnableClientState(GL_COLOR_ARRAY);
    }
    
    if(draw_normals) {
        nbo.Bind();
        glNormalPointer(nbo.datatype, (GLsizei)(nbo.count_per_element * GlDataTypeBytes(nbo.datatype)),0);
        glEnableClientState(GL_NORMAL_ARRAY);
    }

    vbo.Bind();
    glVertexPointer(vbo.count_per_element, vbo.datatype, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    
    if(draw_mesh) {
        ibo.Bind();
        glDrawElements(GL_TRIANGLE_STRIP,ibo.num_elements, ibo.datatype, 0);
        ibo.Unbind();
    }else{
        glPointSize(2.0);
        glDrawArrays(GL_POINTS, 0, vbo.num_elements);
    }
    
    if(draw_color) {
        glDisableClientState(GL_COLOR_ARRAY);
        cbo.Unbind();
    }
    
    if(draw_normals) {
        glDisableClientState(GL_NORMAL_ARRAY);
        nbo.Unbind();
    }

    glDisableClientState(GL_VERTEX_ARRAY);
    vbo.Unbind();
}

inline void RenderVboIboNbo(GlBuffer& vbo, GlBuffer& ibo, GlBuffer& nbo, bool draw_mesh, bool draw_normals)
{
    vbo.Bind();
    glVertexPointer(vbo.count_per_element, vbo.datatype, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);

    if(draw_normals) {
        nbo.Bind();
        glNormalPointer(nbo.datatype, (GLsizei)(nbo.count_per_element * GlDataTypeBytes(nbo.datatype)), 0);
        glEnableClientState(GL_NORMAL_ARRAY);
    }

    if(draw_mesh) {
        ibo.Bind();
        glDrawElements(GL_TRIANGLE_STRIP,ibo.num_elements, ibo.datatype, 0);
        ibo.Unbind();
    }else{
        glPointSize(2.0);
        glDrawArrays(GL_POINTS, 0, vbo.num_elements);
    }

    if(draw_normals) {
        glDisableClientState(GL_NORMAL_ARRAY);
        nbo.Unbind();
    }

    glDisableClientState(GL_VERTEX_ARRAY);
    vbo.Unbind();
}

}

#endif // PANGOLIN_GLVBO_H
