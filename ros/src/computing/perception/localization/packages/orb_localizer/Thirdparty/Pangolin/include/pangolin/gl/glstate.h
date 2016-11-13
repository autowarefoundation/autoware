/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2013 Vincent Mamo, Steven Lovegrove
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

#ifndef PANGOLIN_GLSTATE_H
#define PANGOLIN_GLSTATE_H

#include <pangolin/gl/glinclude.h>
#include <stack>

namespace pangolin
{

class GlState {

    class CapabilityState {
    public:

        CapabilityState(GLenum cap, GLboolean enable)
            : m_cap(cap), m_enable(enable)
        {

        }

        void Apply() {
            if(m_enable) {
                ::glEnable(m_cap);
            }else{
                ::glDisable(m_cap);
            }
        }

        void UnApply() {
            if(m_enable) {
                ::glDisable(m_cap);
            }else{
                ::glEnable(m_cap);
            }
        }

    protected:
        GLenum m_cap;
        GLboolean m_enable;
    };

public:
    GlState()
        : m_DepthMaskCalled(false),
          m_ShadeModelCalled(false),
          m_CullFaceCalled(false),
          m_PointSizeCalled(false),
          m_LineWidthCalled(false),
          m_ColorMaskCalled(false),
          m_ViewportCalled(false)
    {
    }

    ~GlState() {
        //  Restore original state
        while (!m_history.empty()) {
            m_history.top().UnApply();
            m_history.pop();
        }

        if (m_DepthMaskCalled) {
            ::glDepthMask(m_OriginalDepthMask);
        }

        if (m_ShadeModelCalled) {
            ::glShadeModel(m_OriginalShadeModel);
        }

        if (m_CullFaceCalled) {
            ::glCullFace(m_OriginalCullFace);
        }
        
        if(m_PointSizeCalled) {
            ::glPointSize(m_OriginalPointSize);
        }
        
        if(m_LineWidthCalled) {
            ::glLineWidth(m_OriginalLineWidth);
        }        
        
        if (m_ColorMaskCalled) {
            ::glColorMask(m_OriginalColorMask[0], m_OriginalColorMask[1], m_OriginalColorMask[2], m_OriginalColorMask[3]);
        }

        if (m_ViewportCalled) {
            ::glViewport(m_OriginalViewport[0], m_OriginalViewport[1], m_OriginalViewport[2], m_OriginalViewport[3]);
        }
    }

    static inline GLboolean IsEnabled(GLenum cap)
    {
        GLboolean curVal;
        glGetBooleanv(cap, &curVal);
        return curVal;
    }

    inline void glEnable(GLenum cap)
    {
        if(!IsEnabled(cap)) {
            m_history.push(CapabilityState(cap,true));
            ::glEnable(cap);
        }
    }

    inline void glDisable(GLenum cap)
    {
        if(IsEnabled(cap)) {
            m_history.push(CapabilityState(cap,false));
           ::glDisable(cap);
        }
    }

    bool m_DepthMaskCalled;
    GLboolean m_OriginalDepthMask;
    inline void glDepthMask(GLboolean flag)
    {
        if(!m_DepthMaskCalled) {
            m_DepthMaskCalled = true;
            glGetBooleanv(GL_DEPTH_WRITEMASK, &m_OriginalDepthMask);
        }
        ::glDepthMask(flag);
    }

    bool m_ShadeModelCalled;
    GLint m_OriginalShadeModel;
    inline void glShadeModel(GLint mode)
    {
        if(!m_ShadeModelCalled) {
            m_ShadeModelCalled = true;
            glGetIntegerv(GL_SHADE_MODEL, &m_OriginalShadeModel);
        }
        ::glShadeModel(mode);
    }
    
    bool m_CullFaceCalled;
    GLint m_OriginalCullFace;
    void glCullFace(GLenum mode)
    {
        if(!m_ShadeModelCalled) {
            m_ShadeModelCalled = true;
            glGetIntegerv(GL_CULL_FACE_MODE, &m_OriginalCullFace);
        }
        ::glCullFace(mode);        
    }
    
    bool m_PointSizeCalled;
    GLfloat m_OriginalPointSize;
    void glPointSize(GLfloat size)
    {
        if(!m_PointSizeCalled) {
            m_PointSizeCalled = true;
            glGetFloatv(GL_POINT_SIZE, &m_OriginalPointSize);
        }
        ::glPointSize(size);
    }
    
    bool m_LineWidthCalled;
    GLfloat m_OriginalLineWidth;
    void glLineWidth(GLfloat width)
    {
        if(!m_LineWidthCalled) {
            m_LineWidthCalled = true;
            glGetFloatv(GL_LINE_WIDTH, &m_OriginalLineWidth);
        }
        ::glLineWidth(width);
        
    }

    bool m_ColorMaskCalled;
    GLboolean m_OriginalColorMask[4];
    inline void glColorMask(GLboolean red, GLboolean green, GLboolean blue, GLboolean alpha)
    {
        if(!m_ColorMaskCalled) {
            m_ColorMaskCalled = true;
            glGetBooleanv(GL_COLOR_WRITEMASK,  m_OriginalColorMask);
        }
        ::glColorMask(red, green, blue, alpha);
    }

    bool m_ViewportCalled;
    GLint m_OriginalViewport[4];
    inline void glViewport(GLint x, GLint y, GLsizei width, GLsizei height)
    {
        if(!m_ViewportCalled) {
            m_ViewportCalled = true;
            glGetIntegerv(GL_VIEWPORT, m_OriginalViewport);
        }
        ::glViewport(x, y, width, height);
    }

    std::stack<CapabilityState> m_history;
};

}

#endif // PANGOLIN_GLSTATE_H
