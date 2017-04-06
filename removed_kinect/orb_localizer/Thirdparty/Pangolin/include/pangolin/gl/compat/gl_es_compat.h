#pragma once

#include <pangolin/platform.h>

#ifndef PANGOLIN_GL_ES_COMPAT_H
#define PANGOLIN_GL_ES_COMPAT_H

#define GLdouble     GLfloat
#define glClearDepth glClearDepthf
#define glFrustum    glFrustumf

#define glColor4fv(a)       glColor4f(a[0], a[1], a[2], a[3])
#define glColor3fv(a)       glColor4f(a[0], a[1], a[2], 1.0f)
#define glColor3f(a,b,c)    glColor4f(a, b, c, 1.0f)

#define GL_CLAMP                    GL_CLAMP_TO_EDGE

#ifdef HAVE_GLES_2
    #define glGenFramebuffersEXT        glGenFramebuffers
    #define glDeleteFramebuffersEXT     glDeleteFramebuffers
    #define glBindFramebufferEXT        glBindFramebuffer
    #define glDrawBuffers               glDrawBuffers
    #define glFramebufferTexture2DEXT   glFramebufferTexture2D
    #define GL_FRAMEBUFFER_EXT          GL_FRAMEBUFFER
    #define GL_DEPTH_COMPONENT24        GL_DEPTH_COMPONENT16 // <----
    #define GL_COLOR_ATTACHMENT0_EXT    GL_COLOR_ATTACHMENT0
    #define GL_DEPTH_ATTACHMENT_EXT     GL_DEPTH_ATTACHMENT
#else
    #define glOrtho                     glOrthof
    #define glGenFramebuffersEXT        glGenFramebuffersOES
    #define glDeleteFramebuffersEXT     glDeleteFramebuffersOES
    #define glBindFramebufferEXT        glBindFramebufferOES
    #define glDrawBuffers               glDrawBuffersOES
    #define glFramebufferTexture2DEXT   glFramebufferTexture2DOES
    #define GL_FRAMEBUFFER_EXT          GL_FRAMEBUFFER_OES
    #define GL_DEPTH_COMPONENT24        GL_DEPTH_COMPONENT24_OES
    #define GL_COLOR_ATTACHMENT0_EXT    GL_COLOR_ATTACHMENT0_OES
#endif

#define glGetDoublev                glGetFloatv

#ifdef HAVE_GLES_2
#include <pangolin/gl2engine.h>
#endif

inline void glRectf(GLfloat x1, GLfloat y1, GLfloat x2, GLfloat y2)
{
    GLfloat verts[] = { x1,y1,  x2,y1,  x2,y2,  x1,y2 };    
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 0, verts);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glDisableClientState(GL_VERTEX_ARRAY);
}

inline void glRecti(int x1, int y1, int x2, int y2)
{
    GLfloat verts[] = { (float)x1,(float)y1,  (float)x2,(float)y1,
                        (float)x2,(float)y2,  (float)x1,(float)y2 };    
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 0, verts);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glDisableClientState(GL_VERTEX_ARRAY);
}

#endif // PANGOLIN_GL_ES_COMPAT_H
