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

#include <pangolin/gl/glpangoglu.h>
#include <pangolin/utils/simple_math.h>

namespace pangolin {

const GLubyte gNotErrorLookup[] = "XX";

const GLubyte* glErrorString(GLenum /*error*/)
{
    // TODO: Implement glErrorString
    return gNotErrorLookup;
}

// Based on glu implementation.
template<typename P>
int InvertMatrix(const P m[16], P invOut[16])
{
    P inv[16], det;
    int i;

    inv[0] =   m[5]*m[10]*m[15] - m[5]*m[11]*m[14] - m[9]*m[6]*m[15]
             + m[9]*m[7]*m[14] + m[13]*m[6]*m[11] - m[13]*m[7]*m[10];
    inv[4] =  -m[4]*m[10]*m[15] + m[4]*m[11]*m[14] + m[8]*m[6]*m[15]
             - m[8]*m[7]*m[14] - m[12]*m[6]*m[11] + m[12]*m[7]*m[10];
    inv[8] =   m[4]*m[9]*m[15] - m[4]*m[11]*m[13] - m[8]*m[5]*m[15]
             + m[8]*m[7]*m[13] + m[12]*m[5]*m[11] - m[12]*m[7]*m[9];
    inv[12] = -m[4]*m[9]*m[14] + m[4]*m[10]*m[13] + m[8]*m[5]*m[14]
             - m[8]*m[6]*m[13] - m[12]*m[5]*m[10] + m[12]*m[6]*m[9];
    inv[1] =  -m[1]*m[10]*m[15] + m[1]*m[11]*m[14] + m[9]*m[2]*m[15]
             - m[9]*m[3]*m[14] - m[13]*m[2]*m[11] + m[13]*m[3]*m[10];
    inv[5] =   m[0]*m[10]*m[15] - m[0]*m[11]*m[14] - m[8]*m[2]*m[15]
             + m[8]*m[3]*m[14] + m[12]*m[2]*m[11] - m[12]*m[3]*m[10];
    inv[9] =  -m[0]*m[9]*m[15] + m[0]*m[11]*m[13] + m[8]*m[1]*m[15]
             - m[8]*m[3]*m[13] - m[12]*m[1]*m[11] + m[12]*m[3]*m[9];
    inv[13] =  m[0]*m[9]*m[14] - m[0]*m[10]*m[13] - m[8]*m[1]*m[14]
             + m[8]*m[2]*m[13] + m[12]*m[1]*m[10] - m[12]*m[2]*m[9];
    inv[2] =   m[1]*m[6]*m[15] - m[1]*m[7]*m[14] - m[5]*m[2]*m[15]
             + m[5]*m[3]*m[14] + m[13]*m[2]*m[7] - m[13]*m[3]*m[6];
    inv[6] =  -m[0]*m[6]*m[15] + m[0]*m[7]*m[14] + m[4]*m[2]*m[15]
             - m[4]*m[3]*m[14] - m[12]*m[2]*m[7] + m[12]*m[3]*m[6];
    inv[10] =  m[0]*m[5]*m[15] - m[0]*m[7]*m[13] - m[4]*m[1]*m[15]
             + m[4]*m[3]*m[13] + m[12]*m[1]*m[7] - m[12]*m[3]*m[5];
    inv[14] = -m[0]*m[5]*m[14] + m[0]*m[6]*m[13] + m[4]*m[1]*m[14]
             - m[4]*m[2]*m[13] - m[12]*m[1]*m[6] + m[12]*m[2]*m[5];
    inv[3] =  -m[1]*m[6]*m[11] + m[1]*m[7]*m[10] + m[5]*m[2]*m[11]
             - m[5]*m[3]*m[10] - m[9]*m[2]*m[7] + m[9]*m[3]*m[6];
    inv[7] =   m[0]*m[6]*m[11] - m[0]*m[7]*m[10] - m[4]*m[2]*m[11]
             + m[4]*m[3]*m[10] + m[8]*m[2]*m[7] - m[8]*m[3]*m[6];
    inv[11] = -m[0]*m[5]*m[11] + m[0]*m[7]*m[9] + m[4]*m[1]*m[11]
             - m[4]*m[3]*m[9] - m[8]*m[1]*m[7] + m[8]*m[3]*m[5];
    inv[15] =  m[0]*m[5]*m[10] - m[0]*m[6]*m[9] - m[4]*m[1]*m[10]
             + m[4]*m[2]*m[9] + m[8]*m[1]*m[6] - m[8]*m[2]*m[5];

    det = m[0]*inv[0] + m[1]*inv[4] + m[2]*inv[8] + m[3]*inv[12];
    if (det == 0)
        return GL_FALSE;

    det=1.0f/det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;

    return GL_TRUE;
}

// Based on glu implementation
GLint glProject(
    float objx, float objy, float objz,
    const float modelMatrix[16],
    const float projMatrix[16],
    const GLint viewport[4],
    float* winx, float* winy, float* winz)
{
    float t1[4] = {objx, objy, objz, 1.0f};
    float t2[4];

    MatMul<4,4,1,float>(t2, modelMatrix, t1);
    MatMul<4,4,1,float>(t1, projMatrix, t2);

    if (t1[3] == 0.0) {
        return(GL_FALSE);
    }

    // Normalise
    t1[0]/=t1[3];
    t1[1]/=t1[3];
    t1[2]/=t1[3];

    // Map x, y and z to range 0-1
    t1[0]=t1[0]*0.5f+0.5f;
    t1[1]=t1[1]*0.5f+0.5f;
    t1[2]=t1[2]*0.5f+0.5f;

    // Map x,y to viewport
    t1[0]=t1[0] * viewport[2] + viewport[0];
    t1[1]=t1[1] * viewport[3] + viewport[1];

    *winx=t1[0];
    *winy=t1[1];
    *winz=t1[2];

    return GL_TRUE;
}

// Based on glu implementation
GLint glUnProject(
    float winx, float winy, float winz,
    const float mv[16],
    const float proj[16],
    const GLint viewport[4],
    float* objx, float* objy, float* objz)
{
    float t1[16];

    MatMul<4,4,4,float>(t1, proj, mv);

    if (!InvertMatrix<float>(t1, t1)) {
        return(GL_FALSE);
    }

    // Map x and y from window coordinates
    float in[4] = {winx, winy, winz, 1.0f};
    in[0] = (in[0] - viewport[0]) / viewport[2];
    in[1] = (in[1] - viewport[1]) / viewport[3];

    // Map to range -1 to 1
    in[0] = in[0] * 2 - 1;
    in[1] = in[1] * 2 - 1;
    in[2] = in[2] * 2 - 1;

    float out[4];
    MatMul<4,4,1,float>(out, t1, in);

    if (out[3] == 0.0) {
        return(GL_FALSE);
    }

    // Normalise
    out[0] /= out[3];
    out[1] /= out[3];
    out[2] /= out[3];

    // Copy out
    *objx = out[0];
    *objy = out[1];
    *objz = out[2];

    return GL_TRUE;
}

// Based on glu implementation
GLint glProject(
    double objx, double objy, double objz,
    const double modelMatrix[16],
    const double projMatrix[16],
    const GLint viewport[4],
    double* winx, double* winy, double* winz)
{
    double t1[4] = {objx, objy, objz, 1.0f};
    double t2[4];

    MatMul<4,4,1,double>(t2, modelMatrix, t1);
    MatMul<4,4,1,double>(t1, projMatrix, t2);

    if (t1[3] == 0.0) {
        return(GL_FALSE);
    }

    // Normalise
    t1[0]/=t1[3];
    t1[1]/=t1[3];
    t1[2]/=t1[3];

    // Map x, y and z to range 0-1
    t1[0]=t1[0]*0.5f+0.5f;
    t1[1]=t1[1]*0.5f+0.5f;
    t1[2]=t1[2]*0.5f+0.5f;

    // Map x,y to viewport
    t1[0]=t1[0] * viewport[2] + viewport[0];
    t1[1]=t1[1] * viewport[3] + viewport[1];

    *winx=t1[0];
    *winy=t1[1];
    *winz=t1[2];

    return GL_TRUE;
}

// Based on glu implementation
GLint glUnProject(
    double winx, double winy, double winz,
    const double mv[16],
    const double proj[16],
    const GLint viewport[4],
    double* objx, double* objy, double* objz)
{
    double t1[16];

    MatMul<4,4,4,double>(t1, proj, mv);

    if (!InvertMatrix<double>(t1, t1)) {
        return(GL_FALSE);
    }

    // Map x and y from window coordinates
    double in[4] = {winx, winy, winz, 1.0f};
    in[0] = (in[0] - viewport[0]) / viewport[2];
    in[1] = (in[1] - viewport[1]) / viewport[3];

    // Map to range -1 to 1
    in[0] = in[0] * 2 - 1;
    in[1] = in[1] * 2 - 1;
    in[2] = in[2] * 2 - 1;

    double out[4];
    MatMul<4,4,1,double>(out, t1, in);

    if (out[3] == 0.0) {
        return(GL_FALSE);
    }

    // Normalise
    out[0] /= out[3];
    out[1] /= out[3];
    out[2] /= out[3];

    // Copy out
    *objx = out[0];
    *objy = out[1];
    *objz = out[2];

    return GL_TRUE;
}


}
