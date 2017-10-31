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

#ifndef PANGOLIN_SIMPLE_MATH_H
#define PANGOLIN_SIMPLE_MATH_H

#include <iostream>
#include <string.h>
#include <algorithm>
#include <stdarg.h>
#include <cmath>

namespace pangolin
{

static const double Identity3d[] = {1,0,0, 0,1,0, 0,0,1};
static const double Zero3d[]     = {0,0,0, 0,0,0, 0,0,0};
static const double Identity4d[] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
static const double Zero4d[]     = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};

static const float Identity3f[] = {1,0,0, 0,1,0, 0,0,1};
static const float Zero3f[]     = {0,0,0, 0,0,0, 0,0,0};
static const float Identity4f[] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
static const float Zero4f[]     = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};

template<int R, int C, typename P>
void MatPrint(const P m[R*C])
{
    for( int r=0; r < R; ++r)
    {
        for( int c=0; c < C; ++c )
            std::cout << m[R*c+r] << " ";
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

template<int R, int C, typename P>
void MatPrint(const P m[R*C], std::string name)
{
    std::cout << name << " = [" << std::endl;
    for( int r=0; r < R; ++r)
    {
        for( int c=0; c < C; ++c )
            std::cout << m[R*c+r] << " ";
        std::cout << std::endl;
    }
    std::cout << std::endl << "]" << std::endl;
}

// Set array using varadic arguments
template<int R, int C, typename P>
void MatSet(P m[R*C], ...)
{
    va_list ap;
    va_start(ap,m);
    for( int i=0; i< R*C; ++i )
    {
        *m = (P)va_arg(ap,double);
        ++m;
    }
}

// m = zeroes(N)
template<int R, int C, typename P>
void SetZero(P m[R*C] )
{
    std::fill_n(m,R*C,0);
}

// m = identity(N)
template<int N, typename P>
void SetIdentity(P m[N*N] )
{
    std::fill_n(m,N*N,0);
    for( int i=0; i< N; ++i )
        m[N*i+i] = 1;
}

// mo = m1 * m2
template<int R, int M, int C, typename P>
void MatMul(P mo[R*C], const P m1[R*M], const P m2[M*C] )
{
    for( int r=0; r < R; ++r)
        for( int c=0; c < C; ++c )
        {
            mo[R*c+r] = 0;
            for( int m=0; m < M; ++ m) mo[R*c+r] += m1[R*m+r] * m2[M*c+m];
        }
}

// mo = m1 * m2 * s
template<int R, int M, int C, typename P>
void MatMul(P mo[R*C], const P m1[R*M], const P m2[M*C], P s )
{
    for( int r=0; r < R; ++r)
        for( int c=0; c < C; ++c )
        {
            mo[R*c+r] = 0;
            for( int m=0; m < M; ++ m) mo[R*c+r] += m1[R*m+r] * m2[M*c+m] * s;
        }
}

// mo = m1 * transpose(m2)
template<int R, int M, int C, typename P>
void MatMulTranspose(P mo[R*C], const P m1[R*M], const P m2[C*M] )
{
    for( int r=0; r < R; ++r)
        for( int c=0; c < C; ++c )
        {
            mo[R*c+r] = 0;
            for( int m=0; m < M; ++ m) mo[R*c+r] += m1[R*m+r] * m2[C*m+c];
        }
}

// m = m1 + m2
template<int R, int C, typename P>
void MatAdd(P m[R*C], const P m1[R*C], const P m2[R*C])
{
    for( int i=0; i< R*C; ++i )
        m[i] = m1[i] + m2[i];
}

// m = m1 - m2
template<int R, int C, typename P>
void MatSub(P m[R*C], const P m1[R*C], const P m2[R*C])
{
    for( int i=0; i< R*C; ++i )
        m[i] = m1[i] - m2[i];
}

// m = m1 * scalar
template<int R, int C, typename P>
void MatMul(P m[R*C], const P m1[R*C], P scalar)
{
    for( int i=0; i< R*C; ++i )
        m[i] = m1[i] * scalar;
}

// m = m1 + m2
template<int R, int C, typename P>
void MatMul(P m[R*C], P scalar)
{
    for( int i=0; i< R*C; ++i )
        m[i] *= scalar;
}

template<int N, typename P>
void MatTranspose(P out[N*N], const P in[N*N] )
{
    for( int c=0; c<N; ++c )
        for( int r=0; r<N; ++r )
            out[N*c+r] = in[N*r+c];
}

template<int N, typename P>
void MatTranspose(P m[N*N] )
{
    for( int c=0; c<N; ++c )
        for( int r=0; r<c; ++r )
            std::swap<P>(m[N*c+r],m[N*r+c]);
}

// s = skewSymetrixMatrix(v)
template<typename P>
void MatSkew(P s[3*3], const P v[3] )
{
    s[0] = 0;
    s[1] = v[2];
    s[2] = -v[1];
    s[3] = -v[2];
    s[4] = 0;
    s[5] = v[0];
    s[6] = v[1];
    s[7] = -v[0];
    s[8] = 0;
}

template<int N, typename P>
void MatOrtho( P m[N*N] )
{
    // http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/index.htm
    P Itimes3[N*N];
    SetIdentity<N>(Itimes3);
    MatMul<N,N>(Itimes3,(P)3.0);
    
    P mmT[N*N];
    MatMulTranspose<N,N,N>(mmT,m,m);
    
    P _c[N*N];
    MatSub<N,N>(_c,Itimes3,mmT);
    P _m[N*N];
    MatMul<N,N,N>(_m,m,_c,(P)0.5);
    std::copy(_m,_m+(N*N),m);
}

template<typename P>
void Rotation(P R[3*3], P x, P y, P z)
{
    P sx = sin(x);
    P sy = sin(y);
    P sz = sin(z);
    P cx = cos(x);
    P cy = cos(y);
    P cz = cos(z);
    //  P cx = sqrt( (P)1.0 - sx * sx);
    //  P cy = sqrt( (P)1.0 - sy * sy);
    //  P cz = sqrt( (P)1.0 - sz * sz);
    R[0] = cy * cz;
    R[1] = sx * sy * cz + cx * sz;
    R[2] = -cx * sy * cz + sx * sz;
    R[3] = -cy * sz;
    R[4] = -sx * sy * sz + cx * cz;
    R[5] = cx * sy * sz + sx * cz;
    R[6] = sy;
    R[7] = -sx * cy;
    R[8] = cx * cy;
}

template<typename P>
void LieSetIdentity(P T_ba[3*4] )
{
    SetIdentity<3>(T_ba);
    std::fill_n(T_ba+(3*3),3,0);
}

template<typename P>
void LieSetRotation(P T_ba[3*4], const P R_ba[3*3] )
{
    std::copy(R_ba,R_ba+(3*3),T_ba);
}

template<typename P>
void LieSetTranslation(P T_ba[3*4], const P a_b[3*3] )
{
    std::copy(a_b, a_b+3, T_ba+(3*3));
}

template<typename P>
void LieSetSE3(P T_ba[3*4], const P R_ba[3*3], const P a_b[3] )
{
    LieSetRotation<P>(T_ba,R_ba);
    LieSetTranslation<P>(T_ba,a_b);
}

template<typename P>
void LieGetRotation(P R_ba[3*3], const P T_ba[3*4] )
{
    std::copy(T_ba,T_ba+(3*3),R_ba);
}

template<typename P>
void LieApplySO3( P out[3], const P R_ba[3*3], const P in[3] )
{
    MatMul<3,3,1,P>(out,R_ba,in);
}

template<typename P>
void LieApplySE3vec( P x_b[3], const P T_ba[3*4], const P x_a[3] )
{
    P rot[3];
    MatMul<3,3,1,P>(rot,T_ba,x_a);
    MatAdd<3,1,P>(x_b,rot,T_ba+(3*3));
}

template<typename P>
void LieApplySE34x4vec3( P x_b[3], const P T_ba[4*4], const P x_a[3] )
{
    x_b[0] = T_ba[0]*x_a[0] + T_ba[4]*x_a[1] + T_ba[8]*x_a[2] + T_ba[12];
    x_b[1] = T_ba[1]*x_a[0] + T_ba[5]*x_a[1] + T_ba[9]*x_a[2] + T_ba[13];
    x_b[2] = T_ba[2]*x_a[0] + T_ba[6]*x_a[1] + T_ba[10]*x_a[2] + T_ba[14];
}

template<typename P>
void LieMulSO3( P R_ca[3*3], const P R_cb[3*3], const P R_ba[3*3] )
{
    MatMul<3,3,3>(R_ca,R_cb,R_ba);
}

template<typename P>
void LieMulSE3( P T_ca[3*4], const P T_cb[3*4], const P T_ba[3*4] )
{
    LieMulSO3<>(T_ca,T_cb,T_ba);
    P R_cb_times_a_b[3];
    LieApplySO3<>(R_cb_times_a_b,T_cb,T_ba+(3*3));
    MatAdd<3,1>(T_ca+(3*3),R_cb_times_a_b,T_cb+(3*3));
}

template<typename P>
void LiePutSE3in4x4(P out[4*4], const P in[3*4] )
{
    SetIdentity<4>(out);
    std::copy(in,in+3, out);
    std::copy(in+3,in+6, out+4);
    std::copy(in+6,in+9, out+8);
    std::copy(in+9,in+12, out+12);
}

template<typename P>
void LieSE3from4x4(P out[3*4], const P in[4*4] )
{
    std::copy(in,in+4, out);
    std::copy(in+4,in+8, out+3);
    std::copy(in+8,in+12, out+6);
    std::copy(in+12,in+16, out+9);
}

template<typename P>
void LieMul4x4bySE3( P T_ca[4*4], const P T_cb[3*4], const P T_ba[4*4] )
{
    // TODO: fast
    P T_cb4[4*4];
    LiePutSE3in4x4<>(T_cb4,T_cb);
    P res[4*4];
    MatMul<4,4,4>(res,T_cb4,T_ba);
    std::copy(res,res+(4*4),T_ca);
}

template<typename P>
void LieTransposeSO3( P R_ab[3*3], const P R_ba[3*3] )
{
    MatTranspose<3,P>(R_ab,R_ba);
}

template<typename P>
void LieInverseSE3( P T_ab[3*4], const P T_ba[3*4] )
{
    LieTransposeSO3<P>(T_ab,T_ba);
    P minus_b_a[3];
    LieApplySO3(minus_b_a, T_ab, T_ba+(3*3));
    MatMul<3,1,P>(T_ab+(3*3),minus_b_a, -1);
}

// c = a x b
template<typename P>
void CrossProduct( P c[3], const P a[3], const P b[3] )
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

template<int R, typename P>
P Length( P v[R] )
{
    P sum_sq = 0;
    
    for(size_t r = 0; r < R; ++r ) {
        sum_sq += v[r] * v[r];
    }
    
    return sqrt(sum_sq);
}


template<int R, typename P>
void Normalise( P v[R] )
{
    const P length = Length<R,P>(v);
    
    for(size_t r = 0; r < R; ++r ) {
        v[r] /= length;
    }
}

template<typename P>
void EnforceUpT_wc(P T_wc[3*4], const P up_w[3])
{
    // Copy R_wc
    P R_wc[3*3];
    std::copy(T_wc,T_wc+3*3,R_wc);
    
    // New R_wc should go into T_wc
    P* NR_wc = T_wc;
    
    //    // cx_w,cy_w,cz_w are camera axis in world coordinates
    //    // Calculate new camera coordinates (ncx_w,ncy_w,ncz_w)
    
    // ncx_w = up_w x cz_w
    CrossProduct(NR_wc + 0*3, up_w, R_wc + 2*3);
    
    // ncy_w = cz_w x ncx_w
    CrossProduct(NR_wc + 1*3, R_wc + 2*3, NR_wc + 0*3);
    
    // ncz_w = cz_w
    std::copy(R_wc + 2*3, R_wc + 3*3, NR_wc + 2*3);
    
    Normalise<3,P>(NR_wc + 0*3);
    Normalise<3,P>(NR_wc + 1*3);
    Normalise<3,P>(NR_wc + 2*3);
}

template<typename P>
void EnforceUpT_cw(P T_cw_4x4[4*4], const P up_w[3])
{
    // 3x4 from 4x4
    P T_cw[3*4];
    LieSE3from4x4<P>(T_cw,T_cw_4x4);
    
    // Invert T_cw
    P T_wc[3*4];
    LieInverseSE3<P>(T_wc, T_cw);
    
    // Enforce up for T_wc
    EnforceUpT_wc<P>(T_wc, up_w);
    
    // Invert
    LieInverseSE3<P>(T_cw, T_wc);
    
    // 4x4 from 3x4
    LiePutSE3in4x4<P>(T_cw_4x4,T_cw);
}

}

#endif //PANGOLIN_SIMPLE_MATH_H
