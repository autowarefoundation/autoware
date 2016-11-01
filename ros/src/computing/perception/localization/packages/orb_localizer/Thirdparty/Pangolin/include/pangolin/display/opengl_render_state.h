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

#ifndef PANGOLIN_OPENGLRENDERSTATE_H
#define PANGOLIN_OPENGLRENDERSTATE_H

#include <pangolin/platform.h>
#include <pangolin/utils/simple_math.h>

#include <vector>

#if defined(HAVE_EIGEN) && !defined(__CUDACC__) //prevent including Eigen in cuda files
#define USE_EIGEN
#endif

#ifdef USE_EIGEN
#include <Eigen/Eigen>
#endif

#ifdef HAVE_TOON
#include <cstring>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#endif

#ifdef HAVE_OCULUS
#include <pangolin/compat/ovr.h>
#endif

namespace pangolin {

#ifdef HAVE_GLES
    typedef float GLprecision;
#else
    typedef double GLprecision;
#endif

/// Capture OpenGL matrix types in enum to typing.
enum OpenGlStack {
    GlModelViewStack =  0x1700, // GL_MODELVIEW
    GlProjectionStack = 0x1701, // GL_PROJECTION
    GlTextureStack =    0x1702  // GL_TEXTURE
};

enum AxisDirection
{
    AxisNone,
    AxisNegX, AxisX,
    AxisNegY, AxisY,
    AxisNegZ, AxisZ
};

struct CameraSpec {
    GLprecision forward[3];
    GLprecision up[3];
    GLprecision right[3];
    GLprecision img_up[2];
    GLprecision img_right[2];
};

const static CameraSpec CameraSpecOpenGl = {{0,0,-1},{0,1,0},{1,0,0},{0,1},{1,0}};

const static CameraSpec CameraSpecYDownZForward = {{0,0,1},{0,-1,0},{1,0,0},{0,-1},{1,0}};

/// Direction vector for each AxisDirection enum
const static GLprecision AxisDirectionVector[7][3] = {
    {0,0,0},
    {-1,0,0}, {1,0,0},
    {0,-1,0}, {0,1,0},
    {0,0,-1}, {0,0,1}
};

/// Object representing OpenGl Matrix.
struct PANGOLIN_EXPORT OpenGlMatrix {
    static OpenGlMatrix Translate(GLprecision x, GLprecision y, GLprecision z);
    static OpenGlMatrix Scale(GLprecision x, GLprecision y, GLprecision z);
    static OpenGlMatrix RotateX(GLprecision theta_rad);
    static OpenGlMatrix RotateY(GLprecision theta_rad);
    static OpenGlMatrix RotateZ(GLprecision theta_rad);


    template<typename P>
    static OpenGlMatrix ColMajor4x4(const P* col_major_4x4);
    
    OpenGlMatrix();
    
#ifdef USE_EIGEN
    template<typename P>
    OpenGlMatrix(const Eigen::Matrix<P,4,4>& mat);
    
    template<typename P>
    operator Eigen::Matrix<P,4,4>() const;
#endif // USE_EIGEN

#ifdef HAVE_TOON
    OpenGlMatrix(const TooN::SE3<>& T);
    OpenGlMatrix(const TooN::Matrix<4,4>& M);
    operator const TooN::SE3<>() const;
    operator const TooN::Matrix<4,4>() const;
#endif // HAVE_TOON    

#ifdef HAVE_OCULUS
    OpenGlMatrix(const OVR::Matrix4f& M);
    operator const OVR::Matrix4f() const;
#endif // HAVE_OCULUS
    
    // Load matrix on to OpenGl stack
    void Load() const;
    
    void Multiply() const;
    
    void SetIdentity();
    
    OpenGlMatrix Transpose() const;
    
    OpenGlMatrix Inverse() const;

    GLprecision& operator()(int r, int c) {
        return m[4*c +r];
    }

    GLprecision operator()(int r, int c) const {
        return m[4 * c + r];
    }
    
    // Column major Internal buffer
    GLprecision m[16];
};

PANGOLIN_EXPORT
OpenGlMatrix operator*(const OpenGlMatrix& lhs, const OpenGlMatrix& rhs);

PANGOLIN_EXPORT
std::ostream& operator<<(std::ostream& os, const OpenGlMatrix& mat);

/// Deprecated.
struct PANGOLIN_EXPORT OpenGlMatrixSpec : public OpenGlMatrix {
    // Specify which stack this refers to
    OpenGlStack type;
};

/// Object representing attached OpenGl Matrices / transforms.
class PANGOLIN_EXPORT OpenGlRenderState
{
public:
    OpenGlRenderState();
    OpenGlRenderState(const OpenGlMatrix& projection_matrix);
    OpenGlRenderState(const OpenGlMatrix& projection_matrix, const OpenGlMatrix& modelview_matrix);
    
    static void ApplyIdentity();
    
    void Apply() const;
    OpenGlRenderState& SetProjectionMatrix(OpenGlMatrix m);
    OpenGlRenderState& SetModelViewMatrix(OpenGlMatrix m);

    OpenGlMatrix& GetProjectionMatrix();
    OpenGlMatrix GetProjectionMatrix() const;
    
    OpenGlMatrix& GetModelViewMatrix();
    OpenGlMatrix GetModelViewMatrix() const;
    
    OpenGlMatrix GetProjectionModelViewMatrix() const;
    OpenGlMatrix GetProjectiveTextureMatrix() const;
    
    void EnableProjectiveTexturing() const;
    void DisableProjectiveTexturing() const;
    
    //! Seemlessly move OpenGl camera relative to changes in T_wc,
    //! whilst still enabling interaction
    void Follow(const OpenGlMatrix& T_wc, bool follow = true);
    void Unfollow();

    // Experimental - subject to change
    OpenGlMatrix& GetProjectionMatrix(unsigned int view);
    OpenGlMatrix GetProjectionMatrix(unsigned int view) const;
    OpenGlMatrix& GetViewOffset(unsigned int view);
    OpenGlMatrix GetViewOffset(unsigned int view) const;
    OpenGlMatrix GetModelViewMatrix(int i) const;
    void ApplyNView(int view) const;
    
    PANGOLIN_DEPRECATED
    OpenGlRenderState& Set(OpenGlMatrixSpec spec);
    
protected:
    OpenGlMatrix modelview;
    std::vector<OpenGlMatrix> projection;
    std::vector<OpenGlMatrix> modelview_premult;
    OpenGlMatrix T_cw;
    bool follow;
};

PANGOLIN_EXPORT
OpenGlMatrixSpec ProjectionMatrixRUB_BottomLeft(int w, int h, GLprecision fu, GLprecision fv, GLprecision u0, GLprecision v0, GLprecision zNear, GLprecision zFar );

PANGOLIN_EXPORT
OpenGlMatrixSpec ProjectionMatrixRDF_TopLeft(int w, int h, GLprecision fu, GLprecision fv, GLprecision u0, GLprecision v0, GLprecision zNear, GLprecision zFar );

PANGOLIN_EXPORT
OpenGlMatrixSpec ProjectionMatrixRDF_BottomLeft(int w, int h, GLprecision fu, GLprecision fv, GLprecision u0, GLprecision v0, GLprecision zNear, GLprecision zFar );


//! Use OpenGl's default frame RUB_BottomLeft
PANGOLIN_EXPORT
OpenGlMatrixSpec ProjectionMatrix(int w, int h, GLprecision fu, GLprecision fv, GLprecision u0, GLprecision v0, GLprecision zNear, GLprecision zFar );

PANGOLIN_EXPORT
OpenGlMatrixSpec ProjectionMatrixOrthographic(GLprecision l, GLprecision r, GLprecision b, GLprecision t, GLprecision n, GLprecision f );


//! Generate glulookat style model view matrix, looking at (lx,ly,lz)
//! X-Right, Y-Up, Z-Back
PANGOLIN_EXPORT
OpenGlMatrix ModelViewLookAtRUB(GLprecision ex, GLprecision ey, GLprecision ez, GLprecision lx, GLprecision ly, GLprecision lz, GLprecision ux, GLprecision uy, GLprecision uz);

//! Generate glulookat style model view matrix, looking at (lx,ly,lz)
//! X-Right, Y-Down, Z-Forward
PANGOLIN_EXPORT
OpenGlMatrix ModelViewLookAtRDF(GLprecision ex, GLprecision ey, GLprecision ez, GLprecision lx, GLprecision ly, GLprecision lz, GLprecision ux, GLprecision uy, GLprecision uz);

//! Generate glulookat style model view matrix, OpenGL Default camera convention (XYZ=RUB), looking at (lx,ly,lz)
PANGOLIN_EXPORT
OpenGlMatrix ModelViewLookAt(GLprecision x, GLprecision y, GLprecision z, GLprecision lx, GLprecision ly, GLprecision lz, AxisDirection up);

PANGOLIN_EXPORT
OpenGlMatrix ModelViewLookAt(GLprecision ex, GLprecision ey, GLprecision ez, GLprecision lx, GLprecision ly, GLprecision lz, GLprecision ux, GLprecision uy, GLprecision uz);


PANGOLIN_EXPORT
OpenGlMatrix IdentityMatrix();

PANGOLIN_EXPORT
OpenGlMatrixSpec IdentityMatrix(OpenGlStack type);

PANGOLIN_EXPORT
OpenGlMatrixSpec negIdentityMatrix(OpenGlStack type);

#ifdef HAVE_TOON
OpenGlMatrixSpec FromTooN(const TooN::SE3<>& T_cw);
OpenGlMatrixSpec FromTooN(OpenGlStack type, const TooN::Matrix<4,4>& M);
TooN::Matrix<4,4> ToTooN(const OpenGlMatrixSpec& ms);
TooN::SE3<> ToTooN_SE3(const OpenGlMatrixSpec& ms);
#endif

}

// Inline definitions
namespace pangolin
{

template<typename P>
inline OpenGlMatrix OpenGlMatrix::ColMajor4x4(const P* col_major_4x4)
{
    OpenGlMatrix mat;
    std::copy(col_major_4x4, col_major_4x4 + 16, mat.m);
    return mat;
}

inline OpenGlMatrix::OpenGlMatrix() {
}

#ifdef USE_EIGEN
template<typename P> inline
OpenGlMatrix::OpenGlMatrix(const Eigen::Matrix<P,4,4>& mat)
{
    for(int r=0; r<4; ++r ) {
        for(int c=0; c<4; ++c ) {
            m[c*4+r] = mat(r,c);
        }
    }
}

template<typename P>
OpenGlMatrix::operator Eigen::Matrix<P,4,4>() const
{
    Eigen::Matrix<P,4,4> mat;
    for(int r=0; r<4; ++r ) {
        for(int c=0; c<4; ++c ) {
            mat(r,c) = (P)m[c*4+r];
        }
    }
    return mat;
}
#endif // USE_EIGEN

#ifdef HAVE_TOON
inline OpenGlMatrix::OpenGlMatrix(const TooN::SE3<>& T)
{
    TooN::Matrix<4,4,GLprecision,TooN::ColMajor> M;
    M.slice<0,0,3,3>() = T.get_rotation().get_matrix();
    M.T()[3].slice<0,3>() = T.get_translation();
    M[3] = TooN::makeVector(0,0,0,1);
    std::memcpy(m, &(M[0][0]),16*sizeof(GLprecision));
}

inline OpenGlMatrix::OpenGlMatrix(const TooN::Matrix<4,4>& M)
{
    // Read in remembering col-major convension for our matrices
    int el = 0;
    for(int c=0; c<4; ++c)
        for(int r=0; r<4; ++r)
            m[el++] = M[r][c];    
}

inline OpenGlMatrix::operator const TooN::SE3<>() const
{
    const TooN::Matrix<4,4> m = *this;
    const TooN::SO3<> R(m.slice<0,0,3,3>());
    const TooN::Vector<3> t = m.T()[3].slice<0,3>();
    return TooN::SE3<>(R,t);    
}

inline OpenGlMatrix::operator const TooN::Matrix<4,4>() const
{
    TooN::Matrix<4,4> M;
    int el = 0;
    for( int c=0; c<4; ++c )
        for( int r=0; r<4; ++r )
            M(r,c) = m[el++];
    return M;
}

PANGOLIN_DEPRECATED
inline OpenGlMatrixSpec FromTooN(const TooN::SE3<>& T_cw)
{
    TooN::Matrix<4,4,GLprecision,TooN::ColMajor> M;
    M.slice<0,0,3,3>() = T_cw.get_rotation().get_matrix();
    M.T()[3].slice<0,3>() = T_cw.get_translation();
    M[3] = TooN::makeVector(0,0,0,1);
    
    OpenGlMatrixSpec P;
    P.type = GlModelViewStack;
    std::memcpy(P.m, &(M[0][0]),16*sizeof(GLprecision));
    return P;
}

PANGOLIN_DEPRECATED
inline OpenGlMatrixSpec FromTooN(OpenGlStack type, const TooN::Matrix<4,4>& M)
{
    // Read in remembering col-major convension for our matrices
    OpenGlMatrixSpec P;
    P.type = type;
    int el = 0;
    for(int c=0; c<4; ++c)
        for(int r=0; r<4; ++r)
            P.m[el++] = M[r][c];
    return P;
}

PANGOLIN_DEPRECATED
inline TooN::Matrix<4,4> ToTooN(const OpenGlMatrix& ms)
{
    TooN::Matrix<4,4> m;
    int el = 0;
    for( int c=0; c<4; ++c )
        for( int r=0; r<4; ++r )
            m(r,c) = ms.m[el++];
    return m;
}

PANGOLIN_DEPRECATED
inline TooN::SE3<> ToTooN_SE3(const OpenGlMatrix& ms)
{
    TooN::Matrix<4,4> m = ms;
    const TooN::SO3<> R(m.slice<0,0,3,3>());
    const TooN::Vector<3> t = m.T()[3].slice<0,3>();
    return TooN::SE3<>(R,t);
}

#endif // HAVE_TOON

#ifdef HAVE_OCULUS
inline OpenGlMatrix::OpenGlMatrix(const OVR::Matrix4f& mat)
{
    for(int r=0; r<4; ++r )
        for(int c=0; c<4; ++c )
            m[c*4+r] = mat.M[r][c];
}

inline OpenGlMatrix::operator const OVR::Matrix4f() const
{
    OVR::Matrix4f mat;
    for(int r=0; r<4; ++r )
        for(int c=0; c<4; ++c )
            mat.M[r][c] = m[c*4+r];
    return mat;
}
#endif // HAVE_OCULUS


}

#endif // PANGOLIN_OPENGLRENDERSTATE_H
