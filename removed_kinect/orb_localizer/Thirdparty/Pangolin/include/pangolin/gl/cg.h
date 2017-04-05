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

#ifndef PANGOLIN_CG_H
#define PANGOLIN_CG_H

#include <sstream>
#include <algorithm>

// Cg includes
#include <Cg/cg.h>
#include <Cg/cgGL.h>

#include "gl.h"

#ifdef HAVE_TOON
#include <TooN/TooN.h>
#endif // HAVE_TOON

namespace pangolin
{

////////////////////////////////////////////////
// Interface
////////////////////////////////////////////////

/// Lightweight object wrapper for NVidia Cg Shader program objects.
class CgProgram
{
    friend class CgLoader;
public:
    void SetUniform(const std::string& name, GlTexture& tex);
    void SetUniform(const std::string& name, float f);
    
#ifdef HAVE_TOON
    void SetUniform(const std::string& name, const TooN::Vector<2>& v );
    void SetUniform(const std::string& name, const TooN::Vector<3>& v );
    
    template <int R, int C>
    void SetUniform(const std::string& name, const TooN::Matrix<R,C>& M );
#endif
    
    void UpdateParams();
    
protected:
    CGprogram mProg;
    CGcontext mContext;
    CGprofile mProfile;
};

class CgLoader
{
public:
    CgLoader();
    ~CgLoader();
    
    // Call AFTER glutInit (or similar)
    void Initialise();
    
    CgProgram LoadProgramFromFile(const std::string& file, const std::string& function, bool isVertexShader );
    
    void EnableProgram(CgProgram program);
    void DisablePrograms();
    
    void RenderDummyQuad();
    void RenderDummyQuadWithTexCoords(int w, int h);
    
protected:
    CGcontext mContext;
    CGprofile mFragmentProfile;
    CGprofile mVertexProfile;
};




////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////

inline bool cgOkay()
{
    CGerror error;
    const char *string = cgGetLastErrorString(&error);
    
    if (error != CG_NO_ERROR) {
        std::cout << "CG Error: " << string << std::endl;
        //    assert(0);
        return false;
    }
    return true;
}

inline CgLoader::CgLoader()
    :mContext(0)
{
}

inline CgLoader::~CgLoader()
{
    if(mContext)
    {
        // Destroying context destroys all programs associated with it
        cgDestroyContext(mContext);
    }
}


inline void CgLoader::Initialise()
{
    mContext = cgCreateContext();
    cgSetParameterSettingMode(mContext, CG_DEFERRED_PARAMETER_SETTING);
    cgOkay();
    
    mFragmentProfile = cgGLGetLatestProfile(CG_GL_FRAGMENT);
    cgGLSetOptimalOptions(mFragmentProfile);
    cgOkay();
    
    mVertexProfile = cgGLGetLatestProfile(CG_GL_VERTEX);
    cgGLSetOptimalOptions(mVertexProfile);
    cgOkay();
}

inline CgProgram CgLoader::LoadProgramFromFile(const std::string& file, const std::string& function, bool isVertexShader )
{
    if( !mContext ) {
        Initialise();
    }
    
    CgProgram prog;
    
    prog.mContext = mContext;
    prog.mProfile = isVertexShader ? mVertexProfile : mFragmentProfile;
    prog.mProg = cgCreateProgramFromFile( prog.mContext, CG_SOURCE, file.c_str(), prog.mProfile, function.c_str(), NULL);
    
    if( !cgOkay() )
    {
        std::cout << cgGetLastListing(mContext) << std::endl;
        assert(0);
    }
    
    cgGLLoadProgram(prog.mProg);
    if( !cgOkay() )
    {
        const char* err = cgGetProgramString( prog.mProg, CG_COMPILED_PROGRAM );
        int pos;
        glGetIntegerv(GL_PROGRAM_ERROR_POSITION_ARB, &pos);
        std::cout << err << std::endl;
        std::cout << "@ " << pos << std::endl;
        assert(0);
    }
    return prog;
}

inline void CgLoader::EnableProgram(CgProgram program)
{
    cgGLBindProgram(program.mProg);
    cgGLEnableProfile(program.mProfile);
    cgOkay();
}

inline void CgLoader::DisablePrograms()
{
    cgGLDisableProfile(mFragmentProfile);
    cgGLDisableProfile(mVertexProfile);
}

inline void CgLoader::RenderDummyQuad()
{
    glBegin(GL_QUADS);
    glVertex2d(-1,1);
    glVertex2d(1,1);
    glVertex2d(1,-1);
    glVertex2d(-1,-1);
    glEnd();
}

inline void CgLoader::RenderDummyQuadWithTexCoords(int w, int h)
{
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex2d(-1,-1);
    glTexCoord2f(w, 0);
    glVertex2d(1,-1);
    glTexCoord2f(w, h);
    glVertex2d(1,1);
    glTexCoord2f(0, h);
    glVertex2d(-1,1);
    glEnd();
}

void CgProgram::SetUniform(const std::string& name, float f)
{
    CGparameter p = cgGetNamedParameter( mProg, name.c_str());
    cgSetParameter1f( p, f );
    cgUpdateProgramParameters(mProg);
}

void CgProgram::SetUniform(const std::string& name, GlTexture& tex)
{
    CGparameter p = cgGetNamedParameter( mProg, name.c_str());
    cgGLSetTextureParameter(p, tex.tid );
    cgGLEnableTextureParameter(p);
    cgUpdateProgramParameters(mProg);
}

#ifdef HAVE_TOON
void CgProgram::SetUniform(const std::string& name, const TooN::Vector<2>& v )
{
    CGparameter p = cgGetNamedParameter( mProg, name.c_str());
    cgGLSetParameter2f(p, v[0],v[1] );
    cgUpdateProgramParameters(mProg);
}

void CgProgram::SetUniform(const std::string& name, const TooN::Vector<3>& v )
{
    CGparameter p = cgGetNamedParameter( mProg, name.c_str());
    cgGLSetParameter3f(p, v[0],v[1],v[2] );
    cgUpdateProgramParameters(mProg);
}

template <int R, int C>
void CgProgram::SetUniform(const std::string& name, const TooN::Matrix<R,C>& M )
{
    CGparameter p = cgGetNamedParameter( mProg, name.c_str());
    float Mdata[R*C];
    
    int i=0;
    for( int r=0; r<R; ++r )
        for( int c=0; c<C; ++c )
            Mdata[i++] = (float)(M[r][c]);
    
    cgGLSetMatrixParameterfr(p, Mdata );
    cgUpdateProgramParameters(mProg);
}
#endif

void CgProgram::UpdateParams()
{
    cgUpdateProgramParameters(mProg);
}


}

#endif // PANGOLIN_CG_H
