/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2013 Steven Lovegrove
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

#include <pangolin/handler/handler.h>
#include <pangolin/display/display_internal.h>
#include <pangolin/display/view.h>

namespace pangolin
{

// Pointer to context defined in display.cpp
extern __thread PangolinGl* context;

void Handler::Keyboard(View& d, unsigned char key, int x, int y, bool pressed)
{
    View* child = d.FindChild(x,y);
    if( child)
    {
        context->activeDisplay = child;
        if( child->handler)
            child->handler->Keyboard(*child,key,x,y,pressed);
    }
}

void Handler::Mouse(View& d, MouseButton button, int x, int y, bool pressed, int button_state)
{
    View* child = d.FindChild(x,y);
    if( child )
    {
        context->activeDisplay = child;
        if( child->handler)
            child->handler->Mouse(*child,button,x,y,pressed,button_state);
    }
}

void Handler::MouseMotion(View& d, int x, int y, int button_state)
{
    View* child = d.FindChild(x,y);
    if( child )
    {
        context->activeDisplay = child;
        if( child->handler)
            child->handler->MouseMotion(*child,x,y,button_state);
    }
}

void Handler::PassiveMouseMotion(View& d, int x, int y, int button_state)
{
    View* child = d.FindChild(x,y);
    if( child )
    {
        if( child->handler)
            child->handler->PassiveMouseMotion(*child,x,y,button_state);
    }
}

void Handler::Special(View& d, InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state)
{
    View* child = d.FindChild( (int)x, (int)y);
    if( child )
    {
        context->activeDisplay = child;
        if( child->handler)
            child->handler->Special(*child,inType, x,y, p1, p2, p3, p4, button_state);
    }
}

void HandlerScroll::Mouse(View& d, MouseButton button, int x, int y, bool pressed, int button_state)
{
    if( pressed && (button == MouseWheelUp || button == MouseWheelDown) )
    {
        if( button == MouseWheelUp) d.scroll_offset   -= 1;
        if( button == MouseWheelDown) d.scroll_offset += 1;
        d.scroll_offset = std::max(0, std::min(d.scroll_offset, (int)d.views.size()) );
        d.ResizeChildren();
    }else{
        Handler::Mouse(d,button,x,y,pressed,button_state);
    }
}

void HandlerScroll::Special(View& d, InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state)
{
    if( inType == InputSpecialScroll )
    {
        d.scroll_offset -= (int)(p2 / fabs(p2));
        d.scroll_offset = std::max(0, std::min(d.scroll_offset, (int)d.views.size()) );
        d.ResizeChildren();
    }else{
        Handler::Special(d,inType,x,y,p1,p2,p3,p4,button_state);
    }
}

Handler3D::Handler3D(OpenGlRenderState& cam_state, AxisDirection enforce_up, float trans_scale, float zoom_fraction)
    : cam_state(&cam_state), enforce_up(enforce_up), tf(trans_scale), zf(zoom_fraction), cameraspec(CameraSpecOpenGl), last_z(0.8)
{
    SetZero<3,1>(rot_center);
}

void Handler3D::Keyboard(View&, unsigned char key, int x, int y, bool pressed)
{
    // TODO: hooks for reset / changing mode (perspective / ortho etc)
}

void Handler3D::GetPosNormal(pangolin::View& view, int x, int y, GLprecision p[3], GLprecision Pw[3], GLprecision Pc[3], GLprecision n[3], GLprecision default_z)
{
    // TODO: Get to work on android    
    const GLint viewport[4] = {view.v.l,view.v.b,view.v.w,view.v.h};
    const pangolin::OpenGlMatrix proj = cam_state->GetProjectionMatrix();
    const pangolin::OpenGlMatrix mv = cam_state->GetModelViewMatrix();
    //      const pangolin::OpenGlMatrix id = IdentityMatrix();
    
    const int zl = (hwin*2+1);
    const int zsize = zl*zl;
    GLfloat zs[zsize];
    
#ifndef HAVE_GLES    
    glReadBuffer(GL_FRONT);
    glReadPixels(x-hwin,y-hwin,zl,zl,GL_DEPTH_COMPONENT,GL_FLOAT,zs);
#else
    std::fill(zs,zs+zsize, 1);
#endif
    GLfloat mindepth = *(std::min_element(zs,zs+zsize));
    
    if(mindepth == 1) mindepth = (GLfloat)default_z;
    
    p[0] = x; p[1] = y; p[2] = mindepth;
    glUnProject(x, y, mindepth, mv.m, proj.m, viewport, &Pw[0], &Pw[1], &Pw[2]);
//      glUnProject(x, y, mindepth, id.m, proj.m, viewport, &Pc[0], &Pc[1], &Pc[2]);
    LieApplySE34x4vec3(Pc, mv.m, Pw);
    
    GLprecision Pl[3]; GLprecision Pr[3]; GLprecision Pb[3]; GLprecision Pt[3];
    glUnProject(x-hwin, y, zs[hwin*zl + 0],    mv.m, proj.m, viewport, &Pl[0], &Pl[1], &Pl[2]);
    glUnProject(x+hwin, y, zs[hwin*zl + zl-1], mv.m, proj.m, viewport, &Pr[0], &Pr[1], &Pr[2]);
    glUnProject(x, y-hwin, zs[hwin+1],         mv.m, proj.m, viewport, &Pb[0], &Pb[1], &Pb[2]);
    glUnProject(x, y+hwin, zs[zsize-(hwin+1)], mv.m, proj.m, viewport, &Pt[0], &Pt[1], &Pt[2]);
    
    //      n = ((Pr-Pl).cross(Pt-Pb)).normalized();
    GLprecision PrmPl[3]; GLprecision PtmPb[3];
    MatSub<3,1>(PrmPl,Pr,Pl);
    MatSub<3,1>(PtmPb,Pt,Pb);
    CrossProduct(n, PrmPl, PtmPb);
    Normalise<3>(n);
}

void Handler3D::Mouse(View& display, MouseButton button, int x, int y, bool pressed, int button_state)
{
    // mouse down
    last_pos[0] = (float)x;
    last_pos[1] = (float)y;
    
    GLprecision T_nc[3*4];
    LieSetIdentity(T_nc);
    
    if( pressed ) {
        GetPosNormal(display,x,y,p,Pw,Pc,n,last_z);
        if(p[2] < 1.0) {
            last_z = p[2];
            std::copy(Pc,Pc+3,rot_center);
        }
        
        if( button == MouseWheelUp || button == MouseWheelDown)
        {
            LieSetIdentity(T_nc);
            const GLprecision t[] = { 0,0,(button==MouseWheelUp?1:-1)*100*tf};
            LieSetTranslation<>(T_nc,t);
            if( !(button_state & MouseButtonRight) && !(rot_center[0]==0 && rot_center[1]==0 && rot_center[2]==0) )
            {
                LieSetTranslation<>(T_nc,rot_center);
                const GLprecision s = (button==MouseWheelUp?-1.0:1.0) * zf;
                MatMul<3,1>(T_nc+(3*3), s);
            }
            OpenGlMatrix& spec = cam_state->GetModelViewMatrix();
            LieMul4x4bySE3<>(spec.m,T_nc,spec.m);
        }
    }
}

void Handler3D::MouseMotion(View& display, int x, int y, int button_state)
{
    const GLprecision rf = 0.01;
    const float delta[2] = { (float)x - last_pos[0], (float)y - last_pos[1] };
    const float mag = delta[0]*delta[0] + delta[1]*delta[1];
    
    // TODO: convert delta to degrees based of fov
    // TODO: make transformation with respect to cam spec
    
    if( mag < 50.0f*50.0f )
    {
        OpenGlMatrix& mv = cam_state->GetModelViewMatrix();
        const GLprecision* up = AxisDirectionVector[enforce_up];
        GLprecision T_nc[3*4];
        LieSetIdentity(T_nc);
        bool rotation_changed = false;
        
        if( button_state == MouseButtonMiddle )
        {
            // Middle Drag: Rotate around view

            // Try to correct for different coordinate conventions.
            GLprecision aboutx = -rf * delta[1];
            GLprecision abouty = rf * delta[0];
            OpenGlMatrix& pm = cam_state->GetProjectionMatrix();
            abouty *= -pm.m[2 * 4 + 3];

            Rotation<>(T_nc, aboutx, abouty, (GLprecision)0.0);
        }else if( button_state == MouseButtonLeft )
        {
            // Left Drag: in plane translate
            if( last_z != 1 )
            {
                GLprecision np[3];
                display.GetCamCoordinates(*cam_state,x,y,last_z, np[0], np[1], np[2]);
                const GLprecision t[] = { np[0] - rot_center[0], np[1] - rot_center[1], 0};
                LieSetTranslation<>(T_nc,t);
                std::copy(np,np+3,rot_center);
            }else{
                const GLprecision t[] = { -10*delta[0]*tf, 10*delta[1]*tf, 0};
                LieSetTranslation<>(T_nc,t);
            }
        }else if( button_state == (MouseButtonLeft | MouseButtonRight) )
        {
            // Left and Right Drag: in plane rotate about object
            //        Rotation<>(T_nc,0.0,0.0, delta[0]*0.01);
            
            GLprecision T_2c[3*4];
            Rotation<>(T_2c, (GLprecision)0.0, (GLprecision)0.0, delta[0]*rf);
            GLprecision mrotc[3];
            MatMul<3,1>(mrotc, rot_center, (GLprecision)-1.0);
            LieApplySO3<>(T_2c+(3*3),T_2c,mrotc);
            GLprecision T_n2[3*4];
            LieSetIdentity<>(T_n2);
            LieSetTranslation<>(T_n2,rot_center);
            LieMulSE3(T_nc, T_n2, T_2c );
            rotation_changed = true;
        }else if( button_state == MouseButtonRight)
        {
            // Try to correct for different coordinate conventions.
            GLprecision aboutx = -rf * delta[1];
            GLprecision abouty =  rf * delta[0];
            OpenGlMatrix& pm = cam_state->GetProjectionMatrix();
            abouty *= -pm.m[2*4+3];
            
            if(enforce_up) {
                // Special case if view direction is parallel to up vector
                const GLprecision updotz = mv.m[2]*up[0] + mv.m[6]*up[1] + mv.m[10]*up[2];
                if(updotz > 0.98) aboutx = std::min(aboutx, (GLprecision)0.0);
                if(updotz <-0.98) aboutx = std::max(aboutx, (GLprecision)0.0);
                // Module rotation around y so we don't spin too fast!
                abouty *= (1-0.6*fabs(updotz));
            }
            
            // Right Drag: object centric rotation
            GLprecision T_2c[3*4];
            Rotation<>(T_2c, aboutx, abouty, (GLprecision)0.0);
            GLprecision mrotc[3];
            MatMul<3,1>(mrotc, rot_center, (GLprecision)-1.0);
            LieApplySO3<>(T_2c+(3*3),T_2c,mrotc);
            GLprecision T_n2[3*4];
            LieSetIdentity<>(T_n2);
            LieSetTranslation<>(T_n2,rot_center);
            LieMulSE3(T_nc, T_n2, T_2c );
            rotation_changed = true;
        }
        
        LieMul4x4bySE3<>(mv.m,T_nc,mv.m);
        
        if(enforce_up != AxisNone && rotation_changed) {
            EnforceUpT_cw(mv.m, up);
        }
    }
    
    last_pos[0] = (float)x;
    last_pos[1] = (float)y;
}

void Handler3D::Special(View& display, InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state)
{
    if( !(inType == InputSpecialScroll || inType == InputSpecialRotate) )
        return;
    
    // mouse down
    last_pos[0] = x;
    last_pos[1] = y;
    
    GLprecision T_nc[3*4];
    LieSetIdentity(T_nc);
    
    GetPosNormal(display, (int)x, (int)y, p, Pw, Pc, n, last_z);
    if(p[2] < 1.0) {
        last_z = p[2];
        std::copy(Pc,Pc+3,rot_center);
    }
    
    if( inType == InputSpecialScroll ) {
        if(button_state & KeyModifierCmd) {
            const GLprecision rx = -p2 / 1000;
            const GLprecision ry = -p1 / 1000;
            
            Rotation<>(T_nc,rx, ry, (GLprecision)0.0);
            OpenGlMatrix& spec = cam_state->GetModelViewMatrix();
            LieMul4x4bySE3<>(spec.m,T_nc,spec.m);
        }else{
            const GLprecision scrolly = p2/10;
            
            LieSetIdentity(T_nc);
            const GLprecision t[] = { 0,0, -scrolly*100*tf};
            LieSetTranslation<>(T_nc,t);
            if( !(button_state & MouseButtonRight) && !(rot_center[0]==0 && rot_center[1]==0 && rot_center[2]==0) )
            {
                LieSetTranslation<>(T_nc,rot_center);
                MatMul<3,1>(T_nc+(3*3), -scrolly * zf);
            }
            OpenGlMatrix& spec = cam_state->GetModelViewMatrix();
            LieMul4x4bySE3<>(spec.m,T_nc,spec.m);
        }
    }else if(inType == InputSpecialRotate) {
        const GLprecision r = p1 / 20;
        
        GLprecision T_2c[3*4];
        Rotation<>(T_2c, (GLprecision)0.0, (GLprecision)0.0, r);
        GLprecision mrotc[3];
        MatMul<3,1>(mrotc, rot_center, (GLprecision)-1.0);
        LieApplySO3<>(T_2c+(3*3),T_2c,mrotc);
        GLprecision T_n2[3*4];
        LieSetIdentity<>(T_n2);
        LieSetTranslation<>(T_n2,rot_center);
        LieMulSE3(T_nc, T_n2, T_2c );
        OpenGlMatrix& spec = cam_state->GetModelViewMatrix();
        LieMul4x4bySE3<>(spec.m,T_nc,spec.m);
    }
    
}

}
