// This file is part of FivePoint
// 
// Copyright (C) 2016 Vincent Lui (vincent.lui@monash.edu), Monash University
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "GLWindow2.h"
#include <cvd/gl_helpers.h>
#include <gvars3/instances.h>

using namespace CVD;
using namespace TooN;
using namespace GVars3;
using namespace std;

GLWindow2::GLWindow2(ImageRef irSize, std::string sTitle)
    : GLWindow(irSize, sTitle)
{
    v6PoseUpdate = Zeros;
    v6MCUpdate = Zeros;

    irVideoSourceSize = irSize;
}

std::pair<Vector<6>,Vector<6> > GLWindow2::GetPoseUpdate()
{
    Vector<6> v6First = v6PoseUpdate;
    Vector<6> v6Second = v6MCUpdate;
    v6PoseUpdate = Zeros;
    v6MCUpdate = Zeros;

    return std::pair<Vector<6>,Vector<6> >(v6First, v6Second);
}

void GLWindow2::SetupViewport(const ImageRef irPos, const CVD::ImageRef irSize)
{
    glViewport(irPos.x, irPos.y, irSize.x, irSize.y);
}

void GLWindow2::SetupOrthoProj(const TooN::Vector<4> &v4)
{
    // v4[0] - left
    // v4[1] - right
    // v4[2] - bottom
    // v4[3] - top

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(v4[0], v4[1], v4[2], v4[3], -100, 100);
}

void GLWindow2::SetupWindowOrtho(const CVD::ImageRef irSize)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0,irSize.x,irSize.y, 0, -1.0, 1.0);
}

void GLWindow2::SetupPerspectiveProj(const TooN::Vector<4> &v4)
{
    // v4[0] - left
    // v4[1] - right
    // v4[2] - bottom
    // v4[3] - top

    float fNear = 1.0;
    float fFar = 100.0;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(v4[0],v4[1],v4[2],v4[3],fNear,fFar);
}

void GLWindow2::SetupVideoRasterPosAndZoom()
{
    glRasterPos2d(0,0);
    double adZoom[2];
    adZoom[0] = (double) size()[0] / (double) irVideoSourceSize[0];
    adZoom[1] = (double) size()[1] / (double) irVideoSourceSize[1];
    glPixelZoom(adZoom[0], -adZoom[1]);
}

void GLWindow2::on_mouse_move(GLWindow& glw, ImageRef irMouseCurrPos, int state)
{
    ImageRef irMotion = irMouseCurrPos - irMouseLastPos;
    irMouseLastPos = irMouseCurrPos;

    double dSensitivity = 0.01;
    if(state & BUTTON_LEFT && !(state & BUTTON_RIGHT))
    {
        v6MCUpdate[4] -= irMotion[0] * dSensitivity;
        v6MCUpdate[3] += irMotion[1] * dSensitivity;
    }
    else if(state & BUTTON_RIGHT && !(state & BUTTON_LEFT))
    {
        v6PoseUpdate[4] -= irMotion[0] * dSensitivity;
        v6PoseUpdate[3] += irMotion[1] * dSensitivity;
    }
    else if((state & BUTTON_LEFT && state & BUTTON_RIGHT) || state == BUTTON_MIDDLE)
    {
        v6PoseUpdate[2] -= irMotion[1] * dSensitivity;
        v6PoseUpdate[5] += irMotion[0] * dSensitivity;
    }
}

void GLWindow2::on_key_down(GLWindow&, int k)
{
    string s;
    // ASCII chars can be mapped directly:
    if((k >= 48 && k <= 57) || (k >= 97 && k <= 122) || (k >= 65 && k <= 90))
    {
        char c = k;
        if(c >= 65 &&  c <= 90)
            c = c + 32;
        s = c;
    }
    else switch(k) // Some special chars are translated:
    {
    case 33 : s="PageUp"; break;
    case 34 : s="PageDown"; break;
    case 13 : s="Enter"; break;
    case 32 : s="Space"; break;
    case 8 : s="BackSpace"; break;
    case 27 : s="Escape"; break;
    default : break;
    }

    if(s != "")
        GUI.ParseLine("try Keypress " + s);
}

void GLWindow2::HandlePendingEvents()
{
    handle_events(*this);
}
