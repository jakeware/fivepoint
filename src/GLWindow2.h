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

#ifndef GLWINDOW2_H
#define GLWINDOW2_H

#include <cvd/glwindow.h>
#include <TooN/TooN.h>

class GLWindow2 : public CVD::GLWindow, public CVD::GLWindow::EventHandler
{
public:
    GLWindow2(CVD::ImageRef irSize, std::string sTitle);

    // Getter/setter fns
    std::pair<TooN::Vector<6>, TooN::Vector<6> > GetPoseUpdate();

    // Functions to set up window
    void SetupViewport(const CVD::ImageRef irPos, const CVD::ImageRef irSize);
    void SetupOrthoProj(const TooN::Vector<4> &v4);
    void SetupPerspectiveProj(const TooN::Vector<4> &v4);
    void SetupWindowOrtho(const CVD::ImageRef irSize);
    void SetupVideoRasterPosAndZoom();

    // Event handlers
    void HandlePendingEvents();

private:
    // Event handlers
    virtual void on_key_down(GLWindow&, int);
    virtual void on_mouse_move(GLWindow&, CVD::ImageRef, int);
    CVD::ImageRef irMouseLastPos;

    CVD::ImageRef irVideoSourceSize;

    // Updates to view point in GLWindow
    TooN::Vector<6> v6PoseUpdate;
    TooN::Vector<6> v6MCUpdate;
};

#endif // GLWINDOW2_H
