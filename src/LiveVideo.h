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

#ifndef LIVEVIDEO_H
#define LIVEVIDEO_H

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/camera.h>

class System
{
public:

    System();

    void
    Run();

private:

    void
    DrawCamera();

    void
    BuildPyramid(CVD::Image<CVD::byte> &im,
                 std::vector<CVD::Image<CVD::byte> > &pyr);

    enum SELECTION{FRAMEZERO, FRAMEONE, FRAMETWO} selection;

    bool bQuit;
    bool bSpacebar;
    bool bInit;
    int nLevels;
    float fScale;

    cv::Mat descA;
    cv::Mat desc_live;
    std::vector<cv::KeyPoint> kpA;
    std::vector<cv::KeyPoint> kp_live;

    Camera::Linear cam_params;

    static void GUICommandCallBack(void* ptr,
                                   std::string sCommand,
                                   std::string sParams);

    void GUICommandHandler(std::string sCommand,
                           std::string sParams);
};

#endif // LIVEVIDEO_H
