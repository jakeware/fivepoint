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

#ifndef IMAGEGRABBER_H
#define IMAGEGRABBER_H

#include <set>
#include <string>
#include <sys/stat.h>
#include <dirent.h>
#include <cvd/image.h>
#include <cvd/byte.h>

enum OnEndFlag
{
    LOOP,
    EXIT
};

enum FrameFlag
{
    NEXT,
    PREVIOUS,
    NONE
};

class ImageGrabber
{
public:
    ImageGrabber(std::string sDir_,
                 std::string sPrefix_,
                 std::string sSuffix_,
                 OnEndFlag EndBehaviour_);

    bool GrabNextFrame(CVD::Image<CVD::byte> &cvd_im,
                       FrameFlag frame_flag=NEXT);

private:

    bool OpenDir();
    void ExtractFileNames();
    std::string ExtractFileID(std::string sName);

    std::set<std::string> sFileIDs;
    std::set<std::string>::iterator sFileIter;
    OnEndFlag EndBehaviour;

    int len;
    struct dirent *pDirent;
    DIR *pDir;

    std::string sDir;
    std::string sPrefix;
    std::string sSuffix;
};

#endif // IMAGEGRABBER_H
