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

#include "ImageGrabber.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <cvd/image_io.h>

using namespace std;
using namespace CVD;

ImageGrabber::ImageGrabber(string sDir_, string sPrefix_, string sSuffix_, OnEndFlag EndBehaviour_)
    : sDir(sDir_), sPrefix(sPrefix_), sSuffix(sSuffix_), EndBehaviour(EndBehaviour_)
{
    OpenDir();
    ExtractFileNames();
    sFileIter = sFileIDs.begin();
}

bool ImageGrabber::OpenDir()
{
    pDir = opendir(sDir.c_str());
    if(pDir==NULL)
    {
        cerr << "[ImageGrabber:] Cannot open directory!" << endl;
        return false;
    }
    return true;
}
void ImageGrabber::ExtractFileNames()
{
    while((pDirent = readdir(pDir)) != NULL)
    {
        len = strlen(pDirent->d_name);

        if(strcmp(sSuffix.c_str(), &pDirent->d_name[len - (int) sSuffix.length()]) == 0)
            sFileIDs.insert(ExtractFileID(pDirent->d_name));
    }
}

std::string ImageGrabber::ExtractFileID(string sName)
{
    std::size_t start = sName.find(sPrefix);
    if(start != string::npos)
        start += sPrefix.length();

    std::size_t end = sName.find(sSuffix);

    string sFileID = sName.substr(start,end - start);
    return sFileID;
}

bool ImageGrabber::GrabNextFrame(Image<byte> &cvd_im,
                                 FrameFlag frame_flag)
{
    bool bFrameChanged=false;

    if(frame_flag == NEXT)
    {
        advance(sFileIter,1);
        bFrameChanged = true;
    }
    else if(frame_flag == PREVIOUS)
    {
        advance(sFileIter,-1);
        bFrameChanged = true;
    }

    if(sFileIter == sFileIDs.end())
    {
        if(EndBehaviour == LOOP || frame_flag == PREVIOUS)
            sFileIter = sFileIDs.begin();
        else
            return false;
    }

    stringstream ss;
    ss << sDir << "/" << sPrefix << *sFileIter << sSuffix;
    cvd_im = CVD::img_load(ss.str());

    return bFrameChanged;
}
