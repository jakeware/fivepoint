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

#include <iostream>
#include <sstream>
#include "FrameGrabber.h"

using namespace std;
using namespace CVD;

FrameGrabber::FrameGrabber(int w, int h, int dev_num)
{
    stringstream ssRes, ssCspace, ssDev;
    ssRes << "[size=" << w << "x" << h << "]";
    ssCspace << "colourspace:[from=yuv422]";
    ssDev << "/dev/video" << dev_num;
    string sVideoSource = ssCspace.str() + "//" + "v4l2:" + ssRes.str() + "//" + ssDev.str();

    vbBuffer = open_video_source<Rgb<byte> >(sVideoSource);
    mirSize = vbBuffer->size();
}

ImageRef FrameGrabber::size()
{
    return mirSize;
}

void FrameGrabber::GrabFrame(Image<Rgb<byte> > &imRGB, Image<byte> &imB)
{
    imRGB.resize(mirSize);
    imB.resize(mirSize);
    vbBuffer->flush();
    VideoFrame<Rgb<byte> > *vfFrame = vbBuffer->get_frame();
    convert_image(*vfFrame, imRGB);
    convert_image(*vfFrame, imB);
    vbBuffer->put_frame(vfFrame);
}
