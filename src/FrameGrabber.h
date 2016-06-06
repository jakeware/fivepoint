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

#ifndef FRAMEGRABBER_H
#define FRAMEGRABBER_H

#endif // FRAMEGRABBER_H

#include <cvd/videosource.h>

class FrameGrabber
{
public:
    FrameGrabber(int w, int h, int dev_num);
    CVD::ImageRef size();
    void GrabFrame(CVD::Image<CVD::Rgb<CVD::byte> > &imRGB, CVD::Image<CVD::byte> &imB);

private:
    CVD::VideoBuffer<CVD::Rgb<CVD::byte> > *vbBuffer;
    CVD::ImageRef mirSize;
};
