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
