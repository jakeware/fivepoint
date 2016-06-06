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
