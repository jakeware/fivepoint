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
