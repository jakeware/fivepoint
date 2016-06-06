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
