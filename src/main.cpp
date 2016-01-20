#include <iostream>
#include <cvd/image_io.h>
#include <cvd/gl_helpers.h>
#include <cvd/camera.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <gvars3/instances.h>
#include "LevelHelpers.h"
#include "CvOrb.h"
#include "FivePoint.h"
#include "GLWindow2.h"

const int nLevels=4;
const float fScale = std::sqrt(2.0f);
const int nFeatures = 1000;

void
BuildPyramid(CVD::Image<CVD::byte> &im,
             std::vector<CVD::Image<CVD::byte> > &pyr)
{
    pyr.resize(nLevels);
    for(int level = 0; level < nLevels; ++level)
    {
        cv::Size sz;
        if(level != 0)
        {
            sz.width = cvRound(lvlhelpers::LevelNPos(im.size().x,fScale,level));
            sz.height = cvRound(lvlhelpers::LevelNPos(im.size().y,fScale,level));
        }
        else
        {
            sz.width = im.size().x;
            sz.height = im.size().y;
        }

        pyr[level].resize(CVD::ImageRef(sz.width,sz.height));
        cv::Mat cv_im(sz.height,sz.width,
                      CV_8UC1,pyr[level].begin());

        if(level != 0)
        {
            cv::Mat cv_im_prevLvl(pyr[level-1].size().y,
                    pyr[level-1].size().x,
                    CV_8UC1,
                    pyr[level-1].begin());
            cv::resize(cv_im_prevLvl,   // src
                       cv_im,           // dst
                       sz,              // dst size
                       0,               // scale factor x-direction. dsize.width/src.cols when = 0
                       0,               // scale factor y-direction. dsize.height/src.rows when = 0
                       cv::INTER_LINEAR); // bilinear interpolation
        }
        else
            pyr[level] = im;
    }
}

void
DrawCamera()
{
    glBegin(GL_LINES);
    glColor3f(1,0,0);
    glVertex3f(0,0,0);
    glVertex3f(0.1,0,0);
    glColor3f(0,1,0);
    glVertex3f(0,0,0);
    glVertex3f(0,0.1,0);
    glColor3f(0,0,1);
    glVertex3f(0,0,0);
    glVertex3f(0,0,0.1);
    glEnd();
}

int main()
{
    srand(time(NULL));
    std::cout << "Iterative 5-point algorithm demo" << std::endl;

    std::ifstream inFile;
    inFile.open("../data/camera.txt");
    Camera::Linear cam_params;
    cam_params.load(inFile);
    inFile.close();

    std::cout << cam_params.get_parameters() << std::endl;


    CVD::Image<CVD::byte> imA = CVD::img_load("../data/kf0.jpg");
    CVD::Image<CVD::byte> imB = CVD::img_load("../data/kf1.jpg");

    std::vector<CVD::Image<CVD::byte> > pyrA, pyrB;
    BuildPyramid(imA,pyrA);
    BuildPyramid(imB,pyrB);

    // Compute descriptors
    CvOrb cv_orbA(pyrA,nFeatures,fScale);
    CvOrb cv_orbB(pyrB,nFeatures,fScale);

    std::vector<std::vector<cv::KeyPoint> > kpA_all, kpB_all;
    std::vector<cv::KeyPoint> kpA, kpB;
    cv::Mat descA, descB;

    cv_orbA.ComputeKeyPoints(kpA_all);
    cv_orbB.ComputeKeyPoints(kpB_all);
    cv_orbA.ComputeWithKeypoints(kpA_all, kpA, descA);
    cv_orbB.ComputeWithKeypoints(kpB_all, kpB, descB);

    // Here is an example of doing feature
    // matching using approximate nearest neighbours
    // using OpenCV's FLANN wrapper
    std::vector<cv::Mat> ref_descriptors(1,descA);
    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(4,   // table number
                                                                10,  // key size
                                                                1)); // multi-probe level
    matcher.add(ref_descriptors);
    matcher.train();

    std::vector<std::vector<cv::DMatch> > matches;
    matcher.knnMatch(descB, matches, 2); // Get the 2 best approximate matches

    std::vector<cv::KeyPoint> kpA_filt, kpB_filt;
    std::vector<TwoViewMatch> tv_matches;
    for(std::vector<std::vector<cv::DMatch> >::iterator
        it = matches.begin();
        it != matches.end(); ++it)
    {
        std::vector<cv::DMatch> &match = *it;
        if(match.size() < 2)
            continue;
        if(match[0].distance > 44 || match[0].distance/match[1].distance > 0.8)
            continue;

        const cv::DMatch &m = match[0];

        kpA_filt.push_back(kpA[m.trainIdx]);
        kpB_filt.push_back(kpB[m.queryIdx]);

        TooN::Vector<2> v2a = TooN::makeVector(kpA_filt.back().pt.x,kpA_filt.back().pt.y);
        TooN::Vector<2> v2b = TooN::makeVector(kpB_filt.back().pt.x,kpB_filt.back().pt.y);
        TwoViewMatch tvm;
        tvm.v3a = TooN::unproject(cam_params.unproject(v2a));
        tvm.v3b = TooN::unproject(cam_params.unproject(v2b));
        TooN::normalize(tvm.v3a);
        TooN::normalize(tvm.v3b);

        tv_matches.push_back(tvm);
    }

    // Here's how the putative correspondences look like
    CVD::ImageRef irWindowSize = imA.size();
    irWindowSize.x *= 2;
    GLWindow2 glw(irWindowSize, "Putative correspondences");
    glRasterPos2d(0,0);
    CVD::glDrawPixels(imA);
    glRasterPos2d(imA.size().x,0);
    CVD::glDrawPixels(imB);
    glRasterPos2d(0,0);
    for(size_t i = 0; i < kpA_filt.size(); ++i)
    {
        glColor3f(0,1,0);
        glPointSize(2);
        glBegin(GL_POINTS);
        glVertex2f(kpA_filt[i].pt.x,kpA_filt[i].pt.y);
        glVertex2f(kpB_filt[i].pt.x+imA.size().x,kpB_filt[i].pt.y);
        glEnd();
        glPointSize(1);

        glColor3f(0,0,1);
        glBegin(GL_LINES);
        glVertex2f(kpA_filt[i].pt.x,kpA_filt[i].pt.y);
        glVertex2f(kpB_filt[i].pt.x+imA.size().x,kpB_filt[i].pt.y);
        glEnd();
    }
    glw.swap_buffers();
    std::cin.get();

    TwoViewSolver solver;
    solver.Solve(tv_matches, tv_matches, 500);
    TooN::SE3<> se3AtoB = solver.SE3_from_E();
    std::vector<bool> inliers_vec = solver.GetInlierClassification();
    std::vector<TooN::Vector<3> > points;
    solver.Get3DPoints_Cam1(points);

    double dist_avg = 0.0;
    for(size_t i =0; i < points.size(); ++i)
        dist_avg += TooN::norm(points[i]);
    dist_avg /= points.size();
    for(size_t i =0; i < points.size(); ++i)
        points[i] /= dist_avg;
    se3AtoB.get_translation() /= dist_avg;

    glw.set_size(CVD::ImageRef(640,480));
    TooN::SE3<> se3ViewerFromOrigin;
    while(1)
    {
        glClearColor(1,1,1,1);
        glClear(GL_COLOR_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(70,1,0.1,50);

        std::pair<TooN::Vector<6>, TooN::Vector<6> >
                pv6 = glw.GetPoseUpdate();
        TooN::SE3<> se3CamFromMC;
        const TooN::Vector<3> v3MC = TooN::Zeros; // TODO: get real mass center from map
        se3CamFromMC.get_translation() = se3ViewerFromOrigin * v3MC;
        se3ViewerFromOrigin = TooN::SE3<>::exp(pv6.first) *
                se3CamFromMC * TooN::SE3<>::exp(pv6.second) *
                se3CamFromMC.inverse() * se3ViewerFromOrigin;

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        CVD::glMultMatrix(TooN::SO3<>::exp(TooN::makeVector(M_PI,0,0)));
        CVD::glMultMatrix(TooN::SE3<>::exp(TooN::makeVector(0,0,5,0,0,0)));
        CVD::glMultMatrix(se3ViewerFromOrigin);

        DrawCamera();
        glPushMatrix();
        CVD::glMultMatrix(se3AtoB.inverse());
        DrawCamera();
        glPopMatrix();

        glPointSize(2);
        glColor3f(0,0,1);
        for(size_t i=0; i<points.size(); ++i)
        {
            glBegin(GL_POINTS);
            CVD::glVertex(points[i]);
            glEnd();
        }
        glPointSize(1);

        glw.swap_buffers();
        glw.HandlePendingEvents();
    }
    std::cout << solver.SE3_from_E().ln() << std::endl;

    return 0;
}
