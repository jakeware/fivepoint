#include <iostream>
#include <fstream>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/image_io.h>
#include <cvd/camera.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cvd/glwindow.h>
#include <cvd/gl_helpers.h>
#include "LevelHelpers.h"
#include "CvOrb.h"
#include "FivePoint.h"
#include "Timer.h"

const int nFeatures = 1000;
const int nLevels = 4;
const float fScale = std::sqrt(2.0f);

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

int main()
{
    srand(time(NULL));
    std::cout << "Real data experiment for five point algorithm" << std::endl;

    CVD::Image<CVD::byte> imA = CVD::img_load("../data/X/x0.jpg");
    CVD::Image<CVD::byte> imB = CVD::img_load("../data/X/x04.jpg");

    std::ifstream inFile;
    inFile.open("../data/camera.txt");
    Camera::Linear cam_params;
    cam_params.load(inFile);
    inFile.close();

    std::cout << cam_params.get_parameters() << std::endl;

    std::vector<CVD::Image<CVD::byte> > pyrA, pyrB;
    BuildPyramid(imA,pyrA);
    BuildPyramid(imB,pyrB);

    // Compute descriptors
    CvOrb cv_orbA(pyrA,nFeatures,fScale);
    CvOrb cv_orbB(pyrB,nFeatures,fScale);

    std::vector<std::vector<cv::KeyPoint> > kpA_all, kpB_all;
    std::vector<cv::KeyPoint> kpA, kpB;
    cv::Mat descA, descB;
    cv_orbA.ComputeKeyPoints_QuadTree(kpA_all);
    cv_orbB.ComputeKeyPoints_QuadTree(kpB_all);
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
    for(std::vector<std::vector<cv::DMatch> >::iterator
        it = matches.begin();
        it != matches.end(); ++it)
    {
        std::vector<cv::DMatch> &match = *it;
//        if(match.size() < 2)
//            continue;
//        if(match[0].distance > 44 || match[0].distance/match[1].distance > 0.8)
//            continue;

        const cv::DMatch &m = match[0];

        kpA_filt.push_back(kpA[m.trainIdx]);
        kpB_filt.push_back(kpB[m.queryIdx]);
    }

    // Here's how the putative correspondences look like
    CVD::ImageRef irWindowSize = imA.size();
    irWindowSize.x *= 2;
    CVD::GLWindow glw(irWindowSize, "Putative correspondences");
    glRasterPos2d(0,0);
    CVD::glDrawPixels(imA);
    glRasterPos2d(imA.size().x,0);
    CVD::glDrawPixels(imB);
    glRasterPos2d(0,0);
    std::vector<TwoViewMatch> tv_matches;
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

        TooN::Vector<2> v2a = TooN::makeVector(kpA_filt[i].pt.x,kpA_filt[i].pt.y);
        TooN::Vector<2> v2b = TooN::makeVector(kpB_filt[i].pt.x,kpB_filt[i].pt.y);
        TwoViewMatch tvm;
        tvm.v3a = TooN::unproject(cam_params.unproject(v2a));
        tvm.v3b = TooN::unproject(cam_params.unproject(v2b));

//        std::cout << tvm.v3a << " == " << tvm.v3b << std::endl;
        TooN::normalize(tvm.v3a);
        TooN::normalize(tvm.v3b);

        tv_matches.push_back(tvm);
    }
    glw.swap_buffers();
    std::cin.get();

//    std::ofstream matches_file;
//    matches_file.open("../data/Z/matches_z30.txt");
//    for(size_t i=0; i<tv_matches.size(); ++i)
//    {
//        matches_file << tv_matches[i].v3a << " " << tv_matches[i].v3b << std::endl;
//    }
//    matches_file.close();

//    std::ofstream tErr_file;
//    tErr_file.open("../data/Z/tErr_z30.txt");
    double total_time = 0.0;
    int total_matches = 0;
    for(int i=0; i<10000; ++i)
    {
        std::cout << i << std::endl;
        Timer timer;
        timer.start();
        TwoViewSolver solver;
        solver.Solve(tv_matches, tv_matches, 1);
        timer.end();
        total_time += timer.get_time_ms();
        TooN::Matrix<3> m3E = solver.E_from_SO3s();
        solver.SE3_from_E();
        TooN::SE3<> se3AtoB = solver.SE3_from_E();

        TooN::SE3<> se3_err = se3AtoB * TooN::SE3<>::exp(TooN::makeVector(1,0,0,0,0,0));
        double t_err = TooN::norm(/*se3_err.get_translation()*/se3AtoB.get_translation() - TooN::makeVector(1,0,0));
//        std::cout << t_err << std::endl;
//        tErr_file << t_err << std::endl;

//        Timer timer;
//        timer.start();
//        double score = 0.0;
//        for(size_t pt = 0; pt < tv_matches.size(); ++pt)
//        {
//            double dist1 = tv_matches[pt].v3b * (m3E * tv_matches[pt].v3a);
//            double dist2 = tv_matches[pt].v3a * (m3E.T() * tv_matches[pt].v3b);
//            double dist_sum = dist1 + dist2;
//            if(dist_sum < 1e-4)
//                score += 1.0 - dist_sum/1e-4;
//        }
//        timer.end();
//        total_time += timer.get_time_ms();
    }
    std::cout << "Average time: " << total_time/10000 << "ms" << std::endl;

    return 0;
}
