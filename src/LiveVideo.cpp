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

#include <cvd/glwindow.h>
#include <cvd/gl_helpers.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <gvars3/instances.h>
#include "LiveVideo.h"
#include "FrameGrabber.h"
#include "CvOrb.h"
#include "FivePoint.h"
#include "GLWindow2.h"
#include "LevelHelpers.h"

System
::System()
{
    GVars3::GUI.RegisterCommand("quit",GUICommandCallBack,this);
    GVars3::GUI.RegisterCommand("Keypress",GUICommandCallBack,this);
    bQuit=false;
    bSpacebar=false;
    bInit=false;
    selection = System::FRAMEZERO;

    nLevels=4;
    fScale=std::sqrt(2.0f);

    std::ifstream inFile;
    inFile.open("../data/camera.txt");
    cam_params.load(inFile);
    inFile.close();

    std::cout << cam_params.get_parameters() << std::endl;
}

void System
::Run()
{
    FrameGrabber grabber(640,480,1);
    GLWindow2 glw(CVD::ImageRef(640*2,480),"Window");
    TooN::SE3<> se3ViewerFromOrigin;
    while(!bQuit)
    {
        CVD::Image<CVD::Rgb<CVD::byte> > imRGB;
        CVD::Image<CVD::byte> imB;
        grabber.GrabFrame(imRGB, imB);

        std::vector<CVD::Image<CVD::byte> > pyr;
        BuildPyramid(imB, pyr);
        CvOrb cv_orb(pyr, 3000, std::sqrt(2.0f));
        cv_orb.Compute(kp_live,desc_live);
        if(bSpacebar)
        {
            if(selection != System::FRAMEONE)
            {
                kpA = kp_live;
                desc_live.copyTo(descA);
            }

            if(selection == System::FRAMEZERO)
                selection = System::FRAMEONE;
            else if(selection == System::FRAMEONE)
                selection = System::FRAMETWO;
            else if(selection == System::FRAMETWO)
                selection = System::FRAMEONE;

            std::cout << "selection state: " <<  selection << std::endl;

            bSpacebar = false;
            if(bInit == false)
                bInit=true;
        }

        std::vector<cv::KeyPoint> kpA_filt, kpB_filt;
        std::vector<TooN::Vector<2> > v2a_vec, v2b_vec;
        std::vector<TwoViewMatch> tvm_vec;
        std::vector<TooN::Vector<3> > points;
        if(selection != System::FRAMEZERO)
        {
            std::vector<cv::Mat> ref_descriptors(1,descA);
            cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(4,   // table number
                                                                        10,  // key size
                                                                        1)); // multi-probe level
            matcher.add(ref_descriptors);
            matcher.train();

            std::vector<std::vector<cv::DMatch> > matches;
            matcher.knnMatch(desc_live, matches, 2); // Get the 2 best approximate matches

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
                kpB_filt.push_back(kp_live[m.queryIdx]);
                TwoViewMatch tvm;

                TooN::Vector<2> v2a = TooN::makeVector(kpA_filt.back().pt.x,kpA_filt.back().pt.y);
                TooN::Vector<2> v2b = TooN::makeVector(kpB_filt.back().pt.x,kpB_filt.back().pt.y);
                tvm.v3a = TooN::unproject(cam_params.unproject(v2a));
                tvm.v3b = TooN::unproject(cam_params.unproject(v2b));
                TooN::normalize(tvm.v3a);
                TooN::normalize(tvm.v3b);
                tvm_vec.push_back(tvm);
                v2a_vec.push_back(v2a);
                v2b_vec.push_back(v2b);
            }
        }

        TooN::SE3<> se3AtoB;
        if(selection == System::FRAMETWO)
        {
            TwoViewSolver solver;
            solver.Solve(tvm_vec,tvm_vec,1024);
            se3AtoB = solver.SE3_from_E();
            solver.Get3DPoints_Cam1(points);

            double dist_avg = 0.0;
            for(size_t i=0; i<points.size(); ++i)
                dist_avg += TooN::norm(points[i]);
            dist_avg /= points.size();

            for(size_t i=0; i<points.size(); ++i)
                points[i] /= dist_avg;

            se3AtoB.get_translation() = se3AtoB.get_translation()/dist_avg;
        }

        glClearColor(1,1,1,1);
        glClear(GL_COLOR_BUFFER_BIT);
        glw.SetupViewport(CVD::ImageRef(0,0),CVD::ImageRef(640,480));
        glw.SetupWindowOrtho(CVD::ImageRef(640,480));
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glEnable(GL_POINT_SMOOTH);
        glEnable(GL_LINE_SMOOTH);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColorMask(1,1,1,1);
        glDrawPixels(imB);
        glPointSize(2);
        for(size_t i=0; i<kp_live.size(); ++i)
        {
            glColor3f(0,1,0);
            glBegin(GL_POINTS);
            glVertex2f(kp_live[i].pt.x,kp_live[i].pt.y);
            glEnd();
        }
        for(size_t i=0; i<v2a_vec.size(); ++i)
        {
            glColor3f(0,0,1);
            glBegin(GL_LINES);
            glVertex2f(v2a_vec[i][0],v2a_vec[i][1]);
            glVertex2f(v2b_vec[i][0],v2b_vec[i][1]);
            glEnd();
        }
        glPointSize(1);

        glw.SetupViewport(CVD::ImageRef(640,0),CVD::ImageRef(640,480));
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

        glColor3f(0,0,1);
        glPointSize(2);
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
}

void System
::BuildPyramid(CVD::Image<CVD::byte> &im,
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

void System
::DrawCamera()
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

void System
::GUICommandCallBack(void *ptr, std::string sCommand, std::string sParams)
{
    static_cast<System*>(ptr)->GUICommandHandler(sCommand,sParams);
}

void System
::GUICommandHandler(std::string sCommand, std::string sParams)
{
    if(sCommand=="quit")
        bQuit=true;
    if(sCommand == "Keypress" && sParams == "Space")
        bSpacebar=true;
}

int main()
{
    std::cout << "Live demo for iterative 5-point algorithm" << std::endl;

    System system;
    system.Run();

    return 0;
}
