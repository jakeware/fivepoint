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

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#ifndef CVORB_H
#define CVORB_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cvd/image.h>
#include <cvd/byte.h>

// Class to detect FAST keypoints and compute ORB descriptors.
// Example usage:
// CVD::Image<CVD::byte> im = CVD::img_load("im.png");
//
// Frame frame;
// frame.BuildPyramid(im);
// CvOrb cv_orb(pyramid, // image pyramid
//              frame.nFeatures(), // no. of desired features
//              frame.fScale());   // scale factor between pyramid levels
// std::vector<cv::KeyPoint> keypoints;
// cv::Mat descriptors
// cv_orb.Compute(keypoints,descriptors);

class CvOrb
{
    friend class CvOrbTest;
public:
    CvOrb(std::vector<CVD::Image<CVD::byte> > &pyr,
          int nFeatures_, float fScale_);

    // Compute ORB descriptors, assuming FAST keypoints have been
    // computed elsewhere in the form of allKeypoints. Also scales
    // the FAST keypoints provided into their root positions in the
    // base pyramid level.
    void
    ComputeWithKeypoints(std::vector<std::vector<cv::KeyPoint> > &allKeypoints,
                         std::vector<cv::KeyPoint> &keypoints,
                         cv::OutputArray descriptors_);

    // Compute FAST keypoints and ORB descriptors in one go
    void
    Compute(std::vector<cv::KeyPoint> &keypoints,
            cv::OutputArray descriptors_);

    // Detect FAST keypoints, and retain the desired
    // number of keypoints for each pyramid level using
    // Harris scores
    void
    ComputeKeyPoints(std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

    // Detect FAST keypoints using an image grids.
    // Every level of the pyramid has its own image grid.
    void
    ComputeKeyPoints_Grid(std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

    // Detect FAST keypoints using quadtrees.
    // Every level of the pyramid will have its
    // own quadtree. The desired number of keypoints
    // for every level is retrieved using a quadtree
    // traversal algorithm.
    void
    ComputeKeyPoints_QuadTree(std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

private:

    const int FAST_BORDER = 3;
    const int PATCH_SIZE = 31;
    const int HALF_PATCH_SIZE = 15;
    const int EDGE_THRESHOLD = 16;

    // Compute number of columns in each row
    // of the circular patch for a Bresenham circle
    // of radius HALF_PATCH_SIZE, to be stored in umax.
    // This is done for the first 45 degrees, then it
    // is reflected across the first quadrant to get the second octane.
    void
    InitBresenhamNumCols();

    void
    CopyOrbBitPattern();

    void
    ComputeOrientation(std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

    // Compute intensity centroid using circular patch
    // centered on point. Compute angle of intensity centroid
    // from point.
    float
    ICAngle(cv::Point2f &pt, cv::Mat &im);

    void
    ComputeDescriptors(cv::Mat &im,
                       std::vector<cv::KeyPoint> &keypoints,
                       cv::Mat &descriptors,
                       const std::vector<cv::Point>& pattern);

    // Compute 256-bit string descriptor. First rotate keypoint
    // to its orientation, then perform BRIEF-style binary tests
    // to fill in the descriptor values.
    void
    ComputeOrbDescriptor(const cv::KeyPoint &kp, const cv::Mat &im,
                         const cv::Point* pattern, uchar* desc);

    std::vector<cv::Mat> ImagePyramid;
    std::vector<int> nFeaturesPerLevel;

    std::vector<cv::Point> pattern;
    std::vector<int> umax;

    bool bPyramidComputed;
    int nFeatures;
    int nLevels;
    float fScale;
    float fOneOverScale;
};

#endif // CVORB_H
