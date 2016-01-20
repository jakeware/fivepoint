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
