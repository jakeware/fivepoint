#ifndef IMAGEGRID_H
#define IMAGEGRID_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

// Divides the image into uniformly sized cells
// and detects FAST keypoints for every cell.
// Example usage for a 640x480 image:
// ImageGrid image_grid(3);
// image_grid.CreateGrid(4.f/3.f, 1000, 16, 640-16, 16, 480-16);
// image_grid.Detect(im,20,7,ImageGrid::HARRIS_SCORE);
// image_grid.Distribute();
class ImageGrid
{
public:

    enum ScoreType{HARRIS_SCORE, FAST_SCORE};

    ImageGrid(const int nFastBorder);

    // Creates the cells given the image ratio, the number
    // of desired features, and the dimensions in terms of
    // min x, max x, min y, and max y. The last column may
    // be slightly bigger.
    void
    CreateGrid(float fImageRatio, const int nDesiredFeatures,
               int nXMin, int nXMax, int nYMin, int nYMax);

    // Detect keypoints for every cell.
    // First uses a FAST threshold of nFastThresh_max.
    // If the number of keypoints is too little, attempt
    // to detect keypoints again, using a lower threshold of
    // nFastThresh_min.
    void
    Detect(cv::Mat &im,
           const int nFastThresh_max, const int nFastThresh_min,
           ImageGrid::ScoreType score_type=HARRIS_SCORE);

    // Distribute keypoints around the cells.
    // Tries to get the desired number of keypoints by
    // allowing cells with more number of keypoints than
    // the desired number per cell to retain more points.
    void
    Distribute();

    // For each cell, retain the best keypoints according
    // to either Harris or FAST scores.
    void
    Retain(std::vector<cv::KeyPoint> &keypoints, int level,
           const int nScaledPatchSize, int nDesiredFeatures);

private:

    const int FAST_BORDER;

    int nLevelCols; // number of cols
    int nLevelRows; // number of rows
    int nCellW; // width of every cell
    int nCellH; // height of every cell
    int nCells; // number of cells
    int nFeaturesPerCell; // desired number of features per cell
    int nToDistribute; // number of features to distribute around
    int nNoMore;

    std::vector<std::vector<std::vector<cv::KeyPoint> > > allKeypoints;

    // A small partition of the image.
    struct Cell
    {
        int nTopLeftX;  // top left corner, x-position
        int nTopLeftY;  // top left corner, y-position
        int nW;         // width
        int nH;         // height

        bool bNoMore;   // Flag to indicate if there are any more keypoints to distribute around.
        int nToRetain;  // number of keypoints to retain
        int nTotal;     // total number of keypoints

        // Create a cell, given its dimensions
        void
        Create(int nTopLeftX_, int nTopLeftY_, int nW_, int nH_)
        {
            nTopLeftX = nTopLeftX_;
            nTopLeftY = nTopLeftY_;
            nW = nW_;
            nH = nH_;
        }
    };
    std::vector<std::vector<Cell> > cells;
};

#endif // IMAGEGRID_H
