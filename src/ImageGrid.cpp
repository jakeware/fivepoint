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

/**
* This file is part of ORB-SLAM.
* It is based on the file orb.cpp from the OpenCV library (see BSD license below).
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ImageGrid.h"
#include "HarrisScores.h"

ImageGrid::ImageGrid(const int nFastBorder)
    : FAST_BORDER(nFastBorder)
{
}

void ImageGrid::
CreateGrid(float fImageRatio, const int nDesiredFeatures,
           int nXMin, int nXMax, int nYMin, int nYMax)
{
    nLevelCols = std::sqrt((float)nDesiredFeatures/(5*fImageRatio));
    nLevelRows = fImageRatio * nLevelCols;

    const int nTotalW = nXMax - nXMin;
    const int nTotalH = nYMax - nYMin;
    const int nCellW = std::ceil((float)nTotalW/nLevelCols);
    const int nCellH = std::ceil((float)nTotalH/nLevelRows);

    nCells = nLevelCols * nLevelRows;
    nFeaturesPerCell = std::ceil((float)nDesiredFeatures/nCells);
    cells.resize(nLevelRows);

    int nPaddedWidth = nCellW + 2*FAST_BORDER;
    int nPaddedHeight = nCellH + 2*FAST_BORDER;
    int nCurrY = nYMin - FAST_BORDER;
    for(int i = 0; i < nLevelRows; ++i)
    {
        if(i == nLevelRows-1)
            nPaddedHeight = nYMax + FAST_BORDER - nCurrY;

        if(nPaddedHeight <= 0)
            continue;

        int nCurrX = nXMin - FAST_BORDER;
        for(int j = 0; j < nLevelCols; ++j)
        {
            if(j == nLevelCols-1)
                nPaddedWidth = nXMax + FAST_BORDER - nCurrX;

            if(nPaddedWidth <= 0)
                continue;

            Cell grid;
            grid.Create(nCurrX,nCurrY,nPaddedWidth,nPaddedHeight);
            cells[i].push_back(grid);
            nCurrX += nCellW;
        };
        nCurrY += nCellH;
    }
}

void ImageGrid::
Detect(cv::Mat &im,
       const int nFastThresh_max, const int nFastThresh_min,
       ImageGrid::ScoreType score_type)
{
    allKeypoints.resize(nLevelRows);
    nToDistribute = 0;
    nNoMore = 0;
    for(size_t i = 0; i < cells.size(); ++i)
    {
        allKeypoints[i].resize(nLevelCols);
        for(size_t j = 0; j < cells[i].size(); ++j)
        {
            Cell& cell = cells[i][j];
            std::vector<cv::KeyPoint> &cellKeypoints = allKeypoints[i][j];
            cellKeypoints.reserve(nFeaturesPerCell*5);

            cv::Mat sub_image = im.
                    rowRange(cell.nTopLeftY, cell.nTopLeftY + cell.nH).
                    colRange(cell.nTopLeftX, cell.nTopLeftX + cell.nW);
            cv::FAST(sub_image, cellKeypoints, nFastThresh_max, true);

            if(cellKeypoints.size() <= 3)
            {
                cellKeypoints.clear();
                cv::FAST(sub_image, cellKeypoints, nFastThresh_min, true);
            }

            if(score_type==HARRIS_SCORE)
                HarrisScore(sub_image,cellKeypoints);

            const int nKeys = cellKeypoints.size();
            cell.nTotal = nKeys;

            if(nKeys > nFeaturesPerCell)
            {
                cell.bNoMore = false;
                cell.nToRetain = nFeaturesPerCell;
            }
            else
            {
                cell.bNoMore = true;
                cell.nToRetain = nKeys;
                nToDistribute += nFeaturesPerCell - nKeys;
                ++nNoMore;
            }
        };
    }
}

void ImageGrid::
Distribute()
{
    while(nToDistribute > 0 && nNoMore < nCells)
    {
        int nNewFeaturesCell = nFeaturesPerCell + std::ceil((float)nToDistribute/(nCells-nNoMore));
        nToDistribute = 0;

        for(int i=0; i< nLevelRows; ++i)
        {
            for(int j=0; j<nLevelCols; ++j)
            {
                Cell& cell = cells[i][j];
                if(!cell.bNoMore)
                {
                    if(cell.nTotal > nNewFeaturesCell)
                    {
                        cell.nToRetain = nNewFeaturesCell;
                        cell.bNoMore = false;
                    }
                    else
                    {
                        cell.nToRetain = cell.nTotal;
                        nToDistribute += nNewFeaturesCell - cell.nTotal;
                        cell.bNoMore = true;
                        ++nNoMore;
                    }
                }
            };
        }
    }
}

void ImageGrid::
Retain(std::vector<cv::KeyPoint> &keypoints, int level,
       const int nScaledPatchSize, int nDesiredFeatures)
{
    for(int i=0; i<nLevelRows; ++i)
    {
        for(int j=0; j<nLevelCols; ++j)
        {
            Cell& cell = cells[i][j];
            std::vector<cv::KeyPoint> &keysCell = allKeypoints[i][j];
            cv::KeyPointsFilter::retainBest(keysCell, cell.nToRetain);
            if((int)keysCell.size() > cell.nToRetain)
                keysCell.resize(cell.nToRetain);

            for(size_t k = 0, kend = keysCell.size(); k < kend; ++k)
            {
                keysCell[k].pt.x += cells[i][j].nTopLeftX;
                keysCell[k].pt.y += cells[i][j].nTopLeftY;
                keysCell[k].octave = level;
                keysCell[k].size = nScaledPatchSize;
                keypoints.push_back(keysCell[k]);
            }
        };
    }
    if((int)keypoints.size() > nDesiredFeatures)
    {
        cv::KeyPointsFilter::retainBest(keypoints,nDesiredFeatures);
        keypoints.resize(nDesiredFeatures);
    }
}
