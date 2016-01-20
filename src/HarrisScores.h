#ifndef HARRISSCORES_H
#define HARRISSCORES_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

void
HarrisScore(const cv::Mat &img, std::vector<cv::KeyPoint> &pts);

#endif // HARRISSCORES_H
