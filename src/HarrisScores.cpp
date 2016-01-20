#include "HarrisScores.h"

void
HarrisScore(const cv::Mat &img, std::vector<cv::KeyPoint> &pts)
{
    static const int HARRIS_BLOCKSIZE = 7;
    static const float HARRIS_K = 0.04f;

    CV_Assert(img.type() == CV_8UC1 &&
              HARRIS_BLOCKSIZE*HARRIS_BLOCKSIZE <= 2048);

    size_t ptidx, ptsize= pts.size();

    const uchar* ptr00 = img.ptr<uchar>();
    int step = (int)(img.step/img.elemSize1());
    int r = HARRIS_BLOCKSIZE/2;

    float scale = (1 << 2) * HARRIS_BLOCKSIZE * 255.0f;
    scale = 1.0f / scale;
    float scale_sq_sq = scale * scale * scale * scale;

    cv::AutoBuffer<int> ofsbuf(HARRIS_BLOCKSIZE*HARRIS_BLOCKSIZE);
    int* ofs = ofsbuf;
    for(int i = 0; i < HARRIS_BLOCKSIZE; ++i)
        for(int j = 0; j < HARRIS_BLOCKSIZE; ++j)
            ofs[i*HARRIS_BLOCKSIZE + j] = (int)(i*step +j);

    for(ptidx=0; ptidx < ptsize; ++ptidx)
    {
        int x0 = cvRound(pts[ptidx].pt.x - r);
        int y0 = cvRound(pts[ptidx].pt.y - r);

        const uchar* ptr0 = ptr00 + y0*step + x0;
        int a = 0, b = 0, c = 0;

        for(int k = 0; k < HARRIS_BLOCKSIZE*HARRIS_BLOCKSIZE; ++k)
        {
            const uchar* ptr = ptr0 + ofs[k];
            int Ix = (ptr[1] - ptr[-1])*2 + (ptr[-step+1] - ptr[-step-1]) + (ptr[step+1] - ptr[step-1]);
            int Iy = (ptr[step] - ptr[-step])*2 + (ptr[step-1] - ptr[-step-1]) + (ptr[step+1] - ptr[-step+1]);
            a += Ix*Ix;
            b += Iy*Iy;
            c += Ix*Iy;
        };
        pts[ptidx].response = ((float)a * b - (float)c * c -
                               HARRIS_K * ((float)a + b) * ((float)a + b))*scale_sq_sq;
    }
}
