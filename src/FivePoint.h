#ifndef FIVEPOINT_H
#define FIVEPOINT_H

#include <vector>
#include <TooN/TooN.h>
#include <TooN/se3.h>

struct TwoViewMatch
{
    TooN::Vector<3> v3a;
    TooN::Vector<3> v3b;
};

class TwoViewSolver
{
public:
    TwoViewSolver();

    void
    Solve(std::vector<TwoViewMatch> &matchesAll_vec_,
          std::vector<TwoViewMatch> &matches_vec_,
          int nHypothesis_);

    TooN::Matrix<3>
    E_from_SO3s();

    TooN::SE3<>
    SE3_from_E();

    std::vector<bool>
    GetInlierClassification(){return inliersFlag_vec;}

    void
    Get3DPoints_Cam1(std::vector<TooN::Vector<3> > &XYZa_vec_)
    {
        XYZa_vec_ = XYZa_vec;
    }

    void
    Get3DPoints_Cam2(std::vector<TooN::Vector<3> > &XYZb_vec_)
    {
        XYZb_vec_ = XYZb_vec;
    }

private:

    std::pair<TooN::Vector<3>, TooN::Vector<3> >
    Triangulate_from_SO3s(TooN::SO3<> &R1, TooN::SO3<> &R2,
                          TooN::SO3<> &R1inv, TooN::SO3<> &R2inv,
                          TooN::Vector<3> &v3a, TooN::Vector<3> &v3b);

    void
    ComputeBestEpipoles_RANSAC();

    void
    RefineInliers();

    double
    ComputeScore(const TooN::SO3<> &R1, const TooN::SO3<> &R2,
                 TwoViewMatch &match);

    void
    ComputeEpipoles(std::vector<TwoViewMatch> &vSample,
                    TooN::SO3<> &R1, TooN::SO3<> &R2);

    TooN::Vector<5>
    ComputeUpdate(std::vector<TooN::Vector<3> > &RC1,
                  std::vector<TooN::Vector<3> > &RC2);

    double
    ComputeError(const double A1, const double A2);

    double
    ComputeWeight(const TooN::Vector<3> &RC1, const TooN::Vector<3> &RC2);

    TooN::Vector<3>
    ComputeJacobian(const TooN::Vector<3> v3);

    inline double
    myatan2(const double y, const double x)
    {
        const double c1 = 0.25*M_PI;
        const double c2 = 0.273;
        const double fabsy = fabs(y);
        const double fabsx = fabs(x);
        const double v = (fabsy-fabsx)/(fabsy+fabsx);

        //		double phi = 0.25*M_PI*(1+x) - x*(fabs(x)-1)*(0.2447+0.0663*fabs(x));	// Uncomment me if you need more accuracy
        //const double phi = 0.25*M_PI*(1+x)+0.273*x*(1-fabs(x));
        //double phi = 0.25*M_PI*(1+x);

        const double phi = c1 + v * (c1 + c2*(1-fabs(v)));

        if(x>=0){
            if(y>=0){
                return phi;
            } else {
                return -phi;
            }
        } else {
            if(y>=0){
                return M_PI-phi;
            } else {
                return phi-M_PI;
            }
        }
    }

    TooN::SO3<> so3BestR1;
    TooN::SO3<> so3BestR2;

    std::vector<TwoViewMatch> matchesAll_vec;
    std::vector<TwoViewMatch> matches_vec;
    std::vector<TwoViewMatch> inliers_vec;
    std::vector<bool> inliersFlag_vec;

    std::vector<TooN::Vector<3> > XYZa_vec;
    std::vector<TooN::Vector<3> > XYZb_vec;

    int nHypothesis;
};

#endif // FIVEPOINT_H
