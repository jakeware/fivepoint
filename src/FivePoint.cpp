#include "FivePoint.h"
#include "MEstimator.h"
#include <TooN/wls.h>
#include <tag/five_point.h>

TwoViewSolver::TwoViewSolver()
{
}

void
TwoViewSolver::Solve(std::vector<TwoViewMatch> &matchesAll_vec_,
                     std::vector<TwoViewMatch> &matches_vec_,
                     int nHypothesis_)
{
    matchesAll_vec = matchesAll_vec_;
    matches_vec = matches_vec_;
    nHypothesis = nHypothesis_;

    ComputeBestEpipoles_RANSAC();
    RefineInliers();
}

TooN::Matrix<3>
TwoViewSolver::E_from_SO3s()
{
    TooN::Vector<3> T = TooN::makeVector(0,0,1) * so3BestR2;
    return TooN::cross_product_matrix(T) * (so3BestR2.inverse() * so3BestR1);
}

TooN::SE3<>
TwoViewSolver::SE3_from_E()
{
    TooN::Matrix<3> m3E = E_from_SO3s();
    std::vector<SE3<> > P = tag::se3_from_E(m3E);

    // Triangulate 3D points
    XYZa_vec.reserve(inliers_vec.size());
    XYZb_vec.reserve(inliers_vec.size());
    TooN::SO3<> R1inv = so3BestR1.inverse();
    TooN::SO3<> R2inv = so3BestR2.inverse();
    for(size_t i = 0; i < inliers_vec.size(); ++i)
    {
        std::pair<TooN::Vector<3>, TooN::Vector<3> > p =
                Triangulate_from_SO3s(so3BestR1, so3BestR2,
                                      R1inv, R2inv,
                                      inliers_vec[i].v3a,
                                      inliers_vec[i].v3b);
        XYZa_vec.push_back(p.first);
        XYZb_vec.push_back(p.second);
    }

    TooN::Vector<4> error;
    TooN::Vector<4> votes = TooN::Zeros;
    for(size_t pt = 0; pt < XYZa_vec.size(); pt++){
        for(int i = 0; i < 4; i++){
            error[i] = norm(P[i]*XYZa_vec[pt].slice<0,3>() - XYZb_vec[pt]);
        }

        double min_err = error[0];
        int min_err_idx = 0;
        for(int i = 0; i < 4; i++){
            if(error[i] < min_err){
                min_err = error[i];
                min_err_idx = i;
            }
        }

        votes[min_err_idx]++;
    }

    double max_votes = votes[0];
    int max_vote_idx = 0;
    for(int i = 0; i < 4; i++){
        if(votes[i] > max_votes){
            max_votes = votes[i];
            max_vote_idx = i;
        }
    }

    return P[max_vote_idx];
}

std::pair<TooN::Vector<3>, TooN::Vector<3> >
TwoViewSolver::Triangulate_from_SO3s(TooN::SO3<> &R1, TooN::SO3<> &R2,
                                     TooN::SO3<> &R1inv, TooN::SO3<> &R2inv,
                                     TooN::Vector<3> &v3a, TooN::Vector<3> &v3b)
{
    // Rotate point correspondences
    const TooN::Vector<3> v1 = R1 * v3a;
    const TooN::Vector<3> v2 = R2 * v3b;

    // Normalize v1 and v2 so that x^2 + y^2 = 1
    const double mag_xy1 = sqrt(v1[0]*v1[0] + v1[1]*v1[1]);
    const double mag_xy2 = sqrt(v2[0]*v2[0] + v2[1]*v2[1]);
    const TooN::Vector<3> w1 = v1/mag_xy1;
    const TooN::Vector<3> w2 = v2/mag_xy2;

    // Re-normalize p and p' so that q=1 and q'=1
    const TooN::Vector<3> p1 = w1/fabs(w1[2]-w2[2]);
    const TooN::Vector<3> p2 = w2/fabs(w1[2]-w2[2]);

    // Get 3D points
    TooN::Vector<3> P1 = R1inv * p1;
    TooN::Vector<3> P2 = R2inv * p2;

    return std::pair<TooN::Vector<3>, TooN::Vector<3> >(P1,P2);
}

void
TwoViewSolver::RefineInliers()
{
//	ComputeEpipoles(mvInliers, so3BestR1, so3BestR2);

    // Let' try Tukey style cut-off for point pairs
    // with overly small weights.
    for(int k = 0; k < 25; ++k)
    {
        // First, compute sigma
        std::vector<double> v2Errors;
        std::vector<double> vdErrorSquared;
        std::vector<TooN::Vector<5> > vJacobians;
        std::vector<TooN::Vector<3> > RC1(inliers_vec.size());
        std::vector<TooN::Vector<3> > RC2(inliers_vec.size());
        for(size_t i = 0; i < inliers_vec.size(); ++i)
        {
            RC1[i] = so3BestR1 * inliers_vec[i].v3a;
            RC2[i] = so3BestR2 * inliers_vec[i].v3b;

            double A1 = std::atan2(RC1[i][1], RC1[i][0]);
            double A2 = std::atan2(RC2[i][1], RC2[i][0]);

            double dError = ComputeError(A1,A2);
            double dWeight = ComputeWeight(RC1[i], RC2[i]);
            v2Errors.push_back(dError);
            vdErrorSquared.push_back(dError * dError * dWeight);

            TooN::Vector<5> J;
            J.slice<0,3>() = ComputeJacobian(RC1[i]);
            J.slice<3,2>() = - ComputeJacobian(RC2[i]).slice(0,2);
            vJacobians.push_back(J);
        }

        std::vector<double> vdd = vdErrorSquared;
        double dSigmaSquared = Tukey::FindSigmaSquared(vdd);

        // Add re-weighted measurements
        TooN::WLS<5> wls;
        for(size_t i = 0; i < inliers_vec.size(); ++i)
        {
            double t_weight = Tukey::Weight(vdErrorSquared[i], dSigmaSquared);
            wls.add_mJ(v2Errors[i], vJacobians[i], t_weight);
        }
        wls.add_prior(1e-5);
        wls.compute();
        TooN::Vector<5> v5up = wls.get_mu();
        TooN::Vector<3> r1up = -v5up.slice<0,3>();
        TooN::Vector<3> r2up = TooN::makeVector(-v5up[3], -v5up[4], 0);
        so3BestR1 = SO3<>::exp(r1up) * so3BestR1;
        so3BestR2 = SO3<>::exp(r2up) * so3BestR2;
    }
}

void
TwoViewSolver::ComputeBestEpipoles_RANSAC()
{
    const int n_minimal = 7;
    std::vector<int> indices(n_minimal,-1);

    double dBestScore = 0.0;
    inliersFlag_vec.resize(matchesAll_vec.size(), false);
    for(int nH = 0; nH < nHypothesis; ++nH)
    {
        // Draw unique point pairs randomly
        // from point pairs filtered using match thresh
        for(int i = 0; i < n_minimal; ++i)
        {
            bool isUnique = false;
            int n;
            while(!isUnique)
            {
                n = rand() % matches_vec.size();
                isUnique = true;
                for(int j = 0; j < i && isUnique; ++j)
                    if(indices[j] == n)
                        isUnique = false;
            };
            indices[i] = n;
        }

        std::vector<TwoViewMatch> vSample;
        for(int i = 0; i < n_minimal; ++i)
            vSample.push_back(matches_vec[indices[i]]);

        TooN::SO3<> R1, R2;
        ComputeEpipoles(vSample, R1, R2);

        // Compute consensus
        double dTotalScore = 0.0;
        std::vector<bool> candidate_inliers(matchesAll_vec.size(), false);
        for(size_t i = 0; i < matchesAll_vec.size(); ++i)
        {
            double dScore = ComputeScore(R1, R2, matchesAll_vec[i]);
            dTotalScore += dScore;
            if(dScore > 0.0)
                candidate_inliers[i] = true;
        }

        if(dTotalScore > dBestScore)
        {
            dBestScore = dTotalScore;
            so3BestR1 = R1;
            so3BestR2 = R2;
            inliersFlag_vec = candidate_inliers;
        }
    }

    for(size_t i = 0; i < inliersFlag_vec.size(); ++i)
    {
        if(inliersFlag_vec[i] == true)
            inliers_vec.push_back(matchesAll_vec[i]);
    }
}

double
TwoViewSolver::ComputeScore(const TooN::SO3<> &R1, const TooN::SO3<> &R2,
                            TwoViewMatch &match)
{
    const double dThresh = 1e-5;
    TooN::Vector<3> RC1 = R1 * match.v3a;
    TooN::Vector<3> RC2 = R2 * match.v3b;

    double A1 = std::atan2(RC1[1],RC1[0]);
    double A2 = std::atan2(RC2[1],RC2[0]);

    double dErr = ComputeError(A1,A2);
    double dWeight = ComputeWeight(RC1,RC2);
    double dWeightedError = fabs(dErr*dErr) * dWeight;
    if(dWeightedError < dThresh)
        return 1.0 - (dWeightedError / dThresh);
    else
        return 0.0;
}

void
TwoViewSolver::ComputeEpipoles(std::vector<TwoViewMatch> &vSample,
                               TooN::SO3<> &R1, TooN::SO3<> &R2)
{
    int n = vSample.size();
    std::vector<TooN::Vector<3> > RC1(n);
    std::vector<TooN::Vector<3> > RC2(n);

    for(int i = 0; i < 25; ++i)
    {
        for(int pt = 0; pt < n; ++pt)
        {
            RC1[pt] = R1 * vSample[pt].v3a;
            RC2[pt] = R2 * vSample[pt].v3b;
        }

        TooN::Vector<5> v5up = ComputeUpdate(RC1, RC2);
        TooN::Vector<3> r1up = -v5up.slice<0,3>();
        TooN::Vector<3> r2up = TooN::makeVector(-v5up[3], -v5up[4], 0);
        R1 = TooN::SO3<>::exp(r1up) * R1;
        R2 = TooN::SO3<>::exp(r2up) * R2;
    }
}

TooN::Vector<5>
TwoViewSolver::ComputeUpdate(std::vector<TooN::Vector<3> > &RC1,
                             std::vector<TooN::Vector<3> > &RC2)
{
    TooN::WLS<5> wls;
    for(size_t pt = 0; pt < RC1.size(); ++pt)
    {
        double A1 = atan2((RC1[pt])[1],(RC1[pt])[0]);
        double A2 = atan2((RC2[pt])[1],(RC2[pt])[0]);

        double dErr = ComputeError(A1, A2);
        double dWeight = ComputeWeight(RC1[pt], RC2[pt]);

        TooN::Vector<5> J;
        J.slice<0,3>() = ComputeJacobian(RC1[pt]);
        J.slice<3,2>() = - ComputeJacobian(RC2[pt]).slice(0,2);

        wls.add_mJ(dErr, J, dWeight);
    }

    wls.add_prior(1e-5);
    wls.compute();

    return wls.get_mu();
}

double
TwoViewSolver::ComputeError(const double A1, const double A2)
{
    double result = A1 - A2;
    if(result > M_PI){
        result -= 2*M_PI;
    }
    else if(result < -M_PI){
        result += 2*M_PI;
    }
    return result;
}

double
TwoViewSolver::ComputeWeight(const TooN::Vector<3> &RC1,
                             const TooN::Vector<3> &RC2)
{
    double d1 = RC1.slice<0,2>()*RC1.slice<0,2>();
    double d2 = RC2.slice<0,2>()*RC2.slice<0,2>();
    double w = 2.0 / (1.0/(d1) + 1.0/(d2));

    return w;
}

TooN::Vector<3>
TwoViewSolver::ComputeJacobian(const TooN::Vector<3> v3)
{
    double sumsq = (v3[0]*v3[0] + v3[1]*v3[1]);

    TooN::Vector<3> result;
    for (int i = 0; i < 2; ++i)
        result[i] = - (v3[i]*v3[2])/sumsq;
    result[2] = 1;

    return result;
}
