#include <iostream>
#include <fstream>
#include <map>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <cvd/camera.h>
#include <cvd/random.h>
#include "FivePoint.h"

const int nPoints = 100;

struct SyntMeas
{
    TooN::Vector<2> v2;
    TooN::Vector<2> v2_noisy;
    TooN::Vector<2> v2_outlier;
    bool bOutlier;
};

struct SyntLandmark
{
    TooN::Vector<3> v3xyz;
    SyntMeas measA;
    SyntMeas measB;
};

int main()
{
    std::cout << "Outlier experiment" << std::endl;
    srand(time(NULL));

    Camera::Linear cam_params;
    cam_params.get_parameters() = TooN::makeVector(600,600,320,240);

    // Generate synthetic dataset
    TooN::SE3<> se3AtoB = TooN::SE3<>::exp(TooN::makeVector(0.1,0,0,0,0,0));
    std::vector<SyntLandmark> points;
    std::vector<TwoViewMatch> tvm_vec;
    double noise_sigma = 0.1;
    for(int i=0; i<nPoints; ++i)
    {
        bool bSuccess = false;
        while(!bSuccess)
        {
            double x = drand48() * 5.0 - 2.5;
            double y = drand48() * 5.0 - 2.5;
            double z = drand48() * 0.5 + 0.5;

            SyntLandmark lm;
            lm.v3xyz = TooN::makeVector(x,y,z);
            lm.measA.v2_noisy = lm.measA.v2 = cam_params.project(TooN::project(lm.v3xyz));
            lm.measB.v2_noisy = lm.measB.v2 = cam_params.project(TooN::project(se3AtoB.inverse()*lm.v3xyz));
            if(lm.measA.v2[0] < 0 || lm.measA.v2[1] < 0 ||
                    lm.measA.v2[0] > 640 || lm.measA.v2[1] > 480)
                continue;
            if(lm.measB.v2[0] < 0 || lm.measB.v2[1] < 0 ||
                    lm.measB.v2[0] > 640 || lm.measB.v2[1] > 480)
                continue;
            bSuccess = true;

            lm.measA.v2_noisy[0] += CVD::rand_g()*noise_sigma;
            lm.measA.v2_noisy[1] += CVD::rand_g()*noise_sigma;
            lm.measB.v2_noisy[0] += CVD::rand_g()*noise_sigma;
            lm.measB.v2_noisy[1] += CVD::rand_g()*noise_sigma;
            lm.measA.bOutlier = lm.measB.bOutlier = false;

            points.push_back(lm);
        }
    }

    // Create outliers
    double dOutlierFrac = 0.3;
    for(int i=0; i<dOutlierFrac*nPoints; ++i)
    {
        bool bCreated = false;
        while(!bCreated)
        {
            int idx = drand48()*nPoints;
            SyntLandmark &lm = points.at(idx);
            if(lm.measB.bOutlier)
                continue;
            std::cout << idx << std::endl;

            lm.measB.v2_outlier = TooN::makeVector(drand48()*640,
                                                   drand48()*480);
            lm.measB.bOutlier = true;
            bCreated = true;
        }
    }
    std::cout << "Created outliers" << std::endl;

    for(int i=0; i<nPoints; ++i)
    {
        SyntLandmark &lm = points.at(i);
        TooN::Vector<3> v3a = TooN::unproject(cam_params.unproject(lm.measA.v2_noisy));
        TooN::Vector<3> v3b;
        if(lm.measB.bOutlier)
            v3b = TooN::unproject(cam_params.unproject(lm.measB.v2_outlier));
        else
            v3b = TooN::unproject(cam_params.unproject(lm.measB.v2_noisy));
        TooN::normalize(v3a);
        TooN::normalize(v3b);

        TwoViewMatch tvm;
        tvm.v3a = v3a;
        tvm.v3b = v3b;
        tvm_vec.push_back(tvm);
    }

    std::stringstream ss;
    ss << "error_dist_" << noise_sigma << ".txt";

    std::ofstream outFile;
    outFile.open(ss.str());
    int max_k=10000;
    for(int k=0; k<max_k; ++k)
    {
        std::cout << "k: " << k << std::endl;
        TwoViewSolver solver;
        solver.Solve(tvm_vec,tvm_vec,1);
        TooN::Matrix<3> m3E = solver.E_from_SO3s();
        TooN::SE3<> se3_pred;
        if(!std::isnan(m3E(0,0)))
            se3_pred = solver.SE3_from_E();
        else
            continue;

        TooN::SE3<> se3_truth_inv = se3AtoB;
        TooN::normalize(se3_truth_inv.get_translation());

        TooN::SE3<> se3_err = se3_truth_inv * se3_pred;
        double t_err = TooN::norm(se3_err.get_translation());
        double r_err = TooN::norm(se3_err.get_rotation().ln());

        outFile << t_err << " " << r_err << std::endl;
    }
    outFile.close();

    return 0;
}
