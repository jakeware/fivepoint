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

#include <iostream>
#include <fstream>
#include <map>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <cvd/camera.h>
#include <cvd/random.h>
#include "FivePoint.h"
#include "Timer.h"

const int nPoints = 100;

struct SyntMeas
{
    TooN::Vector<2> v2;
    TooN::Vector<2> v2_noisy;
};

struct SyntLandmark
{
    TooN::Vector<3> v3xyz;
    SyntMeas measA;
    SyntMeas measB;
};

int main()
{
    std::cout << "Initialization experiment" << std::endl;
    srand(time(NULL));

    Camera::Linear cam_params;
    cam_params.get_parameters() = TooN::makeVector(600,600,320,240);

    // Generate synthetic dataset
    TooN::SE3<> se3AtoB = TooN::SE3<>::exp(TooN::makeVector(0.0,0,0.1,0,0,0));
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

            points.push_back(lm);
            TooN::Vector<3> v3a = TooN::unproject(cam_params.unproject(lm.measA.v2_noisy));
            TooN::Vector<3> v3b = TooN::unproject(cam_params.unproject(lm.measB.v2_noisy));
            TooN::normalize(v3a);
            TooN::normalize(v3b);

            TwoViewMatch tvm;
            tvm.v3a = v3a;
            tvm.v3b = v3b;
            tvm_vec.push_back(tvm);
        }
    }

    std::stringstream ss;
    ss << "error_dist_" << noise_sigma << ".txt";

    std::ofstream outFile;
    outFile.open(ss.str());
    int max_k=10000;
    double total_time = 0.0;
    for(int k=0; k<max_k; ++k)
    {
        std::cout << "k: " << k << std::endl;
        TwoViewSolver solver;
        Timer timer;
        timer.start();
        solver.Solve(tvm_vec,tvm_vec,1);
        timer.end();
        total_time += timer.get_time_ms();
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

    std::cout << "Average time to generate one hypothesis: " << total_time / max_k << std::endl;

    return 0;
}
