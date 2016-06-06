# FivePoint #

This repository contains a C++ implementation of an iterative 5-point algorithm for computing the relative pose between two images, given a putative set of point correspondences between the two images. More information regarding this work can be found in our BMVC 2013 paper. If you find the code useful, please cite:

* **An Iterative 5-point Algorithm for Fast and Robust Essential Matrix Estimation**, *Vincent Lui and Tom Drummond*, BMVC 2013.

## 1. Installation

FivePoint has been tested using Ubuntu 14.04 and has the following dependencies:

* TooN
* libCVD
* GVars3
* TaG
* OpenCV

TooN, libcvd, GVars3, and TaG are written by members of the Cambridge Machine Intelligence lab and are licensed under the LGPL license. To install the dependencies for these libraries, type the following command in the terminal:

    sudo apt-get install liblapack-dev libblas-dev libjpeg-dev libpng-dev libtiff-dev

The libraries should be installed in the following order: 1. TooN, 2. libCVD, 3. TaG, 4. GVars3.

For TooN,

    git clone git://github.com/edrosten/TooN.git
    cd /path/to/TooN/folder/
    ./configure
    make
    sudo make install

For libCVD,

    git clone git://github.com/edrosten/libcvd.git
    cd /path/to/libCVD/folder/
    ./configure --without-ffmpeg
    make
    sudo make install

For TaG,

    git clone git://github.com/edrosten/tag.git
    cd /path/to/tag/folder
    ./configure
    make
    sudo make install

For GVars3,

    https://github.com/edrosten/gvars.git
    cd /path/to/GVars3/folder/
    ./configure --disable-widgets
    make
    sudo make install

Finally, to install FivePoint,

    git clone https://whvlui@bitbucket.org/whvlui/fivepoint.git
    cd /path/to/fivepoint/folder
    mkdir build
    cd build
    cmake ..
    make

## 2. Usage

There are two examples showing how to use the 5-point algorithm, one in *main.cpp* and the other in *LiveVideo.cpp*. *main.cpp* shows the 5-point algorithm solving the relative pose problem given two images, whereas *LiveVideo.cpp* is a live video application.

*RealExp.cpp*, *InitExp.cpp*, and *OutlierExp.cpp* are just source files that evaluates the performance of the algorithm. *InitExp.cpp* tests the algorithm under different rotation initializations, *OutlierExp.cpp* tests the algorithm under different outlier ratios, and *RealExp.cpp* tests the algorithm using real data.

All other source files are main just helper files to extract frames, putative point correspondences, and to display the output of the five point algorithm in terms of the pose and triangulated 3D points.