/*
* Copyright (C) 2014 ISIR, UPMC
* Author: Darwin Lau, Mingxing Liu
* email: 
*
* Utility functions for working with ORC/Eigen and Whole Body Interface
*/

#ifndef ORCWBIUTIL_H
#define ORCWBIUTIL_H

#include <Eigen/Lgsm>
#include <wbi/wbi.h>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdRm;

    /* Conversions between types/conventions used in Eigen and WBI */
    class orcWbiConversions
    {
    public:
        static bool eigenDispdToWbiFrame(const Eigen::Displacementd &disp, wbi::Frame &frame);
        static bool wbiFrameToEigenDispd(const wbi::Frame &frame, Eigen::Displacementd &disp);
        static bool eigenRowMajorToColMajor(MatrixXdRm &M_rm, Eigen::MatrixXd &M);
    };

#endif

