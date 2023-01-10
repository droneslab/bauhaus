#ifndef CVMAT_H
#define CVMAT_H

#include <opencv2/core/core.hpp>

namespace orb_slam3
{

    struct BindCVMat
    {
        cv::Mat *ptr;
        cv::Mat * operator->() { return ptr; }
        cv::Mat & operator*() { return *ptr; }
        cv::Mat *as_ptr() { return ptr; }
    };

    void debug_DVOS3Mat(const DVOS3Mat  &F1_mDescriptors);
}//namespace ORB_SLAM

#endif
