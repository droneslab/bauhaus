#ifndef DVMAT_H
#define DVMAT_H


namespace orb_slam3
{

    struct DVMat
    {
        void* ptr;
    };

    void debug_DVMat(const DVMat  &F1_mDescriptors);
}//namespace ORB_SLAM

#endif
