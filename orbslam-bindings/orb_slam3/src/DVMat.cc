#include "DVMat.h"
#include<opencv2/core/core.hpp>

namespace orb_slam3
{

    // API to debug DVMat
    void debug_DVMat(const DVMat  &F1_mDescriptors)
    {
            cv::Mat vdesc1_cv = *reinterpret_cast<const cv::Mat *>(&F1_mDescriptors);
            
            for (int i=0;i<vdesc1_cv.rows;i++)
            {                
                printf("%u\n", (unsigned int) *vdesc1_cv.ptr(i,0));
            }        
    }

}