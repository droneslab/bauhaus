#ifndef DVCONVERT_H
#define DVCONVERT_H


#include <vector>
#include<opencv2/core/core.hpp>

namespace orb_slam3
{
    struct DVGrid;
    
    
    struct DVKeyPoint;
    struct DVMat;
    struct DVPoint2f;

    std::vector<std::vector<std::vector<size_t>>>  get_grid_const(const DVGrid  &F2_grid);

    std::vector<cv::KeyPoint> get_keypoints_const(const std::vector<DVKeyPoint>  & F1_mvKeysUn);

    cv::Mat get_descriptor_const(const DVMat& desc);

    std::vector<cv::Point2f>  get_vecofpoint2f_const(const std::vector<DVPoint2f>  & vecOfpts);
}//namespace ORB_SLAM

#endif
