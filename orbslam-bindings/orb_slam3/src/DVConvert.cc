#include "DVConvert.h"

#include "../../../target/cxxbridge/dvos3binding/src/lib.rs.h"


namespace orb_slam3
{

        // API to debug DVMat
    std::vector<std::vector<std::vector<size_t>>>  get_grid_const(const orb_slam3::DVGrid  &F2_grid)
    {
        std::vector<std::vector<std::vector< size_t >>> mGrid ;  

        mGrid.resize(F2_grid.vec.size());

        int i=0;
        for (auto shared : F2_grid.vec) {
            mGrid[i].resize(shared.vec.size());
            int j=0;

            for (auto shared1 : shared.vec) {

                std::copy(shared1.vec.begin(), shared1.vec.end(), std::back_inserter(mGrid[i][j]));

                j++;
            }
            i++;

        }
        return mGrid;
    
    }

    std::vector<cv::KeyPoint> get_keypoints_const(const std::vector<DVKeyPoint>  & F1_mvKeysUn)
    {
        return *reinterpret_cast<const std::vector<cv::KeyPoint>*>(&F1_mvKeysUn);
    }

   cv::Mat get_descriptor_const(const DVMat& desc)
    {
        return *reinterpret_cast<const cv::Mat *>(&desc);
    }

    std::vector<cv::Point2f>  get_vecofpoint2f_const(const std::vector<DVPoint2f>  & vecOfpts)
    {
        return *reinterpret_cast<const std::vector<cv::Point2f>*>(&vecOfpts);
    }

}