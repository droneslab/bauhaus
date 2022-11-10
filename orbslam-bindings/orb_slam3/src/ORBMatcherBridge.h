
#ifndef ORBMATCHERBRIDGE_H
#define ORBMATCHERBRIDGE_H

#include "ORBmatcher.h"
namespace orb_slam3
{

    struct DVKeyPoint;
    struct DVPoint2f;
    struct DVMat;
    struct DVGrid;
    struct VectorOfDVPoint2f;
    struct VectorOfDVi32;

    class ORBMatcherBridge : public ORBmatcher
    {

    public:
            ORBMatcherBridge(int frame_grid_cols, int frame_grid_rows, float minX=0.0, float minY=0.0,  float maxX=0.0, float maxY=0.0,float nnratio=0.6, bool checkOri=true);


        void SearchForInitialization_1(
            const std::vector<orb_slam3::DVKeyPoint>  & F1_mvKeysUn , 
            const std::vector<orb_slam3::DVKeyPoint>  & F2_mvKeysUn, 
            const orb_slam3::DVMat  &F1_mDescriptors,
            const orb_slam3::DVMat  &F2_mDescriptors,
            const orb_slam3::DVGrid  & F2_grid, 
            std::vector<orb_slam3::DVPoint2f>& vbPrevMatched,
            std::vector<int32_t>& vnMatches12,
            int32_t windowSize
        );




        int SearchForInitialization(
        const std::vector<cv::KeyPoint>& F1_mvKeysUn, 
        const std::vector<cv::KeyPoint>& F2_mvKeysUn, 
        const cv::Mat F1_mDescriptors,
        const cv::Mat F2_mDescriptors,
        const std::vector< std::vector <std::vector<size_t> > > F2_grid,
        std::vector<cv::Point2f> &vbPrevMatched, 
        std::vector<int> &vnMatches12, 
        int windowSize=10);


         std::vector<size_t> GetFeaturesInArea(
            const std::vector<cv::KeyPoint> mvKeysUn,
            const std::vector< std::vector <std::vector<size_t> > > mGrid,
            const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    protected:
        float mnMinX;
        float mnMinY;
        float mnMaxX;
        float mnMaxY;
        float mfGridElementWidthInv;
        float mfGridElementHeightInv;
        int mframe_grid_cols;
        int mframe_grid_rows;
    };

    std::unique_ptr<ORBMatcherBridge> new_orb_matcher(
        int frame_grid_cols,
        int frame_grid_rows, 
        float minX, 
        float minY, 
        float maxX,
        float maxY, 
        float nnratio,
        bool checkOri);
}

#endif //ORBMATCHERBRIDGE_H