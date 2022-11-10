#include "ORBMatcherBridge.h"


#include "../../../target/cxxbridge/dvos3binding/src/lib.rs.h"


#include "DVMat.h"
#include "DVConvert.h"

namespace orb_slam3
{


    ORBMatcherBridge::ORBMatcherBridge(int frame_grid_cols, int frame_grid_rows, float minX, float minY, float maxX, float maxY, float nnratio, bool checkOri): mframe_grid_cols(frame_grid_cols), mframe_grid_rows(frame_grid_rows), mnMinX(minX), mnMinY(minY), mnMaxX(maxX), mnMaxY(maxY), ORBmatcher(nnratio,checkOri)
    {

        mfGridElementWidthInv=static_cast<float>(mframe_grid_cols)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(mframe_grid_rows)/(mnMaxY-mnMinY);

    }

    void ORBMatcherBridge::SearchForInitialization_1(
            const std::vector<orb_slam3::DVKeyPoint>  & F1_mvKeysUn , 
            const std::vector<orb_slam3::DVKeyPoint>  & F2_mvKeysUn, 
            const orb_slam3::DVMat  &F1_mDescriptors,
            const orb_slam3::DVMat  &F2_mDescriptors,
            const orb_slam3::DVGrid  & F2_grid,
            std::vector<orb_slam3::DVPoint2f>& vbPrevMatched, 
            std::vector<int32_t>& vnMatches12,
            int32_t windowSize
        )
        {
            auto vKeys1_cv = orb_slam3::get_keypoints_const(F1_mvKeysUn);
            auto vKeys2_cv = orb_slam3::get_keypoints_const(F2_mvKeysUn);

            cv::Mat vdesc1_cv = orb_slam3::get_descriptor_const(F1_mDescriptors);
            cv::Mat vdesc2_cv = orb_slam3::get_descriptor_const(F2_mDescriptors);

            // orb_slam3::debug_DVMat(F1_mDescriptors);
            // orb_slam3::debug_DVMat(F2_mDescriptors);

            auto mGrid = orb_slam3::get_grid_const(F2_grid);

            auto prevMatch_cv = orb_slam3::get_vecofpoint2f_const(vbPrevMatched);
            
            // *reinterpret_cast<std::vector<cv::Point2f>*>(&vbPrevMatched);

            SearchForInitialization(
                vKeys1_cv,
                vKeys2_cv,
                vdesc1_cv,
                vdesc2_cv,
                mGrid,
                prevMatch_cv,
                vnMatches12,
                windowSize
            );

        }


    int ORBMatcherBridge::SearchForInitialization(
    const std::vector<cv::KeyPoint>& F1_mvKeysUn, 
    const std::vector<cv::KeyPoint>& F2_mvKeysUn, 
    const cv::Mat F1_mDescriptors,
    const cv::Mat F2_mDescriptors,
    const std::vector< std::vector <std::vector<size_t> > > F2_grid,std::vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, 
    int windowSize)
    {
        //Needs:
        // mvKeysUn
        // mDescriptors
        int nmatches=0;
        vnMatches12 = vector<int>(F1_mvKeysUn.size(),-1);

        vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f/HISTO_LENGTH;

        vector<int> vMatchedDistance(F2_mvKeysUn.size(),INT_MAX);
        vector<int> vnMatches21(F2_mvKeysUn.size(),-1);

        for(size_t i1=0, iend1=F1_mvKeysUn.size(); i1<iend1; i1++)
        {
            cv::KeyPoint kp1 = F1_mvKeysUn[i1];
            int level1 = kp1.octave;
            if(level1>0)
                continue;

            vector<size_t> vIndices2 = GetFeaturesInArea(F2_mvKeysUn, F2_grid, vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

            if(vIndices2.empty())
                continue;

            cv::Mat d1 = F1_mDescriptors.row(i1);

            int bestDist = INT_MAX;
            int bestDist2 = INT_MAX;
            int bestIdx2 = -1;

            for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
            {
                size_t i2 = *vit;

                cv::Mat d2 = F2_mDescriptors.row(i2);

                int dist = DescriptorDistance(d1,d2);

                if(vMatchedDistance[i2]<=dist)
                    continue;

                if(dist<bestDist)
                {
                    bestDist2=bestDist;
                    bestDist=dist;
                    bestIdx2=i2;
                }
                else if(dist<bestDist2)
                {
                    bestDist2=dist;
                }
            }

            if(bestDist<=TH_LOW)
            {
                if(bestDist<(float)bestDist2*mfNNratio)
                {
                    if(vnMatches21[bestIdx2]>=0)
                    {
                        vnMatches12[vnMatches21[bestIdx2]]=-1;
                        nmatches--;
                    }
                    vnMatches12[i1]=bestIdx2;
                    vnMatches21[bestIdx2]=i1;
                    vMatchedDistance[bestIdx2]=bestDist;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = F1_mvKeysUn[i1].angle-F2_mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(i1);
                    }
                }
            }

        }

        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    int idx1 = rotHist[i][j];
                    if(vnMatches12[idx1]>=0)
                    {
                        vnMatches12[idx1]=-1;
                        nmatches--;
                    }
                }
            }

        }

        //Update prev matched
        for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
            if(vnMatches12[i1]>=0)
                vbPrevMatched[i1]=F2_mvKeysUn[vnMatches12[i1]].pt;

        return nmatches;
    }


     std::vector<size_t> ORBMatcherBridge::GetFeaturesInArea(
            const std::vector<cv::KeyPoint> mvKeysUn,
            const std::vector< std::vector <std::vector<size_t> > > mGrid,
            const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
    {
        size_t N = mvKeysUn.size();
        vector<size_t> vIndices;
        vIndices.reserve(N);

        float factorX = r;
        float factorY = r;

        const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
        if(nMinCellX>=mframe_grid_cols)
        {
            return vIndices;
        }

        const int nMaxCellX = min((int)mframe_grid_cols-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
        if(nMaxCellX<0)
        {
            return vIndices;
        }

        const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
        if(nMinCellY>=mframe_grid_rows)
        {
            return vIndices;
        }

        const int nMaxCellY = min((int)mframe_grid_rows-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
        if(nMaxCellY<0)
        {
            return vIndices;
        }

        const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

        for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
        {
            for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy] ; //(!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
                if(vCell.empty())
                    continue;

                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    const cv::KeyPoint &kpUn =  mvKeysUn[vCell[j]] ; //(Nleft == -1) ? mvKeysUn[vCell[j]] : (!bRight) ? mvKeys[vCell[j]]             : mvKeysRight[vCell[j]];
                    if(bCheckLevels)
                    {
                        if(kpUn.octave<minLevel)
                            continue;
                        if(maxLevel>=0)
                            if(kpUn.octave>maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<factorX && fabs(disty)<factorY)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }



    std::unique_ptr<ORBMatcherBridge> new_orb_matcher(
        int frame_grid_cols,
        int frame_grid_rows, 
        float minX, 
        float minY, 
        float maxX,
        float maxY, 
        float nnratio,
        bool checkOri
    )
    {
        return std::unique_ptr<ORBMatcherBridge>(new ORBMatcherBridge(frame_grid_cols, frame_grid_rows, minX, minY, maxX, maxY, nnratio, checkOri));
    }



}
