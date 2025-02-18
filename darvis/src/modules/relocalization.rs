use derivative::Derivative;
use core::{config::{SETTINGS, SYSTEM}, system::Timestamp};
use crate::{map::{frame::Frame, map::{Id, Map}}, registered_actors::FEATURE_MATCHING_MODULE};

use super::module_definitions::RelocalizationModule;

#[derive(Debug, Clone, Derivative)]
#[derivative(Default)]
pub struct Relocalization {
    pub last_reloc_frame_id: Id,
    pub timestamp_lost: Option<Timestamp>,
}
impl RelocalizationModule for Relocalization {
    type Frame = Frame;
    type Timestamp = Timestamp;
    type Map = Map;

    fn run(&self, current_frame: &mut Frame, map: &Map) -> Result<bool, Box<dyn std::error::Error>> {
        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        // let candidate_kfs = map.detect_relocalization_candidates(current_frame);

        // if candidate_kfs.is_empty() {
        //     return Ok(false);
        // }

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver

        // vector<MLPnPsolver*> vpMLPnPsolvers;
        // vpMLPnPsolvers.resize(nKFs);

        // vector<vector<MapPoint*> > vvpMapPointMatches;
        // vvpMapPointMatches.resize(nKFs);

        // vector<bool> vbDiscarded;
        // vbDiscarded.resize(nKFs);

        // int nCandidates=0;

        // for candidate_kf_id in candidate_kfs {
        //     let candidate_kf = map.get_keyframe(candidate_kf_id);
        //     let nmatches = FEATURE_MATCHING_MODULE.search_by_bow_with_frame(candidate_kf, current_frame, true, 0.7)?;

        //     int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
        //     if(nmatches<15)
        //     {
        //         vbDiscarded[i] = true;
        //         continue;
        //     }
        //     else
        //     {
        //         MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
        //         pSolver->SetRansacParameters(0.99,10,300,6,0.5,5.991);  //This solver needs at least 6 points
        //         vpMLPnPsolvers[i] = pSolver;
        //         nCandidates++;
        //     }
        // }

        // // Alternatively perform some iterations of P4P RANSAC
        // // Until we found a camera pose supported by enough inliers
        // bool bMatch = false;
        // ORBmatcher matcher2(0.9,true);

        // while(nCandidates>0 && !bMatch)
        // {
        //     for(int i=0; i<nKFs; i++)
        //     {
        //         if(vbDiscarded[i])
        //             continue;

        //         // Perform 5 Ransac Iterations
        //         vector<bool> vbInliers;
        //         int nInliers;
        //         bool bNoMore;

        //         MLPnPsolver* pSolver = vpMLPnPsolvers[i];
        //         Eigen::Matrix4f eigTcw;
        //         bool bTcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers, eigTcw);

        //         // If Ransac reachs max. iterations discard keyframe
        //         if(bNoMore)
        //         {
        //             vbDiscarded[i]=true;
        //             nCandidates--;
        //         }

        //         // If a Camera Pose is computed, optimize
        //         if(bTcw)
        //         {
        //             Sophus::SE3f Tcw(eigTcw);
        //             mCurrentFrame.SetPose(Tcw);
        //             // Tcw.copyTo(mCurrentFrame.mTcw);

        //             set<MapPoint*> sFound;

        //             const int np = vbInliers.size();

        //             for(int j=0; j<np; j++)
        //             {
        //                 if(vbInliers[j])
        //                 {
        //                     mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
        //                     sFound.insert(vvpMapPointMatches[i][j]);
        //                 }
        //                 else
        //                     mCurrentFrame.mvpMapPoints[j]=NULL;
        //             }

        //             int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

        //             if(nGood<10)
        //                 continue;

        //             for(int io =0; io<mCurrentFrame.N; io++)
        //                 if(mCurrentFrame.mvbOutlier[io])
        //                     mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

        //             // If few inliers, search by projection in a coarse window and optimize again
        //             if(nGood<50)
        //             {
        //                 int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

        //                 if(nadditional+nGood>=50)
        //                 {
        //                     nGood = Optimizer::PoseOptimization(&mCurrentFrame);

        //                     // If many inliers but still not enough, search by projection again in a narrower window
        //                     // the camera has been already optimized with many points
        //                     if(nGood>30 && nGood<50)
        //                     {
        //                         sFound.clear();
        //                         for(int ip =0; ip<mCurrentFrame.N; ip++)
        //                             if(mCurrentFrame.mvpMapPoints[ip])
        //                                 sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
        //                         nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

        //                         // Final optimization
        //                         if(nGood+nadditional>=50)
        //                         {
        //                             nGood = Optimizer::PoseOptimization(&mCurrentFrame);

        //                             for(int io =0; io<mCurrentFrame.N; io++)
        //                                 if(mCurrentFrame.mvbOutlier[io])
        //                                     mCurrentFrame.mvpMapPoints[io]=NULL;
        //                         }
        //                     }
        //                 }
        //             }


        //             // If the pose is supported by enough inliers stop ransacs and continue
        //             if(nGood>=50)
        //             {
        //                 bMatch = true;
        //                 break;
        //             }
        //         }
        //     }
        // }

        // if(!bMatch)
        // {
        //     return false;
        // }
        // else
        // {
        //     mnLastRelocFrameId = mCurrentFrame.mnId;
        //     cout << "Relocalized!!" << endl;
        //     return true;
        // }



        todo!("Relocalization");
    }
    fn sec_since_lost(&self, current_frame: &Frame) -> Timestamp {
        current_frame.timestamp - self.timestamp_lost.unwrap()
    }

    fn frames_since_lost(&self, current_frame: &Frame) -> i32 {
        current_frame.frame_id - self.last_reloc_frame_id
    }
}

impl Relocalization {
    pub fn _past_cutoff(&self, current_frame: &Frame) -> bool {
        self.sec_since_lost(current_frame) > SETTINGS.get::<i32>(SYSTEM, "recently_lost_cutoff") as f64
    }

}