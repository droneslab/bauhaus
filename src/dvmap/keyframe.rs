use std::{collections::HashMap, iter::FromIterator};
use chrono::{DateTime, Utc};
use abow::{BoW, DirectIdx, Vocabulary};
use dvcore::{matrix::{DVVocabulary}, lockwrap::ReadOnlyWrapper};
use nalgebra::{Vector3, RowVector3};
use serde::{Deserialize, Serialize};
use crate::{
    dvmap::{map::Id, pose::Pose, frame::*, sensor::SensorType},
    utils::{imu::*},
};

use super::{mappoint::{MapPoint, FullMapPoint}, keypoints::KeyPointsData, map::{Map}};

unsafe impl<S: SensorType> Sync for KeyFrame<S> {}
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct KeyFrame<S: SensorType> {
    pub id: Id,
    pub timestamp: DateTime<Utc>,
    pub frame_id: Id, // Id of frame it is based on
    pub origin_map_id: Id, // mnOriginMapId
    pub pose: Pose,

    // Map connections ... Parent, children, neighbors
    pub parent: Option<Id>,
    pub children: Vec<Id>,
    neighbors: Vec<Id>, // also sometimes called covisibility keyframes in ORBSLAM3

    // Image //
    pub image_bounds: ImageBounds,

    // KeyPoints, stereo coordinate and descriptors (all associated by an index) //
    pub keypoints_data: S::KeyPointsData,

    // Mappoints
    // Note: u32 is index in array, Id is mappoint Id ... equal to vector in ORBSLAM3
    // because it just allocates an N-size vector and has a bunch of empty entries
    pub mappoint_matches: HashMap::<u32, Id>, // mvpmappoints 

    // BoW
    pub bow_vec: Vec<abow::BoW>, // mBowVec, Bow and featurevector from dbow2
    pub feature_vec: Option<(BoW, DirectIdx)>, // mFeatVec
    // bow: abow::BoW,
    // bow_db: Arc<BowDB>,
    // scale: f64,
    // depth_threshold: f64,

    // Scale //
    pub num_scale_levels: i32, // mnScaleLevels
    pub scale_factor: f64, // mfScaleFactor
    pub log_scale_factor: f64, // mfLogScaleFactor
    pub scale_factors: Vec<f32>, // mvScaleFactors
    // Used in ORBExtractor, which we haven't implemented
    // we're using detect_and_compute from opencv instead
    // pub level_sigma2: Vec<f32>, // mvLevelSigma2

    // IMU //
    // Preintegrated IMU measurements from previous keyframe
    pub prev_kf_id: Option<Id>,
    pub next_kf_id: Option<Id>,
    pub imu_bias: Option<IMUBias>,
    pub imu_preintegrated: Option<IMUPreIntegrated>,
    // pub imu_calib: IMUCalib,

    // Stereo //
    pub stereo_baseline: f64,

    // Sofiya: I think we can clean this up and get rid of these
    // Variables used by KF database
    pub loop_query: u64, //mnLoopQuery
    pub loop_words: i32, //mnLoopWords
    pub reloc_query: u64, //mnRelocQuery
    pub reloc_words: i32, //mnRelocWords
    pub merge_query: u64, //mnMergeQuery
    pub merge_words: i32, //mnMergeWords
    pub place_recognition_query: u64, //mnPlaceRecognitionQuery
    pub place_recognition_words: i32, //mnPlaceRecognitionWords
    pub place_recognition_score: f32, //mPlaceRecognitionScore
    // Variables used by loop closing
    pub mnBAGlobalForKF: u64,
    // Variables used by merging
    pub mnMergeCorrectedForKF: u64,
    pub mnBALocalForMerge: u64,

    // Don't add these in!! read explanations below
    // mnTrackReferenceForFrame ... used in tracking to decide whether to add a kf/mp into tracking's local map. redundant and easy to mess up/get out of sync. Search for this globally to see an example of how to avoid using it.
}

impl<S: SensorType> KeyFrame<S> {
    pub fn new(frame: &Frame<S>, origin_map_id: Id, vocabulary: &DVVocabulary) -> KeyFrame<S> {
        // let grid = frame.grid.clone();
        // sofiya: do I have to set these?
        // mGrid.resize(mnGridCols);
        // if(F.Nleft != -1)  mGridRight.resize(mnGridCols);
        // for(int i=0; i<mnGridCols;i++)
        // {
        //     mGrid[i].resize(mnGridRows);
        //     if(F.Nleft != -1) mGridRight[i].resize(mnGridRows);
        //     for(int j=0; j<mnGridRows; j++){
        //         mGrid[i][j] = F.mGrid[i][j];
        //         if(F.Nleft != -1){
        //             mGridRight[i][j] = F.mGridRight[i][j];
        //         }
        //     }
        // }
        // if(!F.HasVelocity()) {
        //     mVw.setZero();
        //     mbHasVelocity = false;
        // }
        // else
        // {
        //     mVw = F.GetVelocity();
        //     mbHasVelocity = true;
        // }

        let mut kf = KeyFrame {
            id: -1, // id assigned when putting in map
            timestamp: frame.timestamp,
            frame_id: frame.id,
            mappoint_matches: frame.mappoint_matches.clone(),
            pose: frame.pose.unwrap(), // sofiya: should call set_pose()?
            parent: None,
            children: Vec::new(),
            neighbors: Vec::new(),
            keypoints_data: frame.keypoints_data.clone(),
            origin_map_id,
            bow_vec: frame.bow_vec.as_ref().unwrap().clone(),
            feature_vec: Some(frame.feature_vec.as_ref().unwrap().clone()),
            num_scale_levels: frame.num_scale_levels,
            scale_factor: frame.scale_factor,
            log_scale_factor: frame.log_scale_factor,
            scale_factors: frame.scale_factors.clone(),
            image_bounds: frame.image_bounds.clone(),
            prev_kf_id: None,
            next_kf_id: None,
            imu_bias: frame.imu_bias,
            imu_preintegrated: frame.imu_preintegrated,
            stereo_baseline: 0.0,
            loop_query: 0,
            loop_words: 0,
            reloc_query: 0,
            reloc_words: 0,
            merge_query: 0,
            merge_words: 0,
            place_recognition_query: 0,
            place_recognition_words: 0,
            place_recognition_score: 0.0,
            mnBAGlobalForKF: 0,
            mnMergeCorrectedForKF: 0,
            mnBALocalForMerge: 0,
        };

        kf.compute_bow(vocabulary);

        kf
    }

    pub fn set_pose(&mut self, pose: Pose) {
        self.pose = pose.clone();
        // if (mImuCalib.mbIsSet) // TODO IMU Use a flag instead of the OpenCV matrix
        // {
        //     mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
        // }
    }

    pub fn get_neighbors(&self, N: i32) -> Vec<Id> {
        Vec::from_iter(self.neighbors[0..(N as usize)].iter().cloned())
    }

    pub fn compute_bow(&mut self, voc: &DVVocabulary) {
        // sofiya...this is identical to the frame one, can we combine?
        if self.feature_vec.is_none() {
            self.feature_vec = Some(voc.transform_with_direct_idx(self.keypoints_data.descriptors()));
        }
    }

    pub fn add_mappoint(&mut self, mp: &MapPoint<FullMapPoint>, index: u32) {
        // KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
        self.mappoint_matches.insert(index, mp.map_data.id);
    }

    pub fn update_connections(&mut self) {
        todo!("TODO 10/17 fill");
        // map<KeyFrame*,int> KFcounter;

        // vector<MapPoint*> vpMP;

        // {
        //     unique_lock<mutex> lockMPs(mMutexFeatures);
        //     vpMP = mvpMapPoints;
        // }

        // //For all map points in keyframe check in which other keyframes are they seen
        // //Increase counter for those keyframes
        // for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
        // {
        //     MapPoint* pMP = *vit;

        //     if(!pMP)
        //         continue;

        //     if(pMP->isBad())
        //         continue;

        //     map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        //     for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        //     {
        //         if(mit->first->mnId==mnId || mit->first->isBad() || mit->first->GetMap() != mpMap)
        //             continue;
        //         KFcounter[mit->first]++;

        //     }
        // }

        // // This should not happen
        // if(KFcounter.empty())
        //     return;

        // //If the counter is greater than threshold add connection
        // //In case no keyframe counter is over threshold add the one with maximum counter
        // int nmax=0;
        // KeyFrame* pKFmax=NULL;
        // int th = 15;

        // vector<pair<int,KeyFrame*> > vPairs;
        // vPairs.reserve(KFcounter.size());
        // if(!upParent)
        //     cout << "UPDATE_CONN: current KF " << mnId << endl;
        // for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
        // {
        //     if(!upParent)
        //         cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << endl;
        //     if(mit->second>nmax)
        //     {
        //         nmax=mit->second;
        //         pKFmax=mit->first;
        //     }
        //     if(mit->second>=th)
        //     {
        //         vPairs.push_back(make_pair(mit->second,mit->first));
        //         (mit->first)->AddConnection(this,mit->second);
        //     }
        // }

        // if(vPairs.empty())
        // {
        //     vPairs.push_back(make_pair(nmax,pKFmax));
        //     pKFmax->AddConnection(this,nmax);
        // }

        // sort(vPairs.begin(),vPairs.end());
        // list<KeyFrame*> lKFs;
        // list<int> lWs;
        // for(size_t i=0; i<vPairs.size();i++)
        // {
        //     lKFs.push_front(vPairs[i].second);
        //     lWs.push_front(vPairs[i].first);
        // }

        // {
        //     unique_lock<mutex> lockCon(mMutexConnections);

        //     mConnectedKeyFrameWeights = KFcounter;
        //     mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        //     mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());


        //     if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
        //     {
        //         mpParent = mvpOrderedConnectedKeyFrames.front();
        //         mpParent->AddChild(this);
        //         mbFirstConnection = false;
        //     }

        // }
    }
}