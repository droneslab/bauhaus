use std::collections::HashMap;
use chrono::{DateTime, Utc};
use abow::{BoW, DirectIdx};
use serde::{Deserialize, Serialize};
use crate::{
    map::{
        map::Id, pose::Pose, 
        frame::{Frame, ImageBounds,},
    },
    utils::{imu::*, sensor::SensorType},
};

unsafe impl<S: SensorType> Sync for KeyFrame<S> {}
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct KeyFrame<S: SensorType> {
    pub id: Id,
    pub timestamp: DateTime<Utc>,
    pub frame_id: Id, // Id of frame it is based on
    pub origin_map_id: Id, // mnOriginMapId
    pub pose: Pose,

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

    // Sofiya TODO: I think we can clean this up and get rid of these
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
}

impl<S: SensorType> KeyFrame<S> {
    pub fn new(frame: &Frame<S>, current_map_id: Id) -> KeyFrame<S> {
        // let grid = frame.grid.clone();
        // sofiya todo: do I have to set these?
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

        KeyFrame {
            id: -1, // id assigned when putting in map
            timestamp: frame.timestamp,
            frame_id: frame.id,
            mappoint_matches: frame.mappoint_matches.clone(),
            pose: frame.pose.unwrap(), // sofiya...should call set_pose()?
            keypoints_data: frame.keypoints_data.clone(),
            origin_map_id: current_map_id,
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
        }
    }

    pub fn set_pose(&mut self, pose: Pose) {
        self.pose = pose.clone();
        // Sofiya todo... what is this stuff?
        // mRcw = mTcw.rotationMatrix();
        // mTwc = mTcw.inverse();
        // mRwc = mTwc.rotationMatrix();

        // if (mImuCalib.mbIsSet) // TODO Use a flag instead of the OpenCV matrix
        // {
        //     mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
        // }
    }
}