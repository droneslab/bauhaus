use crate::map::{map::Id, pose::Pose};
use crate::dvutils::*;
use opencv::core::Mat;

#[derive(Debug, Clone)]
pub struct Grid {
    pub grid_cols: i32,
    pub grid_rows: i32,
    pub grid_element_width_inv: f32,
    pub grid_element_height_inv: f32,
}

#[derive(Debug, Clone)]
pub struct ImageBounds {
    pub min_x: i32,
    pub min_y: i32,
    pub max_x: i32,
    pub max_y: i32
}

unsafe impl Sync for KeyFrame {}
#[derive(Debug, Clone)]
pub struct KeyFrame {
    pub id: Id,
    pub frame_id: u64,

    pub grid: Grid, // Grid to speed up feature matching

    pub f_scale: f32, //?
    pub origin_map_id: u32, //?

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    pub num_keypoints: i32,
    pub mv_keys: Vec<DarvisKeyPoint>,
    pub mv_keys_un: Vec<DarvisKeyPoint>,
    pub mvu_right: Vec<f32>, // negative value for monocular points
    pub mv_depth: Vec<f32>, // negative value for monocular points
    pub descriptors: Mat,

    // BoW
    pub bow_vec: Vec<abow::BoW>, //Bow and featurevector from dbow2
    pub feature_vec: Vec<Mat>,

    // Scale
    pub num_scale_levels: i32,
    pub scale_factor: f32,
    pub log_scale_factor: f32,
    pub scale_factors: Vec<f32>,
    pub level_sigma2: Vec<f32>,
    pub inv_level_sigma2: Vec<f32>,

    pub image_bounds: ImageBounds, // Image bounds and calibration

    // Preintegrated IMU measurements from previous keyframe
    pub prev_kf_id: i32,
    pub next_kf_id: i32,
    // pub imu_preintegrated: IMUPreIntegrated,
    // pub imu_calib: IMUCalib,

    pub loop_candidate_kfs: Vec<i32>,
    pub merge_candidate_kfs: Vec<i32>,

    // Variables used by KF database
    pub loop_query: u64,
    pub loop_words: i32,
    pub loop_score: f32,
    pub reloc_query: u64,
    pub reloc_words: i32,
    pub reloc_score: f32,
    pub merge_query: u64,
    pub merge_words: i32,
    pub merge_score: f32,
    pub place_recognition_query: u64,
    pub place_recognition_words: i32,
    pub place_recognition_score: f32,

    // Variables used by loop closing
    // pub mTcwGBA: SophusSE3f,
    // pub mTcwBefGBA: SophusSE3f,
    // pub mVwbGBA: EigenVector3f, // eigen
    // pub mVwbBefGBA: EigenVector3f, // eigen
    // pub mBiasGBA: IMUBias, 
    pub mnBAGlobalForKF: u64,

    // Variables used by merging
    // pub tcw_bef_merge: SophusSE3f,
    // pub tcw_merge: SophusSE3f,
    // pub twc_bef_merge: SophusSE3f,
    // pub vwb_merge: EigenVector3f,
    // pub vwb_bef_merge: EigenVector3f,
    // pub bias_merge: IMUBias,
    pub mnMergeCorrectedForKF: u64,
    pub mnMergeForKF: u64,
    pub mfScaleMerge: f32,
    pub mnBALocalForMerge: u64,

    // IMU position

    timestamp: u64,
    // map_points: Arc<Vec<MapPoint>>,
    // bow: abow::BoW,
    pub pose: Pose,
    // bow_db: Arc<BowDB>,
    // scale: f64,
    // depth_threshold: f64,
}
