use crate::map::map::Id;
use crate::dvutils::*;
use opencv::core::Mat;

#[derive(Debug, Clone)]
pub struct CalibParam {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub inv_fx: f32,
    pub inv_fy: f32,
    pub mbf: f32,
    pub mb: f32,
    pub th_depth: f32,
}

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

    pub grid: Grid, // Grid (to speed up feature matching)

    pub f_scale: f32, //?
    pub origin_map_id: u32, //?

    // Calibration parameters
    pub calibration_params: CalibParam,

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
    // pub mTcwGBA: SophusSE3f, // Sofiya: Sophus?
    // pub mTcwBefGBA: SophusSE3f, // Sofiya: Sophus?
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
    // pose: Pose,
    // bow_db: Arc<BowDB>,
    // scale: f64,
    // depth_threshold: f64,
}
// impl KeyFrame {
//     pub fn new(id: i32) -> Self {
//         Grid {
//             grid_cols: FRAME_GRID_COLS,
//             grid_rows: FRAME_GRID_ROWS,
//             grid_element_width_inv: 0,
//             grid_element_height_inv: 0,
//         }

//         // KeyFrame {
//         //     id: id,
//         //     frame_id: 0,
//         //     timestamp: 0,
//         //     grid: Grid,

            
//         // }


        
//         // mnTrackReferenceForFrame(0), 
//         // mnFuseTargetForKF(0), 
//         // mnBALocalForKF(0), 
//         // mnBAFixedForKF(0), 
//         // mnBALocalForMerge(0),
//         // mnLoopQuery(0), 
//         // mnLoopWords(0), 
//         // mnRelocQuery(0), 
//         // mnRelocWords(0), 
//         // mnMergeQuery(0), 
//         // mnMergeWords(0), 
//         // mnBAGlobalForKF(0),
//         // fx(0), fy(0), cx(0), cy(0), invfx(0), invfy(0), 
//         // mnPlaceRecognitionQuery(0), 
//         // mnPlaceRecognitionWords(0), 
//         // mPlaceRecognitionScore(0),
//         // mbf(0), 
//         // mb(0), 
//         // mThDepth(0), 
//         // N(0), 
//         // mvKeys(static_cast<vector<cv::KeyPoint> >(NULL)), 
//         // mvKeysUn(static_cast<vector<cv::KeyPoint> >(NULL)),
//         // mvuRight(static_cast<vector<float> >(NULL)), 
//         // mvDepth(static_cast<vector<float> >(NULL)), 
//         // mnScaleLevels(0), 
//         // mfScaleFactor(0),
//         // mfLogScaleFactor(0), 
//         // mvScaleFactors(0), 
//         // mvLevelSigma2(0), 
//         // mvInvLevelSigma2(0), 
//         // mnMinX(0), 
//         // mnMinY(0), 
//         // mnMaxX(0),
//         // mnMaxY(0), 
//         // mPrevKF(static_cast<KeyFrame*>(NULL)), 
//         // mNextKF(static_cast<KeyFrame*>(NULL)), 
//         // mbFirstConnection(true), 
//         // mpParent(NULL), 
//         //  mHalfBaseline(0), 
//         //  mbCurrentPlaceRecognition(false), 
//         //  mnMergeCorrectedForKF(0),
//         // NLeft(0),
//         // NRight(0), 
//         // mnNumberOfOpt(0), 
//         // mbHasVelocity(false)



//     }


//     pub fn new(id: i32, frame: &mut Frame) -> Self {


//         bImu(pMap->isImuInitialized()), mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
//     mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
//     mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
//     mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
//     fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
//     mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
//     mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
//     mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
//     mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
//     mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
//     mnMaxY(F.mnMaxY), mK_(F.mK_), mPrevKF(NULL), mNextKF(NULL), mpImuPreintegrated(F.mpImuPreintegrated),
//     mImuCalib(F.mImuCalib), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
//     mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mDistCoef(F.mDistCoef), mbNotErase(false), mnDataset(F.mnDataset),
//     mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap), mbCurrentPlaceRecognition(false), mNameFile(F.mNameFile), mnMergeCorrectedForKF(0),
//     mpCamera(F.mpCamera), mpCamera2(F.mpCamera2),
//     mvLeftToRightMatch(F.mvLeftToRightMatch),mvRightToLeftMatch(F.mvRightToLeftMatch), mTlr(F.GetRelativePoseTlr()),
//     mvKeysRight(F.mvKeysRight), NLeft(F.Nleft), NRight(F.Nright), mTrl(F.GetRelativePoseTrl()), mnNumberOfOpt(0), mbHasVelocity(false)



//         mnId=nNextId++;

//     mGrid.resize(mnGridCols);
//     if(F.Nleft != -1)  mGridRight.resize(mnGridCols);
//     for(int i=0; i<mnGridCols;i++)
//     {
//         mGrid[i].resize(mnGridRows);
//         if(F.Nleft != -1) mGridRight[i].resize(mnGridRows);
//         for(int j=0; j<mnGridRows; j++){
//             mGrid[i][j] = F.mGrid[i][j];
//             if(F.Nleft != -1){
//                 mGridRight[i][j] = F.mGridRight[i][j];
//             }
//         }
//     }



//     if(!F.HasVelocity()) {
//         mVw.setZero();
//         mbHasVelocity = false;
//     }
//     else
//     {
//         mVw = F.GetVelocity();
//         mbHasVelocity = true;
//     }

//     mImuBias = F.mImuBias;
//     SetPose(F.GetPose());

//     mnOriginMapId = pMap->GetId();


// }