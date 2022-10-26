use std::collections::{HashMap, HashSet};
use chrono::{DateTime, Utc};
use abow::{BoW, DirectIdx, Vocabulary};
use dvcore::global_params::{GLOBAL_PARAMS, Sensor};
use serde::{Deserialize, Serialize};

use crate::{global_params::{SYSTEM_SETTINGS}, matrix::*, modules::{imu::IMUBias, imu::IMUPreIntegrated, camera::*,}};
use super::{sensor::SensorType, pose::Pose, map::{Id, Map}, keypoints::{KeyPointsData, FRAME_GRID_ROWS, FRAME_GRID_COLS}};

#[derive(Debug, Clone, Default)]
pub struct Frame<S: SensorType> {
    pub id: Id,
    pub timestamp: DateTime<Utc>,
    pub pose: Option<Pose>,

    // Image and reference KF //
    pub image_bounds: ImageBounds, // min_x, max_x, min_y, max_y
    pub ref_kf_id: Option<Id>, //mpReferenceKF

    // KeyPoints, stereo coordinate and descriptors (all associated by an index) //
    pub keypoints_data: S::KeyPointsData,

    // Mappoints //
    // Note: u32 is index in array, Id is mappoint Id, bool is if it's an oultier
    // equal to the vec in ORBSLAM3 bc it just allocates an N-size vector and has a bunch of empty entries
    pub mappoint_matches: HashMap::<u32, (Id, bool)>, // mvpmappoints , mvbOutlier

    // BoW //
    pub bow_vec: Option<Vec<abow::BoW>>, // mBowVec, Bow and featurevector from dbow2
    pub feature_vec: Option<(BoW, DirectIdx)>, // mFeatVec

    // Scale //
    pub num_scale_levels: i32, // mnScaleLevels
    pub scale_factor: f64, // mfScaleFactor
    pub log_scale_factor: f64, // mfLogScaleFactor
    pub scale_factors: Vec<f32>, // mvScaleFactors
    // Used in ORBExtractor, which we haven't implemented
    // we're using detect_and_compute from opencv instead
    // pub level_sigma2: Vec<f32>, // mvLevelSigma2

    // IMU //
    pub imu_bias: Option<IMUBias>,
    pub imu_preintegrated: Option<IMUPreIntegrated>,

    // Stereo //
    pub stereo_baseline: f64,

    // Idk where to put this
    // depth_threshold: u64,
    pub camera: Camera
}

impl<S: SensorType> Frame<S> {
    pub fn new(
        id: Id, keypoints_vec: &DVVectorOfKeyPoint, descriptors_vec: &DVMatrix,
        im_width: i32, im_height: i32, camera: &Camera
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let image_bounds = ImageBounds::new(im_width, im_height, &camera.dist_coef);
        let kpdata = S::KeyPointsData::new(keypoints_vec, descriptors_vec, &image_bounds, camera);

        let frame = Frame{
            id,
            timestamp: Utc::now(),
            image_bounds,
            keypoints_data: kpdata?,
            num_scale_levels: GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "n_levels"),
            scale_factor: GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "scale_factor"),
            log_scale_factor: (GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "scale_factor") as f64).log(10.0),
            imu_bias: None, // TODO (IMU)
            camera: camera.clone(),
            ..Default::default()
        };

        Ok(frame)
    }

    pub fn get_features_in_area(&self, x: &f64, y: &f64, r: &f64, min_level: &i64, max_level: &i64) -> Vec<usize> {
        // Sofiya: I think this needs to be moved into keypoints.rs
        //GetFeaturesInArea
        let mut indices = Vec::<usize>::new();
        indices.reserve(self.keypoints_data.num_keypoints() as usize);

        let frame_grid_rows = FRAME_GRID_ROWS as i64;
        let frame_grid_cols = FRAME_GRID_COLS as i64;
        let grid_element_width_inv = self.keypoints_data.grid().grid_element_width_inv;
        let grid_element_height_inv = self.keypoints_data.grid().grid_element_height_inv;

        let factor_x = *r;
        let factor_y = *r;

        let min_cell_x = i64::max(0, ((x-self.image_bounds.min_x-factor_x)*grid_element_width_inv).floor() as i64);
        let max_cell_x = i64::min(frame_grid_cols-1, ((x-self.image_bounds.min_x+factor_x)*grid_element_width_inv).ceil() as i64);
        let min_cell_y = i64::max(0, ((y-self.image_bounds.min_y-factor_y)*grid_element_height_inv).floor() as i64);
        let max_cell_y = i64::min(frame_grid_rows-1, ((y-self.image_bounds.min_y+factor_y)*grid_element_height_inv).ceil() as i64);

        if min_cell_x >= frame_grid_cols || max_cell_x < 0 || min_cell_y >= frame_grid_rows || max_cell_y < 0 {
            return indices;
        }

        let b_check_levels = *min_level>0 || *max_level>=0;

        for ix in min_cell_x..max_cell_x + 1 {
            for iy in min_cell_y..max_cell_y + 1 {
                let v_cell = Vec::<usize>::new();
                //const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];

                if v_cell.is_empty() {
                    continue;
                }

                for j in 0..v_cell.len() {
                    //TODO (Stereo) Need to update this if stereo images are processed
                    let kp_un = &self.keypoints_data.keypoints_get(v_cell[j]);
                    if b_check_levels {
                        if kp_un.octave< *min_level as i32 {
                            continue;
                        }
                        if *max_level>=0 {
                            if kp_un.octave> *max_level as i32 {
                                continue;
                            }
                        }
                    }

                    let distx = kp_un.pt.x- (*x as f32);
                    let disty = kp_un.pt.y- (*y as f32);

                    if distx.abs()<(factor_x as f32) && disty.abs() < (factor_y as f32)  {
                        indices.push(v_cell[j]);
                    }
                }
            }
        }
        return indices;
    }

    pub fn compute_bow(&mut self, voc: &DVVocabulary) {
        //Ref code : https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Frame.cc#L740
        if self.feature_vec.is_none() {
            self.feature_vec = Some(voc.transform_with_direct_idx(&self.keypoints_data.descriptors()));
        }
    }

    //* MapPoints */
    pub fn add_mappoint(&mut self, index: u32, mp_id: Id, is_outlier: bool) {
        self.mappoint_matches.insert(index, (mp_id, is_outlier));
    }

    pub fn delete_mappoint_match(&mut self, index: u32) {
        self.mappoint_matches.remove(&index);
        // Sofiya: removed the code that set's mappoint's last_frame_seen to the frame ID
        // I'm not sure we want to be using this
    }

    pub fn clear_mappoints(&mut self) {
        self.mappoint_matches = HashMap::new();
    }
    pub fn is_mp_outlier(&self, index: &u32) -> bool {
        self.mappoint_matches.get(index).unwrap().1
    }

    pub fn set_mp_outlier(&mut self, index: &u32, is_outlier: bool) {
        self.mappoint_matches
            .entry(*index)
            .and_modify(|(_, iso)| *iso = is_outlier);
    }

    pub fn discard_outliers(&mut self) -> i32 {
        let mps_at_start = self.mappoint_matches.len() as i32;
        self.mappoint_matches.retain(|_, (_, is_outlier)| *is_outlier);
        mps_at_start - self.mappoint_matches.len() as i32
    }

    pub fn get_num_mappoints_with_observations(&self, map: &Map<S>) -> i32 {
        (&self.mappoint_matches)
            .into_iter()
            .filter(|(_, (mp_id, _))| map.get_mappoint(mp_id).unwrap().observations().len() > 0)
            .count() as i32
    }

    pub fn delete_mappoints_without_observations(&mut self, map: &Map<S>) {
        self.mappoint_matches
            .retain(|_, (mp_id, _)| map.get_mappoint(mp_id).unwrap().observations().len() == 0);
    }

    pub fn check_close_tracked_mappoints(&self) -> (i32, i32) {
        self.keypoints_data.check_close_tracked_mappoints(self.camera.th_depth as f32, &self.mappoint_matches)
    }

    pub fn is_in_frustum(&self, mp_id: Id, viewing_cos_limit: f64, map: &Map<S>) -> bool {
        match S::sensor_type() {
            Sensor::ImuStereo | Sensor::Stereo => {
                todo!("TODO Stereo");
                //         pMP -> mnTrackScaleLevel = -1;
                //         pMP -> mnTrackScaleLevelR = -1;

                // return isInFrustumChecks(pMP,viewingCosLimit) || isInFrustumChecks(pMP,viewingCosLimit,true)
            },
            _ => {
                todo!("TODO 10/17");
                    // pMP->mTrackProjX = -1;
                    // pMP->mTrackProjY = -1;

                    // 3D in absolute coordinates
                    let P = *map.get_mappoint(&mp_id).unwrap().position;

                    // 3D in camera coordinates
                    // const Eigen::Matrix<float,3,1> Pc = mRcw * P + mtcw;
                    // const float Pc_dist = Pc.norm();

                    // // Check positive depth
                    // const float &PcZ = Pc(2);
                    // const float invz = 1.0f/PcZ;
                    // if(PcZ<0.0f)
                    //     return false;

                    // const Eigen::Vector2f uv = mpCamera->project(Pc);

                    // if(uv(0)<mnMinX || uv(0)>mnMaxX)
                    //     return false;
                    // if(uv(1)<mnMinY || uv(1)>mnMaxY)
                    //     return false;

                    // pMP->mTrackProjX = uv(0);
                    // pMP->mTrackProjY = uv(1);

                    // // Check distance is in the scale invariance region of the MapPoint
                    // const float maxDistance = pMP->GetMaxDistanceInvariance();
                    // const float minDistance = pMP->GetMinDistanceInvariance();
                    // const Eigen::Vector3f PO = P - mOw;
                    // const float dist = PO.norm();

                    // if(dist<minDistance || dist>maxDistance)
                    //     return false;

                    // // Check viewing angle
                    // Eigen::Vector3f Pn = pMP->GetNormal();

                    // const float viewCos = PO.dot(Pn)/dist;

                    // if(viewCos<viewingCosLimit)
                    //     return false;

                    // // Predict scale in the image
                    // const int nPredictedLevel = pMP->PredictScale(dist,this);

                    // // Data used by the tracking
                    // pMP->mbTrackInView = true;
                    // pMP->mTrackProjX = uv(0);
                    // pMP->mTrackProjXR = uv(0) - mbf*invz;

                    // pMP->mTrackDepth = Pc_dist;

                    // pMP->mTrackProjY = uv(1);
                    // pMP->mnTrackScaleLevel= nPredictedLevel;
                    // pMP->mTrackViewCos = viewCos;

                    return false;
            }
        }
    }
}


#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ImageBounds {
    pub min_x: f64,//static float mnMinX;
    pub max_x: f64,//static float mnMaxX;
    pub min_y: f64,//static float mnMinY;
    pub max_y: f64,//static float mnMaxY;
}

impl ImageBounds {
    pub fn new(im_width: i32, im_height: i32, dist_coef: &Option<Vec<f32>>) -> ImageBounds {
        //ComputeImageBounds
        let min_x = 0.0;
        let mut max_x = 0.0;
        let min_y = 0.0;
        let mut max_y = 0.0;

        match dist_coef {
            Some(vec) => {
                todo!("mid priority: implement code if dist_coef is non-zero");
                // cv::Mat mat(4,2,CV_32F);
                // mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
                // mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
                // mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
                // mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

                // mat=mat.reshape(2);
                // cv::undistortPoints(mat,mat,static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
                // mat=mat.reshape(1);

                // // Undistort corners
                // mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
                // mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
                // mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
                // mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));
            },
            None => {
                max_x = im_width as f64;
                max_y = im_height as f64;
            }
        }

        ImageBounds{ min_x, max_x, min_y, max_y }
    }
}