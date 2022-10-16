use std::collections::HashMap;
use chrono::{DateTime, Utc};
use abow::{BoW, DirectIdx};
use dvcore::global_params::GLOBAL_PARAMS;
use serde::{Deserialize, Serialize};

use crate::{
    global_params::{SYSTEM_SETTINGS},
    matrix::*,
    utils::{imu::IMUBias, imu::IMUPreIntegrated, camera::*,},
};

use super::{sensor::SensorType, pose::Pose, map::Id, keypoints::*};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImageBounds {
    pub min_x: f64,//static float mnMinX;
    pub max_x: f64,//static float mnMaxX;
    pub min_y: f64,//static float mnMinY;
    pub max_y: f64,//static float mnMaxY;
}

impl ImageBounds {
    pub fn new(im_width: i32, im_height: i32, dist_coef: &Option<Vec<f64>>) -> ImageBounds {
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

    pub fn default() -> ImageBounds {
        ImageBounds{min_x: 0.0, max_x: 0.0, min_y: 0.0, max_y: 0.0}
    }
}

#[derive(Debug, Clone)]
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
    // Note: u32 is index in array, Id is mappoint Id ... equal to vector in ORBSLAM3
    // because it just allocates an N-size vector and has a bunch of empty entries
    pub mappoint_matches: HashMap::<u32, Id>, // mvpmappoints 
    pub mappoint_outliers: HashMap::<u32, bool>, // mvbOutlier, flag to identify outlier associations.

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
    pub fn fake() -> Frame<S> {
        // Should only be called once ever at the beginning of tracking_backend
        // A way to get around having to use Options around frames inside tracking_backend
        let kpdata = S::KeyPointsData::empty();
        let camera = Camera {
            camera_type: CameraType::Pinhole,
            fx: 0.0, fy: 0.0, cx: 0.0, cy: 0.0,
            inv_fx: 0.0, inv_fy: 0.0, 
            stereo_baseline_times_fx: 0.0,
            stereo_baseline: 0.0, th_depth: 0, 
            dist_coef: None
        };
        Frame{
            id: -1,
            timestamp: Utc::now(),
            pose: None,
            image_bounds: ImageBounds::new(0, 0, &None),
            ref_kf_id: None,
            keypoints_data: kpdata,
            mappoint_matches: HashMap::new(),
            mappoint_outliers: HashMap::new(),
            bow_vec: None,
            feature_vec: None,
            num_scale_levels: 0,
            scale_factor: 0.0,
            log_scale_factor: 0.0,
            scale_factors: Vec::new(),
            imu_bias: None,
            imu_preintegrated: None,
            stereo_baseline: 0.0,
            camera: camera
        }
    }

    pub fn new(
        id: Id,
        keypoints_vec: &DVVectorOfKeyPoint,
        descriptors_vec: &DVMatrix,
        im_width: i32, im_height: i32,
        camera: &Camera
    ) -> Self {
        let image_bounds = ImageBounds::new(im_width, im_height, &camera.dist_coef);
        let kpdata = S::KeyPointsData::new(keypoints_vec, descriptors_vec, &image_bounds);

        let frame = Frame{
            id,
            timestamp: Utc::now(),
            pose: None,
            // Image and reference KF //
            image_bounds,
            ref_kf_id: None,
            // KeyPoints, stereo coordinate and descriptors (all associated by an index) //
            keypoints_data: kpdata,
            // Mappoints //
            mappoint_matches: HashMap::new(),
            mappoint_outliers: HashMap::new(),
            // BoW //
            bow_vec: None,
            feature_vec: None,
            // Scale //
            num_scale_levels: GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "n_levels"),
            scale_factor: GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "scale_factor"),
            log_scale_factor: (GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "scale_factor") as f64).log(10.0),
            scale_factors: Vec::new(),
            // IMU //
            imu_bias: None, // TODO (IMU)
            imu_preintegrated: None,
            // Stereo //
            stereo_baseline: 0.0,
            // Idk where to put this //
            camera: camera.clone()
        };

        frame
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
                    let kp_un = &self.keypoints_data.keypoints_un().get(v_cell[j]).unwrap();
                    //const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[v_cell[j]]
                    //                                         : (!bRight) ? mvKeys[v_cell[j]]
                    //                                                     : mvKeysRight[v_cell[j]];
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

    pub fn set_pose(&mut self, pose: Pose) {
        // Note: Can be changed here without going through map because frames
        // are only ever used in Tracking, and never sent to the map.
        self.pose = Some(pose.clone());

        //UpdatePoseMatrices();
        //mbIsSet = true;
        //mbHasPose = true;]
    }

    pub fn get_pose(&self) -> Pose {
        self.pose.as_ref().unwrap().clone()
    }

    pub fn compute_bow(&mut self) {
        //Ref code : https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Frame.cc#L740
        todo!("IMPORTANT Implement : ComputeBoW");
        // if self.feature_vec.is_none() {
        //     let features = Converter::toDescriptorVector(&self.descriptors);
        //     let k=10;
        //     let l =4;
        //     let voc = Vocabulary::create(&features, k, l);
        //     self.feature_vec = Some(voc.transform_with_direct_idx(&features).unwrap());
        // }
    }

    //* MapPoints */
    pub fn mappoint_is_outlier(&self, index: &u32) -> bool {
        self.mappoint_outliers.contains_key(index) 
    }
    pub fn set_outlier(&mut self, index: &u32, is_outlier: bool) {
        self.mappoint_outliers.insert(*index, is_outlier);
    }
    pub fn discard_outliers(&mut self) -> (i32, Vec<Id>) {
        let mut num_deleted = 0;
        let mut all_left = Vec::new();
        for (index, mp_id) in &mut self.mappoint_matches {
            if self.mappoint_outliers.contains_key(&index) {
                self.delete_mappoint_match(*index);
                num_deleted += 1;
            } else {
                all_left.push(*mp_id);
            }
        }
        (num_deleted, all_left)
    }

    pub fn check_close_tracked_mappoints(&self) -> (i32, i32) {
        self.keypoints_data.check_close_tracked_mappoints(self.camera.th_depth as f32, &self.mappoint_matches, &self.mappoint_outliers)
    }

    pub fn add_mappoint_match(&mut self, index: u32, mp_id: Id) {
        self.mappoint_matches.insert(index, mp_id);
    }

    pub fn delete_mappoint_match(&mut self, index: u32) {
        self.mappoint_matches.remove(&index);
        self.mappoint_outliers.remove(&index);
        // Sofiya note: removed the code that set's mappoint's last_frame_seen to the frame ID
        // I'm not sure we want to be using this
    }

    pub fn clear_mappoints(&mut self) {
        self.mappoint_matches = HashMap::new();
    }

}
