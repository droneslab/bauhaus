use std::collections::{HashMap, HashSet};
use chrono::{DateTime, Utc};
use dvcore::global_params::{GLOBAL_PARAMS, Sensor, FrameSensor};
use serde::{Deserialize, Serialize};

use crate::{global_params::{SYSTEM_SETTINGS}, matrix::*, modules::{imu::IMUBias, imu::IMUPreIntegrated, camera::*,}};
use super::{pose::Pose, map::{Id, Map}, features::{FRAME_GRID_ROWS, FRAME_GRID_COLS, Features}, bow::{DVVocabulary, BoW}};

#[derive(Debug, Clone, Default)]
pub struct Frame {
    pub id: Id,
    pub timestamp: DateTime<Utc>,
    pub pose: Option<Pose>,

    // Image and reference KF //
    pub ref_kf_id: Option<Id>, //mpReferenceKF

    // Vision //
    pub features: Features, // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    pub bow: BoW,

    // Mappoints //
    // Note: u32 is index in array, Id is mappoint Id, bool is if it's an oultier
    // equal to the vec in ORBSLAM3 bc it just allocates an N-size vector and has a bunch of empty entries
    pub mappoint_matches: HashMap::<u32, (Id, bool)>, // mvpmappoints , mvbOutlier

    // Scale //
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
    pub camera: Camera,

    pub sensor: Sensor,
}

impl Frame {
    pub fn new(
        id: Id, keypoints_vec: &DVVectorOfKeyPoint, descriptors_vec: &DVMatrix,
        im_width: i32, im_height: i32, camera: &Camera
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let sensor = GLOBAL_PARAMS.get::<Sensor>(SYSTEM_SETTINGS, "sensor");

        let features = Features::new(keypoints_vec, descriptors_vec, im_width, im_height, camera, sensor);

        let frame = Frame{
            id,
            timestamp: Utc::now(),
            features: features?,
            imu_bias: None, // TODO (IMU)
            camera: camera.clone(),
            sensor,
            ..Default::default()
        };

        Ok(frame)
    }

    pub fn compute_bow(&mut self, voc: &DVVocabulary) {
        //Ref code : https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Frame.cc#L740
        if self.bow.is_empty {
            // voc.transform(self.features.descriptors);
        // }
            voc.transform(&self.features.descriptors, &mut self.bow);
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

    pub fn get_num_mappoints_with_observations(&self, map: &Map) -> i32 {
        (&self.mappoint_matches)
            .into_iter()
            .filter(|(_, (mp_id, _))| map.get_mappoint(mp_id).unwrap().observations().len() > 0)
            .count() as i32
    }

    pub fn delete_mappoints_without_observations(&mut self, map: &Map) {
        self.mappoint_matches
            .retain(|_, (mp_id, _)| map.get_mappoint(mp_id).unwrap().observations().len() == 0);
    }

    pub fn check_close_tracked_mappoints(&self) -> (i32, i32) {
        self.features.check_close_tracked_mappoints(self.camera.th_depth as f32, &self.mappoint_matches)
    }

    pub fn is_in_frustum(&self, mp_id: Id, viewing_cos_limit: f64, map: &Map) -> bool {
        match self.sensor.frame() {
            FrameSensor::Stereo => {
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