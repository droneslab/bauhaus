use opencv::{
    types::{VectorOfPoint3f, VectorOfKeyPoint},
};
use std::collections::HashMap;
use crate::{
    map::{pose::Pose, map::Id},
    utils::twoviewreconstruction::TwoViewReconstruction,
    global_params::*,
    utils::sensor::Sensor,
    dvutils::DVVectorOfKeyPoint
};


#[derive(Debug, Clone)]
pub enum CameraType {
    Pinhole
}

#[derive(Debug, Clone)]
pub struct Camera {
    pub camera_type: CameraType,

    // K in ORBSLAM3 is matrix with the following items:
    pub fx: f64, //(0,0)
    pub fy: f64, //(1,1)
    pub cx: f64, //(0,2)
    pub cy: f64, //(1,2)

    pub inv_fx: f64, //invfx
    pub inv_fy: f64, //invfy
    pub stereo_baseline_times_fx: f64, // mbf
    pub stereo_baseline: f64, //mb
    pub th_depth: i32, //mThDepth
    pub dist_coef: Option<Vec<f64>>, //mDistCoef
    // tvr: Option<TwoViewReconstruction>,
}

impl Camera {
    fn get_cy(&self) -> f64 { self.cy }
    fn get_cx(&self) -> f64 { self.cx }
    fn get_fx(&self) -> f64 { self.fx }
    fn get_fy(&self) -> f64 { self.fy }
    fn get_dist_coef(&self) -> Option<Vec<f64>> { self.dist_coef.clone() }

    pub fn new(camera_type: CameraType) -> Camera {
        // Implementation only for PinholeCamera, see:
        // - void Tracking::newParameterLoader
        // - GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam)
        // - void Settings::readCamera1(cv::FileStorage &fSettings) 
        let fx: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_fx");
        let fy: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_fy");
        let cx: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_cx");
        let cy: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_cy");
        let stereo_baseline_times_fx: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "stereo_baseline_times_fx");
        let th_depth: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "thdepth");

        //Check if we need to correct distortion from the images
        let mut dist_coef = None;
        let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");
        let k1: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_k1");
        if sensor.is_mono() && k1 != 0.0 {
            let k2: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_k2");
            let p1: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_p1");
            let p2: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_p2");
            let k3: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_k3");
            if k3 != 0.0 {
                dist_coef = Some(vec![k1, k2, p1, p2, k3]);
            } else {
                dist_coef = Some(vec![k1, k2, p1, p2]);
            }
        }

        let inv_fx = 1.0 / fx;
        let inv_fy = 1.0 / fy;
        let stereo_baseline = stereo_baseline_times_fx / fx;

        Camera {
            camera_type: camera_type,
            fx: fx, fy: fy, cx: cx, cy: cy, inv_fx: inv_fx, inv_fy: inv_fy, 
            stereo_baseline_times_fx: stereo_baseline_times_fx,
            stereo_baseline: stereo_baseline,
            th_depth: th_depth, dist_coef: dist_coef,
        }
    }

    pub fn reconstruct_with_two_views(
        &mut self, 
        vKeys1: &DVVectorOfKeyPoint, 
        vKeys2: &DVVectorOfKeyPoint,
        vMatches12: &HashMap<u32, Id>,
        T21: &mut Pose,
        vP3D: &mut VectorOfPoint3f,
        vbTriangulated: &mut Vec<bool>
    ) -> bool {
        let mut tvr = TwoViewReconstruction::default();
        return tvr.reconstruct(
            vKeys1,vKeys2,vMatches12,T21, vP3D,vbTriangulated
        );
    }
}

