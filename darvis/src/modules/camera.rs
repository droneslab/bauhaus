use opencv::{types::VectorOfPoint3f, prelude::{Mat, MatTrait}, core::{CV_32F, Scalar}};
use std::collections::HashMap;
use dvcore::{global_params::*, matrix::{DVMatrix3, DVMatrix}};
use crate::{
    dvmap::{pose::Pose, map::Id},
    modules::{twoviewreconstruction::TwoViewReconstruction},
    matrix::DVVectorOfKeyPoint
};


#[derive(Debug, Clone, Default)]
pub enum CameraType {
    #[default] Pinhole
}

#[derive(Debug, Clone, Default)]
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
    pub dist_coef: Option<Vec<f32>>, //mDistCoef
    // tvr: Option<TwoViewReconstruction>,
}

impl Camera {
    pub fn new(camera_type: CameraType) -> Camera {
        // Implementation only for PinholeCamera, see:
        // - void Tracking::newParameterLoader
        // - GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam)
        // - void Settings::readCamera1(cv::FileStorage &fSettings) 
        let fx= GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "camera_fx");
        let fy= GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "camera_fy");
        let cx= GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "camera_cx");
        let cy= GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "camera_cy");
        let stereo_baseline_times_fx= GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "stereo_baseline_times_fx");
        let th_depth= GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "thdepth");

        //Check if we need to correct distortion from the images
        let mut dist_coef = None;
        let sensor= GLOBAL_PARAMS.get::<Sensor>(SYSTEM_SETTINGS, "sensor");
        let k1= GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "camera_k1") as f32;
        if sensor.is_mono() && k1 != 0.0 {
            let k2= GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "camera_k2") as f32;
            let p1= GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "camera_p1") as f32;
            let p2= GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "camera_p2") as f32;
            let k3= GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "camera_k3") as f32;
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
            camera_type,
            fx, fy, cx, cy, inv_fx, inv_fy, 
            stereo_baseline_times_fx,
            stereo_baseline,
            th_depth, dist_coef,
        }
    }

    pub fn reconstruct_with_two_views(
        &mut self, 
        v_keys1: &DVVectorOfKeyPoint, 
        v_keys2: &DVVectorOfKeyPoint,
        v_matches12: &HashMap<u32, Id>,
        t21: &mut Pose,
        v_p3_d: &mut VectorOfPoint3f,
        triangulated: &mut Vec<bool>
    ) -> bool {
        let mut tvr = TwoViewReconstruction::default();
        todo!("TODO 10/17 BINDINGS reconstruct");
        return tvr.reconstruct(
            v_keys1,v_keys2,v_matches12,t21, v_p3_d,triangulated
        );
    }

    pub fn to_K_matrix(&self) -> Result<Mat, Box<dyn std::error::Error>> {
        let mut K = Mat::new_rows_cols_with_default(3,3, CV_32F, Scalar::all(0.0))?;
        *K.at_2d_mut::<f64>(0, 0)? = self.fx;
        *K.at_2d_mut::<f64>(0, 2)? = self.cx;
        *K.at_2d_mut::<f64>(1, 1)? = self.fy;
        *K.at_2d_mut::<f64>(1, 2)? = self.cy;
        *K.at_2d_mut::<f64>(2, 2)? = 1.0;
        Ok(K)
    }
}

