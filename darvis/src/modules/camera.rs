use cxx::CxxVector;
use opencv::{types::VectorOfPoint3f, prelude::{Mat, MatTrait, Boxed}, core::{CV_32F, Scalar}};
use std::collections::HashMap;
use dvcore::{global_params::*, matrix::{DVMatrix3, DVMatrix, DVVectorOfPoint3f, DVVector3}};
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
    // tvr: Option<TwoViewReconstruction>, // If we duplicate camera and have this pointer in here pointing to the same underlying tvr in C++, it is not thread safe
}

// TODO: I don't think we should duplicate camera in each kf/mp, I think we just need one reference?
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

        // Todo Check if we need to correct distortion from the images
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
        matches: &HashMap<u32, u32>,
        t21: &mut Pose,
        v_p3_d: &mut DVVectorOfPoint3f,
        triangulated: &mut Vec<bool>
    ) -> bool {

        let tvr = dvos3binding::ffi::new_two_view_reconstruction(
            self.fx as f32,
            self.cx as f32,
            self.fy as f32,
            self.cy as f32,
            1.0, 200
        );
        todo!("Fix calling bindings");
        // unsafe {
        //     let kps_1_cv = (**v_keys1).into_raw() as *const CxxVector<dvos3binding::ffi::DVKeyPoint>;
        //     let kps_2_cv = (**v_keys2).into_raw() as *const CxxVector<dvos3binding::ffi::DVKeyPoint>;

        //     let matches_cv = matches.into_raw() as *const CxxVector<i32>;
        //     let mut pose = dvos3binding::ffi::Pose{pose : [[0.0;4];4]};
        //     let mut vP3D  = dvos3binding::ffi::VectorOfDVPoint3f{vec:Vec::new() };
        //     let mut vbTriangulated  = dvos3binding::ffi::VectorOfDVBool{ vec:Vec::new() };

        //     tvr.pin_mut().Reconstruct_1(
        //         &*kps_1_cv,
        //         &*kps_2_cv,
        //         &*matches_new, 
        //         &mut pose,
        //         &mut vP3D,
        //         &mut vbTriangulated
        //     )
        // }
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

    pub fn unproject_eig() -> DVVector3<f32> {
        // Eigen::Vector3f Pinhole::unprojectEig(const cv::Point2f &p2D) {
        //     return Eigen::Vector3f((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1],
        //                     1.f);
        // }
        todo!("TODO LOCAL MAPPING");
    }
}

