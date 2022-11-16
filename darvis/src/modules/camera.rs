use cxx::{CxxVector,UniquePtr};
use opencv::{types::VectorOfPoint3f, prelude::{Mat, MatTrait, Boxed, MatTraitConst}, core::{CV_32F, Scalar, CV_64F}};
use std::{collections::HashMap, sync::Mutex};
use dvcore::{config::*, matrix::{DVMatrix3, DVMatrix, DVVectorOfPoint3f, DVVector3}};
use crate::{
    dvmap::{pose::Pose, map::Id},
    matrix::DVVectorOfKeyPoint, registered_modules::MOD_CAMERA
};
use dvos3binding::ffi::TwoViewReconstruction;

lazy_static! {
    pub static ref CAMERA: DVCamera = {
        DVCamera::new().unwrap()
    };
}
// TODO (concurrency): implemented these so that we can pass DVCamera around globally
// This should not cause a problem as long as the TwoViewReconstruction C++ code does not
// mutate the TwoViewReconstruction object. If it does, we need to add mutexes on the C++ side.
unsafe impl Send for DVCamera {}
unsafe impl Sync for DVCamera {}

pub struct DVCamera {
    // K in ORBSLAM3 is matrix with the following items:
    pub k_matrix: DVMatrix,

    pub stereo_baseline_times_fx: f64, // mbf
    pub stereo_baseline: f64, //mb
    pub th_depth: i32, //mThDepth
    pub dist_coef: Option<Vec<f32>>, //mDistCoef
    tvr: UniquePtr<TwoViewReconstruction>,
}

// TODO: I don't think we should duplicate camera in each kf/mp, I think we just need one reference?
impl DVCamera {
    fn new() -> Result<DVCamera, Box<dyn std::error::Error>> {
        // Implementation only for PinholeCamera, see:
        // - void Tracking::newParameterLoader
        // - GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam)
        // - void Settings::readCamera1(cv::FileStorage &fSettings)
        let camera_type = GLOBAL_PARAMS.get::<String>(MOD_CAMERA, "camera_type");
        match camera_type.as_str() {
            "Pinhole" => {
                let fx= GLOBAL_PARAMS.get::<f64>(MOD_CAMERA, "fx");
                let fy= GLOBAL_PARAMS.get::<f64>(MOD_CAMERA, "fy");
                let cx= GLOBAL_PARAMS.get::<f64>(MOD_CAMERA, "cx");
                let cy= GLOBAL_PARAMS.get::<f64>(MOD_CAMERA, "cy");

                let mut k = Mat::new_rows_cols_with_default(3,3, CV_64F, Scalar::all(0.0))?;
                *k.at_2d_mut::<f64>(0, 0)? = fx;
                *k.at_2d_mut::<f64>(0, 2)? = cx;
                *k.at_2d_mut::<f64>(1, 1)? = fy;
                *k.at_2d_mut::<f64>(1, 2)? = cy;
                *k.at_2d_mut::<f64>(2, 2)? = 1.0;

                // Todo (need?) Check if we need to correct distortion from the images
                let mut dist_coef = None;
                let sensor= GLOBAL_PARAMS.get::<Sensor>(SYSTEM_SETTINGS, "sensor");
                let k1= GLOBAL_PARAMS.get::<f64>(MOD_CAMERA, "k1") as f32;
                if sensor.is_mono() && k1 != 0.0 {
                    let k2 = GLOBAL_PARAMS.get::<f64>(MOD_CAMERA, "k2") as f32;
                    let p1 = GLOBAL_PARAMS.get::<f64>(MOD_CAMERA, "p1") as f32;
                    let p2 = GLOBAL_PARAMS.get::<f64>(MOD_CAMERA, "p2") as f32;
                    let k3 = GLOBAL_PARAMS.get::<f64>(MOD_CAMERA, "k3") as f32;
                    if k3 != 0.0 {
                        dist_coef = Some(vec![k1, k2, p1, p2, k3]);
                    } else {
                        dist_coef = Some(vec![k1, k2, p1, p2]);
                    }
                }

                let stereo_baseline_times_fx= GLOBAL_PARAMS.get::<f64>(MOD_CAMERA, "stereo_baseline_times_fx");
                let th_depth= GLOBAL_PARAMS.get::<i32>(MOD_CAMERA, "thdepth");
                let stereo_baseline = stereo_baseline_times_fx / fx;

                let tvr = dvos3binding::ffi::new_two_view_reconstruction(
                    fx as f32,
                    cx as f32,
                    fy as f32,
                    cy as f32,
                    1.0, 200
                );

                Ok(DVCamera {
                    k_matrix: DVMatrix::new(k),
                    stereo_baseline_times_fx,
                    stereo_baseline,
                    th_depth, dist_coef, tvr
                })
            },
            _ => panic!("Incorrect camera type")
        }
    }

    pub fn get_fx(&self) -> f64 { *self.k_matrix.mat().at_2d::<f64>(0,0).unwrap() }
    pub fn get_fy(&self) -> f64 { *self.k_matrix.mat().at_2d::<f64>(1,1).unwrap() }
    pub fn get_cx(&self) -> f64 { *self.k_matrix.mat().at_2d::<f64>(0,2).unwrap() }
    pub fn get_cy(&self) -> f64 { *self.k_matrix.mat().at_2d::<f64>(1,2).unwrap() }
    pub fn get_inv_fx(&self) -> f64 { 1.0 / self.get_fx() }
    pub fn get_inv_fy(&self) -> f64 { 1.0 / self.get_fy() }

    pub fn reconstruct_with_two_views(
        &self, 
        v_keys1: &DVVectorOfKeyPoint, 
        v_keys2: &DVVectorOfKeyPoint,
        matches: &HashMap<u32, u32>,
        t21: &mut Pose,
        v_p3_d: &mut DVVectorOfPoint3f,
        triangulated: &mut Vec<bool>
    ) -> bool {
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

    pub fn unproject_eig() -> DVVector3<f32> {
        // Eigen::Vector3f Pinhole::unprojectEig(const cv::Point2f &p2D) {
        //     return Eigen::Vector3f((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1],
        //                     1.f);
        // }
        todo!("TODO LOCAL MAPPING");
    }

    pub fn project(&self, pos: DVVector3<f64>) -> (f64, f64) {
        // Eigen::Vector2f Pinhole::project(const Eigen::Vector3f &v3D)

        // mvParameters in orbslam are:
        // 0 = fx, 1 = fy, 2 = cx, 3 = cy
        (
            self.get_fx() * pos[0] / pos[2] + self.get_cx(),
            self.get_fy() * pos[1] / pos[2] + self.get_cy()
        )
    }
}

