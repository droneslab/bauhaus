use log::info;
use opencv::{prelude::{Mat, MatTrait, Boxed, MatTraitConst}, core::{Scalar, Point3f, CV_64F, KeyPoint, Point}};
use dvcore::{config::*, matrix::{DVMatrix, DVVectorOfPoint3f, DVVector3, DVVectorOfi32}};
use crate::{
    dvmap::{pose::{Pose, Rotation}},
    matrix::DVVectorOfKeyPoint, registered_modules::CAMERA
};


lazy_static! {
    pub static ref CAMERA_MODULE: Camera = {
        Camera::new(CameraType::Pinhole).unwrap()
    };
}

#[derive(Debug, Clone, Default)]
pub enum CameraType {
    #[default] Pinhole
}

#[derive(Debug, Clone, Default)]
pub struct Camera {
    pub camera_type: CameraType,

    // K in ORBSLAM3 is matrix with the following items:
    pub k_matrix: DVMatrix,

    // Constants
    pub stereo_baseline_times_fx: f64, // mbf
    pub stereo_baseline: f64, //mb
    pub th_depth: i32, //mThDepth
    pub dist_coef: Option<Vec<f32>>, //mDistCoef
    // tvr: Option<TwoViewReconstruction>, // If we duplicate camera and have this pointer in here pointing to the same underlying tvr in C++, it is not thread safe
}

// TODO: I don't think we should duplicate camera in each kf/mp, I think we just need one reference?
impl Camera {
    pub fn new(camera_type: CameraType) -> Result<Camera, Box<dyn std::error::Error>> {
        // Implementation only for PinholeCamera, see:
        // - void Tracking::newParameterLoader
        // - GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam)
        // - void Settings::readCamera1(cv::FileStorage &fSettings) 
        let fx= GLOBAL_PARAMS.get::<f64>(CAMERA, "fx");
        let fy= GLOBAL_PARAMS.get::<f64>(CAMERA, "fy");
        let cx= GLOBAL_PARAMS.get::<f64>(CAMERA, "cx");
        let cy= GLOBAL_PARAMS.get::<f64>(CAMERA, "cy");

        let mut k = Mat::new_rows_cols_with_default(3,3, CV_64F, Scalar::all(0.0))?;
        *k.at_2d_mut::<f64>(0, 0)? = fx;
        *k.at_2d_mut::<f64>(0, 2)? = cx;
        *k.at_2d_mut::<f64>(1, 1)? = fy;
        *k.at_2d_mut::<f64>(1, 2)? = cy;
        *k.at_2d_mut::<f64>(2, 2)? = 1.0;

        // Todo (need?) Check if we need to correct distortion from the images
        let mut dist_coef = None;
        let sensor= GLOBAL_PARAMS.get::<Sensor>(SYSTEM_SETTINGS, "sensor");
        let k1= GLOBAL_PARAMS.get::<f64>(CAMERA, "k1") as f32;
        if sensor.is_mono() && k1 != 0.0 {
            let k2 = GLOBAL_PARAMS.get::<f64>(CAMERA, "k2") as f32;
            let p1 = GLOBAL_PARAMS.get::<f64>(CAMERA, "p1") as f32;
            let p2 = GLOBAL_PARAMS.get::<f64>(CAMERA, "p2") as f32;
            let k3 = GLOBAL_PARAMS.get::<f64>(CAMERA, "k3") as f32;
            if k3 != 0.0 {
                dist_coef = Some(vec![k1, k2, p1, p2, k3]);
            } else {
                dist_coef = Some(vec![k1, k2, p1, p2]);
            }
        }

        let stereo_baseline_times_fx= GLOBAL_PARAMS.get::<f64>(CAMERA, "stereo_baseline_times_fx");
        let th_depth= GLOBAL_PARAMS.get::<i32>(CAMERA, "thdepth");
        let stereo_baseline = stereo_baseline_times_fx / fx;

        Ok(Camera {
            camera_type,
            k_matrix: DVMatrix::new(k),
            stereo_baseline_times_fx,
            stereo_baseline,
            th_depth, dist_coef,
        })
    }

    pub fn get_fx(&self) -> f64 { *self.k_matrix.mat().at_2d::<f64>(0,0).unwrap() }
    pub fn get_fy(&self) -> f64 { *self.k_matrix.mat().at_2d::<f64>(1,1).unwrap() }
    pub fn get_cx(&self) -> f64 { *self.k_matrix.mat().at_2d::<f64>(0,2).unwrap() }
    pub fn get_cy(&self) -> f64 { *self.k_matrix.mat().at_2d::<f64>(1,2).unwrap() }
    pub fn get_inv_fx(&self) -> f64 { 1.0 / self.get_fx() }
    pub fn get_inv_fy(&self) -> f64 { 1.0 / self.get_fy() }

    pub fn reconstruct_with_two_views(
        & self, 
        v_keys1: &DVVectorOfKeyPoint, 
        v_keys2: &DVVectorOfKeyPoint,
        matches: &Vec<i32>,
    ) -> (bool, Pose, DVVectorOfPoint3f, Vec<bool>) {
        let mut tvr = dvos3binding::ffi::new_two_view_reconstruction(
            self.get_fx() as f32,
            self.get_cx() as f32,
            self.get_fy() as f32,
            self.get_cy() as f32,
            1.0, 200
        );

        let mut pose = dvos3binding::ffi::Pose{
            translation: [0.0;3],
            rotation: [0.0;4]
        };
        let mut v_p3d: dvos3binding::ffi::WrapBindCVVectorOfPoint3f = DVVectorOfPoint3f::empty().into();
        let mut vb_triangulated  = Vec::new();

        let reconstructed = tvr.pin_mut().reconstruct(
            & v_keys1.clone().into(),
            & v_keys2.clone().into(),
            matches, 
            &mut pose,
            &mut v_p3d,
            &mut vb_triangulated
        );

        // debug!("pose {:?} \n vP3D {:?} \n reconstructed: {}", pose, v_p3d.vec.len(), reconstructed);
        info!("pose in tvr {:?}", pose);

        (reconstructed, pose.into(), v_p3d.into(), vb_triangulated) 
    }

    pub fn unproject_eig(&self, kp: &opencv::core::Point2f) -> DVVector3<f32> {
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

