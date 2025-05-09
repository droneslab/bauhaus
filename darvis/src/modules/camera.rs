use opencv::{prelude::{Mat, MatTrait, KeyPointTraitConst}, core::{Scalar, CV_64F, KeyPoint}};
use core::{config::*, matrix::{DVMatrix, DVMatrix3, DVVector3, DVVectorOfPoint3f}, sensor::Sensor};
use crate::{
    map::{pose::Pose, keyframe::KeyFrame},
    matrix::DVVectorOfKeyPoint, registered_actors::CAMERA
};
use crate::modules::module_definitions::CameraModule;


#[derive(Debug, Clone, Default)]
pub enum CameraType {
    #[default] Pinhole
}

#[derive(Debug, Clone, Default)]
pub struct Camera {
    pub camera_type: CameraType,

    // Three diff ways of representing K...
    pub k_matrix: DVMatrix,
    k_matrix_nalgebra: nalgebra::Matrix3<f64>,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,

    // Constants
    pub stereo_baseline_times_fx: f64, // mbf
    pub stereo_baseline: f64, //mb
    pub th_depth: i32, //mThDepth
    pub dist_coef: Option<Vec<f32>>, //mDistCoef
    pub uncertainty: f32, //function call to uncertainty2
}

impl CameraModule for Camera {
    type Keys = DVVectorOfKeyPoint;
    type Pose = Pose;
    type ResultPoints = DVVectorOfPoint3f;
    type ResultTriangulated = Vec<bool>;
    type KeyPoint = opencv::core::KeyPoint;
    type KeyFrame = KeyFrame;
    type Point = opencv::core::Point2f;
    type Matches = Vec<i32>;

    fn two_view_reconstruction(
        & self, 
        v_keys1: &DVVectorOfKeyPoint, 
        v_keys2: &DVVectorOfKeyPoint,
        matches: &Vec<i32>,
    ) -> Option<(Pose, DVVectorOfPoint3f, Vec<bool>)> {
        let mut tvr = dvos3binding::ffi::new_two_view_reconstruction(
            self.fx as f32,
            self.cx as f32,
            self.fy as f32,
            self.cy as f32,
            1.0, 200
        );

        let mut pose = dvos3binding::ffi::Pose {
            translation: [0.0;3],
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        };
        let mut v_p3d: dvos3binding::ffi::WrapBindCVVectorOfPoint3f = DVVectorOfPoint3f::empty().into();
        let mut vb_triangulated  = Vec::new();

        // TODO (timing) ... cloning vkeys1 and 2
        let reconstructed = tvr.pin_mut().reconstruct(
            & v_keys1.clone().into(),
            & v_keys2.clone().into(),
            matches, 
            &mut pose,
            &mut v_p3d,
            &mut vb_triangulated
        );

        if reconstructed {
            return Some((pose.into(), v_p3d.into(), vb_triangulated));
        } else {
            return None;
        }
    }

    fn unproject_eig(&self, kp: &opencv::core::Point2f) -> DVVector3<f64> {
        // Eigen::Vector3f Pinhole::unprojectEig(const cv::Point2f &p2D)
        // mvParameters in orbslam are:
        // 0 = fx, 1 = fy, 2 = cx, 3 = cy
        DVVector3::new_with(
            (kp.x as f64 - self.cx) / self.fx,
            (kp.y as f64 - self.cy) / self.fy,
            1.0
        )
    }

    fn unproject_stereo(&self, _kf: &KeyFrame, _idx: usize) -> Option<DVVector3<f64>> {
        todo!("TODO (Stereo)");
        // bool KeyFrame::UnprojectStereo(int i, Eigen::Vector3f &x3D)
        // {
        //     const float z = mvDepth[i];
        //     if(z>0)
        //     {
        //         const float u = mvKeys[i].pt().x;
        //         const float v = mvKeys[i].pt().y;
        //         const float x = (u-cx)*z*invfx;
        //         const float y = (v-cy)*z*invfy;
        //         Eigen::Vector3f x3Dc(x, y, z);

        //         unique_lock<mutex> lock(mMutexPose);
        //         x3D = mRwc * x3Dc + mTwc.translation();
        //         return true;
        //     }
        //     else
        //         return false;
        // }
    }

    fn project(&self, pos: DVVector3<f64>) -> (f64, f64) {
        // Eigen::Vector2f Pinhole::project(const Eigen::Vector3f &v3D)
        let u = (self.fx * pos[0]) / pos[2] + self.cx;
        let v = (self.fy * pos[1]) / pos[2] + self.cy;
        (u, v)
    }

    fn epipolar_constrain(&self, kp1: &KeyPoint, kp2: &KeyPoint, r12: &DVMatrix3<f64>, t12: &DVVector3<f64>, unc: f32) -> bool {
        // bool Pinhole::epipolarConstrain(GeometricCamera* pCamera2,  const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc) {

        //Compute Fundamental Matrix
        let rot = **r12;
        let trans = *t12;
        let t12x = nalgebra::Matrix3::new(
            0.0, -trans[2], trans[1],
            trans[2], 0.0, -trans[0],
            -trans[1], trans[0], 0.0
        );

        let k1 = self.k_matrix_nalgebra;
        let k2 = k1.clone(); // TODO (Stereo) this should be whatever camera kf2 uses, the K1 above should be whatevere camera kf1 uses
        let f12 = k1.transpose().try_inverse().unwrap() * t12x * rot * k2.try_inverse().unwrap();

        // Epipolar line in second image l = x1'F12 = [a b c]
        let ptx = kp1.pt().x as f64;
        let pty = kp1.pt().y as f64;
        let a = ptx * f12[(0,0)] + pty * f12[(1,0)] + f12[(2,0)];
        let b = ptx * f12[(0,1)] + pty * f12[(1,1)] + f12[(2,1)];
        let c = ptx * f12[(0,2)] + pty * f12[(1,2)] + f12[(2,2)];

        let num = a * (kp2.pt().x as f64) + b * (kp2.pt().y as f64) + c;
        let den = a * a + b * b;
        if den == 0.0 {
            return false;
        }
        let dsqr = num * num / den;

        // Note: we're always doing these f64/f32 and i32/u32/usize conversions when there isn't
        // usually a good reason for why we have any variable be one or the other in the first place.
        dsqr < 3.84 * (unc as f64)
    }


}

impl Camera {
    pub fn new(camera_type: CameraType) -> Result<Camera, Box<dyn std::error::Error>> {
        // Implementation only for PinholeCamera, see:
        // - void Tracking::newParameterLoader
        // - GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam)
        // - void Settings::readCamera1(cv::FileStorage &fSettings) 
        let fx= SETTINGS.get::<f64>(CAMERA, "fx");
        let fy= SETTINGS.get::<f64>(CAMERA, "fy");
        let cx= SETTINGS.get::<f64>(CAMERA, "cx");
        let cy= SETTINGS.get::<f64>(CAMERA, "cy");

        // K as opencv matrix
        let mut k = Mat::new_rows_cols_with_default(3,3, CV_64F, Scalar::all(0.0))?;
        *k.at_2d_mut::<f64>(0, 0)? = fx;
        *k.at_2d_mut::<f64>(0, 2)? = cx;
        *k.at_2d_mut::<f64>(1, 1)? = fy;
        *k.at_2d_mut::<f64>(1, 2)? = cy;
        *k.at_2d_mut::<f64>(2, 2)? = 1.0;

        // K as nalgebra matrix
        // Could be an into() from opencv version if we made a KMatrix type but not sure if it's needed
        let k_matrix_nalgebra = nalgebra::Matrix3::new(
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        );

        let mut dist_coef = None;
        let sensor= SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let k1= SETTINGS.get::<f64>(CAMERA, "k1") as f32;
        if sensor.is_mono() && k1 != 0.0 {
            let k2 = SETTINGS.get::<f64>(CAMERA, "k2") as f32;
            let p1 = SETTINGS.get::<f64>(CAMERA, "p1") as f32;
            let p2 = SETTINGS.get::<f64>(CAMERA, "p2") as f32;
            let k3 = SETTINGS.get::<f64>(CAMERA, "k3") as f32;
            if k3 != 0.0 {
                dist_coef = Some(vec![k1, k2, p1, p2, k3]);
            } else {
                dist_coef = Some(vec![k1, k2, p1, p2]);
            }
        }

        let stereo_baseline_times_fx= SETTINGS.get::<f64>(CAMERA, "stereo_baseline_times_fx");
        let th_depth= SETTINGS.get::<i32>(CAMERA, "thdepth");
        let stereo_baseline = stereo_baseline_times_fx / fx;

        Ok(Camera {
            camera_type,
            k_matrix: DVMatrix::new(k),
            k_matrix_nalgebra,
            stereo_baseline_times_fx,
            stereo_baseline,
            th_depth,
            dist_coef,
            fx,
            fy,
            cx,
            cy,
            uncertainty: 1.0
        })
    }
}