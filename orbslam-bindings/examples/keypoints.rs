extern crate dvos3binding;

use std::{env, ops::Deref};

use cxx::UniquePtr;
use opencv::{imgcodecs, core::{KeyPoint}, prelude::{Mat, Boxed, MatTraitConst}, types::{VectorOfKeyPoint, VectorOfPoint2f, VectorOfi32}};

pub struct DVORBextractor {
    pub extractor: UniquePtr<dvos3binding::ffi::ORBextractor>,
    pub max_features: i32
}
impl DVORBextractor {
    pub fn new(max_features: i32) -> Self {
        DVORBextractor{
            max_features,
            extractor: dvos3binding::ffi::new_orb_extractor(
                max_features,
                1.2,
                8,
                20,
                7,
                // 0,
                // 1000
            )
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        panic!("
            [ERROR] Invalid number of input parameters.
            Usage: cargo run -- [PATH_TO_IMAGE]
        ");
    }
    let path = args[1].to_owned();

    let image = imgcodecs::imread(&path, imgcodecs::IMREAD_GRAYSCALE).expect("Could not read image.");
    let image_dv: dvos3binding::ffi::WrapBindCVMat = DVMatrix::new(image.clone()).into();

    // let params = Vector::new();
    // let _ = imgcodecs::imwrite("results/BLA.png", &image, &params).expect("Could not read image.");

    let mut descriptors: dvos3binding::ffi::WrapBindCVMat = DVMatrix::default().into();
    let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints = DVVectorOfKeyPoint::empty().into();
    let mut orb_extractor = DVORBextractor::new(2000*5);
    orb_extractor.extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);

    // This function reads 000001.png and prints output to image
    // orb_extractor.extractor.pin_mut().extract2(&mut keypoints, &mut descriptors);
}


#[derive(Clone, Debug, Default)]
pub struct DVMatrix (opencv::core::Mat);
unsafe impl Sync for DVMatrix {}
impl DVMatrix {
    pub fn empty() -> Self {
        Self ( Mat::default() )
    }
    pub fn new(mat: opencv::core::Mat) -> Self {
        Self ( mat )
    }
    pub fn mat(&self) -> &opencv::core::Mat { &self.0 }
    pub fn row(&self, index: u32) -> Result<Mat, opencv::Error> { self.0.row(index as i32) }
}
impl Deref for DVMatrix {
    type Target = opencv::core::Mat;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
// For interop with custom C++ bindings to ORBSLAM
impl From<DVMatrix> for dvos3binding::ffi::WrapBindCVMat {
    fn from(mat: DVMatrix) -> dvos3binding::ffi::WrapBindCVMat {
        dvos3binding::ffi::WrapBindCVMat { 
            mat_ptr: dvos3binding::BindCVMat {
                mat_ptr: mat.0
            } 
        }
    }
}
impl From<dvos3binding::ffi::WrapBindCVMat> for DVMatrix {
    fn from(mat: dvos3binding::ffi::WrapBindCVMat) -> DVMatrix {
        DVMatrix::new(mat.mat_ptr.mat_ptr)
    }
}
impl From<&DVMatrix> for dvos3binding::ffi::WrapBindCVRawPtr {
    fn from(mat: &DVMatrix) -> dvos3binding::ffi::WrapBindCVRawPtr {
        dvos3binding::ffi::WrapBindCVRawPtr { 
            raw_ptr: dvos3binding::BindCVRawPtr {
                raw_ptr: mat.0.as_raw()
            } 
        }
    }
}
impl<'a> From<&'a DVMatrix> for dvos3binding::BindCVMatRef<'a> {
    fn from(mat: &'a DVMatrix) -> dvos3binding::BindCVMatRef<'a> {
        dvos3binding::BindCVMatRef { 
            mat_ptr: mat
        }
    }
}


pub struct DVVectorOfKeyPoint ( opencv::types::VectorOfKeyPoint );
unsafe impl Sync for DVVectorOfKeyPoint {}
impl DVVectorOfKeyPoint {
    // Constructors
    pub fn empty() -> Self {
        Self ( VectorOfKeyPoint::new() )
    }
    pub fn new(vec: opencv::types::VectorOfKeyPoint) -> Self {
        Self ( vec )
    }
    pub fn clone(&self) -> DVVectorOfKeyPoint {
        Self ( self.0.clone() )
    }

    pub fn len(&self) -> i32 { self.0.len() as i32 }
    pub fn get(&self, index: usize) -> Result<KeyPoint, opencv::Error> { self.0.get(index) }
    pub fn convert(&self, vec_of_points: &mut VectorOfPoint2f, index: &VectorOfi32) -> Result<(), opencv::Error> {
        opencv::core::KeyPoint::convert(&self.0, vec_of_points, index)
    }

    pub fn clear(&mut self) { self.0.clear() }
}
impl Deref for DVVectorOfKeyPoint {
    type Target = opencv::types::VectorOfKeyPoint;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
// From implementations to make it easier to pass this into opencv functions
impl From<DVVectorOfKeyPoint> for opencv::types::VectorOfKeyPoint {
    fn from(vec: DVVectorOfKeyPoint) -> opencv::types::VectorOfKeyPoint { vec.0 }
}
// For interop with custom C++ bindings to ORBSLAM
impl From<DVVectorOfKeyPoint> for dvos3binding::ffi::WrapBindCVKeyPoints {
    fn from(kp: DVVectorOfKeyPoint) -> dvos3binding::ffi::WrapBindCVKeyPoints {
        dvos3binding::ffi::WrapBindCVKeyPoints { 
            kp_ptr: dvos3binding::BindCVKeyPoints {
                kp_ptr: kp.0
            } 
        }
    }
}
impl From<dvos3binding::ffi::WrapBindCVKeyPoints> for DVVectorOfKeyPoint {
    fn from(kp: dvos3binding::ffi::WrapBindCVKeyPoints) -> DVVectorOfKeyPoint {
        DVVectorOfKeyPoint::new(kp.kp_ptr.kp_ptr)
    }
}
impl From<&DVVectorOfKeyPoint> for dvos3binding::ffi::WrapBindCVRawPtr {
    fn from(kp: &DVVectorOfKeyPoint) -> dvos3binding::ffi::WrapBindCVRawPtr {
        dvos3binding::ffi::WrapBindCVRawPtr { 
            raw_ptr: dvos3binding::BindCVRawPtr {
                raw_ptr: kp.0.as_raw()
            } 
        }
    }
}

impl<'a> From<&'a DVVectorOfKeyPoint> for dvos3binding::BindCVKeyPointsRef<'a> {
    fn from(kp: &'a DVVectorOfKeyPoint) -> dvos3binding::BindCVKeyPointsRef<'a> {
        dvos3binding::BindCVKeyPointsRef { 
            kp_ptr: kp
        }
    }
}

