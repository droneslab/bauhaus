use opencv::{
    prelude::*,
    core::*,
    features2d,
    features2d::{Feature2DTrait, ORB},
    highgui,
    imgproc,
    videoio,
    imgcodecs,
    types::{PtrOfORB, VectorOfKeyPoint},
};
use serde::{Deserialize, Serialize};
use std::convert::TryInto;
extern crate nalgebra as na;
use na::*;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct DarvisKeyPoint {
    p2f: Vec<f32>,
    size: f32,
    angle: f32,
    response: f32,
    octave: i32,
    class_id: i32
}

//Type defs to be used
pub type DVMatrixGrayscale = na::DMatrix<u8>;
pub type DVVectorOfKeyPoint = Vec<DarvisKeyPoint>;
pub type DVVector3 = na::Vector3<f64>;
pub type DVMatrix3 = na::Matrix3<f64>;



pub trait DarvisMatrix
{
    fn grayscale_to_cv_mat(&self) -> opencv::core::Mat;
    fn grayscale_mat(&self) -> na::DMatrix<u8>;
}


impl DarvisMatrix for opencv::core::Mat
{
    fn grayscale_to_cv_mat(&self) -> opencv::core::Mat
    {
        self.clone()
    }
    fn grayscale_mat(&self) -> na::DMatrix<u8>
    {
        let mut dmat = DMatrix::from_element(self.rows().try_into().unwrap(), self.cols().try_into().unwrap(), 0u8);
        for i in 0..self.rows() {
                for j in 0..self.cols() {
                       let val = *self.at_2d::<u8>(i, j).unwrap(); // Grayscale 1 channel uint8
                       let r: usize = i.try_into().unwrap();
                       let c: usize = j.try_into().unwrap();
                       dmat[(r, c)] = val;
                }
        }
        dmat      
    }
}


impl DarvisMatrix for na::DMatrix<u8>
{
    fn grayscale_to_cv_mat(&self) -> opencv::core::Mat
    {
        let mut mat = Mat::new_rows_cols_with_default(self.nrows().try_into().unwrap(),self.ncols().try_into().unwrap(),CV_8UC1, opencv::core::Scalar::all(0.0)).unwrap();

        for i in 0..self.nrows() {
                for j in 0..self.ncols() {
                        let r: usize = i.try_into().unwrap();
                        let c: usize = j.try_into().unwrap();
                        unsafe {*mat.at_2d_unchecked_mut::<u8>(i.try_into().unwrap(), j.try_into().unwrap()).unwrap() = self[(r, c)];}
                }
        }
        return mat;
    }
    fn grayscale_mat(&self) -> na::DMatrix<u8>
    {
        self.clone()     
    }
}






pub trait DarvisVectorOfKeyPoint
{
    fn cv_vector_of_keypoint(&self) -> VectorOfKeyPoint;
    fn darvis_vector_of_keypoint(&self) -> DVVectorOfKeyPoint;
}

impl DarvisVectorOfKeyPoint for VectorOfKeyPoint
{
    fn cv_vector_of_keypoint(&self) -> VectorOfKeyPoint
    {
        self.clone()
    }
    fn darvis_vector_of_keypoint(&self) -> DVVectorOfKeyPoint
    {

        let mut dmat_vkp = DVVectorOfKeyPoint::new();

        for i in 0..self.len() {
            let kp = self.get(i).unwrap();
            let dkp = DarvisKeyPoint {
                    p2f: vec![kp.pt.x, kp.pt.y],
                    size: kp.size,
                    angle: kp.angle,
                    response: kp.response,
                    octave: kp.octave,
                    class_id: kp.class_id
                    };
            //dmat_vkp[i.try_into().unwrap()] = dkp;
            dmat_vkp.push(dkp);
        }
    
        return dmat_vkp;        
    }
}


impl DarvisVectorOfKeyPoint for DVVectorOfKeyPoint
{
    fn cv_vector_of_keypoint(&self) -> VectorOfKeyPoint
    {
        let mut cv_vkp = VectorOfKeyPoint::new();

        for i in 0..self.len() {
            let dkp_instance = &self[i];
            let p2f = Point_::new(dkp_instance.p2f[0], dkp_instance.p2f[1]);
            let cvkp = KeyPoint::new_point(
                            p2f,
                            dkp_instance.size,
                            dkp_instance.angle,
                            dkp_instance.response,
                            dkp_instance.octave,
                            dkp_instance.class_id
                            );
            cv_vkp.push(cvkp.unwrap());
        }
    
        return cv_vkp;
    }
    fn darvis_vector_of_keypoint(&self) -> DVVectorOfKeyPoint
    {
        self.clone()       
    }    
}