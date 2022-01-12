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

#[derive(Debug, Serialize, Deserialize)]
pub struct DmatKeyPoint {
    p2f: Vec<f32>,
    size: f32,
    angle: f32,
    response: f32,
    octave: i32,
    class_id: i32
}



#[derive(Debug, Serialize, Deserialize, Clone, PartialEq, Copy)]
pub struct DVVec3f64(na::Vector3<f64>);

impl DVVec3f64
{
    pub fn zeros() -> Self
    {
        DVVec3f64(na::Vector3::<f64>::zeros())
    }

    pub fn from(vec : &na::Vector3::<f64>) -> Self
    {
        DVVec3f64(*vec)
    }
}

use std::ops::{Deref, DerefMut};

impl Deref for DVVec3f64
{
    type Target = na::Vector3::<f64>;
    fn deref(&self) -> &na::Vector3<f64>{&self.0}
}

impl DerefMut for DVVec3f64
{
    fn deref_mut(&mut self) -> &mut na::Vector3<f64>{&mut self.0}
}

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq, Copy)]
pub struct DVMat3f64(na::Matrix3<f64>);

impl DVMat3f64
{
    pub fn zeros() -> Self
    {
        DVMat3f64(na::Matrix3::<f64>::zeros())
    }

    pub fn from(mat : &na::Matrix3::<f64>) -> Self
    {
        DVMat3f64(*mat)
    }
}

impl Deref for DVMat3f64
{
    type Target = na::Matrix3::<f64>;
    fn deref(&self) -> &na::Matrix3<f64>{&self.0}
}

impl DerefMut for DVMat3f64
{
    fn deref_mut(&mut self) -> &mut na::Matrix3<f64>{&mut self.0}
}













#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct DVMat(na::DMatrix<u8>);

impl DVMat  {

    pub fn from(mat : &opencv::core::Mat) -> Self
    {  
        DVMat(DVMat::cv_mat_to_dmatrix_u8(mat))        
    }

    fn cv_mat_to_dmatrix_u8(mat: &Mat) -> DMatrix<u8> {
        // Iterate through image print pixel values
        // println!("{}", mat.rows());
        // println!("{}", mat.cols());
        
        let mut dmat = DMatrix::from_element(mat.rows().try_into().unwrap(), mat.cols().try_into().unwrap(), 0u8);
        for i in 0..mat.rows() {
                for j in 0..mat.cols() {
                       let val = *mat.at_2d::<u8>(i, j).unwrap(); // Grayscale 1 channel uint8
                       let r: usize = i.try_into().unwrap();
                       let c: usize = j.try_into().unwrap();
                       dmat[(r, c)] = val;
                }
        }
        return dmat;
    }

// Function to convert cv matrix to na matrix - For descriptors which is usually a 2D array
// Currently, type used is u8 because orb.detect_and_compute returns Mat with u8 type
// Can use mat.convert_to to convert the elements to the desired type. Need to check on the syntax of the function.
//cv_mat_to_na_grayscale
pub fn dmatrix_u8(&self) -> DMatrix<u8> {
    // Iterate through image print pixel values
    // println!("{}", mat.rows());
    // println!("{}", mat.cols());
    return self.clone_owned();
}

// Function to print matrix
pub fn print_matrix(&self) {
    let mat = self.cv_mat();
    for i in 0..mat.rows() {
        for j in 0..mat.cols() {
            let val = *mat.at_2d::<f64>(i, j).unwrap();
            print!("{:} ", val);
        }
        println!("");
    }

}

 pub fn cv_mat(&self) -> Mat {

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

}



impl Deref for DVMat
{
    type Target = na::DMatrix::<u8>;
    fn deref(&self) -> &na::DMatrix<u8>{&self.0}
}

impl DerefMut for DVMat
{
    fn deref_mut(&mut self) -> &mut na::DMatrix<u8>{&mut self.0}
}




#[derive(Debug, Serialize, Deserialize)]
pub struct DVVectorOfKeyPoint(Vec<DmatKeyPoint>);

impl DVVectorOfKeyPoint  {


    pub fn from(vkp: &VectorOfKeyPoint) -> Self
    {  
        DVVectorOfKeyPoint(DVVectorOfKeyPoint::cv_vector_of_keypoint_to_na(vkp))

    }

// Function to convert cv vector of keypoints to rust vector.
// Needed because KeyPoint is a structure with Point2f cv type holding the points.
// Need to convert the Point2f for serialization
// Currently using rust Vec type since its simpler and trivial than using DMatrix
    fn cv_vector_of_keypoint_to_na(vkp: &VectorOfKeyPoint) -> Vec<DmatKeyPoint> {

    //let mut dmat_vkp = na::DVector::<DmatKeyPoint>;//::from_element(vkp.len().try_into().unwrap(), dummy);

    let mut dmat_vkp = Vec::<DmatKeyPoint>::new();

    for i in 0..vkp.len() {
        let kp = vkp.get(i).unwrap();
        let dkp = DmatKeyPoint {
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



pub fn cv_vector_of_keypoint(&self) -> VectorOfKeyPoint {

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

}


impl Deref for DVVectorOfKeyPoint
{
    type Target = Vec<DmatKeyPoint>;
    fn deref(&self) -> &Vec::<DmatKeyPoint>{&self.0}
}

impl DerefMut for DVVectorOfKeyPoint
{
    fn deref_mut(&mut self) -> &mut Vec::<DmatKeyPoint>{&mut self.0}
}
