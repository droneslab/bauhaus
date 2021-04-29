extern crate nalgebra as na;

#[allow(unused_imports)]
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
use na::*;
use std::convert::TryInto;


pub struct DmatKeyPoint {
    p2f: Vec<f32>,
    size: f32,
    angle: f32,
    response: f32,
    octave: i32,
    class_id: i32
}

// Function to convert cv matrix to na matrix - For descriptors which is usually a 2D array
// Currently, type used is u8 because orb.detect_and_compute returns Mat with u8 type
// Can use mat.convert_to to convert the elements to the desired type. Need to check on the syntax of the function.

pub fn cv_mat_to_na_grayscale(mat: Mat) -> na::DMatrix<u8> {
    // Iterate through image print pixel values
    // println!("{}", mat.rows());
    // println!("{}", mat.cols());
    let mut dmat = na::DMatrix::from_element(mat.rows().try_into().unwrap(), mat.cols().try_into().unwrap(), 0u8);
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


// Function to convert cv vector of keypoints to rust vector.
// Needed because KeyPoint is a structure with Point2f cv type holding the points.
// Need to convert the Point2f for serialization
// Currently using rust Vec type since its simpler and trivial than using DMatrix

pub fn cv_vector_of_keypoint_to_na(vkp: VectorOfKeyPoint) -> Vec<DmatKeyPoint> {

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

pub fn na_grayscale_to_cv_mat(dmat: na::DMatrix<f64>) -> Mat {
    let mut mat = Mat::new_rows_cols_with_default(dmat.nrows().try_into().unwrap(),dmat.ncols().try_into().unwrap(),CV_64FC1, opencv::core::Scalar::all(1.0)).unwrap();

    for i in 0..dmat.nrows() {
            for j in 0..dmat.ncols() {
                    let r: usize = i.try_into().unwrap();
                    let c: usize = j.try_into().unwrap();
                    unsafe {*mat.at_2d_unchecked_mut::<f64>(i.try_into().unwrap(), j.try_into().unwrap()).unwrap() = dmat[(r, c)];}
            }
    }
    return mat;

}
