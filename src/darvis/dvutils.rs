use na::DMatrix;
use opencv::{
    prelude::*,
    core::*,
    types::{VectorOfKeyPoint},
};
use serde::{Deserialize, Serialize};
use std::convert::TryInto;
extern crate nalgebra as na;
//use na::*;
use opencv::core::Mat;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct DarvisKeyPoint {
    p2f: Vec<f32>,
    size: f32,
    angle: f32,
    response: f32,
    octave: i32,
    class_id: i32
}

use cv_convert::{FromCv, IntoCv, TryFromCv, TryIntoCv};

/// Used to handle Grayscale images 
pub type DVMatrixGrayscale = na::DMatrix<u8>;
/// Used to handle Vector of KeyPoint
pub type DVVectorOfKeyPoint = Vec<DarvisKeyPoint>;
/// Used to handle 3 dimentional column vector
pub type DVVector3 = na::Vector3<f64>;
/// Used to handle 3x3 matrix 
pub type DVMatrix3 = na::Matrix3<f64>;

/// Traits useful fo converting between OPENCV and Algebra library format.
pub trait DarvisMatrix {
    fn grayscale_to_cv_mat(&self) -> opencv::core::Mat;
    fn grayscale_mat(&self) -> DVMatrixGrayscale;
}

/// Matrix Trait implementation for OpenCV Mat
impl DarvisMatrix for opencv::core::Mat {
    /// get OpenCV Mat
    fn grayscale_to_cv_mat(&self) -> opencv::core::Mat {
        self.clone()
    }

    /// Convert OpenCV mat to Grayscale matrix
    fn grayscale_mat(&self) -> DVMatrixGrayscale {
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

/// Matrix Trait implementation for Algebra Matrix
impl DarvisMatrix for DVMatrixGrayscale {
    /// Convert Grayscale matrix to OpenCV mat
    fn grayscale_to_cv_mat(&self) -> opencv::core::Mat {
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

    /// Get Algebra Matrix
    fn grayscale_mat(&self) -> DVMatrixGrayscale {
        self.clone()
    }
}

/// Traits useful fo converting between OPENCV and Darvis format of Keypoint.
pub trait DarvisVectorOfKeyPoint {
    fn cv_vector_of_keypoint(&self) -> VectorOfKeyPoint;
    fn darvis_vector_of_keypoint(&self) -> DVVectorOfKeyPoint;
}

/// Trait implementation for OpenCV vector of KeyPoint
impl DarvisVectorOfKeyPoint for VectorOfKeyPoint {
    /// get OpenCV vector of KeyPoint
    fn cv_vector_of_keypoint(&self) -> VectorOfKeyPoint {
        self.clone()
    }
    /// Convert opencv to Darvis vector of KeyPoint
    fn darvis_vector_of_keypoint(&self) -> DVVectorOfKeyPoint {
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

/// Trait implementation for Darvis vector of KeyPoint
impl DarvisVectorOfKeyPoint for DVVectorOfKeyPoint {
    /// Convert Darvis to opencv vector of KeyPoint
    fn cv_vector_of_keypoint(&self) -> VectorOfKeyPoint {
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
    /// get Darvis vector of KeyPoint
    fn darvis_vector_of_keypoint(&self) -> DVVectorOfKeyPoint {
        self.clone()
    }
}



// Functions to serialize and deserialize Mat objects, since they cannot be 
// inferred with serde.

// // Serde calls this the definition of the remote type. It is just a copy of the
// // remote data structure. The `remote` attribute gives the path to the actual
// // type we intend to derive code for.
// #[derive(Serialize, Deserialize)]
// #[serde(remote = "Mat")]
// struct DurationDef {
//     secs: i64,
//     nanos: i32,
// }

// // Now the remote type can be used almost like it had its own Serialize and
// // Deserialize impls all along. The `with` attribute gives the path to the
// // definition for the remote type. Note that the real type of the field is the
// // remote type, not the definition type.
// #[derive(Serialize, Deserialize)]
// struct Process {
//     command_line: String,

//     #[serde(with = "DurationDef")]
//     wall_time: Duration,
// }


pub struct Converter{

}

use abow::Desc;

impl Converter {
    //std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
    pub fn toDescriptorVector(Descriptors : &Mat) -> Vec<Desc>
    {
        let mut vDesc = vec![];
        vDesc.reserve(Descriptors.rows() as usize);

        let desc_vec = Descriptors.to_vec_2d::<u8>().unwrap();

        for j in 0..Descriptors.rows()
        {
            let desc_j = desc_vec.get(j as usize).unwrap();
            let mut desc_val : Desc = [0; 32];
            for (&x, p) in desc_j.iter().zip(desc_val.iter_mut()) {
                *p = x;
            }
            vDesc.push(desc_val);

        }

        return vDesc;
    }
}
