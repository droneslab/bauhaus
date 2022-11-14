/// *** Structs to wrap opencv/nalgebra objects, so we can implement traits on them. *** ///
///
/// Watch out for thread safety here, opencv::core::Mat is not 
/// inherently thread-safe so we have to be careful how we deal with it.
/// Current strategy is to hide the inner matrices and require creation
/// of the struct to go through a new() constructor, which clones the
/// incoming matrix so it definitely won't point to the same underlying data.
///
/// Note: multiplication/addition/etc doesn't work on these objects.
/// Currently two options:
/// 1 - struct.vec() + struct2.vec()
/// 2 - do operations on interim opencv/nalgebra structs, then save to DV struct.
/// Currently using option 2 more. First option doesn't work when it requires
/// that the shape constraint trait is satisfied, which is private in nalgebra...
/// We can fix this but honestly I don't think it's worth it.

use std::{fmt::Debug, convert::TryInto, ops::Index};
use std::ops::Deref;
use cxx::{UniquePtr, let_cxx_string};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

use na::{DMatrix, ComplexField};
use opencv::{
    prelude::*, core::*,
    types::{VectorOfKeyPoint, VectorOfi32, VectorOfPoint2f, VectorOfPoint3f},
};
extern crate nalgebra as na;

//////////////////////////* OPENCV TYPES //////////////////////////

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct DVKeyPoint {
    p2f: Vec<f32>,
    size: f32,
    angle: f32,
    response: f32,
    octave: i32,
    class_id: i32
}

#[derive(Clone, Debug, Default)]
pub struct DVMatrix (opencv::core::Mat);
unsafe impl Sync for DVMatrix {}
impl DVMatrix {
    pub fn empty() -> Self {
        Self ( Mat::default() )
    }
    pub fn new(mat: opencv::core::Mat) -> Self {
        Self ( mat.clone() )
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
impl Serialize for DVMatrix {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
        todo!("(low priority) serialize");
        }
}
impl<'de> Deserialize<'de> for DVMatrix {
    fn deserialize<D>(deserializer: D) -> Result<DVMatrix, D::Error>
    where
        D: Deserializer<'de>,
    {
        todo!("(low priority) deserialize");
    }
}
// impl From<DVMatrix> for *const dvos3binding::ffi::DVMat {
//     fn from(mat: DVMatrix) -> Self { 
//         mat.into_raw() as *const dvos3binding::ffi::DVMat
//     }
// }


#[derive(Clone, Debug, Default)]
pub struct DVVectorOfKeyPoint ( opencv::types::VectorOfKeyPoint );
unsafe impl Sync for DVVectorOfKeyPoint {}
impl DVVectorOfKeyPoint {
    // Constructors
    pub fn empty() -> Self {
        Self ( VectorOfKeyPoint::new() )
    }
    pub fn new(vec: opencv::types::VectorOfKeyPoint) -> Self {
        Self ( vec.clone() )
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
impl Serialize for DVVectorOfKeyPoint {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        todo!("(low priority) serialize");
    }
}
impl<'de> Deserialize<'de> for DVVectorOfKeyPoint {
    fn deserialize<D>(deserializer: D) -> Result<DVVectorOfKeyPoint, D::Error>
    where
        D: Deserializer<'de>,
    {
        todo!("(low priority) deserialize");
    }
}


#[derive(Clone, Debug, Default)]
pub struct DVVectorOfPoint3f ( opencv::types::VectorOfPoint3f );
unsafe impl Sync for DVVectorOfPoint3f {}
impl DVVectorOfPoint3f {
    // Constructors
    pub fn empty() -> Self {
        Self ( VectorOfPoint3f::new() )
    }
    pub fn new(vec: VectorOfPoint3f) -> Self {
        Self ( vec.clone() ) 
    }
    pub fn clone(&self) -> DVVectorOfPoint3f {
        Self ( self.0.clone() )
    }
    pub fn get(&self, i: usize) -> Result<opencv::core::Point3f, opencv::Error> {
        self.0.get(i)
    }

    pub fn clear(&mut self) { self.0.clear() }
}


//////////////////////////* Nalgebra TYPES //////////////////////////

#[derive(Clone, Copy, Debug)]
pub struct DVVector3<T> ( na::Vector3<T> ); // 3 dimensional column vector

impl<T: Debug + Clone + na::Scalar + num_traits::identities::Zero + ComplexField> DVVector3<T> {
    // Constructors
    pub fn new(vec: na::Vector3<T>) -> Self {
        DVVector3 ( vec.clone() )
    }
    pub fn new_with(x: T, y: T, z: T) -> Self {
        DVVector3 ( na::Vector3::<T>::new(x,y,z) )
    }
    pub fn zeros<T2: Debug + Clone + na::Scalar + num_traits::identities::Zero>() -> Self {
        DVVector3 ( na::Vector3::<T>::zeros() )
    }
    pub fn clone(&self) -> Self {
        DVVector3 ( self.0.clone() )
    }

    pub fn is_zero(&self) -> bool { self.0 == na::Vector3::<T>::zeros() }

    // Note: I have no idea why the compiler hates these
    // Getting around it rn by calling *struct.norm() and *struct.div_assign()
    // pub fn norm(&self) -> T { self.vec.norm() }
    // pub fn div_assign(&mut self, divisor: T) { self.vec.div_assign(divisor) }
}
impl<T> Deref for DVVector3<T> {
    type Target = na::Vector3<T>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
// From implementations ... not sure why I need to have a separate 
// from for DVVector3 and &DVVector3 though.
impl<T: Debug + Clone + Copy + na::Scalar + num_traits::identities::Zero + ComplexField> From<DVVector3<T>> for [T; 3] {
    fn from(vec: DVVector3<T>) -> [T; 3] { [vec[0], vec[1], vec[2]] }
}
impl<T: Debug + Clone + Copy + na::Scalar + num_traits::identities::Zero + ComplexField> From<&DVVector3<T>> for [T; 3] {
    fn from(vec: &DVVector3<T>) -> [T; 3] { [vec[0], vec[1], vec[2]] }
}
impl<T: Debug + Clone + na::Scalar + num_traits::identities::Zero> From<DVVector3<T>> for na::Vector3<T> {
    fn from(vec: DVVector3<T>) -> na::Vector3<T> { vec.0.clone() }
}
impl<T: Debug + Clone + na::Scalar + num_traits::identities::Zero> From<&DVVector3<T>> for na::Vector3<T> {
    fn from(vec: &DVVector3<T>) -> na::Vector3<T> { vec.0.clone() }
}
impl From<DVVector3<f32>> for [f64; 3] {
    fn from(vec: DVVector3<f32>) -> [f64; 3] { [vec[0] as f64, vec[1] as f64, vec[2] as f64] }
}
// So we can do vector3[i] without having to call a getter or setter function
impl<T> Index<usize> for DVVector3<T> {
    type Output = T;
    fn index(&self, i: usize) -> &Self::Output { &self.0[i] }
}
impl<T> Serialize for DVVector3<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        todo!("(low priority) serialize");
    }
}
impl<'de, T> Deserialize<'de> for DVVector3<T> {
    fn deserialize<D>(deserializer: D) -> Result<DVVector3<T>, D::Error>
    where
        D: Deserializer<'de>,
    {
        todo!("(low priority) deserialize");
    }
}


#[derive(Clone, Debug)]
pub struct DVMatrix3<T> ( na::Matrix3<T> ); // 3x3 matrix

impl<T: Debug + Clone + na::Scalar + num_traits::identities::Zero + num_traits::One + ComplexField> DVMatrix3<T> {
    // Constructors
    pub fn new(vec: na::Matrix3<T>) -> Self {
        DVMatrix3 ( vec.clone() )
    }
    pub fn zeros<T2: Debug + Clone + na::Scalar + num_traits::identities::Zero>() -> Self {
        DVMatrix3::new(na::Matrix3::<T>::zeros())
    }
    pub fn identity() -> Self { 
        DVMatrix3::new(na::Matrix3::<T>::identity())
    }

    pub fn is_zero(&self) -> bool { self.0 == na::Matrix3::<T>::zeros() }

    pub fn vec(&self) -> &na::Matrix3<T> { &self.0 }
    pub fn transpose(&self) -> na::Matrix3<T> { self.0.transpose() }
    pub fn determinant(&self) -> T { self.0.determinant() }

    pub fn neg_mut(&mut self) { self.0.neg_mut() }
}
impl<T> Deref for DVMatrix3<T> {
    type Target = na::Matrix3<T>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
// From implementations...
impl<T: Clone> From<DVMatrix3<T>> for na::Matrix3<T> {
    fn from(vec: DVMatrix3<T>) -> na::Matrix3<T> { vec.0.clone() }
}
impl<T: Clone> From<&DVMatrix3<T>> for na::Matrix3<T> {
    fn from(vec: &DVMatrix3<T>) -> na::Matrix3<T> { vec.0.clone() }
}
// Two index implementations, one for matrix3[(x,y)] and one for [(x)]
impl<T> Index<(usize, usize)> for DVMatrix3<T> {
    type Output = T;

    fn index(&self, i: (usize, usize)) -> &Self::Output {
        &self.0[i]
    }
}
impl<T> Index<usize> for DVMatrix3<T> {
    type Output = T;

    fn index(&self, i: usize) -> &Self::Output {
        &self.0[i]
    }
}
impl<T> Serialize for DVMatrix3<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        todo!("(low priority) serialize");
    }
}
impl<'de, T> Deserialize<'de> for DVMatrix3<T> {
    fn deserialize<D>(deserializer: D) -> Result<DVMatrix3<T>, D::Error>
    where
        D: Deserializer<'de>,
    {
        todo!("(low priority) deserialize");
    }
}


#[derive(Clone, Debug)]
pub struct DVMatrixGrayscale ( na::DMatrix<u8> );

impl DVMatrixGrayscale {
    // Constructors
    pub fn new(vec: na::DMatrix<u8>) -> Self {
        DVMatrixGrayscale ( vec.clone() )
    }
}
// &DVMatrixGrayscale implemented instead of DVMatrixGrayscale because this avoids having to take 
// ownership of the matrix when calling .into() ... this is useful because this matrix is usually
// inside an Arc<ImageMsg> and you cannot move out of an arc. Alternatively, we could copy the matrix
// but that takes too much time/memory.
impl From<&DVMatrixGrayscale> for opencv::core::Mat {
    fn from(mat: &DVMatrixGrayscale) -> opencv::core::Mat {
        let mut new_mat = Mat::new_rows_cols_with_default(mat.0.nrows().try_into().unwrap(),mat.0.ncols().try_into().unwrap(),CV_8UC1, opencv::core::Scalar::all(0.0)).unwrap();

        for i in 0..mat.0.nrows() {
            for j in 0..mat.0.ncols() {
                let r: usize = i.try_into().unwrap();
                let c: usize = j.try_into().unwrap();
                unsafe {*new_mat.at_2d_unchecked_mut::<u8>(i.try_into().unwrap(), j.try_into().unwrap()).unwrap() = mat.0[(r, c)];}
            }
        }
        new_mat
    }
}
impl From<opencv::core::Mat> for DVMatrixGrayscale {
    fn from(mat: opencv::core::Mat) -> DVMatrixGrayscale {
        let mut dmat = DMatrix::from_element(mat.rows().try_into().unwrap(), mat.cols().try_into().unwrap(), 0u8);
        for i in 0..mat.rows() {
            for j in 0..mat.cols() {
                let val = *mat.at_2d::<u8>(i, j).unwrap(); // Grayscale 1 channel uint8
                let r: usize = i.try_into().unwrap();
                let c: usize = j.try_into().unwrap();
                dmat[(r, c)] = val;
            }
        }
        DVMatrixGrayscale(dmat)
    }
}



// This is from the old implementation of DarvisVectorOfKeyPoint

// Trait implementation for OpenCV vector of KeyPoint
// impl DarvisVectorOfKeyPoint for VectorOfKeyPoint {
//     /// get OpenCV vector of KeyPoint
//     fn cv_vector_of_keypoint(&self) -> VectorOfKeyPoint {
//         self.clone()
//     }
//     /// Convert opencv to Darvis vector of KeyPoint
//     fn darvis_vector_of_keypoint(&self) -> DVVectorOfKeyPoint {
//         let mut dmat_vkp = DVVectorOfKeyPoint::new();

//         for i in 0..self.len() {
//             let kp = self.get(i).unwrap();
//             let dkp = DVKeyPoint {
//                 p2f: vec![kp.pt.x, kp.pt.y],
//                 size: kp.size,
//                 angle: kp.angle,
//                 response: kp.response,
//                 octave: kp.octave,
//                 class_id: kp.class_id
//             };
//             //dmat_vkp[i.try_into().unwrap()] = dkp;
//             dmat_vkp.push(dkp);
//         }
//         return dmat_vkp;
//     }
// }

// Trait implementation for Darvis vector of KeyPoint
// impl DarvisVectorOfKeyPoint for DVVectorOfKeyPoint {
//     /// Convert Darvis to opencv vector of KeyPoint
//     fn cv_vector_of_keypoint(&self) -> VectorOfKeyPoint {
//         let mut cv_vkp = VectorOfKeyPoint::new();

//         for i in 0..self.len() {
//             let dkp_instance = &self[i];
//             let p2f = Point_::new(dkp_instance.p2f[0], dkp_instance.p2f[1]);
//             let cvkp = KeyPoint::new_point(
//                 p2f,
//                 dkp_instance.size,
//                 dkp_instance.angle,
//                 dkp_instance.response,
//                 dkp_instance.octave,
//                 dkp_instance.class_id
//             );
//             cv_vkp.push(cvkp.unwrap());
//         }
//         return cv_vkp;
//     }
//     /// get Darvis vector of KeyPoint
//     fn darvis_vector_of_keypoint(&self) -> DVVectorOfKeyPoint {
//         self.clone()
//     }
// }
