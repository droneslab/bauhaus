// Structs to wrap opencv functions, so we can implement traits on them.
// Watch out for thread safety here, opencv::core::Mat is not 
// inherently thread-safe so we have to be careful how we deal with it.

// Todo (one day): matrix multiplication/addition/etc doesn't work on these objects.
// Currently need to get struct.vec() and do the operations on that, ie
// struct.vec() + struct2.vec()
use std::{
    fmt::Debug, convert::TryInto, ops::{Add, Neg, Index}
};
use na::{DMatrix, ComplexField};
use opencv::{
    prelude::*, core::*,
    types::{VectorOfKeyPoint, VectorOfi32, VectorOfPoint2f},
};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
extern crate nalgebra as na;
use opencv::core::Mat;
use abow::Desc;


#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct DVKeyPoint {
    p2f: Vec<f32>,
    size: f32,
    angle: f32,
    response: f32,
    octave: i32,
    class_id: i32
}


// Note: only thread-safe because of clone in new() below.
// vec is private to prevent making this struct without
// using the constructor
#[derive(Clone, Debug)]
pub struct DVMatrix {
    mat: opencv::core::Mat
}
unsafe impl Sync for DVMatrix {}
impl DVMatrix {
    pub fn new(mat: opencv::core::Mat) -> Self {
        Self { mat: mat.clone() }
    }
    pub fn mat(&self) -> &opencv::core::Mat { &self.mat }
    pub fn row(&self, index: i32) -> Result<Mat, opencv::Error> { self.mat.row(index) }
}
impl Serialize for DVMatrix {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // let helper = MatrixGrid { 
        //     width: self.cols(),
        //     height: self.rows(),
        // }

        // let mut rows = serializer.serialize_seq(Some(helper.height))?;
        // for idx in 0..self.height {
        //     rows.serialize_element(
        //         &Row { grid: self, start_idx: idx * self.width }
        //     )?;
        // }
        // rows.end()

        // let mut dmat = DMatrix::from_element(self.rows().try_into().unwrap(), self.cols().try_into().unwrap(), 0u8);
        // for i in 0..self.rows() {
        //     for j in 0..self.cols() {
        //         let val = *self.at_2d::<u8>(i, j).unwrap(); // Grayscale 1 channel uint8
        //         let r: usize = i.try_into().unwrap();
        //         let c: usize = j.try_into().unwrap();
        //         seq.serialize_element(val);
        //         dmat[(r, c)] = val;
        //     }
        // }
        // dmat


        // let mut seq = serializer.serialize_seq(Some(self.len()))?;
        // for e in self {
        //     seq.serialize_element(e)?;
        // }
        // seq.end()


        // serializer.serialize_i32(*self)
        todo!("immediate: serialization for opencv matrix");
    }
}
impl<'de> Deserialize<'de> for DVMatrix {
    fn deserialize<D>(deserializer: D) -> Result<DVMatrix, D::Error>
    where
        D: Deserializer<'de>,
    {
        // deserializer.deserialize_i32(I32Visitor)
        todo!("immediate: deserialization for opencv matrix");
    }
}


// Note: only thread-safe because of clone in new() below.
// vec is private to prevent making this struct without
// using the constructor
#[derive(Clone, Debug)]
pub struct DVVectorOfKeyPoint {
    vec: opencv::types::VectorOfKeyPoint
}
unsafe impl Sync for DVVectorOfKeyPoint {}
impl DVVectorOfKeyPoint {
    pub fn empty() -> Self {
        Self { vec: VectorOfKeyPoint::new() }
    }
    pub fn new(vec: opencv::types::VectorOfKeyPoint) -> Self {
        Self { vec: vec.clone() }
    }

    pub fn len(&self) -> i32 { self.vec.len() as i32 }
    pub fn clone(&self) -> DVVectorOfKeyPoint {
        Self { vec: self.vec.clone() }
    }
    pub fn get(&self, index: usize) -> Result<KeyPoint, opencv::Error> { self.vec.get(index) }
    pub fn convert(&self, vec_of_points: &mut VectorOfPoint2f, index: &VectorOfi32) -> Result<(), opencv::Error> {
        opencv::core::KeyPoint::convert(&self.vec, vec_of_points, index)
    }

    pub fn clear(&mut self) { self.vec.clear() }
}
impl From<DVVectorOfKeyPoint> for opencv::types::VectorOfKeyPoint {
    fn from(vec: DVVectorOfKeyPoint) -> opencv::types::VectorOfKeyPoint {
        vec.vec
    }
}
impl Serialize for DVVectorOfKeyPoint {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // let helper = MatrixGrid { 
        //     width: self.cols(),
        //     height: self.rows(),
        // }

        // let mut rows = serializer.serialize_seq(Some(helper.height))?;
        // for idx in 0..self.height {
        //     rows.serialize_element(
        //         &Row { grid: self, start_idx: idx * self.width }
        //     )?;
        // }
        // rows.end()

        // let mut dmat = DMatrix::from_element(self.rows().try_into().unwrap(), self.cols().try_into().unwrap(), 0u8);
        // for i in 0..self.rows() {
        //     for j in 0..self.cols() {
        //         let val = *self.at_2d::<u8>(i, j).unwrap(); // Grayscale 1 channel uint8
        //         let r: usize = i.try_into().unwrap();
        //         let c: usize = j.try_into().unwrap();
        //         seq.serialize_element(val);
        //         dmat[(r, c)] = val;
        //     }
        // }
        // dmat


        // let mut seq = serializer.serialize_seq(Some(self.len()))?;
        // for e in self {
        //     seq.serialize_element(e)?;
        // }
        // seq.end()


        // serializer.serialize_i32(*self)
        todo!("immediate: serialization for opencv matrix");
    }
}
impl<'de> Deserialize<'de> for DVVectorOfKeyPoint {
    fn deserialize<D>(deserializer: D) -> Result<DVVectorOfKeyPoint, D::Error>
    where
        D: Deserializer<'de>,
    {
        // deserializer.deserialize_i32(I32Visitor)
        todo!("immediate: deserialization for opencv matrix");
    }
}


// Used to handle 3 dimensional column vector
#[derive(Clone, Debug)]
pub struct DVVector3<T> {
    vec: na::Vector3<T>
}
impl<T: Debug + Clone + Copy + na::Scalar + num_traits::identities::Zero + ComplexField> DVVector3<T> {
    // Constructors
    pub fn new(vec: na::Vector3<T>) -> Self {
        DVVector3 { vec: vec.clone() }
    }
    pub fn new_with(x: T, y: T, z: T) -> Self {
        DVVector3 { vec: na::Vector3::<T>::new(x,y,z) }
    }
    pub fn zeros<T2: Debug + Copy + Clone + na::Scalar + num_traits::identities::Zero>() -> Self {
        DVVector3 { vec: na::Vector3::<T>::zeros() }
    }
    pub fn clone(&self) -> Self {
        DVVector3 { vec: self.vec.clone() }
    }

    pub fn is_zero(&self) -> bool { self.vec == na::Vector3::<T>::zeros() }

    pub fn vec(&self) -> &na::Vector3<T> { &self.vec }

    // Sofiya note: I have no idea why the compiler hates these
    // Getting around it rn by calling struct.vec().norm() and 
    // struct.vec().div_assign()
    // pub fn norm(&self) -> T { self.vec.norm() }
    // pub fn div_assign(&mut self, divisor: T) { self.vec.div_assign(divisor) }
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
    fn from(vec: DVVector3<T>) -> na::Vector3<T> { vec.vec.clone() }
}
impl<T: Debug + Clone + na::Scalar + num_traits::identities::Zero> From<&DVVector3<T>> for na::Vector3<T> {
    fn from(vec: &DVVector3<T>) -> na::Vector3<T> { vec.vec.clone() }
}
impl<T> Index<usize> for DVVector3<T> {
    type Output = T;

    fn index(&self, i: usize) -> &Self::Output {
        &self.vec[i]
    }
}
impl<T> Serialize for DVVector3<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        todo!("immediate: serialization for opencv matrix");
    }
}
impl<'de, T> Deserialize<'de> for DVVector3<T> {
    fn deserialize<D>(deserializer: D) -> Result<DVVector3<T>, D::Error>
    where
        D: Deserializer<'de>,
    {
        todo!("immediate: deserialization for opencv matrix");
    }
}


/// Used to handle 3x3 matrix 
#[derive(Clone, Debug)]
pub struct DVMatrix3<T> {
    vec: na::Matrix3<T>
}
impl<T: Debug + Clone + na::Scalar + num_traits::identities::Zero + num_traits::One + ComplexField> DVMatrix3<T> {
    pub fn new(vec: na::Matrix3<T>) -> Self {
        DVMatrix3 { vec: vec.clone() }
    }
    pub fn zeros<T2: Debug + Clone + na::Scalar + num_traits::identities::Zero>() -> Self {
        DVMatrix3::new(na::Matrix3::<T>::zeros())
    }
    pub fn identity() -> Self { 
        DVMatrix3::new(na::Matrix3::<T>::identity())
    }
    pub fn is_zero(&self) -> bool { self.vec == na::Matrix3::<T>::zeros() }

    pub fn vec(&self) -> &na::Matrix3<T> { &self.vec }
    pub fn transpose(&self) -> na::Matrix3<T> { self.vec.transpose() }
    pub fn determinant(&self) -> T { self.vec.determinant() }

    pub fn neg_mut(&mut self) { self.vec.neg_mut() }
    // pub fn div_assign(&mut self, divisor: T) { self.vec.div_assign(divisor) }
}
// impl<T> Eq for DVMatrix3<T> {}
// impl<T> PartialEq for DVMatrix3<T> {
//     fn eq(&self, other: &Self) -> bool {
//         self.vec == other.vec
//     }
// }
impl<T: Clone> From<DVMatrix3<T>> for na::Matrix3<T> {
    fn from(vec: DVMatrix3<T>) -> na::Matrix3<T> { vec.vec.clone() }
}
impl<T: Clone> From<&DVMatrix3<T>> for na::Matrix3<T> {
    fn from(vec: &DVMatrix3<T>) -> na::Matrix3<T> { vec.vec.clone() }
}
impl<T> Index<(usize, usize)> for DVMatrix3<T> {
    type Output = T;

    fn index(&self, i: (usize, usize)) -> &Self::Output {
        &self.vec[i]
    }
}
impl<T> Index<(usize)> for DVMatrix3<T> {
    type Output = T;

    fn index(&self, i: (usize)) -> &Self::Output {
        &self.vec[i]
    }
}
impl<T> Serialize for DVMatrix3<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        todo!("immediate: serialization for opencv matrix");
    }
}
impl<'de, T> Deserialize<'de> for DVMatrix3<T> {
    fn deserialize<D>(deserializer: D) -> Result<DVMatrix3<T>, D::Error>
    where
        D: Deserializer<'de>,
    {
        todo!("immediate: deserialization for opencv matrix");
    }
}






/// Used to handle Grayscale images 
// TODO immediate: convert old code for darvismatrix below
// into the same structure as code above

// #[derive(Clone, Debug)]
// pub struct DVMatrixGrayscale {
//     mat: na::DMatrix<u8>
// }
// impl DVMatrixGrayscale {
//     pub fn new(vec: na::Matrix3<T>) -> Self {
//         DVMatrix3 { vec: vec.clone() }
//     }
// }
pub type DVMatrixGrayscale = na::DMatrix<u8>;

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

/// Trait implementation for OpenCV vector of KeyPoint
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

/// Trait implementation for Darvis vector of KeyPoint
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


pub struct Converter {}

impl Converter {
    //std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
    pub fn toDescriptorVector(Descriptors : &Mat) -> Vec<Desc> {
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
