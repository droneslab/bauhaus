/// *** Structs to wrap opencv/nalgebra objects, so we can implement traits on them. *** ///
///
/// Watch out for thread safety here, opencv::core::Mat is not 
/// inherently thread-safe so we have to be careful how we deal with it.
/// Current strategy is to hide the inner matrices and require creation
/// of the struct to go through a new() constructor, which consumes the
/// incoming matrix so it definitely won't point to the same underlying data.
///
/// Note: multiplication/addition/etc doesn't work on these objects.
/// Currently two options:
/// 1 - struct.vec() + struct2.vec()
/// 2 - do operations on interim opencv/nalgebra structs, then save to DV struct.
/// Currently using option 2 more. First option doesn't work when it requires
/// that the shape constraint trait is satisfied, which is private in nalgebra...
/// We can fix this but honestly I don't think it's worth it.

// TODO (timing) ... clones in matrix
// Lot of clones in here, a large part of them are converting 
// from a darvis structure back into nalgebra. Does it make sense to clone in
// this case? That feels correct because then I can hold both the darvis structure
// and the nalgebra structure without pointing to the same underlying data,
// but that might actually not be intuitive if we expect to use into() to convert
// rather than make copies.

use std::{fmt::Debug, convert::TryInto, ops::Index};
use std::ops::{Deref, DerefMut};
use opencv::platform_types::size_t;
use serde::{Deserialize, Serialize};

use opencv::{
    prelude::*, core::*, hub_prelude::MatExprTraitConst
};
// extern crate nalgebra as na;

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
        Self ( mat )
    }
    pub fn new_expr(mat: opencv::core::MatExpr) -> Self {
        Self ( mat.to_mat().unwrap() )
    }
    pub fn new_expr_res<T: opencv::prelude::MatExprTraitConst>(mat: opencv::core::MatExprResult<T>) -> Self {
        Self ( mat.into_result().unwrap().to_mat().unwrap() )
    }

    pub fn new_clone(mat: &opencv::core::Mat) -> Self {
        Self ( mat.clone() )
    }
    pub fn mat(&self) -> &opencv::core::Mat { &self.0 }
    pub fn row(&self, index: u32) -> DVMatrix {
        DVMatrix::new(
            self.0.row(index as i32).unwrap()
        )
    }
    pub fn row_range(&self, start: i32, end: i32) -> DVMatrix {
        DVMatrix::new(
            self.0.row_range(&Range::new(start,end).unwrap()).unwrap()
        )
    }
    pub fn col_range(&self, start: i32, end: i32) -> DVMatrix {
        DVMatrix::new(
            self.0.col_range(&Range::new(start,end).unwrap()).unwrap()
    )
    }
    pub fn divide_by_scalar(&self, scalar: f32) -> DVMatrix {
        // Should be equal to self.0 / scalar
        DVMatrix::new(
            self.0.mul(&((1.0 / scalar) as f64), 1.).unwrap().to_mat().unwrap()
        )
    }
    pub fn neg(&self) -> DVMatrix {
        DVMatrix::new(
            self.0.mul(&(-1.0 as f64), 1.).unwrap().to_mat().unwrap()
        )
    }
    pub fn norm(&self) -> f64 {
        norm(&self.0, NORM_L2, &Mat::default()).unwrap()
    }
    pub fn at(&self, index: i32) -> f64 {
        *self.0.at::<f64>(index).unwrap()
    }
    pub fn at_2d(&self, row: i32, col: i32) -> f64 {
        *self.0.at_2d::<f64>(row, col).unwrap()
    }

}
impl Deref for DVMatrix {
    type Target = opencv::core::Mat;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl std::ops::Mul<&DVMatrix> for DVMatrix {
    type Output = DVMatrix;

    fn mul(self, rhs: &DVMatrix) -> DVMatrix {
        DVMatrix::new_expr_res(self.mat() * rhs.mat())
    }
}

// For interop with custom C++ bindings to ORBSLAM
impl From<&DVMatrix> for dvos3binding::ffi::WrapBindCVMat {
    fn from(mat: &DVMatrix) -> dvos3binding::ffi::WrapBindCVMat {
        dvos3binding::ffi::WrapBindCVMat { 
            mat_ptr: dvos3binding::BindCVMat {
                mat_ptr: mat.0.clone()
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
impl From<&DVVector3<f64>> for opencv::core::Mat {
    fn from(mat: &DVVector3<f64>) -> opencv::core::Mat { 
        Mat::from_slice_2d(&[
            [mat[0]],
            [mat[1]],
            [mat[2]],
        ]).unwrap()
    }
}

impl From<&DVMatrix3<f64>> for opencv::core::Mat {
    fn from(mat: &DVMatrix3<f64>) -> opencv::core::Mat { 
        Mat::from_slice_2d(&[
            [mat[0], mat[3], mat[6]],
            [mat[1], mat[4], mat[7]],
            [mat[2], mat[5], mat[8]]
        ]).unwrap()
    }
}
impl<T: opencv::prelude::DataType> From<&DVMatrix4<T>> for DVMatrix {
    fn from(mat: &DVMatrix4<T>) -> DVMatrix { 
        DVMatrix::new(
            Mat::from_slice_2d(&[
                [mat[0], mat[1], mat[2], mat[3]],
                [mat[4], mat[5], mat[6], mat[7]],
                [mat[8], mat[9], mat[10], mat[11]],
                [mat[12], mat[13], mat[14], mat[15]]
            ]).unwrap()
        )
    }
}
impl<T: nalgebra::ComplexField> From<[T; 3]> for DVVector3<T> {
    fn from(vec: [T; 3]) -> DVVector3<T> { 
        DVVector3::new(nalgebra::Vector3::<T>::new(vec[0].clone(), vec[1].clone(), vec[2].clone()))
    }
}

#[derive(Clone, Debug, Default)]
pub struct DVVectorOfKeyPoint ( opencv::types::VectorOfKeyPoint );
unsafe impl Sync for DVVectorOfKeyPoint {}
impl DVVectorOfKeyPoint {
    // Constructors
    pub fn empty() -> Self {
        Self ( opencv::types::VectorOfKeyPoint::new() )
    }
    pub fn new(vec: opencv::types::VectorOfKeyPoint) -> Self {
        Self ( vec )
    }
    pub fn clone(&self) -> DVVectorOfKeyPoint {
        Self ( self.0.clone() )
    }

    pub fn len(&self) -> i32 { self.0.len() as i32 }
    pub fn get(&self, index: usize) -> Result<KeyPoint, opencv::Error> { self.0.get(index) }
    pub fn convert(&self, vec_of_points: &mut opencv::types::VectorOfPoint2f, index: &opencv::types::VectorOfi32) -> Result<(), opencv::Error> {
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
impl DerefMut for DVVectorOfKeyPoint {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
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



#[derive(Clone, Debug, Default)]
pub struct DVVectorOfPoint3f ( opencv::types::VectorOfPoint3f );
unsafe impl Sync for DVVectorOfPoint3f {}
impl DVVectorOfPoint3f {
    // Constructors
    pub fn empty() -> Self {
        Self ( opencv::types::VectorOfPoint3f::new() )
    }
    pub fn new(vec: opencv::types::VectorOfPoint3f) -> Self {
        Self ( vec ) 
    }
    pub fn clone(&self) -> DVVectorOfPoint3f {
        Self ( self.0.clone() )
    }
    pub fn get(&self, i: usize) -> Result<opencv::core::Point3f, opencv::Error> {
        self.0.get(i)
    }

    pub fn clear(&mut self) { self.0.clear() }

    pub fn push(&mut self, pt: opencv::core::Point3f)
    {
        self.0.push(pt);
    }
}
// For interop with custom C++ bindings to ORBSLAM
impl From<DVVectorOfPoint3f> for dvos3binding::ffi::WrapBindCVVectorOfPoint3f {
    fn from(vec: DVVectorOfPoint3f) -> dvos3binding::ffi::WrapBindCVVectorOfPoint3f {
        dvos3binding::ffi::WrapBindCVVectorOfPoint3f { 
            vec_ptr: dvos3binding::BindCVVectorOfPoint3f {
                vec_ptr: vec.0
            } 
        }
    }
}
impl From<dvos3binding::ffi::WrapBindCVVectorOfPoint3f> for DVVectorOfPoint3f {
    fn from(vec: dvos3binding::ffi::WrapBindCVVectorOfPoint3f) -> DVVectorOfPoint3f {
        DVVectorOfPoint3f::new(vec.vec_ptr.vec_ptr)
    }
}

#[derive(Clone, Debug, Default)]
pub struct DVVectorOfPoint2f ( opencv::types::VectorOfPoint2f );
unsafe impl Sync for DVVectorOfPoint2f {}
impl DVVectorOfPoint2f {
    pub fn empty() -> Self {
        Self ( opencv::types::VectorOfPoint2f::new() )
    }
    pub fn new(vec: opencv::types::VectorOfPoint2f) -> Self {
        Self ( vec ) 
    }
    pub fn len(&self) -> i32 { self.0.len() as i32 }
    pub fn clone(&self) -> Self {
        Self ( self.0.clone() )
    }
    pub fn get(&self, i: usize) -> Result<opencv::core::Point2f, opencv::Error> {
        self.0.get(i)
    }
    pub fn clear(&mut self) { self.0.clear() }
    pub fn push(&mut self, pt: opencv::core::Point2f) {
        self.0.push(pt);
    }
}
// For interop with custom C++ bindings to ORBSLAM
impl From<DVVectorOfPoint2f> for dvos3binding::ffi::WrapBindCVVectorOfPoint2f {
    fn from(vec: DVVectorOfPoint2f) -> dvos3binding::ffi::WrapBindCVVectorOfPoint2f {
        dvos3binding::ffi::WrapBindCVVectorOfPoint2f { 
            vec_ptr: dvos3binding::BindCVVectorOfPoint2f {
                vec_ptr: vec.0
            } 
        }
    }
}
impl From<dvos3binding::ffi::WrapBindCVVectorOfPoint2f> for DVVectorOfPoint2f {
    fn from(vec: dvos3binding::ffi::WrapBindCVVectorOfPoint2f) -> DVVectorOfPoint2f {
        DVVectorOfPoint2f::new(vec.vec_ptr.vec_ptr)
    }
}
impl<'a> From<&'a mut DVVectorOfPoint2f> for dvos3binding::BindCVVectorOfPoint2fRef<'a> {
    fn from(vec: &'a mut DVVectorOfPoint2f) -> dvos3binding::BindCVVectorOfPoint2fRef<'a> {
        dvos3binding::BindCVVectorOfPoint2fRef { 
            vec_ptr: &mut vec.0
        }
    }
}



#[derive(Clone, Debug, Default)]
pub struct DVVectorOfi32 ( opencv::types::VectorOfi32 );
unsafe impl Sync for DVVectorOfi32 {}
impl DVVectorOfi32 {
    pub fn empty() -> Self {
        Self ( opencv::types::VectorOfi32::new() )
    } 
    pub fn new(vec: opencv::types::VectorOfi32) -> Self {
        Self ( vec ) 
    }
	pub fn set(&mut self, index: size_t, val: i32) -> std::result::Result<(), opencv::Error> {
        self.0.set(index, val)
    }
}
impl Deref for DVVectorOfi32 {
    type Target = opencv::types::VectorOfi32;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
// For interop with custom C++ bindings to ORBSLAM
impl From<DVVectorOfi32> for dvos3binding::ffi::WrapBindCVVectorOfi32 {
    fn from(vec: DVVectorOfi32) -> dvos3binding::ffi::WrapBindCVVectorOfi32 {
        dvos3binding::ffi::WrapBindCVVectorOfi32 { 
            vec_ptr: dvos3binding::BindCVVectorOfi32 {
                vec_ptr: vec.0
            } 
        }
    }
}
impl From<dvos3binding::ffi::WrapBindCVVectorOfi32> for DVVectorOfi32 {
    fn from(vec: dvos3binding::ffi::WrapBindCVVectorOfi32) -> DVVectorOfi32 {
        DVVectorOfi32::new(vec.vec_ptr.vec_ptr)
    }
}
impl From<& DVVectorOfi32> for dvos3binding::ffi::WrapBindCVRawPtr {
    fn from(vec: & DVVectorOfi32) -> dvos3binding::ffi::WrapBindCVRawPtr {
        dvos3binding::ffi::WrapBindCVRawPtr { 
            raw_ptr: dvos3binding::BindCVRawPtr {
                raw_ptr: vec.0.as_raw()
            } 
        }
    }
}

//////////////////////////* Nalgebra TYPES //////////////////////////

#[derive(Clone, Copy, Debug)]
pub struct DVVector3<T> ( nalgebra::Vector3<T> ); // 3 dimensional column vector

impl<T: Debug + Clone + nalgebra::Scalar + num_traits::identities::Zero + nalgebra::ComplexField> DVVector3<T> {
    pub fn new(vec: nalgebra::Vector3<T>) -> Self {
        DVVector3 ( vec )
    }
    pub fn new_with(x: T, y: T, z: T) -> Self {
        DVVector3 ( nalgebra::Vector3::<T>::new(x,y,z) )
    }
    pub fn zeros<T2: Debug + Clone + nalgebra::Scalar + num_traits::identities::Zero>() -> Self {
        DVVector3 ( nalgebra::Vector3::<T>::zeros() )
    }
    pub fn clone(&self) -> Self {
        DVVector3 ( self.0.clone() )
    }

    pub fn is_zero(&self) -> bool { self.0 == nalgebra::Vector3::<T>::zeros() }

    // Note: I have no idea why the compiler hates these
    // Getting around it rn by calling *struct.norm() and *struct.div_assign()
    // pub fn norm(&self) -> T { self.vec.norm() }
    // pub fn div_assign(&mut self, divisor: T) { self.vec.div_assign(divisor) }
}
impl<T> Deref for DVVector3<T> {
    type Target = nalgebra::Vector3<T>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
// From implementations ... not sure why I need to have a separate 
// from for DVVector3 and &DVVector3 though.
impl<T: Debug + Clone + Copy + nalgebra::Scalar + num_traits::identities::Zero + nalgebra::ComplexField> From<DVVector3<T>> for [T; 3] {
    fn from(vec: DVVector3<T>) -> [T; 3] { [vec[0], vec[1], vec[2]] }
}
impl<T: Debug + Clone + Copy + nalgebra::Scalar + num_traits::identities::Zero + nalgebra::ComplexField> From<&DVVector3<T>> for [T; 3] {
    fn from(vec: &DVVector3<T>) -> [T; 3] { [vec[0], vec[1], vec[2]] }
}
impl<T: Debug + Clone + nalgebra::Scalar + num_traits::identities::Zero> From<DVVector3<T>> for nalgebra::Vector3<T> {
    fn from(vec: DVVector3<T>) -> nalgebra::Vector3<T> { vec.0.clone() }
}
impl<T: Debug + Clone + nalgebra::Scalar + num_traits::identities::Zero> From<&DVVector3<T>> for nalgebra::Vector3<T> {
    fn from(vec: &DVVector3<T>) -> nalgebra::Vector3<T> { vec.0.clone() }
}
impl From<DVVector3<f32>> for [f64; 3] {
    fn from(vec: DVVector3<f32>) -> [f64; 3] { [vec[0] as f64, vec[1] as f64, vec[2] as f64] }
}
impl From<nalgebra::Vector3<f64>> for DVVector3<f64> {
    fn from(vec: nalgebra::Vector3<f64>) -> DVVector3<f64> { DVVector3::new(vec) }
}
// So we can do vector3[i] without having to call a getter or setter function
impl<T> Index<usize> for DVVector3<T> {
    type Output = T;
    fn index(&self, i: usize) -> &Self::Output { &self.0[i] }
}
impl From<&opencv::core::Mat> for DVVector3<f64> {
    fn from(mat: &opencv::core::Mat) -> DVVector3<f64> { 
        DVVector3::new(nalgebra::Vector3::<f64>::new(
            *mat.at::<f64>(0).unwrap(),
            *mat.at::<f64>(1).unwrap(),
            *mat.at::<f64>(2).unwrap()
        ))
    }
}

#[derive(Clone, Debug)]
pub struct DVMatrix3<T> ( nalgebra::Matrix3<T> ); // 3x3 matrix

impl<T: Debug + Clone + nalgebra::Scalar + num_traits::identities::Zero + num_traits::One + nalgebra::ComplexField> DVMatrix3<T> {
    pub fn new(vec: nalgebra::Matrix3<T>) -> Self {
        DVMatrix3 ( vec )
    }
    pub fn zeros() -> Self {
        DVMatrix3::new(nalgebra::Matrix3::<T>::zeros())
    }
    pub fn identity() -> Self { 
        DVMatrix3::new(nalgebra::Matrix3::<T>::identity())
    }

    pub fn is_zero(&self) -> bool { self.0 == nalgebra::Matrix3::<T>::zeros() }

    pub fn vec(&self) -> &nalgebra::Matrix3<T> { &self.0 }
    pub fn transpose(&self) -> nalgebra::Matrix3<T> { self.0.transpose() }
    pub fn determinant(&self) -> T { self.0.determinant() }

    pub fn neg_mut(&mut self) { self.0.neg_mut() }
}
impl<T> Deref for DVMatrix3<T> {
    type Target = nalgebra::Matrix3<T>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
// From implementations...
impl<T: Clone> From<DVMatrix3<T>> for nalgebra::Matrix3<T> {
    fn from(vec: DVMatrix3<T>) -> nalgebra::Matrix3<T> { vec.0.clone() }
}
impl<T: Clone> From<&DVMatrix3<T>> for nalgebra::Matrix3<T> {
    fn from(vec: &DVMatrix3<T>) -> nalgebra::Matrix3<T> { vec.0.clone() }
}
impl From<opencv::core::Mat> for DVMatrix3<f64> {
    fn from(mat: opencv::core::Mat) -> DVMatrix3<f64> { 
        DVMatrix3::new(nalgebra::Matrix3::<f64>::new(
            *mat.at_2d::<f64>(0,0).unwrap(),
            *mat.at_2d::<f64>(0, 1).unwrap(),
            *mat.at_2d::<f64>(0, 2).unwrap(),
            *mat.at_2d::<f64>(1, 0).unwrap(),
            *mat.at_2d::<f64>(1, 1).unwrap(),
            *mat.at_2d::<f64>(1, 2).unwrap(),
            *mat.at_2d::<f64>(2, 0).unwrap(),
            *mat.at_2d::<f64>(2, 1).unwrap(),
            *mat.at_2d::<f64>(2, 2).unwrap(),
        ))
    }
}
impl<T: nalgebra::ComplexField> From<[[T; 3]; 3]> for DVMatrix3<T> {
    fn from(mat: [[T; 3]; 3]) -> DVMatrix3<T> { 
        DVMatrix3::new(nalgebra::Matrix3::<T>::new(
            mat[0][0].clone(), mat[0][1].clone(), mat[0][2].clone(),
            mat[1][0].clone(), mat[1][1].clone(), mat[1][2].clone(),
            mat[2][0].clone(), mat[2][1].clone(), mat[2][2].clone()
        ))
    }
}
impl<T: nalgebra::ComplexField> From<& DVMatrix3<T>> for [[T; 3]; 3] {
    fn from(mat: & DVMatrix3<T>) -> [[T; 3]; 3] { 
        [
            [mat[(0,0)].clone(), mat[(0,1)].clone(), mat[(0,2)].clone()],
            [mat[(1,0)].clone(), mat[(1,1)].clone(), mat[(1,2)].clone()],
            [mat[(2,0)].clone(), mat[(2,1)].clone(), mat[(2,2)].clone()]
        ]
    }
}
impl<T: nalgebra::ComplexField> From<&mut DVMatrix3<T>> for [[T; 3]; 3] {
    fn from(mat: &mut DVMatrix3<T>) -> [[T; 3]; 3] { 
        [
            [mat[(0,0)].clone(), mat[(0,1)].clone(), mat[(0,2)].clone()],
            [mat[(1,0)].clone(), mat[(1,1)].clone(), mat[(1,2)].clone()],
            [mat[(2,0)].clone(), mat[(2,1)].clone(), mat[(2,2)].clone()]
        ]
    }
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


#[derive(Clone, Debug)]
pub struct DVMatrix4<T> ( nalgebra::Matrix4<T> ); // 4x4 matrix

impl<T: Debug + Clone + nalgebra::Scalar + num_traits::identities::Zero + num_traits::One + nalgebra::ComplexField> DVMatrix4<T> {
    pub fn new(vec: nalgebra::Matrix4<T>) -> Self {
        DVMatrix4 ( vec )
    }
    pub fn zeros<T2: Debug + Clone + nalgebra::Scalar + num_traits::identities::Zero>() -> Self {
        DVMatrix4::new(nalgebra::Matrix4::<T>::zeros())
    }
    pub fn identity() -> Self { 
        DVMatrix4::new(nalgebra::Matrix4::<T>::identity())
    }
    pub fn inverse(&self) -> Self {
        DVMatrix4::new(self.0.clone().try_inverse().unwrap())
    }

    pub fn is_zero(&self) -> bool { self.0 == nalgebra::Matrix4::<T>::zeros() }

    pub fn vec(&self) -> &nalgebra::Matrix4<T> { &self.0 }
    pub fn transpose(&self) -> nalgebra::Matrix4<T> { self.0.transpose() }
    pub fn determinant(&self) -> T { self.0.determinant() }

    pub fn neg_mut(&mut self) { self.0.neg_mut() }
}
impl<T> Deref for DVMatrix4<T> {
    type Target = nalgebra::Matrix4<T>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
// From implementations...
impl<T: Clone> From<DVMatrix4<T>> for nalgebra::Matrix4<T> {
    fn from(vec: DVMatrix4<T>) -> nalgebra::Matrix4<T> { vec.0.clone() }
}
impl<T: Clone> From<&DVMatrix4<T>> for nalgebra::Matrix4<T> {
    fn from(vec: &DVMatrix4<T>) -> nalgebra::Matrix4<T> { vec.0.clone() }
}
impl From<&DVMatrix4<f64>> for Vec<dvos3binding::ffi::DoubleVec> {
    fn from(mat: &DVMatrix4<f64>) -> Vec<dvos3binding::ffi::DoubleVec> {
        let mut res = Vec::new();

        for i in 0..mat.0.nrows() {
            let mut row = Vec::new();
            for j in 0..mat.0.ncols() {
                let r: usize = i.try_into().unwrap();
                let c: usize = j.try_into().unwrap();
                row.push(mat.0[(r, c)]);
            }
            res.push(dvos3binding::ffi::DoubleVec { 
                vec: row
            });
        }
        res
    }
}
// Two index implementations, one for matrix3[(x,y)] and one for [(x)]
impl<T> Index<(usize, usize)> for DVMatrix4<T> {
    type Output = T;

    fn index(&self, i: (usize, usize)) -> &Self::Output {
        &self.0[i]
    }
}
impl<T> Index<usize> for DVMatrix4<T> {
    type Output = T;

    fn index(&self, i: usize) -> &Self::Output {
        &self.0[i]
    }
}
pub struct DVMatrix7x7<T> ( nalgebra::SMatrix<T, 7, 7> ); // 7x7 matrix
impl<T: Debug + Clone + nalgebra::Scalar + num_traits::identities::Zero + num_traits::One + nalgebra::ComplexField> DVMatrix7x7<T> {
    pub fn new(vec: nalgebra::SMatrix<T, 7, 7>) -> Self {
        DVMatrix7x7 ( vec )
    }
    pub fn zeros<T2: Debug + Clone + nalgebra::Scalar + num_traits::identities::Zero>() -> Self {
        DVMatrix7x7::new(nalgebra::SMatrix::<T, 7, 7>::zeros())
    }
    pub fn identity() -> Self { 
        DVMatrix7x7::new(nalgebra::SMatrix::<T, 7, 7>::identity())
    }

    pub fn is_zero(&self) -> bool { self.0 == nalgebra::SMatrix::<T, 7, 7>::zeros() }

    pub fn vec(&self) -> &nalgebra::SMatrix<T, 7, 7> { &self.0 }
    pub fn transpose(&self) -> nalgebra::SMatrix<T, 7, 7> { self.0.transpose() }
    pub fn determinant(&self) -> T { self.0.determinant() }

    pub fn neg_mut(&mut self) { self.0.neg_mut() }
}
impl<T> Deref for DVMatrix7x7<T> {
    type Target = nalgebra::SMatrix<T, 7, 7>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
// From implementations...
impl<T: Clone> From<DVMatrix7x7<T>> for nalgebra::SMatrix<T, 7, 7> {
    fn from(vec: DVMatrix7x7<T>) -> nalgebra::SMatrix<T, 7, 7> { vec.0.clone() }
}
impl<T: Clone> From<&DVMatrix7x7<T>> for nalgebra::SMatrix<T, 7, 7> {
    fn from(vec: &DVMatrix7x7<T>) -> nalgebra::SMatrix<T, 7, 7> { vec.0.clone() }
}
// Two index implementations, one for matrix7x7[(x,y)] and one for [(x)]
impl<T> Index<(usize, usize)> for DVMatrix7x7<T> {
    type Output = T;

    fn index(&self, i: (usize, usize)) -> &Self::Output {
        &self.0[i]
    }
}
impl<T> Index<usize> for DVMatrix7x7<T> {
    type Output = T;

    fn index(&self, i: usize) -> &Self::Output {
        &self.0[i]
    }
}



#[derive(Clone, Debug)]
pub struct DVMatrixDynamic<T> ( nalgebra::DMatrix<T> );

impl<T> DVMatrixDynamic<T> {
    // Constructors
    pub fn new(vec: nalgebra::DMatrix<T>) -> Self {
        DVMatrixDynamic ( vec )
    }
}
// DVMatrixDynamic<u8> is used for greyscale images!
// &DVMatrixDynamic implemented instead of DVMatrixDynamic because this avoids having to take 
// ownership of the matrix when calling .into() ... this is useful because this matrix is usually
// inside an Arc<ImageMsg> and you cannot move out of an arc. Alternatively, we could copy the matrix
// but that takes too much time/memory.
impl From<&DVMatrixDynamic<u8>> for opencv::core::Mat {
    fn from(mat: &DVMatrixDynamic<u8>) -> opencv::core::Mat {
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
impl From<opencv::core::Mat> for DVMatrixDynamic<u8> {
    fn from(mat: opencv::core::Mat) -> DVMatrixDynamic<u8> {
        let mut dmat = nalgebra::DMatrix::from_element(mat.rows().try_into().unwrap(), mat.cols().try_into().unwrap(), 0u8);
        for i in 0..mat.rows() {
            for j in 0..mat.cols() {
                let val = *mat.at_2d::<u8>(i, j).unwrap(); // Grayscale 1 channel uint8
                let r: usize = i.try_into().unwrap();
                let c: usize = j.try_into().unwrap();
                dmat[(r, c)] = val;
            }
        }
        DVMatrixDynamic(dmat)
    }
}

impl From<&DVMatrixDynamic<f64>> for Vec<dvos3binding::ffi::DoubleVec> {
    fn from(mat: &DVMatrixDynamic<f64>) -> Vec<dvos3binding::ffi::DoubleVec> {
        let mut res = Vec::new();

        for i in 0..mat.0.nrows() {
            let mut row = Vec::new();
            for j in 0..mat.0.ncols() {
                let r: usize = i.try_into().unwrap();
                let c: usize = j.try_into().unwrap();
                row.push(mat.0[(r, c)]);
            }
            res.push(dvos3binding::ffi::DoubleVec { 
                vec: row
            });
        }
        res
    }
}
impl From<&Vec<dvos3binding::ffi::DoubleVec>> for DVMatrixDynamic<f64> {
    fn from(mat: &Vec<dvos3binding::ffi::DoubleVec>) -> DVMatrixDynamic<f64> {
        let rows = mat.len();
        let cols = mat[0].vec.len();
        let mut res = nalgebra::DMatrix::zeros(rows, cols);

        for i in 0..mat.len() {
            let row = & mat[i].vec;
            for j in 0..row.len() {
                let r: usize = i.try_into().unwrap();
                let c: usize = j.try_into().unwrap();
                res[(c, r)] = mat[i].vec[j];
            }
        }
        DVMatrixDynamic::<f64>::new(res)
    }
}
impl<T: Clone> Deref for DVMatrixDynamic<T> {
    type Target = nalgebra::DMatrix<T>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
