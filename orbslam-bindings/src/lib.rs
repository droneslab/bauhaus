use std::ffi::c_void;
use opencv;
use cxx::{type_id, ExternType};

// Note: The structs BindCV[x] below allow us to pass rust openCV objects
// (created using the rust opencv crate) to the orbslam C++ code.
// The rust opencv objects (opencv::core::Vector<opencv::core::KeyPoint>, opencv::core::Mat) 
// are actually just pointers to data allocated in C++, so we should be able to
// just pass those raw pointers from rust to the orbslam C++ code.
// Of course, we could accidentally pass the wrong pointer and end up with a
// memory error. The code below helps prevent that by linking the opencv rust
// pointers (ex, opencv::core::Vector<opencv::core::KeyPoint>) to the orbslam C++ pointers
// (ex, orb_slam3::BindCVKeyPoints, see CVConvert.h). Unfortunately we need the 
// wrapper BindCVKeyPoints and BindCVMat because we can't implement ExternType
// on a struct declared outside our repository.
// So, to pass an opencv matrix allocated from the rust opencv crate into these
// orbslam C++ bindings, the entire workflow looks like this:
// In Rust...
//   Read an image using the opencv crate:
//     let opencv_crate_img = imgcodecs::imread(&path, imgcodecs::IMREAD_GRAYSCALE)?;
//   Wrap the image into a struct that can be passed to the orbslam C++ code:
//     let dv_mat_img = DVMatrix::new(opencv_crate_img);
//     let ready_for_orbslam_img: dvos3binding::ffi::WrapBindCVMat = dv_mat_img.into();
//   Call the orbslam C++ binding:
//     dvos3binding::ffi::send_mat(ready_for_orbslam_img);
// In C++...
//     void send_mat(const orb_slam3::WrapBindCVMat & image) {
//       Convert the wrapped image into a regular opencv mat:
//         opencv::Mat * regular_mat_img = *image.mat_ptr;
//     }
//
// The C++ equivalent of these structs is defined in CVConvert.h
#[derive(Debug, Clone)]
pub struct BindCVKeyPoints {
    pub kp_ptr: opencv::core::Vector<opencv::core::KeyPoint>
}
unsafe impl ExternType for BindCVKeyPoints {
    type Id = type_id!("orb_slam3::BindCVKeyPoints");
    type Kind = cxx::kind::Trivial;
}
pub struct BindCVKeyPointsRef<'a> {
    pub kp_ptr: &'a opencv::core::Vector<opencv::core::KeyPoint>
}
unsafe impl<'a> ExternType for BindCVKeyPointsRef<'a> {
    type Id = type_id!("orb_slam3::BindCVKeyPointsRef");
    type Kind = cxx::kind::Trivial;
}

#[derive(Debug, Clone)]
pub struct BindCVMat {
    pub mat_ptr: opencv::core::Mat
}
unsafe impl ExternType for BindCVMat {
    type Id = type_id!("orb_slam3::BindCVMat");
    type Kind = cxx::kind::Trivial;
}
pub struct BindCVMatRef<'a> {
    pub mat_ptr: &'a opencv::core::Mat
}
unsafe impl<'a> ExternType for BindCVMatRef<'a> {
    type Id = type_id!("orb_slam3::BindCVMatRef");
    type Kind = cxx::kind::Trivial;
}

#[derive(Debug, Clone)]
pub struct BindCVVectorOfi32 {
    pub vec_ptr: opencv::core::Vector<i32>
}
unsafe impl ExternType for BindCVVectorOfi32 {
    type Id = type_id!("orb_slam3::BindCVVectorOfi32");
    type Kind = cxx::kind::Trivial;
}
#[derive(Debug, Clone)]
pub struct BindCVVectorOfPoint2f {
    pub vec_ptr: opencv::core::Vector<opencv::core::Point2f>
}
unsafe impl ExternType for BindCVVectorOfPoint2f {
    type Id = type_id!("orb_slam3::BindCVVectorOfPoint2f");
    type Kind = cxx::kind::Trivial;
}

#[derive(Debug, Clone)]
pub struct BindCVVectorOfPoint3f {
    pub vec_ptr: opencv::core::Vector<opencv::core::Point3f>
}
unsafe impl ExternType for BindCVVectorOfPoint3f {
    type Id = type_id!("orb_slam3::BindCVVectorOfPoint3f");
    type Kind = cxx::kind::Trivial;
}
pub struct BindCVVectorOfPoint2fRef<'a> {
    pub vec_ptr:  &'a mut opencv::core::Vector<opencv::core::Point2f>
}
unsafe impl<'a> ExternType for BindCVVectorOfPoint2fRef<'a> {
    type Id = type_id!("orb_slam3::BindCVVectorOfPoint2fRef");
    type Kind = cxx::kind::Trivial;
}

// Note: Don't use this (and/or RawCVPtr below) if you can avoid it.
// It's safer to use BindCVKeyPoints and BindCVMat because then C++
// is sure to do the cast correctly. This is only useful in the 
// specific circumstance where you don't have ownership of a DVMatrix
// or a DVVectorOfKeyPoint but you also can't clone the data.
#[derive(Debug, Clone)]
pub struct BindCVRawPtr {
    pub raw_ptr: *const c_void
}
unsafe impl ExternType for BindCVRawPtr {
    type Id = type_id!("orb_slam3::BindCVRawPtr");
    type Kind = cxx::kind::Trivial;
}

pub struct Feature {
    pub node_id: u32,
    pub feature_id: u32,
}

#[cxx::bridge(namespace = "orb_slam3")]
pub mod ffi {
    // Shared structs with fields visible to both languages.
    pub struct WrapBindCVRawPtr {
        pub raw_ptr: BindCVRawPtr
    }
    pub struct WrapBindCVKeyPoints {
        pub kp_ptr: BindCVKeyPoints
    }
    pub struct WrapBindCVMat {
        pub mat_ptr: BindCVMat
    }
    pub struct WrapBindCVVectorOfi32 {
        pub vec_ptr: BindCVVectorOfi32
    }
    pub struct WrapBindCVVectorOfPoint2f {
        pub vec_ptr: BindCVVectorOfPoint2f
    }
    pub struct WrapBindCVVectorOfPoint3f {
        pub vec_ptr: BindCVVectorOfPoint3f
    }

    #[derive(Debug, Clone)]
    pub struct Pose {
        pub translation: [f32;3],
        pub rotation: [[f32; 3]; 3]
    }

    #[derive(Debug, Clone)]
    pub struct Grid {

        pub vec: Vec<VectorOfVecusize>
    }

    #[derive(Debug, Clone)]
    pub struct VectorOfVecusize
    {
        pub vec: Vec<VectorOfusize>
    }

    #[derive(Debug, Clone)]
    pub struct VectorOfusize
    {
        pub vec: Vec<usize>
    }

    struct DoubleVec {
        vec: Vec<f64>,
    }
    struct SVDResult {
        singular_values: Vec<f64>,
        u: Vec<DoubleVec>,
        v: Vec<DoubleVec>,
    }
    enum SVDComputeType {
        ThinUThinV,
        FullV,
    }


    unsafe extern "C++" {
        include!("orb_slam3/src/CVConvert.h");
        type BindCVMat = crate::BindCVMat;
        type BindCVKeyPoints = crate::BindCVKeyPoints;
        type BindCVRawPtr = crate::BindCVRawPtr;
        type BindCVVectorOfi32 = crate::BindCVVectorOfi32;
        type BindCVVectorOfPoint2f = crate::BindCVVectorOfPoint2f;
        type BindCVVectorOfPoint3f = crate::BindCVVectorOfPoint3f;
        type BindCVKeyPointsRef<'a> = crate::BindCVKeyPointsRef<'a>;
        type BindCVMatRef<'a> = crate::BindCVMatRef<'a>;
        type BindCVVectorOfPoint2fRef<'a> = crate::BindCVVectorOfPoint2fRef<'a>;

        // ORB extractor
        include!("orb_slam3/src/ORBextractor.h");
        type ORBextractor;
        fn new_orb_extractor(
            features: i32,
            scale_factor: f32,
            levels: i32,
            ini_th_fast: i32,
            min_th_fast: i32,
            overlap_begin: i32,
            overlap_end: i32,
        ) -> UniquePtr<ORBextractor>;
        #[rust_name = "extract_with_existing_points"]
        fn extract_with_existing_points_rust(
            self: Pin<&mut ORBextractor>,
            image: &WrapBindCVMat,
            points: &BindCVVectorOfPoint2f,
            keypoints: &mut WrapBindCVKeyPoints,
            descriptors: &mut WrapBindCVMat
        ) -> i32;
        #[rust_name = "extract"]
        fn extract_rust(
            self: Pin<&mut ORBextractor>,
            image: &WrapBindCVMat,
            keypoints: &mut WrapBindCVKeyPoints,
            descriptors: &mut WrapBindCVMat
        ) -> i32;
        fn extract2(
            self: Pin<&mut ORBextractor>,
            keypoints: &mut WrapBindCVKeyPoints,
            descriptors: &mut WrapBindCVMat
        ) -> i32;

        // Two view reconstruction
        include!("orb_slam3/src/TwoViewReconstruction.h");
        type TwoViewReconstruction;
        fn new_two_view_reconstruction(
            fx: f32,
            cx: f32,
            fy: f32,
            cy: f32,
            sigma: f32,
            iterations: i32
        ) -> UniquePtr<TwoViewReconstruction>;
        #[rust_name = "reconstruct"]
        fn reconstruct_rust(
            self: Pin<&mut TwoViewReconstruction>,
            vKeys1: & WrapBindCVKeyPoints,
            vKeys2: & WrapBindCVKeyPoints,
            vMatches12: & Vec<i32>,
            T21: &mut Pose,
            vP3D: &mut WrapBindCVVectorOfPoint3f,
            vbTriangulated: &mut Vec<bool>
        )-> bool;

        // ORB Matcher
        include!("orb_slam3/src/ORBmatcher.h");
        type ORBmatcher;
        fn descriptor_distance(
            a: &WrapBindCVRawPtr,
            b: &WrapBindCVRawPtr,
            print: bool
        ) -> i32;
        fn print_descriptors(
            a: &WrapBindCVRawPtr,
        );
        fn new_orb_matcher(
            frame_grid_cols : i32, 
            frame_grid_rows : i32,
            minX : f32,
            minY : f32,
            maxX: f32, 
            maxY: f32,
            nnratio: f32,
            checkOri: bool
        ) -> UniquePtr<ORBmatcher>;
        #[rust_name = "search_for_initialization"]
        fn search_for_initialization_rust(
            self: &ORBmatcher,
            F1_mvKeysUn : &BindCVKeyPointsRef, 
            F2_mvKeysUn: &BindCVKeyPointsRef, 
            F1_mDescriptors: &BindCVMatRef,
            F2_mDescriptors: &BindCVMatRef ,
            F2_grid: &Grid,
            vbPrevMatched : &mut BindCVVectorOfPoint2fRef,
            vnMatches12 : &mut Vec<i32>,
            windowSize: i32
        ) -> i32;
    }

    #[namespace = "DBoW2"]
    unsafe extern "C++" {
        include!("orb_slam3/DBoW2/DBoW2/ORBVocabulary.h");

        type ORBVocabulary;
        type BowVector;
        type FeatureVector;

        fn new_bow_vec() -> UniquePtr<BowVector>;
        fn clone(self: &BowVector) -> UniquePtr<BowVector>;

        fn new_feat_vec() -> UniquePtr<FeatureVector>;
        fn clone(self: &FeatureVector) -> UniquePtr<FeatureVector>;
        fn get_all_nodes(self: &FeatureVector) -> Vec<u32>;

        fn get_feat_from_node(self: &FeatureVector, node_id: u32) -> Vec<u32>;
        fn vec_size(self: &FeatureVector, node_id: u32) -> u32;
        fn vec_get(self: &FeatureVector, node_id: u32, index: u32) -> u32;

        fn get_all_word_ids(self: &BowVector) -> Vec<u32>;
        // fn vec_get(self: &BowVector, node_id: u32, index: u32) -> Vec<u32>;

        fn load_vocabulary_from_text_file(file: &CxxString) -> UniquePtr<ORBVocabulary>;
        fn size(self: &ORBVocabulary) -> usize;
        fn transform(
            self: &ORBVocabulary,
            descriptors: &WrapBindCVMat,
            bow_vector: Pin<&mut BowVector>,
            feature_vector: Pin<&mut FeatureVector>,
            levelsup: i32
        );
        fn score(
            self: &ORBVocabulary,
            bow_vec1: &UniquePtr<BowVector>,
            bow_vec2: &UniquePtr<BowVector>
        ) -> f32;

    }

    unsafe extern "C++" {
        include!("orb_slam3/src/Extra.h");
        fn normalize_rotation(
            rotation: [[f64; 3]; 3]
        ) -> [[f64; 3]; 3];
        fn svd(
            mat: Vec<DoubleVec>,
            compute_type: SVDComputeType,
        ) -> SVDResult;
    }

    #[namespace = "DUtils"]
    unsafe extern "C++" {

        include!("orb_slam3/DBoW2/DUtils/Random.h");
        #[rust_name = "RandomInt"]
        fn RandomInt(min: i32, max: i32) -> i32;
    }
}

// TODO (mvp): Need to make sure these are actually safe
// They should be ok as long as they are only used within a UniquePtr.
// But can we enforce using them in a UniquePtr?
// These are needed because:
// ORBVocabulary ...
//    - used in map, which is owned by map actor, and cxx requires state
//    - in actor to be send + sync
// ORBBowVector, ORBFeatureVector ... 
//    - used in KeyFrame, which is sent to/from map actor, and cxx requires
//    - messages to be send + sync
unsafe impl Send for ffi::ORBVocabulary {}
unsafe impl Sync for ffi::ORBVocabulary {}
unsafe impl Send for ffi::BowVector {}
unsafe impl Sync for ffi::BowVector {}
unsafe impl Send for ffi::FeatureVector {}
unsafe impl Sync for ffi::FeatureVector {}
