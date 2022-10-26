
use cxx::{CxxVector, SharedPtr};

#[cxx::bridge(namespace = "orb_slam3")]
pub mod ffi {
    // Shared structs with fields visible to both languages.
    #[derive(Debug, Clone)]
    pub struct DVPoint2f
    {
        pub x : f32,
        pub y : f32
    }
    #[derive(Debug, Clone)]
    pub struct DVKeyPoint {
        /// coordinates of the keypoints
        pub pt: DVPoint2f,
        /// diameter of the meaningful keypoint neighborhood
        pub size: f32,
        /// computed orientation of the keypoint (-1 if not applicable);
        /// it's in [0,360) degrees and measured relative to
        /// image coordinate system, ie in clockwise.
        pub angle: f32,
        /// the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling
        pub response: f32,
        /// octave (pyramid layer) from which the keypoint has been extracted
        pub octave: i32,
        /// object class (if the keypoints need to be clustered by an object they belong to)
        pub class_id: i32,
    }

    #[derive(Debug, Clone)]
    pub struct DVPoint3f {
        pub x : f32,
        pub y : f32,
        pub z: f32
    }

    pub struct Pose{
    
        pub pose: [[f32;4];4]
    }
    #[derive(Debug, Clone)]
    pub struct DVbool{
    
        pub val: bool
    }

    pub struct VectorOfDVPoint3f
    {
        pub vec: Vec<DVPoint3f>
    }
    pub struct VectorOfDVBool
    {
        pub vec: Vec<bool>
    }
    pub struct VectorOfDVi32
    {
        pub vec: Vec<i32>
    }
    pub struct VectorOfDVPoint2f
    {
        pub vec: Vec<DVPoint2f>
    }

    #[derive(Debug, Clone)]
    pub struct VectorOff32
    {
        pub vec: Vec<f32>
    }

    #[derive(Debug, Clone)]
    pub struct VectorOfusize
    {
        pub vec: Vec<usize>
    }

    #[derive(Debug, Clone)]
    pub struct VectorOfVecusize
    {
        pub vec: Vec<VectorOfusize>
    }

    #[derive(Debug, Clone)]
    pub struct DVGrid {

        pub vec: Vec<VectorOfVecusize>
    }

    unsafe extern "C++" {
        // Opaque types which both languages can pass around
        // but only C++ can see the fields.

        include!("orb_slam3/src/TwoViewReconstruction.h");
        include!("orb_slam3/src/ORBmatcher.h");
        include!("orb_slam3/src/DVMat.h");
        //include!("orb_slam3/src/DVGrid.h");
        type TwoViewReconstruction;
        type ORBmatcher;
        type DVMat;
        

        fn new_two_view_reconstruction(
            fx: f32,
            cx: f32,
            fy: f32,
            cy: f32,
            sigma: f32,
            iterations: i32
        ) -> UniquePtr<TwoViewReconstruction>;

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


         fn Reconstruct_1(
            self: Pin<&mut TwoViewReconstruction>,
            vKeys1: &CxxVector<DVKeyPoint>,
            vKeys2:  &CxxVector<DVKeyPoint>,
            vMatches12: &CxxVector<i32>,
            T21: &mut Pose,
            vP3D: &mut VectorOfDVPoint3f,
            vbTriangulated: &mut VectorOfDVBool
        )-> bool;


        fn SearchForInitialization_1(
            self: Pin<&mut ORBmatcher>,
            F1_mvKeysUn : &CxxVector<DVKeyPoint>, 
            F2_mvKeysUn: &CxxVector<DVKeyPoint>, 
            F1_mDescriptors: &DVMat,
            F2_mDescriptors: &DVMat ,
            F2_grid: &DVGrid,
             vbPrevMatched : Pin<&mut CxxVector<DVPoint2f>>, 
            vnMatches12 : Pin<&mut CxxVector<i32>>,
            windowSize: i32
        );

        // SearchForInitialization(
        //     const std::vector<cv::KeyPoint>& F1_mvKeysUn, 
        //     const std::vector<cv::KeyPoint>& F2_mvKeysUn, 
        //     const cv::Mat F1_mDescriptors,
        //     const cv::Mat F2_mDescriptors,
        //     const std::vector< std::vector <std::vector<size_t> > > F2_grid,
        //     std::vector<cv::Point2f> &vbPrevMatched, 
        //     std::vector<int> &vnMatches12, 
        //     int windowSize=10)

    }
}




