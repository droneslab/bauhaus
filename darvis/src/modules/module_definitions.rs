use core::{matrix::{DVMatrix, DVMatrix3, DVVector3, DVVectorOfKeyPoint, DVVectorOfPoint2f}, sensor::Sensor, system::Module};
use std::{collections::{BTreeSet, HashMap}, fmt::Debug};

use downcast_rs::{impl_downcast, Downcast};

use crate::{actors::tracking_backend::TrackedMapPointData, map::{frame::Frame, keyframe::KeyFrame, map::Id, pose::Sim3}, MapLock};

/// *** Traits for modules. *** //


/// *** Camera *** //
pub trait CameraModule {
    // Vision
    type Keys;
    type KeyPoint;
    type Point;
    type Matches;

    // Map references
    type KeyFrame;
    type Pose;

    // Results
    type ResultPoints;
    type ResultTriangulated;

    fn two_view_reconstruction(
        &self, 
        v_keys1: &Self::Keys, 
        v_keys2: &Self::Keys,
        matches: &Self::Matches,
    ) -> Option<(Self::Pose, Self::ResultPoints, Self::ResultTriangulated)>;
    fn unproject_eig(&self, kp: &Self::Point) -> DVVector3<f64>;
    fn unproject_stereo(&self, kf: &Self::KeyFrame, _idx: usize) -> Option<DVVector3<f64>>;
    fn project(&self, pos: DVVector3<f64>) -> (f64, f64) ;
    fn epipolar_constrain(&self, kp1: &Self::KeyPoint, kp2: &Self::KeyPoint, r12: &DVMatrix3<f64>, t12: &DVVector3<f64>, unc: f32) -> bool;
}


/// *** Vocabulary *** //
pub trait VocabularyModule {
    type BoWModule;
    type Descriptors;

    fn access(&self);
    fn load(filename: String) -> Self;
    fn size(&self) -> usize;
    fn transform(&self, descriptors: &Self::Descriptors, bow: &mut Self::BoWModule);
    fn score(&self, kf1: &Self::BoWModule, kf2: &Self::BoWModule) -> f32;
}

/// *** Bag of words *** //
pub trait BoWModule {
    type BoWVector;
    type FeatureVector;

    fn get_bow_vec(&self) -> &Self::BoWVector;
    fn get_feat_vec(&self) -> &Self::FeatureVector;
} 


/// *** IMU *** //
pub trait ImuModule {
    fn ready(&self) -> bool;
    fn predict_state(&self) -> bool;
    fn preintegrate(&self);
    fn initialize(&self);
}

/// *** Feature Extraction *** //
pub trait FeatureExtractionModule {
    fn extract(&mut self, image: DVMatrix) -> Result<(DVVectorOfKeyPoint, DVMatrix), Box<dyn std::error::Error>>;
}

impl Module for dyn FeatureExtractionModule { }
impl Debug for dyn FeatureExtractionModule {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        todo!()
    }
}


/// *** Feature Matching. This consists of several traits, which are combined into a supertrait. See ORBMatcherTrait for an example. *** //

// Generic trait
pub trait FeatureMatchingModule: Downcast {
    fn search_by_projection(
        &self, 
        frame: &mut Frame, mappoints: &mut BTreeSet<Id>, th: i32, ratio: f64,
        track_in_view: &HashMap<Id, TrackedMapPointData>, track_in_view_right: &HashMap<Id, TrackedMapPointData>, 
        map: &MapLock, sensor: Sensor
    ) -> Result<(HashMap<Id, i32>, i32), Box<dyn std::error::Error>>;
    fn search_by_projection_with_threshold (
        &self, 
        current_frame: &mut Frame, last_frame: &mut Frame, th: i32,
        should_check_orientation: bool,
        map: &MapLock, sensor: Sensor
    ) -> Result<i32, Box<dyn std::error::Error>>;
    fn _search_by_projection_reloc (
        &self, 
        _current_frame: &mut Frame, _keyframe: &KeyFrame,
        _th: i32, _should_check_orientation: bool, _ratio: f64,
        _track_in_view: &HashMap<Id, TrackedMapPointData>, _track_in_view_right: &HashMap<Id, TrackedMapPointData>,
        _map: &MapLock, _sensor: Sensor
    ) -> Result<i32, Box<dyn std::error::Error>>;
    fn search_by_projection_for_loop_detection1(
        &self, 
        map: &MapLock, kf_id: &Id, scw: &Sim3, 
        candidates: &Vec<Id>, // vpPoints
        kfs_for_candidates: &Vec<Id>, // vpPointsKFs
        matches: &mut Vec<Option<Id>>, // vpMatched
        threshold: i32, hamming_ratio: f64
    ) -> Result<(i32, Vec<Option<Id>>), Box<dyn std::error::Error>>;
    fn search_by_projection_for_loop_detection2(
        &self, 
        map: &MapLock, kf_id: &Id, scw: &Sim3, 
        candidates: &Vec<Id>,
        matches: &mut Vec<Option<Id>>,
        threshold: i32, hamming_ratio: f64
    ) -> Result<i32, Box<dyn std::error::Error>>;
}
impl Debug for dyn FeatureMatchingModule {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        todo!()
    }
}

pub trait SearchForTriangulationTrait {
    fn search_for_triangulation(
        &self, 
        kf_1 : &KeyFrame, kf_2 : &KeyFrame,
        should_check_orientation: bool, _only_stereo: bool, course: bool,
        sensor: Sensor
    ) -> Result<Vec<(usize, usize)>, Box<dyn std::error::Error>> ;
}
pub trait SearchBySim3Trait {
    fn search_by_sim3(&self, map: &MapLock, kf1_id: Id, kf2_id: Id, matches: &mut HashMap<usize, i32>, sim3: &Sim3, th: f32) -> i32 ;
}

pub trait FuseTrait {
    fn fuse_from_loop_closing(&self,  kf_id: &Id, scw: &Sim3, mappoints: &Vec<Id>, map: &MapLock, th: i32) ->  Result<Vec<Option<Id>>, Box<dyn std::error::Error>>;
    fn fuse(&self,  kf_id: &Id, fuse_candidates: &Vec<Option<(Id, bool)>>, map: &MapLock, th: f32, is_right: bool);
}

pub trait SearchByBoWTrait {
    fn search_by_bow_with_frame(
        &self, 
        kf: &KeyFrame, frame: &mut Frame,
        should_check_orientation: bool, ratio: f64
    ) -> Result<u32, Box<dyn std::error::Error>>;
    fn search_by_bow_with_keyframe(
        &self, 
        kf_1 : &KeyFrame, kf_2 : &KeyFrame, should_check_orientation: bool, 
        ratio: f64
    ) -> Result<HashMap<u32, Id>, Box<dyn std::error::Error>>;
}

pub trait SearchForInitializationTrait {
    fn search_for_initialization(&self, f1: &Frame, f2: &Frame, vb_prev_matched: &mut DVVectorOfPoint2f, window_size: i32) -> (i32, Vec<i32>);

}
pub trait DescriptorDistanceTrait {
    fn descriptor_distance(&self, desc1: &opencv::core::Mat, desc2: &opencv::core::Mat) -> i32;
}



/// *** Map initialization *** //
pub trait MapInitializationModule {
    type Frame;
    type Map;
    type InitializationResult;

    fn try_initialize(&mut self, current_frame: &Self::Frame) -> Result<bool, Box<dyn std::error::Error>>;
    fn create_initial_map(&mut self, map: &mut Self::Map) -> Self::InitializationResult ;
}

/// *** Map initialization *** //
pub trait RelocalizationModule {
    type Frame;
    type Timestamp;
    type Map;

    fn run(&self, current_frame: &mut Self::Frame, map: &Self::Map) -> Result<bool, Box<dyn std::error::Error>>;
    fn sec_since_lost(&self, current_frame: &Self::Frame) -> Self::Timestamp;
    fn frames_since_lost(&self, current_frame: &Self::Frame) -> i32;
}

/// *** Loop detection *** //
pub trait LoopDetectionModule {
    fn detect_loop(&mut self, map: &MapLock, current_kf_id: Id) -> Result<(Option<Id>, Option<Id>, Option<Sim3>, Vec<Id>, Vec<Option<Id>>), Box<dyn std::error::Error>>;
}

/// *** Local map optimization *** //
pub trait LocalMapOptimizationModule {
    fn optimize(&self, map: &MapLock, keyframe_id: Id);
}

/// *** Full map optimization *** //
pub trait FullMapOptimizationModule {
    fn optimize(&self, map: &mut MapLock, iterations: i32, robust: bool, loop_kf: Id) ;
}
