/// *** Traits for modules. *** //


pub trait CameraModule {
    type Keys;
    type Pose;
    type ResultPoints;
    type ResultTriangulated;
    type KeyPoint;
    type Matrix3;
    type Vector3;
    type KeyFrame;

    fn two_view_reconstruction(
        &self, 
        v_keys1: &Self::Keys, 
        v_keys2: &Self::Keys,
        matches: &Vec<i32>,
    ) -> Option<(Self::Pose, Self::ResultPoints, Self::ResultTriangulated)>;
    fn unproject_eig(&self, kp: &opencv::core::Point2f) -> Self::Vector3;
    fn unproject_stereo(&self, kf: &Self::KeyFrame, _idx: usize) -> Option<Self::Vector3>;
    fn project(&self, pos: Self::Vector3) -> (f64, f64) ;
    fn epipolar_constrain(&self, kp1: &Self::KeyPoint, kp2: &Self::KeyPoint, r12: &Self::Matrix3, t12: &Self::Vector3, unc: f32) -> bool;
}


pub trait VocabularyModule {
    type BoWModule;
    type Descriptors;
    type ItemToScore;

    fn access(&self);
    fn load(filename: String) -> Self;
    fn size(&self) -> usize;
    fn transform(&self, descriptors: &Self::Descriptors, bow: &mut Self::BoWModule);
    fn score(&self, kf1: &Self::ItemToScore, kf2: &Self::ItemToScore) -> f32;
}

pub trait BoWModule { } 



pub trait ImuModule {
    fn ready(&self) -> bool;
    fn predict_state(&self) -> bool;
    fn preintegrate(&self);
    fn initialize(&self);
}

pub trait FeatureExtractionModule {
    type Image;

    fn extract(&mut self, image: Self::Image) -> Result<(opencv::types::VectorOfKeyPoint, opencv::core::Mat), Box<dyn std::error::Error>>;
}

pub trait FeatureMatchingModule {
    type Descriptor;

    fn descriptor_distance(desc1: &Self::Descriptor, desc2: &Self::Descriptor) -> i32;
}

pub trait MapInitializationModule {
    type Frame;
    type Map;
    type InitializationResult;

    fn try_initialize(&mut self, current_frame: &Self::Frame) -> Result<bool, Box<dyn std::error::Error>>;
    fn create_initial_map(&mut self, map: &mut Self::Map) -> Self::InitializationResult ;
}

pub trait RelocalizationModule {
    type Frame;
    type Timestamp;

    fn run(&self) -> bool;
    fn sec_since_lost(&self, current_frame: &Self::Frame) -> Self::Timestamp;
    fn frames_since_lost(&self, current_frame: &Self::Frame) -> i32;
}

pub trait Sim3SolverModule {

}