// Code related to SLAM task, but that isn't an actor.

pub mod camera;
pub mod orbslam_matcher;
pub mod optimizer; // Some optimization code that isn't separated into its own modules
pub mod imu;
pub mod map_initialization;
pub mod relocalization;
pub mod geometric_tools;
pub mod image;
pub mod sim3solver;
pub mod bow;
pub mod orbslam_extractor; // orb extractor from orbslam3, using bindings to C++ code
pub mod opencv_extractor; // orb extractor using opencv feature detection 
pub mod good_features_to_track; // good features to track using opencv feature detection
pub mod orbslam3_loop_detection;
pub mod local_bundle_adjustment; 
pub mod global_bundle_adjustment;

pub mod module_definitions;
