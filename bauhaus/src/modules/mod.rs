// Code related to SLAM task, but that isn't an actor.

pub mod bow;
pub mod camera;
pub mod geometric_tools;
pub mod global_bundle_adjustment;
pub mod good_features_to_track; // good features to track using opencv feature detection
pub mod image;
pub mod imu;
pub mod local_bundle_adjustment;
pub mod map_initialization;
pub mod opencv_extractor; // orb extractor using opencv feature detection
pub mod optimizer; // Some optimization code that isn't separated into its own modules
pub mod orbslam3_loop_detection;
pub mod orbslam_extractor; // orb extractor from orbslam3, using bindings to C++ code
pub mod orbslam_matcher;
pub mod relocalization;
pub mod sim3solver;
pub mod opengv_translation_only_sac; // Implementation of opengv TranslationOnlySacProblem and Ransac

pub mod module_definitions;
