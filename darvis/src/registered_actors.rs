use core::{config::{SETTINGS, SYSTEM}, system::{Actor, System}};
use std::sync::Arc;
use log::error;
use parking_lot::RwLock;

use crate::{map::read_only_lock::ReadWriteMap, modules::{bow::Vocabulary, camera::{Camera, CameraType}, imu::IMU, module_definitions::{FeatureExtractionModule, FullMapOptimizationModule, LocalMapOptimizationModule, LoopDetectionModule}, orbslam_matcher::ORBMatcherTrait}};
use crate::modules::module_definitions::VocabularyModule;

// USER-DEFINED ACTORS: add a string to name your actor here
// These are the names that actors in the system use to find/communicate with each other
// You can overload actor names between different config files. It can be useful to use the same name for two different
// implementations of ACTOR B if you want ACTOR A to find it without knowing its internals (eg, LOCAL_MAPPING refers to
// TRACKING_BACKEND without knowing whether the implementation is tracking_backend.rs or tracking_full.rs).
pub static TRACKING_FRONTEND: &str = "TRACKING_FRONTEND";
pub static TRACKING_BACKEND: &str = "TRACKING_BACKEND";
pub static LOCAL_MAPPING: &str = "LOCAL_MAPPING";
pub static LOOP_CLOSING: &str = "LOOP_CLOSING";
pub static VISUALIZER: &str = "VISUALIZER";

// USER-DEFINED MODULES: add a string to name your module here
pub static FEATURE_DETECTION: &str = "FEATURE_DETECTION";
pub static FEATURE_MATCHER: &str = "FEATURE_MATCHER";
pub static CAMERA: &str = "CAMERA";
pub static FEATURES: &str = "FEATURES";
pub static LOOP_DETECTION: &str = "LOOP_DETECTION";
pub static LOCAL_MAP_OPTIMIZATION: &str = "LOCAL_MAP_OPTIMIZATION";
pub static FULL_MAP_OPTIMIZATION: &str = "FULL_MAP_OPTIMIZATION";
pub static IMU: &str = "IMU";

// DARVIS SYSTEM ACTORS
pub static SHUTDOWN_ACTOR: &str = "SHUTDOWN";

// Modules without internal state ... can have one global instance since we don't have to worry about sharing data between threads
lazy_static! {
    pub static ref CAMERA_MODULE: Camera = {
        Camera::new(CameraType::Pinhole).unwrap()
    };
    pub static ref VOCABULARY_MODULE: Vocabulary = {
        let filename = SETTINGS.get::<String>(SYSTEM, "vocabulary_file");
        Vocabulary::load(filename)
    };
    pub static ref FEATURE_MATCHING_MODULE: Box<dyn ORBMatcherTrait> = {
        let module_tag = SETTINGS.get::<String>(FEATURE_MATCHER, "module_tag");
        match module_tag.as_ref() {
            str if str == "orbslam feature matching".to_string() => {
                Box::new(crate::modules::orbslam_matcher::ORBMatcher::new() )
            },
            _ => {
                error!("Module not implemented: {}", module_tag);
                panic!();
            },
        }    
    };
    pub static ref FULL_MAP_OPTIMIZATION_MODULE: Box<dyn FullMapOptimizationModule + Send + Sync> = {
        let module_tag = SETTINGS.get::<String>(FULL_MAP_OPTIMIZATION, "module_tag");
        match module_tag.as_ref() {
            str if str == "bundle adjustment".to_string() => {
                Box::new(crate::modules::global_bundle_adjustment::GlobalBundleAdjustment { } )
            },
            _ => {
                error!("Module not implemented: {}", module_tag);
                panic!();
            },
        }
    };
    pub static ref LOCAL_MAP_OPTIMIZATION_MODULE: Box<dyn LocalMapOptimizationModule + Send + Sync> = {
        let module_tag = SETTINGS.get::<String>(LOCAL_MAP_OPTIMIZATION, "module_tag");
        match module_tag.as_ref() {
            str if str == "bundle adjustment".to_string() => {
                Box::new(crate::modules::local_bundle_adjustment::LocalBundleAdjustment { } )
            },
            _ => {
                error!("Module not implemented: {}", module_tag);
                panic!();
            },
        }
    };
    pub static ref IMU_MODULE: Arc<RwLock<IMU>> = {
        Arc::new(RwLock::new(IMU::new()))
    };

}



pub fn spawn_actor(
    actor_tag: String, system: System, map: Option<ReadWriteMap>
) {
    // Spawn actors using the actor TAG, not NAME. These match up your intended actor with the actual file that implements it.
    // If it is clearer to you, you could make these tags equivalent to the name of the file that implements the actor.
    // But you don't HAVE to (ie, we don't actually use the file name at all).
    match actor_tag.as_ref() {
        str if str == "orbslam tracking frontend".to_string() => {
            crate::actors::tracking_frontend::TrackingFrontEnd::spawn(system, ())
        },
        str if str == "orbslam tracking backend".to_string() => {
            crate::actors::tracking_backend::TrackingBackend::spawn(system, map.expect("Tracking backend needs the map!"))
        },
        // str if str == "end-to-end tracking".to_string() => {
        //     crate::actors::tracking_full::TrackingFull::spawn(system, map.expect("Tracking needs the map!"))
        // },
        str if str == "orbslam local mapping".to_string() => {
            crate::actors::local_mapping::LocalMapping::spawn(system, map.expect("Local mapping needs the map!"))
        },
        str if str == "orbslam3 loop closing".to_string() => {
            crate::actors::loop_closing::LoopClosing::spawn(system, map.expect("Loop closing needs the map!"))
        },
        str if str == "visualizer".to_string() => {
            crate::actors::visualizer::DarvisVisualizer::spawn(system, map.expect("Visualizer needs the map!"))
        },
        // str if str == "optical flow tracking".to_string() => {
        //     crate::actors::tracking_optical_flow::TrackingOpticalFlow::spawn(system, map.expect("Tracking needs the map!"))
        // },
        str if str == "gtsam tracking frontend".to_string() => {
            crate::actors::tracking_frontend_gtsam::TrackingFrontendGTSAM::spawn(system, map.expect("Tracking needs the map!"))
        },
        str if str == "gtsam tracking backend".to_string() => {
            crate::actors::tracking_backend_gtsam::TrackingBackendGTSAM::spawn(system, map.expect("Tracking needs the map!"))
        },

        str if str == SHUTDOWN_ACTOR.to_string() => {
            crate::actors::shutdown::ShutdownActor::spawn(system, map.expect("Shutdown needs the map!"))
        },
        _ => {
            error!("Actor not implemented: {}", actor_tag);
        },
    };
}

// MODULES with internal state
pub fn new_feature_extraction_module(is_ini: bool) -> Box<dyn FeatureExtractionModule> {
    let module_tag = SETTINGS.get::<String>(FEATURE_DETECTION, "module_tag");
    match module_tag.as_ref() {
        str if str == "orbslam feature detection".to_string() => {
            Box::new(crate::modules::orbslam_extractor::ORBExtractor::new(is_ini))
        },
        str if str == "opencv feature detection".to_string() => {
            Box::new(crate::modules::opencv_extractor::OpenCVExtractor::new(is_ini))
        },
        str if str == "good features to track".to_string() => {
            Box::new(crate::modules::good_features_to_track::GoodFeaturesExtractor::new())
        }
        _ => {
            error!("Module not implemented: {}", module_tag);
            panic!();
        },
    }
}

pub fn new_loop_detection_module() -> Box<dyn LoopDetectionModule> {
    let module_tag = SETTINGS.get::<String>(LOOP_DETECTION, "module_tag");
    match module_tag.as_ref() {
        str if str == "orbslam3 loop detection".to_string() => {
            Box::new(crate::modules::orbslam3_loop_detection::ORBSLAM3LoopDetection::new() )
        },
        _ => {
            error!("Module not implemented: {}", module_tag);
            panic!();
        },
    }
}

