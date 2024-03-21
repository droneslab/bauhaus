use core::{config::{SETTINGS, SYSTEM}, system::{Actor, System}};
use std::collections::HashMap;
use log::error;

use crate::{modules::{bow::DVVocabulary, camera::{Camera, CameraType}}, MapLock};

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
pub static CAMERA: &str = "CAMERA";
pub static MATCHER: &str = "MATCHER";
pub static FEATURES: &str = "FEATURES";

// DARVIS SYSTEM ACTORS
pub static SHUTDOWN_ACTOR: &str = "SHUTDOWN";

lazy_static! {
    pub static ref CAMERA_MODULE: Camera = {
        Camera::new(CameraType::Pinhole).unwrap()
    };
    pub static ref VOCABULARY: DVVocabulary = {
        let filename = SETTINGS.get::<String>(SYSTEM, "vocabulary_file");
        DVVocabulary::load(filename)
    };
}


pub fn spawn_actor(
    actor_tag: String, system: System, map: Option<MapLock>
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
        str if str == "end-to-end tracking".to_string() => {
            crate::actors::tracking_full::TrackingFull::spawn(system, map.expect("Tracking needs the map!"))
        },
        str if str == "orbslam local mapping".to_string() => {
            crate::actors::local_mapping::LocalMapping::spawn(system, map.expect("Local mapping needs the map!"))
        },
        str if str == "orbslam2 loop closing".to_string() => {
            crate::actors::loop_closing::LoopClosing::spawn(system, map.expect("Loop closing needs the map!"))
        },
        str if str == "visualizer".to_string() => {
            crate::actors::visualizer::DarvisVisualizer::spawn(system, map.expect("Visualizer needs the map!"))
        },
        str if str == SHUTDOWN_ACTOR.to_string() => {
            crate::actors::shutdown::ShutdownActor::spawn(system, ())
        },
        _ => {
            error!("Actor not implemented: {}", actor_tag);
        },
    };
}
