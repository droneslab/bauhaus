use dvcore::{lockwrap::ReadOnlyWrapper, base::{ActorSystem}};
use crate::{dvmap::{map::Map}, };

// USER-DEFINED ACTORS: add a string to name your actor here
pub static TRACKING_FRONTEND: &str = "TRACKING_FRONTEND";
pub static TRACKING_BACKEND: &str = "TRACKING_BACKEND";
pub static LOCAL_MAPPING: &str = "LOCAL_MAPPING";
pub static LOOP_CLOSING: &str = "LOOP_CLOSING";

// USER-DEFINED MODULES: add a string to name your module here
pub static FEATURE_DETECTION: &str = "FEATURE_DETECTION";
pub static CAMERA: &str = "CAMERA";
pub static MATCHER: &str = "MATCHER";

// DARVIS SYSTEM ACTORS
pub static SHUTDOWN_ACTOR: &str = "SHUTDOWN";
pub static VISUALIZER: &str = "VISUALIZER";
pub static MAP_ACTOR: &str = "MAP_ACTOR"; 


pub fn run_actor(actor_name: String, map: ReadOnlyWrapper<Map>, actor_system: ActorSystem) {
    match actor_name.as_ref() {
        str if str == TRACKING_FRONTEND.to_string() => {
            let mut actor = crate::actors::tracking_frontend::DarvisTrackingFront::new(actor_system);
            actor.run();
        },
        str if str == TRACKING_BACKEND.to_string() => {
            let mut actor = crate::actors::tracking_backend::DarvisTrackingBack::new(map, actor_system);
            actor.run();
        },
        str if str == LOCAL_MAPPING.to_string() => {
            let mut actor = crate::actors::local_mapping::DarvisLocalMapping::new(map, actor_system);
            actor.run();
        },
        str if str == LOOP_CLOSING.to_string() => {
            let mut actor = crate::actors::loop_closing::DarvisLoopClosing::new(map, actor_system);
            actor.run();
        },
        _ => {
            panic!("Actor {} Not Implemented!!", actor_name);
        },
    } 
}