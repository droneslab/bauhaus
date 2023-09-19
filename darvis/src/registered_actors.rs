use dvcore::{lockwrap::ReadOnlyWrapper, base::{ActorChannels, Actor}};
use rerun::RecordingStream;
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


pub fn get_actor(
    actor_name: String, actor_channels: ActorChannels, map: Option<ReadOnlyWrapper<Map>>, rec_stream: Option<RecordingStream>
) -> Box<dyn Actor> {
    match actor_name.as_ref() {
        str if str == TRACKING_FRONTEND.to_string() => {
            return Box::new(crate::actors::tracking_frontend::DarvisTrackingFront::new(actor_channels))
        },
        str if str == TRACKING_BACKEND.to_string() => {
            return Box::new(crate::actors::tracking_backend::DarvisTrackingBack::new(map.expect("Tracking backend needs the map!"), actor_channels))
        },
        str if str == LOCAL_MAPPING.to_string() => {
            return Box::new(crate::actors::local_mapping::DarvisLocalMapping::new(map.expect("Local mapping needs the map!"), actor_channels))
        },
        str if str == LOOP_CLOSING.to_string() => {
            return Box::new(crate::actors::loop_closing::DarvisLoopClosing::new(map.expect("Loop closing needs the map!"), actor_channels))
        },
        str if str == VISUALIZER.to_string() => {
            return Box::new(crate::actors::visualizer::DarvisVisualizer::new(actor_channels, map.expect("Visualizer needs the map!"), rec_stream.expect("Visualizer needs the recording stream!")))
        },
        str if str == SHUTDOWN_ACTOR.to_string() => {
            return Box::new(crate::actors::shutdown::ShutdownActor::new(actor_channels))
        },
        _ => {
            return Box::new(dvcore::base::DarvisNone::new(actor_name.clone()))
        },
    };
}
