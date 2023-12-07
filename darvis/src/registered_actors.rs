use dvcore::{maplock::{ReadOnlyMap, ReadWriteMap}, actor::{ActorChannels, Actor}};
use crate::{dvmap::map::Map, MapLock};

// USER-DEFINED ACTORS: add a string to name your actor here
pub static TRACKING_FRONTEND: &str = "TRACKING_FRONTEND";
pub static TRACKING_BACKEND: &str = "TRACKING_BACKEND";
pub static LOCAL_MAPPING: &str = "LOCAL_MAPPING";
pub static LOOP_CLOSING: &str = "LOOP_CLOSING";
pub static VISUALIZER: &str = "VISUALIZER";

// USER-DEFINED MODULES: add a string to name your module here
pub static FEATURE_DETECTION: &str = "FEATURE_DETECTION";
pub static CAMERA: &str = "CAMERA";
pub static MATCHER: &str = "MATCHER";

// DARVIS SYSTEM ACTORS
pub static SHUTDOWN_ACTOR: &str = "SHUTDOWN";


pub fn spawn(
    actor_name: String, actor_channels: ActorChannels, map: Option<MapLock>
) {
    match actor_name.as_ref() {
        str if str == TRACKING_FRONTEND.to_string() => {
            crate::actors::tracking_frontend::DarvisTrackingFront::spawn(actor_channels, ())
        },
        str if str == TRACKING_BACKEND.to_string() => {
            crate::actors::tracking_backend::DarvisTrackingBack::spawn(actor_channels, map.expect("Tracking backend needs the map!"))
        },
        str if str == LOCAL_MAPPING.to_string() => {
            crate::actors::local_mapping::DarvisLocalMapping::spawn(actor_channels, map.expect("Local mapping needs the map!"))
        },
        str if str == LOOP_CLOSING.to_string() => {
            crate::actors::loop_closing::DarvisLoopClosing::spawn(actor_channels, map.expect("Loop closing needs the map!"))
        },
        str if str == VISUALIZER.to_string() => {
            crate::actors::visualizer::DarvisVisualizer::spawn(actor_channels, map.expect("Visualizer needs the map!"))
        },
        str if str == SHUTDOWN_ACTOR.to_string() => {
            crate::actors::shutdown::ShutdownActor::spawn(actor_channels, ())
        },
        _ => {
            dvcore::actor::NullActor::spawn(actor_channels, ())
        },
    };
}

