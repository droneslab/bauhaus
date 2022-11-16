use axiom::prelude::*;
use dvcore::{plugin_functions::*, lockwrap::ReadOnlyWrapper};
use crate::dvmap::{map::Map};

// REGISTER MODULE: add a string to refer to your module here
pub static FRAME_LOADER: &str = "FRAME_LOADER";
pub static TRACKING_FRONTEND: &str = "TRACKING_FRONTEND";
pub static TRACKING_BACKEND: &str = "TRACKING_BACKEND";
pub static LOCAL_MAPPING: &str = "LOCAL_MAPPING";
pub static LOOP_CLOSING: &str = "LOOP_CLOSING";
pub static TRACKER: &str = "TRACKER";
pub static VISUALIZER: &str = "VISUALIZER";

pub static FEATURE_DETECTION: &str = "FEATURE_DETECTION";
pub static CAMERA: &str = "CAMERA";
pub static MATCHER: &str = "CAMERA";

/// REGISTER MODULE: All the features/actors that are implemented, 
/// need to be registered through this function.

/// Returns a actor method of type FunctionProxy
///
/// # Arguments
///
/// * `fnname` - A string slice that holds the name of the actor function to be instanciated and called later
/// * `id` - A string slice that holds the unique id for each actor function created. Help to call a unique actor.
///
/// # Examples
///
/// ```
/// # use darvis::register_modules::getmethod;
/// let orb_extract_fn = getmethod("orb_extract".to_string(), "orb_extract".to_string());
/// ```
/// 
pub fn getmethod(fnname: &String, id: &String, map: ReadOnlyWrapper<Map>) -> FunctionProxy
{
    match fnname.as_ref()
    {
        "frameloader" => FunctionProxy {function: Box::new(crate::actors::frameloader::DarvisFrameLoader::new())},
        "tracking_frontend" => FunctionProxy {function: Box::new(crate::actors::tracking_frontend::DarvisTrackingFront::new(map))},
        "tracking_backend" => FunctionProxy {function: Box::new(crate::actors::tracking_backend::DarvisTrackingBack::new(map))},
        "local_mapping" => FunctionProxy {function: Box::new(crate::actors::local_mapping::DarvisLocalMapping::new(map))},
        "loop_closing" => FunctionProxy {function: Box::new(crate::actors::loop_closing::DarvisLoopClosing::new(map))},
        "vis" => FunctionProxy {function: Box::new(crate::actors::vis::DarvisVis::new(id.clone()))},
        "fast_extract" => FunctionProxy {function: Box::new(crate::actors::fast::DarvisFast::new())},
        _ => FunctionProxy {function: Box::new(dvcore::plugin_functions::DarvisNone)},
    }
}

pub struct FeatureManager {
    pub object: FunctionProxy,
}

impl FeatureManager {
    pub fn new(fnname: &String, id: &String, map: ReadOnlyWrapper<Map>) -> FeatureManager {
        FeatureManager { object: getmethod(fnname, id, map) }
    }

    pub async fn handle(mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<Self> {
        self.object.handle(_context, message).unwrap();
        Ok(Status::done(self))    
    }
}