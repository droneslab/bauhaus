use std::marker::PhantomData;
use axiom::prelude::*;
use dvcore::{plugin_functions::*, lockwrap::ReadOnlyWrapper};
use crate::dvmap::{sensor::SensorType, map::Map};

// REGISTER MODULE: add a string to refer to your module here
pub static FRAME_LOADER: &str = "FRAME_LOADER";
pub static TRACKING_FRONTEND: &str = "TRACKING_FRONTEND";
pub static TRACKING_BACKEND: &str = "TRACKING_BACKEND";
pub static LOCAL_MAPPING: &str = "LOCAL_MAPPING";
pub static TRACKER: &str = "TRACKER";
pub static VISUALIZER: &str = "VISUALIZER";

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
pub fn getmethod<S: SensorType + 'static>(fnname: &String, id: &String, map: ReadOnlyWrapper<Map<S>>) -> FunctionProxy
{
    match fnname.as_ref()
    {
        "frameloader" => FunctionProxy {function: Box::new(crate::modules::frameloader::DarvisFrameLoader::new())},
        "tracking_frontend" => FunctionProxy {function: Box::new(crate::modules::tracking_frontend::DarvisTrackingFront::new(map))},
        "tracking_backend" => FunctionProxy {function: Box::new(crate::modules::tracking_backend::DarvisTrackingBack::new(map))},
        "local_mapping" => FunctionProxy {function: Box::new(crate::modules::localmapping::DarvisLocalMapping::new(map))},
        "vis" => FunctionProxy {function: Box::new(crate::modules::vis::DarvisVis::new(id.clone()))},
        "fast_extract" => FunctionProxy {function: Box::new(crate::modules::fast::DarvisFast::new())},
        _ => FunctionProxy {function: Box::new(dvcore::plugin_functions::DarvisNone)},
    }
}

pub struct FeatureManager<S: SensorType> {
    pub object: FunctionProxy,
    _pd: PhantomData<S>
}

impl<S: SensorType + 'static> FeatureManager<S> {
    pub fn new(fnname: &String, id: &String, map: ReadOnlyWrapper<Map<S>>) -> FeatureManager<S> {
        FeatureManager { object: getmethod(fnname, id, map), _pd: PhantomData }
    }

    pub async fn handle(mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<Self> {
        self.object.handle(_context, message).unwrap();
        Ok(Status::done(self))    
    }
}