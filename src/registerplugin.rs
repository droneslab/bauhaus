
use axiom::prelude::*;

use crate::pluginfunction::*;

/// All the features/actors that are implemented, need to be registered through this function.

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
/// # use darvis::registerplugin::getmethod;
/// let orb_extract_fn = getmethod("orb_extract".to_string(), "orb_extract".to_string());
/// ```
/// 
pub fn getmethod(fnname: String, id: String) -> FunctionProxy
{
    match fnname.as_ref()
    {
        "orb_extract" => FunctionProxy {function: Box::new(crate::orb::DarvisOrb::new())}
        ,
        "align" => FunctionProxy {function: Box::new(crate::align::DarvisAlign::new())}
        ,
        "vis" => FunctionProxy {function: Box::new(crate::vis::DarvisVis::new(id))}
        ,        
        "sift_extract" => FunctionProxy {function: Box::new(crate::sift::DarvisSift::new())}
        ,
        _ => FunctionProxy {function: Box::new(crate::pluginfunction::DarvisNone)}
        ,
    }
}


pub struct FeatureManager{
    pub object: FunctionProxy,
}

impl FeatureManager
{
    pub fn new(fnname: String, id: String) -> FeatureManager
    {
        FeatureManager { object: getmethod(fnname, id)}
    }

    pub async fn handle(mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<Self> {
        
        self.object.handle(_context, message).unwrap();
        Ok(Status::done(self))    
    }
}