
use axiom::prelude::*;

use crate::pluginfunction::*;


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