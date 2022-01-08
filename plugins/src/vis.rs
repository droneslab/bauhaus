use plugins_core::{Function};

use axiom::prelude::*;

#[derive(Debug, Clone)]
pub struct Visualization
{
    pub object: Vis,
}

use plugins_core::vis::Vis;
use plugins_core::darvismsg::DarvisMessage;
use std::collections::HashMap;

impl Function for Visualization {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.object.visualize(_context, message).unwrap();
        Ok(Status::done(()))
    }

    fn send_new(&mut self, message: DarvisMessage, aids:  &HashMap<String, axiom::actors::Aid>) -> ActorResult<()>
    {
        Ok(Status::done(()))
    }
}
