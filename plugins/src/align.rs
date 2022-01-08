use plugins_core::{Function};

use axiom::prelude::*;


#[derive(Debug, Clone, PartialEq)]
pub struct Alignment;

use plugins_core::align::*;
use plugins_core::darvismsg::DarvisMessage;
use std::collections::HashMap;

impl Function for Alignment {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        align((), _context, message).unwrap();
        Ok(Status::done(()))
    } 

    fn send_new(&mut self, message: DarvisMessage, aids:  &HashMap<String, axiom::actors::Aid>) -> ActorResult<()>
    {
        Ok(Status::done(()))
    }

}
