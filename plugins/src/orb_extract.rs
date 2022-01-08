use plugins_core::{Function};

use axiom::prelude::*;


#[derive(Debug, Clone, PartialEq)]
pub struct OrbExtract;

//use std::any::{TypeId};

use plugins_core::orb::*;
use plugins_core::darvismsg::DarvisMessage;
use std::collections::HashMap;

impl Function for OrbExtract {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        //println!("call from plugins OrbMsg {:?}", TypeId::of::<OrbMsg>());
        orb_extract((), _context, message).unwrap();
        Ok(Status::done(()))
    } 

    fn send_new(&mut self, message: DarvisMessage, aids: &HashMap<String, axiom::actors::Aid>) -> ActorResult<()>
    {   

        Ok(Status::done(()))
    }

}
