use plugins_core::{Function, InvocationError, PluginRegistrar};


use axiom::prelude::*;

// plugins_core::export_plugin!(register_orb_extract);

// extern "C" fn register_orb_extract(registrar: &mut dyn PluginRegistrar) {
//     registrar.register_function("orb_extract", Box::new(OrbExtract));
// }

#[derive(Debug, Clone, PartialEq)]
pub struct OrbExtract;

use std::any::{TypeId};

use plugins_core::orb::*;

//#[async_trait]
impl Function for OrbExtract {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        //println!("call from plugins OrbMsg {:?}", TypeId::of::<OrbMsg>());
        orb_extract((), _context, message).unwrap();
        Ok(Status::done(()))
    } 
}
