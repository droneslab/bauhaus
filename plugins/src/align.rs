use plugins_core::{Function, InvocationError, PluginRegistrar};


use axiom::prelude::*;

// plugins_core::export_plugin!(register_align);

// extern "C" fn register_align(registrar: &mut dyn PluginRegistrar) {
//     registrar.register_function("alignment", Box::new(Alignment));
// }

#[derive(Debug, Clone, PartialEq)]
pub struct Alignment;

use std::any::{TypeId};

use plugins_core::align::*;

//#[async_trait]
impl Function for Alignment {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        //println!("call from plugins AlignMsg {:?}", TypeId::of::<AlignMsg>());
        align((), _context, message);
        Ok(Status::done(()))
    } 
}
