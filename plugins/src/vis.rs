use plugins_core::{Function, InvocationError, PluginRegistrar};


use axiom::prelude::*;

// plugins_core::export_plugin!(register_visualization);

// extern "C" fn register_visualization(registrar: &mut dyn PluginRegistrar) {
//     registrar.register_function("visualization", Box::new(Visualization{object: Vis::new()}));
// }

#[derive(Debug, Clone)]
pub struct Visualization
{
    pub object: Vis,
}

use std::any::{TypeId};

use plugins_core::vis::*;

//#[async_trait]
impl Function for Visualization {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        println!("call from plugins VisMsg {:?}", TypeId::of::<VisMsg>());

        self.object.visualize1(_context, message);
        //Vis::visualize(self, _context, message);
        Ok(Status::done(()))
    }
}
