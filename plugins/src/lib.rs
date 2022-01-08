
pub mod orb_extract;
pub mod align;
pub mod vis;

use plugins_core::{PluginRegistrar};

use plugins_core::vis::Vis;

plugins_core::export_plugin!(register);

extern "C" fn register(registrar: &mut dyn PluginRegistrar) {
    registrar.register_function("orb_extract", Box::new(orb_extract::OrbExtract));
    registrar.register_function("alignment", Box::new(align::Alignment));
    registrar.register_function("visualization", Box::new(vis::Visualization{object: Vis::new()}));

}