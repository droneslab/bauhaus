// Base data structures to be used across the framwork.
pub mod base;
// Internal core framework trait/abstraction functions to be implemented.
pub mod plugin_functions;
// Logic to get/set global configuration parameters
// and load data from config file.
pub mod config;

/// *** Map *** ///
// Wraps Arc<RwLock> to give it the option of being read-only. 
// Used for the map, but extensible for other types.
pub mod lockwrap;

/// *** Misc. *** ///
// Wrapping opencv/algebra structs with our own (ie DVMatrix) so that we can
// implement traits on them. Notably, serialize/deserialize.
pub mod matrix;
