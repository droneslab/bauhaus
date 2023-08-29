// Base data structures to be used across the framwork.
pub mod base;
// Logic to get/set global configuration parameters
// and load data from config file.
pub mod config;
// Sensor data structures. Needs to be in dvcore because ConfigValueBox needs to hold a sensor
pub mod sensor;

// Wraps Arc<RwLock> to give it the option of being read-only. 
// Used for the map, but extensible for other types.
pub mod lockwrap;

// Wrapping opencv/algebra structs with our own (ie DVMatrix) so that we can
// implement traits on them. Notably, serialize/deserialize.
pub mod matrix;

// Internal core framework trait/abstraction functions to be implemented.
// pub mod plugin_functions;
