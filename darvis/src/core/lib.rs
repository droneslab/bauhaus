// Base data structures to be used across the framwork.
pub mod system;
pub mod module;
// Logic to get/set global configuration parameters
// and load data from config file.
pub mod config;
// Sensor data structures.
pub mod sensor;

// Wraps Arc<RwLock> to give it the option of being read-only. 
// Used for the map, but extensible for other types.
pub mod read_only_lock;

// Wrapping opencv/algebra structs with our own (ie DVMatrix) so that we can
// implement traits on them. Notably, serialize/deserialize.
pub mod matrix;
