// Base data structures to be used across the framwork.
pub mod system;
// Logic to get/set global configuration parameters
// and load data from config file.
pub mod config;
// Sensor data structures.
pub mod sensor;

// Wrapping opencv/algebra structs with our own (ie DVMatrix) so that we can
// implement traits on them. Notably, serialize/deserialize.
pub mod matrix;