//For testing all the implemented features using cargo test --doc 

/// Base data structures to be used across the framwork.
pub mod base;
/// Handles logic to load data from config file.
pub mod load_config;
/// This is documentation of all development and how to's to enable graceful enhancement.
pub mod doc;

/// [NOTE] DO NOT MODIFY THESE FILES:
/// Logic to get/set global configuration parameters.
pub mod global_params;
/// Internal core framework trait/abstraction functions to be implemented.
pub mod plugin_functions;
// Wraps Arc<RwLock> to give it the option of being read-only. 
// Used for the map, but extensible for other types.
pub mod lockwrap;

/// Algebric utilitiy wrapper to support abstract implementation of 3rd party library used.
pub mod dvutils;
// Map implementation
// Contains map data structures + logic for reading/writing and the map actor
pub mod map;
