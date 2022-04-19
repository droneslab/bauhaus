//For testing all the implemented features using cargo test --doc 

pub mod base;

/// Utilities to get/set global configuration parameters, 
pub mod config;

/// Handles logic to load data from config file.
pub mod load_config;

/// Internal core framework trait/abstraction functions to be implemented. [NOTE] DO NOT MODIFY THIS FILE
pub mod plugin_functions;

/// Algebric utilitiy wrapper to support abstract implementation of 3rd party library used.
pub mod dvutils;

/// This is documentation of all development and how to's to enable graceful enhancement.
pub mod doc;

// Global map
pub mod map;
