//For testing all the implemented features using cargo test --doc 

/// Base data structures to be used across the framwork.
pub mod base;
/// Orb Feature extraction 
pub mod orb;
/// Alignment of two frames and getting alignment translation and rotation.
pub mod align;

/// Configuration file handling utilities.
pub mod config;

/// Visualization of the Frames and Trajectory traced
pub mod vis;

/// Internal core framework trait/abstraction functions to be implemented. [NOTE] DO NOT MODIFY THIS FILE
pub mod pluginfunction;

/// Register new plugins in this module.
pub mod registerplugin;

/// Algebric utilitiy wrapper to support abstract implementation of 3rd party library used.
pub mod dvutils;

/// This is documentation of all development and how to's to enable graceful enhancement.
pub mod doc;

/// Implemented optical flow feature extraction and tracking
pub mod opflow;