/// Tracking front end  ... feature extraction and initialization
pub mod tracking_frontend;

/// Tracking back end ... Alignment of two frames and getting alignment translation and rotation.
pub mod tracking_backend;

pub mod localmapping;

/// Visualization of the Frames and Trajectory traced
pub mod vis;

/// Frame Loader to load frames and stream to the Feature Extraction actor
pub mod frameloader;

/// FAST Feature extraction 
pub mod fast;

// Messages sent to/from actors
pub mod messages;

// Optimization code, uses g2orust crate to call g2o in C++
pub mod optimizer; 

