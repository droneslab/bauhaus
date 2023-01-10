/// Tracking front end  ... feature extraction and initialization
pub mod tracking_frontend;

/// Tracking back end ... Alignment of two frames and getting alignment translation and rotation.
pub mod tracking_backend;

pub mod local_mapping;

pub mod loop_closing;

/// Visualization of the Frames and Trajectory traced
pub mod vis;

/// FAST Feature extraction 
pub mod fast;

// Messages sent to/from actors
pub mod messages;