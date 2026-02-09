pub mod local_mapping; // orbslam3 local mapping
pub mod loop_closing;
pub mod tracking_backend; // orbslam3 tracking backend
pub mod tracking_frontend; // orbslam3 tracking frontend // orbslam3 loop closing.

pub mod local_mapping_gtsam;
pub mod tracking_backend_gtsam;
pub mod tracking_frontend_gtsam;

// Bauhaus-defined actors
pub mod shutdown;
pub mod visualizer; // handles CTRL+C

// General messages sent to/from actors
pub mod messages;

pub mod garbage_tracking;
