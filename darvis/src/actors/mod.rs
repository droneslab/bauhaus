pub mod tracking_frontend; // orbslam3 tracking frontend
pub mod tracking_backend; // orbslam3 tracking backend
pub mod local_mapping; // orbslam3 local mapping
pub mod loop_closing; // orbslam2 loop closing
// pub mod loop_closing3; // orbslam3 loop closing. NOT FINISHED!

pub mod tracking_full; // end-to-end tracking

// Darvis-defined actors
pub mod visualizer;
pub mod shutdown; // handles CTRL+C

// General messages sent to/from actors
pub mod messages;
