pub mod tracking_frontend; // orbslam tracking frontend
pub mod tracking_backend; // orbslam tracking backend
pub mod local_mapping; // orbslam local mapping
// pub mod loop_closing3; // orbslam3 loop closing. NOT FINISHED!
pub mod loop_closing2; // orbslam2 loop closing

pub mod tracking_full; // end-to-end tracking

// Darvis-defined actors
pub mod visualizer;
pub mod shutdown; // handles CTRL+C

// General messages sent to/from actors
pub mod messages;
