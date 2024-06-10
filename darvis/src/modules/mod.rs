// Code related to SLAM task, but that isn't an actor.

pub mod camera;
pub mod orbmatcher;
pub mod optimizer; // Optimization code, uses g2o crate to call g2o in C++
pub mod imu;
pub mod map_initialization;
pub mod relocalization;
pub mod geometric_tools;
pub mod image;
pub mod sim3solver;
pub mod bow;
pub mod orbextractor;