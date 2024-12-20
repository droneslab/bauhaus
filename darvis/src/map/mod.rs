// Map implementation
// Contains map data structures + logic for reading/writing

pub mod map;

pub mod frame;
pub mod keyframe;
pub mod mappoint;
pub mod pose;
pub mod features;
pub mod keyframe_database;

// Wraps Arc<RwLock> to give it the option of being read-only. 
// Used for the map, but extensible for other types.
pub mod read_only_lock;
