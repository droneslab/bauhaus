use serde::{Deserialize, Serialize};
use darvis::map::pose::Pose;

/// Public message struct for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct VisMsg {
    /// Pose of image paths to read in/extract, Poses take 2 matrixes, pos and rot <int type, # rows, # col, data storage?>
    pub new_pose: Pose,
    /// all actor ids
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>
}

impl VisMsg {
    pub fn new(pose: Pose, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            new_pose: pose,
            actor_ids: ids,
        }
    }
}
