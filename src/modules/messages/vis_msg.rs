use serde::{Deserialize, Serialize};
use darvis::map::pose::Pose;

#[derive(Debug, Serialize, Deserialize)]
pub struct VisMsg {
    pub new_pose: Pose,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>
}

impl VisMsg {
    pub fn new(pose: Pose, actor_ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            new_pose: pose,
            actor_ids: actor_ids,
        }
    }
}
