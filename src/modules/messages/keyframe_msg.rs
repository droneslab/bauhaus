use serde::{Deserialize, Serialize};
use darvis::map::map::Id;

// Message type for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct KeyFrameMsg {
    // pub kf_id: Id,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl KeyFrameMsg {
    pub fn new(kf_id: Id, actor_ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            // kf_id: kf_id,
            actor_ids: actor_ids,
        }
    }
}