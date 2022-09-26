use serde::{Deserialize, Serialize};
use darvis::dvutils::DVMatrixGrayscale;

#[derive(Debug, Serialize, Deserialize)]
pub struct ImageMsg {
    frame: DVMatrixGrayscale,
    actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl ImageMsg {
    pub fn new(img: DVMatrixGrayscale, actor_ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            frame: img,
            actor_ids: actor_ids,
        }
    }

    pub fn get_frame(&self) -> &DVMatrixGrayscale {
        &self.frame
    }

    pub fn get_actor_ids(&self) -> &std::collections::HashMap<String, axiom::actors::Aid> {
        &self.actor_ids
    }
}