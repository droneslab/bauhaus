use serde::{Deserialize, Serialize};
use darvis::dvutils::DVMatrixGrayscale;

// Message type for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct ImageMsg {
    // Vector of image paths to read in/extract
    frame: DVMatrixGrayscale,
    actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl ImageMsg {
    pub fn new(img: DVMatrixGrayscale, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            frame: img,
            actor_ids: ids,
        }
    }

    pub fn get_frame(&self ) -> &DVMatrixGrayscale {
        &self.frame
    }

    pub fn get_actor_ids(&self ) -> &std::collections::HashMap<String, axiom::actors::Aid> {
        &self.actor_ids
    }
}