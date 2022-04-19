use serde::{Deserialize, Serialize};

// Message type for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct ImagesMsg {
    // Vector of image paths to read in/extract
    pub img_paths: Vec<String>,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl ImagesMsg {
    pub fn new(vec: Vec<String>, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            img_paths: vec,
            actor_ids: ids,
        }
    }

    pub fn get_img_paths(&self ) -> &Vec<String> {
        &self.img_paths
    }

    pub fn get_actor_ids(&self ) -> &std::collections::HashMap<String, axiom::actors::Aid> {
        &self.actor_ids
    }
}