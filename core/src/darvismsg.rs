use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub enum DarvisMessage
{
    ImagePaths(Vec<String>, std::collections::HashMap<String, axiom::actors::Aid>),
}