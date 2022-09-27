use nalgebra::Vector3;
use serde::{Serialize, Deserialize};

use super::map::Id;




#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LocalMapping {
    pub mnMatchesInliers: i32,
}


impl LocalMapping
{

}
