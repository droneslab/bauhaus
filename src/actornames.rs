
use lazy_static::*;
use std::collections::HashMap;
lazy_static! {
    #[derive(Debug)]
    pub static ref GLOBAL_MAP: std::sync::RwLock<HashMap<String, String>> = std::sync::RwLock::new(HashMap::new());
}

pub fn get_global_param( feature : &String, attirbute: &String) -> String
{
    let curr_key = format!("{}_{}", feature, attirbute);
    GLOBAL_MAP.read().unwrap().get(&curr_key).unwrap().clone()

}

pub static FRAME_LOADER: &str = "FRAME_LOADER";
pub static FEATURE_EXTRACTOR: &str = "FEATURE_EXTRACTOR";
pub static TRACKER: &str = "TRACKER";
pub static VISUALIZER: &str = "VISUALIZER";
pub static TRIANGULATION: &str = "TRIANGULATION";
