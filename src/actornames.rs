use lazy_static::*;
use std::collections::HashMap;

pub static FRAME_LOADER: &str = "FRAME_LOADER";
pub static FEATURE_EXTRACTOR: &str = "FEATURE_EXTRACTOR";
pub static TRACKER: &str = "TRACKER";
pub static VISUALIZER: &str = "VISUALIZER";
pub static SYSTEM_SETTINGS: &str = "SYSTEM_SETTINGS"; 

pub trait OverloadedConfigParams<T> {
    fn get_param(&self, module : String, param: String) -> T;
    fn insert_param(&self, key: String, value: T);
}

pub struct Box {
    string_field: Option<String>,
    bool_field: Option<bool>
}

pub struct GlobalParams {
    pub params: std::sync::RwLock<HashMap<String, Box>>,
}

lazy_static! {
    #[derive(Debug)]
    pub static ref GLOBAL_PARAMS: GlobalParams = GlobalParams {
        params: std::sync::RwLock::new(HashMap::new())
    };
}

impl GlobalParams {
    pub fn insert<T>(&self, key : String, value: T)
    where Self: OverloadedConfigParams<T> {
        self.insert_param(key, value);
    }
    pub fn get<T>(&self, module : String, param: String) -> T
    where Self: OverloadedConfigParams<T> {
        self.get_param(module, param)
    }
}

impl OverloadedConfigParams<String> for GlobalParams {
    fn get_param(&self, module : String, param: String) -> String {
        let curr_key = format!("{}_{}", module, param);
        let unlocked_params = GLOBAL_PARAMS.params.read().unwrap();
        unlocked_params.get(&curr_key).unwrap().string_field.as_ref().unwrap().to_string()
    }
    fn insert_param(&self, key: String, value: String) {
        let boxed_value = Box {
            string_field: Some(value.clone()),
            bool_field: None
        };
        let mut unlocked_params = GLOBAL_PARAMS.params.write().unwrap();
        unlocked_params.insert(key.clone(), boxed_value);
    }
}

impl OverloadedConfigParams<bool> for GlobalParams {
    fn get_param(&self, module : String, param: String) -> bool {
        let curr_key = format!("{}_{}", module, param);
        let unlocked_params = GLOBAL_PARAMS.params.read().unwrap();
        *unlocked_params.get(&curr_key).unwrap().bool_field.as_ref().unwrap()
    }
    fn insert_param(&self, key: String, value: bool) {
        let boxed_value = Box {
            string_field: None,
            bool_field: Some(value)
        };
        let mut unlocked_params = GLOBAL_PARAMS.params.write().unwrap();
        unlocked_params.insert(key.clone(), boxed_value);
    }
}
