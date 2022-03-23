use lazy_static::*;
use std::collections::HashMap;

pub static FRAME_LOADER: &str = "FRAME_LOADER";
pub static FEATURE_EXTRACTOR: &str = "FEATURE_EXTRACTOR";
pub static TRACKER: &str = "TRACKER";
pub static VISUALIZER: &str = "VISUALIZER";
pub static SYSTEM_SETTINGS: &str = "SYSTEM_SETTINGS"; 

pub trait OverloadedConfigParams<T> {
    fn get_value_from_box(&self, boxed_value : &ConfigValueBox) -> T;
    fn make_box_from_value(&self, value: T) -> ConfigValueBox;
}

pub struct ConfigValueBox {
    string_field: Option<String>,
    bool_field: Option<bool>,
    float_field: Option<f64>,
    int_field: Option<i32>
}

pub struct GlobalParams {
    // Sofiya: Not sure that lock is necessary here
    // config only ever written by 1 thread at the beginning
    pub params: std::sync::RwLock<HashMap<String, ConfigValueBox>>,
}

lazy_static! {
    #[derive(Debug)]
    pub static ref GLOBAL_PARAMS: GlobalParams = GlobalParams {
        params: std::sync::RwLock::new(HashMap::new())
    };
}

impl GlobalParams {
    // Search code for GLOBAL_PARAMS.insert and GLOBAL_PARAMS.get for examples 
    pub fn get<T>(&self, module : String, param: String) -> T
    where Self: OverloadedConfigParams<T> {
        let key = format!("{}_{}", module, param);
        let unlocked_params = GLOBAL_PARAMS.params.read().unwrap();
        let boxed_value = unlocked_params.get(&key).unwrap();
        return self.get_value_from_box(boxed_value);
    }
    pub fn insert<T>(&self, key_module: String, key_param: String, value: T)
    where Self: OverloadedConfigParams<T> {
        let key = format!("{}_{}", key_module, key_param);
        let value = self.make_box_from_value(value);
        let mut unlocked_params = GLOBAL_PARAMS.params.write().unwrap();
        unlocked_params.insert(key, value);
    }
}

impl OverloadedConfigParams<String> for GlobalParams {
    fn get_value_from_box(&self, boxed_value : &ConfigValueBox) -> String {
        return boxed_value.string_field.as_ref().unwrap().to_string();
    }
    fn make_box_from_value(&self, value: String) -> ConfigValueBox {
        return ConfigValueBox {
            string_field: Some(value.clone()),
            bool_field: None,
            float_field: None,
            int_field: None
        };
    }
}

impl OverloadedConfigParams<bool> for GlobalParams {
    fn get_value_from_box(&self, boxed_value : &ConfigValueBox) -> bool {
        return *boxed_value.bool_field.as_ref().unwrap();
    }
    fn make_box_from_value(&self, value: bool) -> ConfigValueBox  {
        return ConfigValueBox {
            string_field: None,
            bool_field: Some(value),
            float_field: None,
            int_field: None
        };
    }
}

impl OverloadedConfigParams<f64> for GlobalParams {
    fn get_value_from_box(&self, boxed_value : &ConfigValueBox) -> f64 {
        return *boxed_value.float_field.as_ref().unwrap();
    }
    fn make_box_from_value(&self, value: f64) -> ConfigValueBox  {
        return ConfigValueBox {
            string_field: None,
            bool_field: None,
            float_field: Some(value),
            int_field: None
        };
    }
}

impl OverloadedConfigParams<i32> for GlobalParams {
    fn get_value_from_box(&self, boxed_value : &ConfigValueBox) -> i32 {
        return *boxed_value.int_field.as_ref().unwrap();
    }
    fn make_box_from_value(&self, value: i32) -> ConfigValueBox  {
        return ConfigValueBox {
            string_field: None,
            bool_field: None,
            float_field: None,
            int_field: Some(value)
        };
    }
}
