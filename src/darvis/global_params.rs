/// *** Structs to help get/set global configuration parameters that are read from the config file. *** //
/// 
/// This section is a little complicated and the details don't matter too much if you're just using it.
/// Basically, this implementation of GlobalParams allows inserting a parameter of 4 different types
/// (string, bool, f64, and i32) without needing to call a specific function for each type.
/// 
/// To insert a new parameter into GLOBAL_PARAMS:
///     GLOBAL_PARAMS.insert(SYSTEM_SETTINGS, "show_ui", show_ui);
/// To get a parameter from GLOBAL_PARAMS:
///     let max_features: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "max_features".to_string());
/// 
use std::collections::HashMap;
use lazy_static::*;
use parking_lot::RwLock;
use crate::utils::sensor::Sensor;

pub static SYSTEM_SETTINGS: &str = "SYSTEM_SETTINGS"; 

pub struct GlobalParams {
    // Lock is necessary because GLOBAL_PARAMS is a static variable
    // https://stackoverflow.com/questions/34832583/global-mutable-hashmap-in-a-library
    pub params: RwLock<HashMap<String, ConfigValueBox>>,
}

lazy_static! {
    #[derive(Debug)]
    pub static ref GLOBAL_PARAMS: GlobalParams = GlobalParams {
        params: RwLock::new(HashMap::new())
    };
}

impl GlobalParams {
    pub fn get<T>(&self, module : &str, param: &str) -> T
    where Self: OverloadedConfigParams<T> {
        let key = format!("{}_{}", module, param);
        let unlocked_params = GLOBAL_PARAMS.params.read();
        let boxed_value = unlocked_params.get(&key).unwrap();
        return self.get_value_from_box(boxed_value);
    }
    pub fn insert<T>(&self, key_module: &str, key_param: &str, value: T)
    where Self: OverloadedConfigParams<T> {
        let key = format!("{}_{}", key_module, key_param);
        let value = self.make_box_from_value(value);
        let mut unlocked_params = GLOBAL_PARAMS.params.write();
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
            int_field: None,
            sensor_field: None
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
            int_field: None,
            sensor_field: None
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
            int_field: None,
            sensor_field: None
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
            int_field: Some(value),
            sensor_field: None
        };
    }
}

impl OverloadedConfigParams<Sensor> for GlobalParams {
    fn get_value_from_box(&self, boxed_value : &ConfigValueBox) -> Sensor {
        return *boxed_value.sensor_field.as_ref().unwrap();
    }
    fn make_box_from_value(&self, value: Sensor) -> ConfigValueBox  {
        return ConfigValueBox {
            string_field: None,
            bool_field: None,
            float_field: None,
            int_field: None,
            sensor_field: Some(value)
        };
    }
}

pub trait OverloadedConfigParams<T> {
    fn get_value_from_box(&self, boxed_value : &ConfigValueBox) -> T;
    fn make_box_from_value(&self, value: T) -> ConfigValueBox;
}

pub struct ConfigValueBox {
    string_field: Option<String>,
    bool_field: Option<bool>,
    float_field: Option<f64>,
    int_field: Option<i32>,
    sensor_field: Option<Sensor>
}
