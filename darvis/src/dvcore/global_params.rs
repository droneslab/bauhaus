/// *** Structs to help get/set global configuration parameters that are read from the config file. *** //
/// 
/// This section is a little complicated and the details don't matter too much if you're just using it.
/// Basically, this implementation of GlobalParams allows inserting a parameter of 4 different types
/// (string, bool, f64, and i32) without needing to call a specific function for each type.
/// 
/// To insert a new parameter into GLOBAL_PARAMS:
///     GLOBAL_PARAMS.insert(SYSTEM_SETTINGS, "show_ui", show_ui);
/// To get a parameter from GLOBAL_PARAMS:
///     let max_features= GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "max_features".to_string());
/// 
use std::{collections::HashMap, any::Any, sync::RwLock};
use once_cell::sync::OnceCell;
use lazy_static::*;

use std::fs::File;
use std::io::Read;
extern crate yaml_rust;
use yaml_rust::yaml;
use yaml_rust::yaml::Yaml;
use crate::{
    base::ActorConf,
};

pub static SYSTEM_SETTINGS: &str = "SYSTEM_SETTINGS"; 

pub struct GlobalParams {
    // Lock is necessary because GLOBAL_PARAMS is a static variable
    // https://stackoverflow.com/questions/34832583/global-mutable-hashmap-in-a-library
    params: RwLock<HashMap<String, ConfigValueBox>>,
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
        let unlocked_params = GLOBAL_PARAMS.params.read().unwrap();
        let boxed_value = unlocked_params.get(&key).unwrap();
        return self.get_value_from_box(boxed_value);
    }
    pub fn insert<T>(&self, key_module: &str, key_param: &str, value: T)
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



pub fn read_config_file(file_name: &String) -> String {
    let mut config_as_string = String::new();
    let mut f = File::open(file_name).unwrap();
    f.read_to_string(&mut config_as_string).unwrap();
    config_as_string
}

pub fn load_modules_from_config(config_string: &String, all_modules: &mut Vec<ActorConf>) {
    let yaml_document = &yaml::YamlLoader::load_from_str(config_string).unwrap()[0];
    let v = yaml_document["modules"].as_vec().unwrap();

    for i in 0..v.len() {
        let h = &v[i].as_hash().unwrap();
        let mut mconf = ActorConf::default();
        mconf.name = h[&Yaml::String("name".to_string())].as_str().unwrap().to_string();
        mconf.file = h[&Yaml::String("file".to_string())].as_str().unwrap().to_string();
        mconf.actor_message = h[&Yaml::String("actor_message".to_string())].as_str().unwrap().to_string();
        mconf.actor_function = h[&Yaml::String("actor_function".to_string())].as_str().unwrap().to_string();
        mconf.ip_address = h[&Yaml::String("address".to_string())].as_str().unwrap().to_string();
        mconf.port = h[&Yaml::String("port".to_string())].as_str().unwrap().to_string();
        mconf.multithreaded = h[&Yaml::String("multithreaded".to_string())].as_bool().unwrap();
        mconf.threads = h[&Yaml::String("threads".to_string())].as_i64().unwrap();
        let paths = &h[&Yaml::String("possible_paths".to_string())].as_vec().unwrap();
        let mut hmap = HashMap::<String, String>::new();
        for p in 0..paths.len() {
            let path = paths[p].as_hash().unwrap();
            hmap.insert(path[&Yaml::String("from".to_string())].as_str().unwrap().to_string()
                                        , path[&Yaml::String("to".to_string())].as_str().unwrap().to_string());
        }
        mconf.possible_paths = hmap;
        all_modules.push(mconf.clone());

        let actname = mconf.name.clone();

        GLOBAL_PARAMS.insert(&actname, "file", mconf.file);
        GLOBAL_PARAMS.insert(&actname, "actor_message", mconf.actor_message);
        GLOBAL_PARAMS.insert(&actname, "actor_function", mconf.actor_function);
    }
}

pub fn add_system_setting_bool(yaml: &Yaml, param_name: &str) {
    let value = yaml[param_name].as_bool().unwrap();
    GLOBAL_PARAMS.insert(SYSTEM_SETTINGS, param_name, value);
    println!("\t {}: {}", param_name, value);
}

pub fn add_system_setting_i32(yaml: &Yaml, param_name: &str) {
    let value: i32 = yaml[param_name].as_i64().unwrap() as i32;
    GLOBAL_PARAMS.insert(SYSTEM_SETTINGS, param_name, value);
    println!("\t {}: {}", param_name, value);
}

pub fn add_system_setting_f64(yaml: &Yaml, param_name: &str) {
    let value: f64 = yaml[param_name].as_f64().unwrap();
    GLOBAL_PARAMS.insert(SYSTEM_SETTINGS, param_name, value);
    println!("\t {}: {}", param_name, value);
}

pub fn add_system_setting_string(yaml: &Yaml, param_name: &str) {
    let value: &str = yaml[param_name].as_str().unwrap();
    GLOBAL_PARAMS.insert(SYSTEM_SETTINGS, param_name, value.to_string());
    println!("\t {}: {}", param_name, value);
}

pub fn add_system_setting_sensor(yaml: &Yaml) {
    let param_name = "sensor";
    let value_str = yaml["sensor"].as_str().unwrap();
    let value = match value_str {
        "Mono" => Sensor::Mono,
        "IMU_Mono" => Sensor::ImuMono,
        "Stereo" => Sensor::Stereo,
        "IMU_STEREO" => Sensor::ImuStereo,
        "RGBD" => Sensor::Rgbd,
        "IMU_RGBD" => Sensor::ImuRgbd,
        _ => panic!("Incompatible sensor type"),
    };
    GLOBAL_PARAMS.insert(SYSTEM_SETTINGS, param_name, value);
    println!("\t {}: {}", param_name, value_str);
}





// Note: sensor has to be in here because ConfigValueBox needs to hold a sensor
// Tried working with the commented code below which could be close but ran into errors. 
// Don't think it's worth it to optimize this rn.
#[derive(Clone, Copy, Debug)]
pub enum Sensor {
    Mono,
    ImuMono,
    Stereo,
    ImuStereo,
    Rgbd,
    ImuRgbd
}

impl Sensor {
    pub fn is_imu(&self) -> bool {
        match *self {
            Sensor::ImuMono | Sensor::ImuStereo | Sensor::ImuRgbd => true,
            _ => false
        }
    }

    pub fn is_mono(&self) -> bool {
        match *self {
            Sensor::ImuMono | Sensor::Mono => true,
            _ => false
        }
    }
}
// pub static GLOBAL_PARAMS: OnceCell<GlobalParams> = OnceCell::INIT;

// pub struct GlobalParams {
//     pub params: HashMap<String, Box<dyn Any + Send + Sync + 'static>>,
// }

// impl GlobalParams {
//     pub fn global() -> &'static GlobalParams {
//         GLOBAL_PARAMS.get().expect("logger is not initialized")
//     }

//     pub fn get(&self, module : &str, param: &str) -> &Box<dyn Any + Send + Sync> {
//         let key = format!("{}_{}", module, param);
//         self.params.get(&key).unwrap()
//     }
//     pub fn insert<T: Send + Sync + 'static>(&mut self, key_module: &str, key_param: &str, value: T) {
//         let key = format!("{}_{}", key_module, key_param);
//         let value = Box::new(value);
//         self.params.insert(key, value);
//     }
// }
