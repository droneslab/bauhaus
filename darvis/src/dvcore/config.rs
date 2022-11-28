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
use std::{collections::HashMap, sync::RwLock, fmt};
use lazy_static::*;
use linked_hash_map::LinkedHashMap;
use log::info;

use std::fs::File;
use std::io::Read;
extern crate yaml_rust;
use yaml_rust::yaml;
use yaml_rust::yaml::Yaml;
use crate::{
    base::{ActorConf, ModuleConf},
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
    pub fn insert<T: std::fmt::Display>(&self, namespace: &str, key_param: &str, value: T)
    where Self: OverloadedConfigParams<T> {
        let key = format!("{}_{}", namespace, key_param);
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





pub fn load_config(file_name: &String) -> Option<(Vec<ActorConf>, Vec<ModuleConf>)> {
    let mut config_string = String::new();
    let mut f = File::open(file_name).unwrap();
    f.read_to_string(&mut config_string).unwrap();
    let yaml_document = &yaml::YamlLoader::load_from_str(&config_string).unwrap()[0];

    println!("SYSTEM SETTINGS");

    // Load additional custom settings from config file
    let system_settings = &yaml::YamlLoader::load_from_str(&config_string).unwrap()[0]["system_settings"];
    add_setting_bool(SYSTEM_SETTINGS, "show_ui", &system_settings["show_ui"]);
    add_setting_bool(SYSTEM_SETTINGS, "localization_only_mode", &system_settings["localization_only_mode"]);
    add_setting_bool(SYSTEM_SETTINGS, "should_profile", &system_settings["should_profile"]);
    add_setting_string(SYSTEM_SETTINGS, "vocabulary_file", &system_settings["vocabulary_file"]);
    add_setting_string(SYSTEM_SETTINGS, "trajectory_file_name", &system_settings["trajectory_file_name"]);
    add_setting_f64(SYSTEM_SETTINGS, "fps", &system_settings["fps"]);

    // Load sensor settings
    let framesensor = match system_settings["framesensor"].as_str()? {
        "Mono" => FrameSensor::Mono,
        "Stereo" => FrameSensor::Stereo,
        "RGBD" => FrameSensor::Rgbd,
        _ => panic!("Incompatible frame sensor type"),
    };
    let imusensor = match system_settings["imusensor"].as_str()? {
        "Some" => ImuSensor::Some,
        "None" => ImuSensor::None,
        _ => panic!("Incompatible IMU sensor type")
    };
    let sensor = Sensor(framesensor, imusensor);
    GLOBAL_PARAMS.insert(SYSTEM_SETTINGS, "sensor", sensor);
    println!("\t {} = {}", "SENSOR", sensor);

    // Load actors
    let mut actor_info = Vec::<ActorConf>::new();
    for actor in yaml_document["actors"].as_vec()? {
        let h = &actor.as_hash()?;

        let paths = get_val(h, "possible_paths").as_vec()?;
        let mut hmap = HashMap::<String, String>::new();
        for p in 0..paths.len() {
            let path = paths[p].as_hash()?;
            hmap.insert(
                get_val(path, "from").as_str()?.to_string(),
                get_val(path, "to").as_str()?.to_string(),
            );
        }

        let a_conf = ActorConf {
            name: get_val(h, "name").as_str()?.to_string(),
            file: get_val(h, "file").as_str()?.to_string(),
            actor_message: get_val(h, "actor_message").as_str()?.to_string(),
            actor_function: get_val(h, "actor_function").as_str()?.to_string(),
            ip_address: get_val(h, "address").as_str()?.to_string(),
            port: get_val(h, "port").as_str()?.to_string(),
            multithreaded: get_val(h, "multithreaded").as_bool()?,
            threads: get_val(h, "threads").as_i64()?,
            possible_paths: hmap
        };

        actor_info.push(a_conf.clone());

        add_settings(get_val(h, "settings").as_vec()?, &a_conf.name);

        GLOBAL_PARAMS.insert(&a_conf.name, "file", a_conf.file);
        GLOBAL_PARAMS.insert(&a_conf.name, "actor_message", a_conf.actor_message);
        GLOBAL_PARAMS.insert(&a_conf.name, "actor_function", a_conf.actor_function);
    }

    // Load modules
    let mut module_info = Vec::<ModuleConf>::new();
    for module in yaml_document["modules"].as_vec()? {
        let h = module.as_hash()?;
        let m_conf = ModuleConf {
            name: get_val(h, "name").as_str()?.to_string()
        };
        add_settings(get_val(h, "settings").as_vec()?, &m_conf.name);
        module_info.push(m_conf);
    }

    Some((actor_info, module_info))
}


fn add_settings(settings: &Vec<Yaml>, namespace: &String) -> Option<()> {
    for s in 0..settings.len() {
        let setting = settings[s].as_hash()?;
        let s_name = setting[&Yaml::String("name".to_string())].as_str()?.to_string();
        let s_value = &setting[&Yaml::String("value".to_string())];
        let s_type = setting[&Yaml::String("type".to_string())].as_str()?.to_string();

        match s_type.as_ref() {
            "bool" => add_setting_bool(namespace, &s_name, s_value),
            "i32" => add_setting_i32(namespace, &s_name, s_value),
            "f64" => add_setting_f64(namespace, &s_name, s_value),
            "string" => add_setting_string(namespace, &s_name, s_value),
            _ => panic!("Incompatible type {} for setting {} in {}", s_type, s_value.as_str()?.to_string(), s_name)
        };
    }
    Some(())
}

fn add_setting_bool(namespace: &str, key: &str, value: &Yaml) {
    let val = value.as_bool().unwrap();
    GLOBAL_PARAMS.insert(&namespace, &key, val);
    println!("\t {} {} = {}", namespace, key, val);
}
fn add_setting_i32(namespace: &str, key: &str, value: &Yaml) {
    let val = value.as_i64().unwrap() as i32;
    GLOBAL_PARAMS.insert(&namespace, &key, val);
    println!("\t {} {} = {}", namespace, key, val);
}
fn add_setting_f64(namespace: &str, key: &str, value: &Yaml) {
    let val = value.as_f64().unwrap();
    GLOBAL_PARAMS.insert(&namespace, &key, val);
    println!("\t {} {} = {}", namespace, key, val);
}
fn add_setting_string(namespace: &str, key: &str, value: &Yaml) {
    let val = value.as_str().unwrap().to_string();
    GLOBAL_PARAMS.insert(&namespace, &key, val.clone());
    println!("\t {} {} = {}", namespace, key, val);
}

fn get_val<'a>(hashmap: &'a LinkedHashMap<Yaml, Yaml>, string: &str) -> &'a Yaml {
    &hashmap[&Yaml::String(string.to_string())]
}


// Note: sensor has to be in dvcore because ConfigValueBox needs to hold a sensor
#[derive(Clone, Copy, Debug, Default)]
pub enum FrameSensor {
    #[default] Mono,
    Stereo,
    Rgbd,
}

#[derive(Clone, Copy, Debug, Default)]
pub enum ImuSensor {
    #[default] None,
    Some
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Sensor (pub FrameSensor, pub ImuSensor);

impl Sensor {
    pub fn is_mono(&self) -> bool {
        matches!(self.0, FrameSensor::Mono)
    }
    pub fn is_imu(&self) -> bool {
        matches!(self.1, ImuSensor::Some)
    }
    pub fn frame(&self) -> FrameSensor {
        self.0
    }
    pub fn imu(&self) -> ImuSensor {
        self.1
    }
}

impl fmt::Display for Sensor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let framesensor_str = match self.0 {
            FrameSensor::Mono => "Mono",
            FrameSensor::Stereo => "Stereo",
            FrameSensor::Rgbd => "Rgbd",
        };
        let imusensor_str = match self.1 {
            ImuSensor::None => "No",
            ImuSensor::Some => "Yes",
        };

        write!(f, "(Frame: {}, Imu: {})", framesensor_str, imusensor_str)
    }
}
