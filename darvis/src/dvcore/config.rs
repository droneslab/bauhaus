/// *** Structs to help get/set global configuration parameters that are read from the config file. *** //
/// 
/// This section is a little complicated and the details don't matter too much if you're just using it.
/// Basically, this implementation of Settings allows inserting a setting of 4 different types
/// (string, bool, f64, and i32) without needing to call a specific function for each type.
/// 
/// To insert a new parameter into SETTINGS:
///     SETTINGS.insert(SYSTEM, "show_visualizer", show_visualizer);
/// To get a parameter from SETTINGS:
///     let max_features= SETTINGS.get::<i32>(SYSTEM, "max_features".to_string());
/// 
use std::{collections::HashMap, sync::RwLock};
use lazy_static::*;
use linked_hash_map::LinkedHashMap;

use std::fs::File;
use std::io::Read;
extern crate yaml_rust;
use yaml_rust::yaml;
use yaml_rust::yaml::Yaml;

use crate::sensor::{Sensor, FrameSensor, ImuSensor};

pub static SYSTEM: &str = "SYSTEM"; 

//* GLOBAL SETTING PARAMETERS FOR LOOKUP */
pub struct Settings {
    // Lock is necessary because Settings is a static variable
    // https://stackoverflow.com/questions/34832583/global-mutable-hashmap-in-a-library
    settings: RwLock<HashMap<String, SettingBox>>,
}

lazy_static! {
    #[derive(Debug)]
    pub static ref SETTINGS: Settings = Settings {
        settings: RwLock::new(HashMap::new())
    };
}

impl Settings {
    pub fn get<T>(&self, module : &str, param: &str) -> T
    where Self: OverloadedSetting<T> {
        let key = format!("{}_{}", module, param);
        let unlocked_params = SETTINGS.settings.read().unwrap();
        let boxed_value = unlocked_params.get(&key).unwrap();
        return self.get_value_from_box(boxed_value);
    }
    pub fn insert<T: std::fmt::Display>(&self, namespace: &str, key_param: &str, value: T)
    where Self: OverloadedSetting<T> {
        let key = format!("{}_{}", namespace, key_param);
        let value = self.make_box_from_value(value);
        let mut unlocked_params = SETTINGS.settings.write().unwrap();
        unlocked_params.insert(key, value);
    }
}

impl OverloadedSetting<String> for Settings {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> String {
        return boxed_value.string_field.as_ref().unwrap().to_string();
    }
    fn make_box_from_value(&self, value: String) -> SettingBox {
        return SettingBox {
            string_field: Some(value.clone()),
            bool_field: None,
            float_field: None,
            int_field: None,
            sensor_field: None
        };
    }
}

impl OverloadedSetting<bool> for Settings {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> bool {
        return *boxed_value.bool_field.as_ref().unwrap();
    }
    fn make_box_from_value(&self, value: bool) -> SettingBox  {
        return SettingBox {
            string_field: None,
            bool_field: Some(value),
            float_field: None,
            int_field: None,
            sensor_field: None
        };
    }
}

impl OverloadedSetting<f64> for Settings {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> f64 {
        return *boxed_value.float_field.as_ref().unwrap();
    }
    fn make_box_from_value(&self, value: f64) -> SettingBox  {
        return SettingBox {
            string_field: None,
            bool_field: None,
            float_field: Some(value),
            int_field: None,
            sensor_field: None
        };
    }
}

impl OverloadedSetting<i32> for Settings {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> i32 {
        return *boxed_value.int_field.as_ref().unwrap();
    }
    fn make_box_from_value(&self, value: i32) -> SettingBox  {
        return SettingBox {
            string_field: None,
            bool_field: None,
            float_field: None,
            int_field: Some(value),
            sensor_field: None
        };
    }
}

impl OverloadedSetting<Sensor> for Settings {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> Sensor {
        return *boxed_value.sensor_field.as_ref().unwrap();
    }
    fn make_box_from_value(&self, value: Sensor) -> SettingBox  {
        return SettingBox {
            string_field: None,
            bool_field: None,
            float_field: None,
            int_field: None,
            sensor_field: Some(value)
        };
    }
}

pub trait OverloadedSetting<T> {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> T;
    fn make_box_from_value(&self, value: T) -> SettingBox;
}

pub struct SettingBox {
    string_field: Option<String>,
    bool_field: Option<bool>,
    float_field: Option<f64>,
    int_field: Option<i32>,
    sensor_field: Option<Sensor>
}




// * LOADING CONFIGURATION FROM FILE *//

#[derive(Debug, Default, Clone)]
// Struct holding configuration parameters for a given actor
pub struct ActorConf{
    pub name: String,
    pub file: String,
    pub receiver_bound: Option<usize>,
}

#[derive(Debug, Default, Clone)]
// Struct holding configuration parameters for a given module
pub struct ModuleConf{
    pub name: String
}

pub fn load_config(file_name: &String) -> Result<(Vec<ActorConf>, Vec<ModuleConf>, String), Box<dyn std::error::Error>> {
    let mut config_string = String::new();
    println!("{}",file_name);
    let mut f = File::open(file_name).unwrap();
    f.read_to_string(&mut config_string).unwrap();
    let yaml_document = &yaml::YamlLoader::load_from_str(&config_string).unwrap()[0];

    println!("SYSTEM SETTINGS");

    // Load additional custom settings from config file
    let system_settings = &yaml::YamlLoader::load_from_str(&config_string).unwrap()[0]["system"];
    add_setting_bool(SYSTEM, "localization_only_mode", &system_settings["localization_only_mode"]);
    add_setting_bool(SYSTEM, "create_flamegraph", &system_settings["create_flamegraph"]);
    add_setting_string(SYSTEM, "vocabulary_file", &system_settings["vocabulary_file"]);
    add_setting_string(SYSTEM, "trajectory_file_name", &system_settings["trajectory_file_name"]);
    add_setting_string(SYSTEM, "results_folder", &system_settings["results_folder"]);
    add_setting_bool(SYSTEM, "use_timestamps_file", &system_settings["use_timestamps_file"]);
    add_setting_f64(SYSTEM, "fps", &system_settings["fps"]);
    add_setting_bool(SYSTEM, "show_visualizer", &system_settings["show_visualizer"]);
    let log_level = system_settings["log_level"].as_str().unwrap().to_owned();

    // Load sensor settings
    let framesensor = match system_settings["framesensor"].as_str().unwrap() {
        "Mono" => FrameSensor::Mono,
        "Stereo" => FrameSensor::Stereo,
        "RGBD" => FrameSensor::Rgbd,
        _ => panic!("Incompatible frame sensor type"),
    };
    let imusensor = match system_settings["imusensor"].as_str().unwrap() {
        "Some" => ImuSensor::Some,
        "None" => ImuSensor::None,
        _ => panic!("Incompatible IMU sensor type")
    };
    let sensor = Sensor(framesensor, imusensor);
    SETTINGS.insert(SYSTEM, "sensor", sensor);
    println!("\t {} = {}", "SENSOR", sensor);

    // Load actors
    let mut actor_info = Vec::<ActorConf>::new();
    for actor in yaml_document["actors"].as_vec().unwrap() {
        let h = &actor.as_hash().unwrap();

        let a_conf = ActorConf {
            name: get_val(h, "name").as_str().unwrap().to_string(),
            file: get_val(h, "file").as_str().unwrap().to_string(),
            receiver_bound: match get_val(h, "receiver_bound").as_i64().unwrap() == -1 {
                true => None,
                false => Some(get_val(h, "receiver_bound").as_i64().unwrap() as usize)
            }
        };

        actor_info.push(a_conf.clone());

        add_settings(get_val(h, "settings").as_vec().unwrap(), &a_conf.name);

        SETTINGS.insert(&a_conf.name, "file", a_conf.file);
    }

    // Load modules
    let mut module_info = Vec::<ModuleConf>::new();
    for module in yaml_document["modules"].as_vec().unwrap() {
        let h = module.as_hash().unwrap();
        let m_conf = ModuleConf {
            name: get_val(h, "name").as_str().unwrap().to_string()
        };
        add_settings(get_val(h, "settings").as_vec().unwrap(), &m_conf.name);
        module_info.push(m_conf);
    }

    Ok((actor_info, module_info, log_level))
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
    SETTINGS.insert(&namespace, &key, val);
    println!("\t {} {} = {}", namespace, key, val);
}
fn add_setting_i32(namespace: &str, key: &str, value: &Yaml) {
    let val = value.as_i64().unwrap() as i32;
    SETTINGS.insert(&namespace, &key, val);
    println!("\t {} {} = {}", namespace, key, val);
}
fn add_setting_f64(namespace: &str, key: &str, value: &Yaml) {
    let val = value.as_f64().unwrap();
    SETTINGS.insert(&namespace, &key, val);
    println!("\t {} {} = {}", namespace, key, val);
}
fn add_setting_string(namespace: &str, key: &str, value: &Yaml) {
    let val = value.as_str().unwrap().to_string();
    SETTINGS.insert(&namespace, &key, val.clone());
    println!("\t {} {} = {}", namespace, key, val);
}

fn get_val<'a>(hashmap: &'a LinkedHashMap<Yaml, Yaml>, string: &str) -> &'a Yaml {
    &hashmap[&Yaml::String(string.to_string())]
}

