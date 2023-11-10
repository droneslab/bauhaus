/// *** Structs to help get/set global configuration parameters that are read from the config file. *** //
/// 
/// This section is a little complicated and the details don't matter too much if you're just using it.
/// Basically, this implementation of GlobalParams allows inserting a parameter of 4 different types
/// (string, bool, f64, and i32) without needing to call a specific function for each type.
/// 
/// To insert a new parameter into GLOBAL_PARAMS:
///     GLOBAL_PARAMS.insert(SYSTEM_SETTINGS, "show_visualizer", show_visualizer);
/// To get a parameter from GLOBAL_PARAMS:
///     let max_features= GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "max_features".to_string());
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

pub static SYSTEM_SETTINGS: &str = "SYSTEM_SETTINGS"; 

//* GLOBAL CONFIG PARAMETERS FOR LOOKUP */
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




// * LOADING CONFIGURATION FROM FILE *//

#[derive(Debug, Default, Clone)]
// Struct holding configuration parameters for a given actor
pub struct ActorConf{
    pub name: String,
    pub file: String,
    // Other stuff that used to be in the config file that we are not using right now
    // pub actor_message: String,
    // pub actor_function: String,
    // pub ip_address: String,
    // pub port: String,
    // pub multithreaded: bool,
    // pub threads: i64,
    // pub possible_paths: HashMap<String, String>
}

#[derive(Debug, Default, Clone)]
// Struct holding configuration parameters for a given module
pub struct ModuleConf{
    pub name: String
}

pub fn load_config(file_name: &String) -> Result<(Vec<ActorConf>, Vec<ModuleConf>), Box<dyn std::error::Error>> {
    let mut config_string = String::new();
    println!("{}",file_name);
    let mut f = File::open(file_name).unwrap();
    f.read_to_string(&mut config_string).unwrap();
    let yaml_document = &yaml::YamlLoader::load_from_str(&config_string).unwrap()[0];

    println!("SYSTEM SETTINGS");

    // Load additional custom settings from config file
    let system_settings = &yaml::YamlLoader::load_from_str(&config_string).unwrap()[0]["system_settings"];
    add_setting_bool(SYSTEM_SETTINGS, "localization_only_mode", &system_settings["localization_only_mode"]);
    add_setting_bool(SYSTEM_SETTINGS, "should_profile", &system_settings["should_profile"]);
    add_setting_string(SYSTEM_SETTINGS, "vocabulary_file", &system_settings["vocabulary_file"]);
    add_setting_string(SYSTEM_SETTINGS, "trajectory_file_name", &system_settings["trajectory_file_name"]);
    add_setting_f64(SYSTEM_SETTINGS, "fps", &system_settings["fps"]);

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
    GLOBAL_PARAMS.insert(SYSTEM_SETTINGS, "sensor", sensor);
    println!("\t {} = {}", "SENSOR", sensor);

    // Load actors
    let mut actor_info = Vec::<ActorConf>::new();
    for actor in yaml_document["actors"].as_vec().unwrap() {
        let h = &actor.as_hash().unwrap();

        // Other stuff that used to be in the config file that we aren't using right now.
        // let paths = get_val(h, "possible_paths").as_vec().unwrap();
        // let mut hmap = HashMap::<String, String>::new();
        // for p in 0..paths.len() {
        //     let path = paths[p].as_hash().unwrap();
        //     hmap.insert(
        //         get_val(path, "from").as_str().unwrap().to_string(),
        //         get_val(path, "to").as_str().unwrap().to_string(),
        //     );
        // }

        let a_conf = ActorConf {
            name: get_val(h, "name").as_str().unwrap().to_string(),
            file: get_val(h, "file").as_str().unwrap().to_string(),
            // Other stuff that used to be in the config file that we are not using right now
            // actor_message: get_val(h, "actor_message").as_str().unwrap().to_string(),
            // actor_function: get_val(h, "actor_function").as_str().unwrap().to_string(),
            // ip_address: get_val(h, "address").as_str().unwrap().to_string(),
            // port: get_val(h, "port").as_str().unwrap().to_string(),
            // multithreaded: get_val(h, "multithreaded").as_bool().unwrap(),
            // threads: get_val(h, "threads").as_i64().unwrap(),
            // possible_paths: hmap
        };

        actor_info.push(a_conf.clone());

        add_settings(get_val(h, "settings").as_vec().unwrap(), &a_conf.name);

        GLOBAL_PARAMS.insert(&a_conf.name, "file", a_conf.file);
        // Other stuff that used to be in the config file that we are not using right now
        // GLOBAL_PARAMS.insert(&a_conf.name, "actor_message", a_conf.actor_message);
        // GLOBAL_PARAMS.insert(&a_conf.name, "actor_function", a_conf.actor_function);
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

    Ok((actor_info, module_info))
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

