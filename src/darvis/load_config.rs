/// *** Functions to load actor info from the config file *** ///
use std::fs::File;
use std::collections::HashMap;
use std::io::Read;
extern crate yaml_rust;
use yaml_rust::yaml;
use yaml_rust::yaml::Yaml;
use crate::{
    global_params::*,
    base::ActorConf,
};

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