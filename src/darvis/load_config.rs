use std::fs::File;
use std::collections::HashMap;
use std::io::Read;
extern crate yaml_rust;
use yaml_rust::yaml;
use yaml_rust::yaml::Yaml;
use crate::{
    config::*,
    base::ActorConf,
};

/// *** Functions to load actor info from the config file *** ///
pub fn load_config(config_file: &String, all_modules: &mut Vec<ActorConf>) {
    let mut config_as_string = String::new();
    read_config_file(&mut config_as_string, &config_file);
    let yaml_document = &yaml::YamlLoader::load_from_str(&config_as_string).unwrap()[0];
    load_module_info(yaml_document, all_modules);
}

pub fn read_config_file(str: &mut String, file_name: &String) {
    let mut f = File::open(file_name).unwrap();
    f.read_to_string(str).unwrap();
 }

fn load_module_info(doc: &Yaml, all_modules: &mut Vec<ActorConf>) {
    let v = doc["modules"].as_vec().unwrap();

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

        GLOBAL_PARAMS.insert(actname.clone(), "file".to_string(), mconf.file);
        GLOBAL_PARAMS.insert(actname.clone(), "actor_message".to_string(), mconf.actor_message);
        GLOBAL_PARAMS.insert(actname.clone(), "actor_function".to_string(), mconf.actor_function);
    }
}
