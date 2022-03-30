use std::collections::HashMap;
use std::env;
use axiom::prelude::*;
use glob::glob;
use log::LevelFilter;
use yaml_rust::yaml;

#[allow(unused_imports)]
use opencv::{
    prelude::*,
    core::*,
    features2d,
    features2d::{Feature2DTrait, ORB},
    highgui,
    imgproc,
    videoio,
    imgcodecs,
    types::{PtrOfORB, VectorOfKeyPoint},
};

use darvis::base;
use darvis::config::*;
use darvis::load_config::{load_config, read_config_file};

mod modules;
mod registered_modules;
use registered_modules::*;

fn load_custom_settings(config_file: &String) {
    let mut config_as_string = String::new();
    read_config_file(&mut config_as_string, &config_file);
    let yaml_document = &yaml::YamlLoader::load_from_str(&config_as_string).unwrap()[0];

    println!("SYSTEM SETTINGS");

    let system_settings = &yaml_document["system_settings"];

    let show_ui = system_settings["show_ui"].as_bool().unwrap();
    GLOBAL_PARAMS.insert(SYSTEM_SETTINGS.to_string(), "show_ui".to_string(), show_ui);
    println!("\t show_ui: {}", show_ui);

    let max_features: i32 = system_settings["max_features"].as_i64().unwrap() as i32;
    GLOBAL_PARAMS.insert(SYSTEM_SETTINGS.to_string(), "max_features".to_string(), max_features);
    println!("\t orb extractor: max_features: {}", max_features);

    let scale_factor: f64 = system_settings["scale_factor"].as_f64().unwrap();
    GLOBAL_PARAMS.insert(SYSTEM_SETTINGS.to_string(), "scale_factor".to_string(), scale_factor);
    println!("\t orb extractor: scale_factor: {}", scale_factor);

    let n_levels: i32 = system_settings["n_levels"].as_i64().unwrap() as i32;
    GLOBAL_PARAMS.insert(SYSTEM_SETTINGS.to_string(), "n_levels".to_string(), n_levels);
    println!("\t orb extractor: n_levels: {}", n_levels);

    let fast_threshold: i32 = system_settings["fast_threshold"].as_i64().unwrap() as i32;
    GLOBAL_PARAMS.insert(SYSTEM_SETTINGS.to_string(), "fast_threshold".to_string(), fast_threshold);
    println!("\t orb extractor: fast_threshold: {}", fast_threshold);
}

fn main() {
    env_logger::builder() 
        .filter_level(LevelFilter::Warn)
        .try_init()
        .unwrap();

    let args: Vec<String> = env::args().collect();

    if args.len() < 3 {
        println!("[ERROR] Invalid number of input parameters.");
        println!("Usage: cargo run -- [PATH_TO_DATA_DIRECTORY] [PATH_TO_CONFIG_FILE]");
        return;
    }

    let img_dir = args[1].to_owned();
    let config_file = args[2].to_owned();

    // Populate global config parameters
    // Get configuration for each actor (aka the "modules" of the system)
    let mut modules = Vec::<base::ActorConf>::new();
    load_config(&config_file, &mut modules);
    load_custom_settings(&config_file);

    let mut glob_str = img_dir.to_owned();
    glob_str.push_str("/*.png");

    let mut img_paths = Vec::new();

    for entry in glob(&glob_str).expect("Failed to read glob pattern") {
        match entry {
            Ok(path) => match path.to_str() {
                Some(path_str) => img_paths.push(path_str.to_owned()),
                None => println!("Invalid path found!"),
            },
            Err(e) => println!("{:?}", e),
        }
    }
    
    //TODO: Use Cluster manager to do remote agent calls

    let mut systems  = HashMap::<String, ActorSystem>::new();

    let mut aids = HashMap::new();

    // Note: was used in connect_with_channels call
    //let mut i =0;
    //let mut features = vec![];

    // Loop through the config to initialize actor system using the default config
    for actor_conf in modules {
        let actname = actor_conf.name.clone();
        

        let system_current = ActorSystem::create(ActorSystemConfig::default());
        

        systems.insert(actname.clone(), system_current.clone());

        let c_aid  = system_current.spawn().name(&actor_conf.name).with(FeatureManager::new(&actor_conf.actor_function,&actor_conf.actor_function), FeatureManager::handle).unwrap();
        
        aids.insert(actname.clone(), c_aid.clone());

        // TODO: Identify the role of using connect_with_channels, as the system communications are working without doing the following.
        //features.push(actname.clone());
        // if i > 0 { 
        //     ActorSystem::connect_with_channels(&systems.get(&features[0]).unwrap(), &system_current);            
        // }
        //i+=1;
    
    }

    //println!("{:?}",aids);

    let system = systems.get(FRAME_LOADER).unwrap();

    let feat_aid = system.find_aid_by_name(FRAME_LOADER).unwrap();

    // Kickoff the pipeline by sending the feature extraction module images
    feat_aid.send_new(modules::frameloader::ImagesMsg::new(img_paths, aids.clone())).unwrap();
   
    system.await_shutdown(None);
}
