use std::env;
use glob::glob;
use axiom::prelude::*;
use std::collections::HashMap;
use log::LevelFilter;
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

mod base;
mod orb;
mod tracker;
mod dvutils;
mod load_config;
mod vis;
mod pluginfunction;
mod registerplugin;
mod opflow;
mod frameloader;
mod config;
mod fast;
mod tracker_klt;

use config::*;


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
    load_config::load_config(&config_file, &mut modules);

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

        let c_aid  = system_current.spawn().name(&actor_conf.name).with(registerplugin::FeatureManager::new(&actor_conf.actor_function,&actor_conf.actor_function), registerplugin::FeatureManager::handle).unwrap();
        
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
    feat_aid.send_new(frameloader::ImagesMsg::new(img_paths, aids.clone())).unwrap();
   
    system.await_shutdown(None);
}
