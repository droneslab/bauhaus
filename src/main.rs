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
mod align;
mod utils;
mod config;
mod vis;
mod sift;

fn main() {

    env_logger::builder() 
        .filter_level(LevelFilter::Warn)
        .try_init()
        .unwrap();

    let args: Vec<String> = env::args().collect();

    //TODO: Check input arguments/error if not enough
    let img_dir = args[1].to_owned();
    let config_file = args[2].to_owned();

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
    
    // Load the config file 
    let mut conf_str = String::new();
    config::read_config_file(&mut conf_str, &config_file);

    // Get configuration for each actor (aka the "modules" of the system)
    let mut modules = Vec::<base::ActorConf>::new();
    config::load_config(&mut conf_str, &mut modules);

    // First we initialize the actor system using the default config
    let config = ActorSystemConfig::default();
    let system = ActorSystem::create(config);

    // TODO: Spawn all actors based on config, need a table to map string names to functions
//     for actor_conf in modules {
//         // TODO: Need to add message/function tables (hashmaps) somewhere that map sttrings to Fn pointers
//         // https://stackoverflow.com/questions/30540807/calling-a-function-only-known-at-runtime
//         println!("{:?}", actor_conf);
//         // let test = actor_conf.file as module;
//         // let new_aid = system.spawn().name(actor_conf.name).with((), actor_conf.file::actor_conf.actor_function).unwrap();
//     }

    // Next we spawn each actor
    let feat_aid = system.spawn().name("feature_extraction").with((), sift::sift_extract).unwrap();
    let align_aid = system.spawn().name("alignment").with((), align::align).unwrap();
    let vis_aid = system.spawn().name("visulization").with(vis::Vis::new(), vis::Vis::visualize).unwrap();

    // Save spawned actor ID's for lookup later
    let mut aids = HashMap::new();
    aids.insert("feat".to_string(), feat_aid.clone());
    aids.insert("align".to_string(), align_aid.clone());
    aids.insert("vis".to_string(), vis_aid.clone());

    // Kickoff the pipeline by sending the feature extraction module images
    feat_aid.send_new(sift::SiftMsg::new(img_paths, aids.clone())).unwrap();
   
    system.await_shutdown(None);
}
