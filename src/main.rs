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
mod vis;


fn main() {

    env_logger::builder() 
        .filter_level(LevelFilter::Warn)
        .try_init()
        .unwrap();

    let args: Vec<String> = env::args().collect();
    let img_dir = args[1].to_owned();
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

    // First we initialize the actor system using the default config
    let config = ActorSystemConfig::default();
    let system = ActorSystem::create(config);

    // Next we spawn each actor
    let feat_aid = system.spawn().name("feature_extraction").with((), orb::orb_extract).unwrap();
    let align_aid = system.spawn().name("alignment").with((), align::align).unwrap();
    let vis_aid = system.spawn().name("visulization").with(vis::Vis::new(), vis::Vis::visualize).unwrap();

    // Save spawned actor ID's for lookup later
    let mut aids = HashMap::new();
    aids.insert("feat".to_string(), feat_aid.clone());
    aids.insert("align".to_string(), align_aid.clone());
    aids.insert("vis".to_string(), vis_aid.clone());

    // Kickoff the pipeline by sending the feature extraction module images
    feat_aid.send_new(orb::OrbMsg::new(img_paths, aids.clone())).unwrap();
   
    system.await_shutdown(None);
}
