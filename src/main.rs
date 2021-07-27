use std::env;
use glob::glob;
use axiom::prelude::*;
use axiom::cluster::*;

use std::collections::HashMap;
use std::net::{SocketAddr};
use std::time::Duration;
use log::LevelFilter;

mod base;
mod orb;
mod align;
mod utils;
mod vis;

fn main() {

    env_logger::builder() 
        .filter_level(LevelFilter::Info)
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

    let mut aids = HashMap::new();

    // First we initialize the actor system using the default config
    let config = ActorSystemConfig::default();
    let system = ActorSystem::create(config);

    // Next we spawn each actor
    let feat_aid = system.spawn().name("orb_extract").with((), orb::orb_extract).unwrap();
    let align_aid = system.spawn().name("alignment").with((), align::align).unwrap();
    let vis_aid = system.spawn().name("vis_extract").with((), vis::Vis_extract).unwrap();

    // Save spawned actor ID's for lookup later
    aids.insert("feat".to_string(), feat_aid.clone());
    aids.insert("alilgn".to_string(), align_aid.clone());
    aids.insert("vis".to_string(), vis_aid.clone());

    // Kickoff the pipeline by sending the feature extraction module images
    feat_aid.send_new(orb::OrbMsg::new(img_paths, aids)).unwrap();
   
    system.await_shutdown(None);
}
