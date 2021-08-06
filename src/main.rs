use std::env;
use glob::glob;
use axiom::prelude::*;
use axiom::cluster::*;

use std::collections::HashMap;
use std::net::{SocketAddr};
use std::time::Duration;
use log::LevelFilter;

#[allow(unused_imports)]
use opencv::{
    prelude::*,
    core,
    features2d,
    features2d::{Feature2DTrait, ORB},
    highgui,
    imgproc,
    videoio,
    imgcodecs,
    types::{PtrOfORB, VectorOfKeyPoint},
};
use opencv::core::CV_32FC1;

extern crate nalgebra as na;
use na::*;

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

    //blank image for graph using opencv
    let mut img: Mat = Mat::new_rows_cols_with_default(400, 400, CV_32FC1, opencv::core::Scalar::all(0.0)).unwrap();                // create 400x400 image
    let mut img_na = utils::cv_mat_to_na_grayscale(&img);

    let mut aids = HashMap::new();

    // First we initialize the actor system using the default config
    let config = ActorSystemConfig::default();
    let system = ActorSystem::create(config);

    // Next we spawn each actor
    let feat_aid = system.spawn().name("orb_extract").with((), orb::orb_extract).unwrap();
    let align_aid = system.spawn().name("alignment").with((), align::align).unwrap();
    let vis_aid = system.spawn().name("vis_extract").with((), vis::vis_extract).unwrap();

    // Save spawned actor ID's for lookup later
    aids.insert("feat".to_string(), feat_aid.clone());
    aids.insert("alilgn".to_string(), align_aid.clone());
    aids.insert("vis".to_string(), vis_aid.clone());

    // Kickoff the pipeline by sending the feature extraction module images
    feat_aid.send_new(orb::OrbMsg::new(img_paths, img_na.clone(), aids)).unwrap();
   
    system.await_shutdown(None);
}
