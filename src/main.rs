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
mod dvutils;
mod config;
mod vis;
mod pluginfunction;
mod registerplugin;

use axiom::cluster::*;
use std::net::{SocketAddr};
use std::time::Duration;


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
    //let config = ActorSystemConfig::default();
    //let system = ActorSystem::create(config);

    // TODO: Spawn all actors based on config, need a table to map string names to functions
//     for actor_conf in modules {
//         // TODO: Need to add message/function tables (hashmaps) somewhere that map sttrings to Fn pointers
//         // https://stackoverflow.com/questions/30540807/calling-a-function-only-known-at-runtime
//         println!("{:?}", actor_conf);
//         // let test = actor_conf.file as module;
//         // let new_aid = system.spawn().name(actor_conf.name).with((), actor_conf.file::actor_conf.actor_function).unwrap();
//     }

    // let mut socket_addr1 = "localhost:7717".to_socket_addrs().unwrap().next().unwrap();
    // //let socket_addr1 = SocketAddr::from(addrs_iter);
    // let cluster_mgr1 = TcpClusterMgr::create(&system, socket_addr1);

    let mut features = vec![];
    let mut systems  = HashMap::<String, ActorSystem>::new();

    let mut clusters  = HashMap::<String, TcpClusterMgr>::new();

    let mut socket_addrs  = HashMap::<String, SocketAddr>::new();

    let mut aids = HashMap::new();

    for actor_conf in modules {
        let actname = actor_conf.name.clone();
        features.push(actname.clone());

        let port = actor_conf.port.parse::<u16>().unwrap();
        let socket_addr1 = SocketAddr::from(([127, 0, 0, 1], port));
        socket_addrs.insert(actname.clone(), socket_addr1.clone());
        let system1 = ActorSystem::create(ActorSystemConfig::default());
        
        systems.insert(actname.clone(), system1.clone());
        let cluster_mgr1 = TcpClusterMgr::create(&system1, socket_addr1);
        clusters.insert(actname.clone(), cluster_mgr1);
        

        let c_aid  = system1.spawn().name(&actor_conf.name).with(registerplugin::FeatureManager::new(&actor_conf.actor_function,&actor_conf.actor_function), registerplugin::FeatureManager::handle).unwrap();
        
        aids.insert(actname.clone(), c_aid.clone());

    }

    let cluster_mgr1 = clusters.get("feature_extraction").unwrap();
    for (i, feature) in features.iter().enumerate() {
        if i == 0 { continue; }
        println!("Index = {}, Feature = {}", i, feature);
        
        let socket_addr2 = socket_addrs.get(feature).unwrap();
        cluster_mgr1
        .connect(*socket_addr2, Duration::from_millis(2000))
        .unwrap();

        ActorSystem::connect_with_channels(&systems.get(&features[0]).unwrap(), &systems.get(feature).unwrap());
    }



    let system = systems.get("feature_extraction").unwrap();


    let feat_aid = system.find_aid_by_name("feature_extraction").unwrap();
    //Kickoff the pipeline by sending the feature extraction module images
    feat_aid.send_new(orb::OrbMsg::new(img_paths.clone(), aids)).unwrap();
   
    system.await_shutdown(None);
}




