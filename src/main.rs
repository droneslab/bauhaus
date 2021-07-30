use std::env;
use glob::glob;
use axiom::prelude::*;
use axiom::cluster::*;
use std::io::prelude::*;
// use std::io::{BufReader, BufWriter};
use std::net::{SocketAddr};
// use std::sync::atomic::{AtomicBool, Ordering};
// use std::sync::{Arc, RwLock};
// use std::sync::{Condvar, Mutex};
// use std::thread;
// use std::thread::JoinHandle;
use std::time::Duration;
use log::LevelFilter;

mod base;
mod orb;
mod align;
mod utils;
mod config;

fn main() {

    env_logger::builder() 
        .filter_level(LevelFilter::Info)
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

    // Spawn all actors
    for actor_conf in modules {
        // TODO: Need to add message/function tables (hashmaps) somewhere that map sttrings to Fn pointers
        // https://stackoverflow.com/questions/30540807/calling-a-function-only-known-at-runtime
        println!("{:?}", actor_conf);
        // let test = actor_conf.file as module;
        // let new_aid = system.spawn().name(actor_conf.name).with((), actor_conf.file::actor_conf.actor_function).unwrap();
    }

    // let align_aid = system.spawn().name("alignment").with((), align::align).unwrap();
    //let align_aid = system.spawn().name(all_modules[0].module_name).with((), all_modules[0].module_file::all_modules[0].module).unwrap();
    
    // let feat_aid = system.spawn().name("orb_extract").with((), orb::orb_extract).unwrap();
    //let feat_aid = system.spawn().name(all_modules[1].module_name).with((), all_modules[1].module_file::all_modules[1].module).unwrap();
    // feat_aid.send_new(orb::OrbMsg::new(img_paths, align_aid)).unwrap();

    /***********************************************************************************************/

    /*let socket_addr1 = SocketAddr::from(([127, 0, 0, 1], 7717));
    let system1 = ActorSystem::create(ActorSystemConfig::default().thread_pool_size(2));
    let cluster_mgr1 = TcpClusterMgr::create(&system1, socket_addr1);

    let socket_addr2 = SocketAddr::from(([127, 0, 0, 1], 7727));
    let system2 = ActorSystem::create(ActorSystemConfig::default().thread_pool_size(2));
    let _cluster_mgr2 = TcpClusterMgr::create(&system2, socket_addr2);

    // Connect both actor systems
    cluster_mgr1.connect(socket_addr2, Duration::from_millis(1000)).unwrap();

    // Spawn actors, orb on sys1 align on sys2
    let feat_aid = system1.spawn().name("orb_extract").with((), orb::orb_extract).unwrap();
    let align_aid = system2.spawn().name("alignment").with((), align::align).unwrap();

    // Send images
    feat_aid.send_new(orb::OrbMsg::new(img_paths, align_aid)).unwrap();

    // The actor will trigger shutdown, we just wait for it
    system1.await_shutdown(None);
    system2.await_shutdown(None);*/
    system.await_shutdown(None);
}
