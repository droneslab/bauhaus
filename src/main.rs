use std::env;
use glob::glob;
use axiom::prelude::*;
use axiom::cluster::*;

// use std::collections::HashMap;
// use std::io::prelude::*;
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

    // First we initialize the actor system using the default config
    // let config = ActorSystemConfig::default();
    // let system = ActorSystem::create(config);

    // // Spawn actors
    // let feat_aid = system.spawn().name("orb_extract").with((), orb::orb_extract).unwrap();
    // let align_aid = system.spawn().name("alignment").with((), align::align).unwrap();

    // // Send images
    // feat_aid.send_new(orb::OrbMsg::new(img_paths)).unwrap();

    /***********************************************************************************************/

    let socket_addr1 = SocketAddr::from(([127, 0, 0, 1], 7717));
    let system1 = ActorSystem::create(ActorSystemConfig::default().thread_pool_size(2));
    let cluster_mgr1 = TcpClusterMgr::create(&system1, socket_addr1);

    let socket_addr2 = SocketAddr::from(([127, 0, 0, 1], 7727));
    let system2 = ActorSystem::create(ActorSystemConfig::default().thread_pool_size(2));
    let _cluster_mgr2 = TcpClusterMgr::create(&system2, socket_addr2);

    cluster_mgr1
        .connect(socket_addr2, Duration::from_millis(1000))
        .unwrap();

    // The actor will trigger shutdown, we just wait for it
    // system.await_shutdown(None);
}
