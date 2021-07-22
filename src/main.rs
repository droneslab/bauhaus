extern crate yaml_rust;
use yaml_rust::YamlLoader;
use yaml_rust::yaml;
use yaml_rust::yaml::Yaml;
use yaml_rust::yaml::Hash;
use std::env;
use glob::glob;
use axiom::prelude::*;
use axiom::cluster::*;
use std::fs::File;
use std::collections::HashMap;
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

fn read_config_file(str: &mut String, file_name: &String) {
   let mut f = File::open(file_name).unwrap();
   let mut s = String::new();
   f.read_to_string(str).unwrap();
}

fn load_config(str: &mut String, all_modules: &mut Vec<base::ModuleConf>) {

    let docs = yaml::YamlLoader::load_from_str(&str).unwrap();
    for doc in &docs {

        let v = doc["modules"].as_vec().unwrap();
        for i in 0..v.len() {

            let h = &v[i].as_hash().unwrap();
            let mut mconf = base::ModuleConf::default();
            mconf.module_name = h[&Yaml::String("name".to_string())].as_str().unwrap().to_string();
            mconf.module_file = h[&Yaml::String("file".to_string())].as_str().unwrap().to_string();
            mconf.module = h[&Yaml::String("module".to_string())].as_str().unwrap().to_string();
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

        }
    }

}

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
    
    // Load the config file 
    let mut conf_str = String::new();
    read_config_file(&mut conf_str, &args[2]);

    // Create module configuration objects
    let mut all_modules = Vec::<base::ModuleConf>::new();
    load_config(&mut conf_str, &mut all_modules);
    

    // First we initialize the actor system using the default config
    let config = ActorSystemConfig::default();
    let system = ActorSystem::create(config);

    let align_aid = system.spawn().name("alignment").with((), align::align).unwrap();
    //let align_aid = system.spawn().name(all_modules[0].module_name).with((), all_modules[0].module_file::all_modules[0].module).unwrap();
    
    let feat_aid = system.spawn().name("orb_extract").with((), orb::orb_extract).unwrap();
    //let feat_aid = system.spawn().name(all_modules[1].module_name).with((), all_modules[1].module_file::all_modules[1].module).unwrap();
    feat_aid.send_new(orb::OrbMsg::new(img_paths, align_aid)).unwrap();

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
