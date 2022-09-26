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
use darvis::{
    base,
    map::map_actor::{MapActor, MAP_ACTOR},
    lockwrap::ReadWriteWrapper,
    map::map::Map,
    load_config::*,
};
mod modules;
mod registered_modules;
use registered_modules::*;
use crate::modules::messages::images_msg::ImagesMsg;

fn load_config(config_file: String) -> Vec<base::ActorConf> {
    let mut module_info = Vec::<base::ActorConf>::new();

    let config_string = read_config_file(&config_file);
    load_modules_from_config(&config_string, &mut module_info);

    // Load additional custom settings from config file
    let yaml = &yaml::YamlLoader::load_from_str(&config_string).unwrap()[0]["system_settings"];
    println!("SYSTEM SETTINGS");
    add_system_setting_bool(yaml, "show_ui");
    add_system_setting_bool(yaml, "localization_only_mode");
    add_system_setting_sensor(yaml);
    // camera calibration
    add_system_setting_f64(yaml, "camera_fx");
    add_system_setting_f64(yaml, "camera_fy");
    add_system_setting_f64(yaml, "camera_cx");
    add_system_setting_f64(yaml, "camera_cy");
    add_system_setting_f64(yaml, "camera_k1");
    add_system_setting_f64(yaml, "camera_k2");
    add_system_setting_f64(yaml, "camera_p1");
    add_system_setting_f64(yaml, "camera_p2");
    add_system_setting_i32(yaml, "camera_width");
    add_system_setting_i32(yaml, "camera_height");
    add_system_setting_f64(yaml, "fps");
    add_system_setting_f64(yaml, "camera_bf");
    add_system_setting_i32(yaml, "thdepth");
    // feature detection
    add_system_setting_i32(yaml, "max_features");
    add_system_setting_f64(yaml, "scale_factor");
    add_system_setting_i32(yaml, "n_levels");
    add_system_setting_i32(yaml, "fast_threshold");
    // tracking
    add_system_setting_i32(yaml, "recently_lost_cutoff");
    add_system_setting_i32(yaml, "frames_to_reset_IMU");
    add_system_setting_bool(yaml, "insert_KFs_when_lost");
    add_system_setting_i32(yaml, "min_num_features");

    module_info
}

fn generate_image_paths(img_dir: String) -> Vec<String> {
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
    img_paths
}

fn initialize_actor_system(
    modules: Vec::<base::ActorConf>, writeable_map: &ReadWriteWrapper<Map>
) -> (HashMap::<String, ActorSystem>, HashMap::<String, Aid>) {
    let mut systems  = HashMap::<String, ActorSystem>::new();
    let mut aids = HashMap::new();

    //TODO: Use Cluster manager to do remote agent calls

    // Loop through the config to initialize actor system using the default config
    for actor_conf in modules {
        let actname = actor_conf.name.clone();
        let system_current = ActorSystem::create(ActorSystemConfig::default());
        systems.insert(actname.clone(), system_current.clone());

        // read_only() is important here, otherwise all actors can
        // access the write lock of the map
        let c_aid  = system_current.spawn().name(&actor_conf.name).with(
            FeatureManager::new(
                &actor_conf.actor_function,
                &actor_conf.actor_function, 
                writeable_map.read_only()
            ),
            FeatureManager::handle
        ).unwrap();
        aids.insert(actname.clone(), c_aid.clone());

        // TODO: Identify the role of using connect_with_channels, as the system communications are working without doing the following.
        //features.push(actname.clone());
        // if i > 0 { 
        //     ActorSystem::connect_with_channels(&systems.get(&features[0]).unwrap(), &system_current);            
        // }
        //i+=1;
    }
    //println!("{:?}",aids);

    (systems, aids)
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

    // Load config, including custom settings and actor information
    let module_info = load_config(config_file);
    // Get image paths from directory
    let img_paths = generate_image_paths(img_dir);
    // Create the global map
    let writeable_map = ReadWriteWrapper::new(Map::new());
    // Initialize actor system from config
    let (systems, mut aids) = initialize_actor_system(module_info, &writeable_map);
    // Create map actor
    let map_actor_aid = MapActor::spawn(writeable_map);
    aids.insert(MAP_ACTOR.to_string(), map_actor_aid);

    // Kickoff the pipeline by sending the feature extraction module images
    let system = systems.get(FRAME_LOADER).unwrap();
    let feat_aid = system.find_aid_by_name(FRAME_LOADER).unwrap();
    feat_aid.send_new(ImagesMsg::new(img_paths, aids.clone())).unwrap();

    system.await_shutdown(None);
}
