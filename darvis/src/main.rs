use std::{collections::HashMap, env};
use axiom::prelude::*;
use dvmap::map_actor::{MAP_ACTOR, MapActor};
use glob::glob;
use log::LevelFilter;
use yaml_rust::yaml;
use dvcore::{*, lockwrap::ReadWriteWrapper, global_params::*};

mod actors;
mod registered_modules;
mod dvmap;
mod modules;

use crate::actors::messages::ImagesMsg;
use crate::dvmap::{sensor::*, map::Map};
use registered_modules::{FeatureManager, FRAME_LOADER};

fn main() {
    setup_logger();

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

    // Note: This is so annoying but the map initialization needs to be in 
    // its own function, otherwise the compiler complains that it doesn't
    // know the type of S in the map. This way it generates duplicate
    // version of init() for each SensorType. Another option is to use
    // dyn, but this allows us to keep static dispatching. The map and S
    // are used quite a few times so it seems like a huge slowdown to use
    // dynamic dispatch.
    let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");
    match GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor") {
        Sensor::Mono => init::<MonoSensor>(module_info, img_dir),
        Sensor::ImuMono => init::<ImuMonoSensor>(module_info, img_dir),
        Sensor::Stereo => init::<StereoSensor>(module_info, img_dir),
        Sensor::ImuStereo => init::<ImuStereoSensor>(module_info, img_dir),
        Sensor::Rgbd => init::<RgbdSensor>(module_info, img_dir),
        Sensor::ImuRgbd => init::<ImuRgbdSensor>(module_info, img_dir),
    }
}

fn init<S: SensorType + 'static> (module_info: Vec<base::ActorConf>, img_dir: String) {
    // Get image paths from directory
    let img_paths = generate_image_paths(img_dir);
    // Create the global map
    let writeable_map: ReadWriteWrapper<Map<S>> = ReadWriteWrapper::new(Map::new::<S>());
    // Initialize actor system from config
    let (systems, mut aids) = initialize_actor_system(module_info, &writeable_map);
    // Create map actor
    let map_actor_aid = MapActor::<S>::spawn(writeable_map);
    aids.insert(MAP_ACTOR.to_string(), map_actor_aid);

    // Kickoff the pipeline by sending the feature extraction module images
    let system = systems.get(FRAME_LOADER).unwrap();
    let feat_aid = system.find_aid_by_name(FRAME_LOADER).unwrap();
    feat_aid.send_new(ImagesMsg{ img_paths }).unwrap();

    system.await_shutdown(None);
}

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
    add_system_setting_string(yaml, "vocabulary_file");
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
    add_system_setting_f64(yaml, "stereo_baseline_times_fx");
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
    // local mapping
    add_system_setting_f64(yaml, "far_points_threshold");

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

fn initialize_actor_system<S: SensorType + 'static>(
    modules: Vec::<base::ActorConf>, writeable_map: &ReadWriteWrapper<Map<S>>
) -> (HashMap::<String, ActorSystem>, HashMap::<String, Aid>) {
    let mut systems  = HashMap::<String, ActorSystem>::new();
    let mut aids = HashMap::new();

    //TODO (low priority): Use Cluster manager to do remote agent calls

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

        // TODO (low priority): Identify the role of using connect_with_channels, as the system communications are working without doing the following.
        //features.push(actname.clone());
        // if i > 0 { 
        //     ActorSystem::connect_with_channels(&systems.get(&features[0]).unwrap(), &system_current);            
        // }
        //i+=1;
    }
    //println!("{:?}",aids);

    (systems, aids)
}

fn setup_logger() -> Result<(), fern::InitError> {
    fern::Dispatch::new()
        .format(|out, message, record| {
            out.finish(format_args!(
                "{}[{}][{}] {}",
                chrono::Local::now().format("[%H:%M:%S]"),
                record.target(),
                record.level(),
                message
            ))
        })
        .level(log::LevelFilter::Debug)
        .chain(std::io::stdout())
        .chain(fern::log_file("output.log")?)
        .apply()?;
    Ok(())
}
