use std::{collections::HashMap, env};
use axiom::prelude::*;
use dvmap::map_actor::{MAP_ACTOR, MapActor};
use fern::colors::{ColoredLevelConfig, Color};
use glob::glob;
use log::{warn, info, error};
use yaml_rust::yaml;
use dvcore::{*, lockwrap::ReadWriteWrapper, config::*};

mod actors;
mod registered_modules;
mod dvmap;
mod modules;

use crate::actors::messages::ImagesMsg;
use crate::dvmap::map::Map;
use registered_modules::{FeatureManager, FRAME_LOADER};

fn main() {
    setup_logger().unwrap();

    let args: Vec<String> = env::args().collect();
    if args.len() < 3 {
        println!("[ERROR] Invalid number of input parameters.");
        println!("Usage: cargo run -- [PATH_TO_DATA_DIRECTORY] [PATH_TO_CONFIG_FILE]");
        return;
    }
    let img_dir = args[1].to_owned();
    let config_file = args[2].to_owned();

    // Load config, including custom settings and actor information
    if let Some((actor_info, module_info)) = load_config(&config_file) {
        // Get image paths from directory
        let img_paths = generate_image_paths(img_dir);
        // Create the global map
        let writeable_map = ReadWriteWrapper::new(Map::new());
        // Initialize actor system from config
        let actor_system = initialize_actor_system(actor_info, &writeable_map);
        // Create map actor
        let map_actor_aid = MapActor::spawn(&actor_system, writeable_map);

        info!("System ready to receive messages");

        // Kickoff the pipeline by sending the feature extraction module images
        let feat_aid = actor_system.find_aid_by_name(FRAME_LOADER).unwrap();

        feat_aid.send_new(ImagesMsg{ img_paths }).unwrap();

        actor_system.await_shutdown(None);
    } else {
        error!("Could not load config");
    }
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
) -> ActorSystem {
    let system = ActorSystem::create(ActorSystemConfig::default());

    //TODO (low priority): Use Cluster manager to do remote agent calls

    // Loop through the config to initialize actor system using the default config
    for actor_conf in modules {
        let actname = actor_conf.name.clone();
        info!("Spawning actor;{}", &actor_conf.name);

        // Note: read_only() is important here, otherwise all actors can
        // access the write lock of the map
        let _ = system.spawn().name(&actor_conf.name).with(
            FeatureManager::new(
                &actor_conf.actor_function,
                &actor_conf.actor_function, 
                writeable_map.read_only()
            ),
            FeatureManager::handle
        ).unwrap();

        // TODO (low priority): Identify the role of using connect_with_channels, as the system communications are working without doing the following.
        //features.push(actname.clone());
        // if i > 0 { 
        //     ActorSystem::connect_with_channels(&systems.get(&features[0]).unwrap(), &system_current);            
        // }
        //i+=1;
    }

    system
}

fn setup_logger() -> Result<(), fern::InitError> {
    let colors = ColoredLevelConfig::new()
        .info(Color::Green)
        .warn(Color::Yellow)
        .error(Color::Red);

    // fern::Dispatch::new()
    // .format(move |out, message, record| {
    //     out.finish(format_args!(
    //         "{color_line}[{date}][{target}][{level}{color_line}] {message}\x1B[0m",
    //         color_line = format_args!(
    //             "\x1B[{}m",
    //             colors.get_color(&record.level()).to_fg_str()
    //         ),
    //         date = chrono::Local::now().format("%Y-%m-%d %H:%M:%S"),
    //         target = record.target(),
    //         level = colors.color(record.level()),
    //         message = message,
    //     ));
    // });


    fern::Dispatch::new()
        .format(move |out, message, record| {
            out.finish(format_args!(
                "{color_line}[{time}][{target}][{level}{color_line}] {message}\x1B[0m",
                color_line = format_args!(
                    "\x1B[{}m",
                    colors.get_color(&record.level()).to_fg_str()
                ),
                level = colors.color(record.level()),
                time = chrono::Local::now().format("%s%.6f"), // seconds since 1970-01-01 00:00 UTC
                target = record.target(),
                message = message
            ))
        })
        .level(log::LevelFilter::Info)
        .level_for("axiom", log::LevelFilter::Warn)
        .chain(std::io::stdout())
        .chain(fern::log_file("output.log")?)
        .apply()?;
    Ok(())
}
