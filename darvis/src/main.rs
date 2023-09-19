#![feature(map_many_mut)]
extern crate flame;
use std::{fs::{OpenOptions}, path::Path, env};
use fern::colors::{ColoredLevelConfig, Color};
use glob::glob;
use log::{warn, info};
use spin_sleep::LoopHelper;
#[macro_use]
extern crate lazy_static;

use dvcore::{*, config::*, base::{ActorChannels}};
use crate::{actors::{messages::{ImageMsg, ShutdownMessage}}, registered_actors::{TRACKING_FRONTEND, VISUALIZER}};
use crate::dvmap::{bow::VOCABULARY, map::Id};

mod actors;
mod registered_actors;
mod initialize_system;
mod dvmap;
mod modules;
mod tests;

pub static RESULTS_FOLDER: &str = "results";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    setup_logger()?;

    // Process arguments
    let args: Vec<String> = env::args().collect();
    if args.len() < 3 {
        panic!("
            [ERROR] Invalid number of input parameters.
            Usage: cargo run -- [PATH_TO_DATA_DIRECTORY] [PATH_TO_CONFIG_FILE]
        ");
    }
    let img_dir = args[1].to_owned();
    let config_file = args[2].to_owned();

    // Load config, including custom settings and actor information
    let (actor_info, _module_info) = load_config(&config_file).expect("Could not load config");

    // Actor that launches the pipeline
    let first_actor_name = match GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "show_visualizer") {
        true => VISUALIZER.to_string(),
        false => TRACKING_FRONTEND.to_string()
    };

    // Launch actor system
    let (shutdown_flag, first_actor_tx, shutdown_tx) = initialize_system::initialize_actors(actor_info.clone(), first_actor_name)?;

    // Load vocabulary
    VOCABULARY.access();

    info!("System ready to receive messages");

    // Run loop at fps rate
    let target_fps = GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "fps");
    let mut loop_helper = LoopHelper::builder()
        .report_interval_s(0.5)
        .build_with_target_rate(target_fps);

    let current_timestamp = target_fps; // TODO (MVP): For evaluation this needs to be a real timestamp from the associated timestamp file.
    // Process images
    for path in &generate_image_paths(img_dir) {
        if *shutdown_flag.lock().unwrap() { break; }
        let _delta = loop_helper.loop_start(); 

        first_actor_tx.send(Box::new(ImageMsg{image_path: path.clone()}))?;

        info!("Read image {}", path.split("/").last().unwrap().split(".").nth(0).unwrap());

        // ORBSLAM3 frame loader scales and resizes image, do we need this?
        // After looking into it for a while, I think not. They have the code to scale,
        // but then never set the variable mImageScale to anything but 1.0
        // https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Examples/RGB-D/rgbd_tum.cc#L89

        loop_helper.loop_sleep(); 
    }
    shutdown_tx.send(Box::new(ShutdownMessage{}))?;

    Ok(())
}

fn generate_image_paths(img_dir: String) -> Vec<String> {
    let mut glob_str = img_dir.to_owned();
    glob_str.push_str("/*.png");
    let mut img_paths = Vec::new();

    for entry in glob(&glob_str).expect("Failed to read glob pattern") {
        match entry {
            Ok(path) => match path.to_str() {
                Some(path_str) => img_paths.push(path_str.to_owned()),
                None => panic!("Invalid path found!"),
            },
            Err(e) => panic!("{:?}", e),
        }
    }
    img_paths
}


fn setup_logger() -> Result<(), fern::InitError> {
    let colors = ColoredLevelConfig::new()
        .info(Color::Green)
        .warn(Color::Yellow)
        .error(Color::Red);

    // Two loggers in chain - one for terminal output (colored, easier to read)
    // and another for file output (not colored, info on each line deliminated by |
    // for easier parsing in python scripts)
    let start_time = chrono::Local::now();
    fern::Dispatch::new()
    .level(log::LevelFilter::Debug)
    .level_for("axiom", log::LevelFilter::Warn)
    .chain(
    fern::Dispatch::new()
            .format(move |out, message, record| {
                out.finish(format_args!(
                    "{color_line}[{time}][{target}][{level}{color_line}] {message}\x1B[0m",
                    color_line = format_args!(
                        "\x1B[{}m",
                        colors.get_color(&record.level()).to_fg_str()
                    ),
                    level = colors.color(record.level()),
                    time = (chrono::Local::now() - start_time).num_milliseconds() as f64 / 1000.0,
                    target = record.target(),
                    message = message
                ))
            })
            .chain(std::io::stdout()),
    )
    .chain(
    fern::Dispatch::new()
            .format(move |out, message, record| {
                out.finish(format_args!(
                    "{time}|{target}|{message}",
                    time = (chrono::Local::now() - start_time).num_milliseconds() as f64  / 1000.0,
                    target = record.target(),
                    message = message
                ))
            })
            .chain(OpenOptions::new()
                .write(true)
                .create(true)
                .open(Path::new(RESULTS_FOLDER).join("output.log"))?
            )
    )
    .apply()?;

    Ok(())
}
